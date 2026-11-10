"""Session coordination. Apex owns Pico/body/grippers; chassis owns motion output.

ControlGate is an in-process lifecycle gate, NOT a robot-wide ownership lock.
Native body/gripper emergency stopping remains the operator's physical E-stop.
"""
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]


class ControlGate:
    def __init__(self):
        self.owner = None
        self.armed = False
        self.deadline = 0.0
        self.connected = False
        self.adapter = None

    def acquire(self, owner):
        if owner not in ('teleop', 'reset', 'replay') or self.owner is not None:
            raise ValueError('invalid or already owned control gate')
        self.owner = owner
        self.stop()

    def connect(self):
        self.connected = True
        self.stop()

    def arm(self, ttl_s):
        if not self.owner or not self.connected or not math.isfinite(ttl_s) or not 0 < ttl_s <= 1:
            raise ValueError('cannot arm')
        self.deadline = time.monotonic() + ttl_s
        self.armed = True

    def stop(self):
        self.armed = False
        self.deadline = 0.0

    def disconnect(self):
        self.connected = False
        self.stop()

    def release(self):
        self.stop()
        self.owner = None
        self.adapter = None

    def check(self):
        if time.monotonic() >= self.deadline:
            self.stop()
        return self.armed and self.connected

    def execute(self, *args, **kwargs):
        if args or kwargs or not self.check() or self.owner != 'teleop' or self.adapter is None:
            raise ValueError('execution requires a fresh gate and chassis adapter')
        return self.adapter.start()


# Sent over SSH in memory. It only subscribes and performs HTTP GET requests.
# No ROS CLI daemon, publishers, service calls, files, or native service changes.
NATIVE_OBSERVER = r'''
import json,math,os,sys,threading,time,urllib.request,select,socket,struct
import rclpy
from std_msgs.msg import Bool,Float32MultiArray
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
rclpy.init(args=['--ros-args','--disable-external-lib-logs'])
node=rclpy.create_node('gento_teleop_observer',enable_rosout=False)
latest={}; backend={}; backend_at=None; backend_error=None
opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
def query(path):
    with opener.open('http://127.0.0.1:8080/api/v1/'+path,timeout=.5) as response:
        result=json.load(response)
    if result.get('ok') is not True:raise ValueError('backend rejected GET')
    return result['data']
def backend_loop():
    global backend,backend_at,backend_error
    while True:
        try:
            status=query('robot/status'); modules=query('modules')
            backend={'status':status,'modules':modules}
            backend_at=time.monotonic();backend_error=None
        except Exception as exc:backend_error=type(exc).__name__+': '+str(exc)
        time.sleep(.2)
threading.Thread(target=backend_loop,daemon=True).start()
def received(name,message):
    data=list(message.position) if name=='body' else message.data
    if name!='vr':
        data=list(data)
        if not data or not all(math.isfinite(v) for v in data):return
    latest[name]=(time.monotonic(),data)
subscriptions=[]
for name,topic,kind in [
    ('vr','/tj/info/vr_connected',Bool),
    ('body','/tj/joint_states',JointState),
    ('left_gripper','/tj/info/gripper_feedback_L',Float32MultiArray),
    ('right_gripper','/tj/info/gripper_feedback_R',Float32MultiArray)]:
    subscriptions.append(node.create_subscription(kind,topic,lambda m,n=name:received(n,m),qos_profile_sensor_data))
# Passive listeners only. These IDs match the deployed Apex dm_gripper.yaml.
# Never count outgoing MIT commands (slave ID) as motor feedback.
can_sockets={}; can_error=None
try:
    for name,interface,motor in [('left_gripper_can','vcan0',1),('right_gripper_can','vcan1',2)]:
        sock=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
        can_sockets[sock]=(name,motor)
        sock.setsockopt(socket.SOL_CAN_RAW,socket.CAN_RAW_FILTER,
            struct.pack('=IIII',motor+0x10,0xC00007FF,0,0xC00007FF))
        sock.bind((interface,));sock.setblocking(False)
except OSError as exc:
    can_error=str(exc)
    for sock in can_sockets:sock.close()
    can_sockets={}
def read_motor_feedback():
    for sock,(name,motor) in can_sockets.items():
        for _ in range(256):
            try:frame=sock.recv(16)
            except BlockingIOError:break
            if len(frame)!=16:continue
            cid,dlc=struct.unpack('=IB',frame[:5]);payload=frame[8:]
            if cid not in (0,motor+0x10) or dlc!=8 or payload[0]&15!=motor:continue
            latest[name]=(time.monotonic(),{'can_id':cid,'state':payload[0]>>4})
sequence=0; pending=b''
os.set_blocking(sys.stdin.fileno(),False)
try:
    while rclpy.ok():
        rclpy.spin_once(node,timeout_sec=.02)
        read_motor_feedback()
        now=time.monotonic()
        if not select.select([sys.stdin],[],[],0)[0]:continue
        raw=os.read(sys.stdin.fileno(),4096)
        if not raw:break
        pending+=raw
        if len(pending)>4096:raise ValueError('observer request too large')
        if b'\n' not in pending:continue
        line,pending=pending.split(b'\n',1)
        request=json.loads(line)
        if request.get('type')!='poll':raise ValueError('invalid observer request')
        sequence+=1
        samples={k:{'age_s':now-v[0],'value':v[1]} for k,v in latest.items()}
        print(json.dumps({'type':'native_status','sequence':sequence,'nonce':request['nonce'],
            'ros_domain_id':int(os.environ['ROS_DOMAIN_ID']), 'samples':samples,
            'backend':backend,'backend_age_s':None if backend_at is None else now-backend_at,
            'backend_error':backend_error,'gripper_can_error':can_error},allow_nan=False),flush=True)
finally:
    for sock in can_sockets:sock.close()
    node.destroy_node();rclpy.shutdown()
'''


def native_command(config):
    robot = config['robot_ssh']
    domain = config['ros_domain_id']
    if not isinstance(robot, str) or not re.fullmatch(r'[A-Za-z0-9_.@:-]+', robot) or robot.startswith('-'):
        raise ValueError('invalid robot SSH destination')
    if type(domain) is not int or not 0 <= domain <= 232:
        raise ValueError('ROS domain must be an integer in 0..232')
    script = ('source /etc/apex/apex_ros_env.sh >&2; '
              'export PYTHONDONTWRITEBYTECODE=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_DOMAIN_ID=' + str(domain) + '; '
              'exec python3 -u -B -c ' + shlex.quote(NATIVE_OBSERVER))
    return ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
            '-o', 'ConnectTimeout=5', '-o', 'ServerAliveInterval=1',
            '-o', 'ServerAliveCountMax=3', robot, 'bash -e -c ' + shlex.quote(script)]


def native_problems(row, config):
    problems = []
    def fresh(value, limit=None):
        return type(value) in (int, float) and math.isfinite(value) and 0 <= value < (config['feedback_timeout_s'] if limit is None else limit)
    if row.get('ros_domain_id') != config['ros_domain_id']:
        problems.append('native ROS domain mismatch')
    if row.get('backend_error') or not fresh(row.get('backend_age_s')):
        problems.append('native backend unavailable or stale')
    backend = row.get('backend', {})
    modules = {m.get('id'): m for m in backend.get('modules', [])}
    for name in ('robot', 'teleop', 'tool'):
        module = modules.get(name, {})
        if module.get('active_state') != 'active' or module.get('error') is not False:
            problems.append('native module unavailable: ' + name)
    status = backend.get('status', {})
    if status.get('control_ready') is not True or status.get('control_error') is not False or status.get('control_state_healthy') is not True:
        problems.append('native robot not ready')
    for key, expected in (('current_input_mode', 1), ('current_ik_input_source', 1), ('base_mode', 0)):
        if status.get(key) != expected:
            problems.append('native mode mismatch: ' + key)
    samples = row.get('samples', {})
    for name in ('vr', 'body', 'left_gripper', 'right_gripper'):
        sample = samples.get(name, {})
        if not fresh(sample.get('age_s'), config['vr_timeout_s'] if name == 'vr' else None):
            problems.append('missing/stale native feedback: ' + name)
        if name == 'vr' and sample.get('value') is not True:
            problems.append('Pico disconnected')
    if row.get('gripper_can_error'):
        problems.append('gripper CAN observer unavailable: ' + row['gripper_can_error'])
    for name in ('left_gripper_can', 'right_gripper_can'):
        sample = samples.get(name, {})
        if not fresh(sample.get('age_s')):
            problems.append('missing/stale motor CAN feedback: ' + name)
        # DM status 1 is enabled; 0 is disabled and other values are faults.
        if sample.get('value', {}).get('state') != 1:
            problems.append('motor not confirmed enabled: ' + name)
    return problems


class JsonProcess:
    """Bounded nonblocking JSON reader, so a child cannot hang the coordinator."""
    def __init__(self, command, errors, env=None, input_pipe=False):
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE if input_pipe else subprocess.DEVNULL, stdout=subprocess.PIPE,
                                        stderr=errors, env=env, start_new_session=True, bufsize=0)
        os.set_blocking(self.process.stdout.fileno(), False)
        self.buffer = b''
        self.eof = False

    def receive(self):
        rows = []
        for _ in range(32):
            try:
                raw = os.read(self.process.stdout.fileno(), 65536)
            except BlockingIOError:
                break
            if not raw:
                self.eof = True
                break
            self.buffer += raw
            if len(self.buffer) > 1048576:
                raise ValueError('child output exceeded bounded buffer')
            while b'\n' in self.buffer:
                line, self.buffer = self.buffer.split(b'\n', 1)
                if line:
                    row = json.loads(line)
                    if not isinstance(row, dict):raise ValueError('invalid child JSON object')
                    rows.append(row)
        return rows

    def terminate(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGTERM)

    def close(self):
        self.terminate()
        try:self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=2)
        self.process.stdout.close()
        if self.process.stdin is not None:self.process.stdin.close()


class ChassisAdapter:
    """Uses the existing sender including its feedback, A return and stop handshake."""
    def __init__(self, config, errors, seconds, env=None):
        self.config, self.errors, self.seconds, self.env = config, errors, seconds, env
        self.child = None
        self.stop_confirmed = False

    def command(self, mode='execute'):
        c = self.config
        return [str(ROOT/'.venv/bin/python'), '-u', '-B',
                str(ROOT/'src/teleop/chassis/src/teleop/sender.py'), '--'+mode,
                '--json-output', '--robot', c['robot_ssh'], '--remote-dir', c['chassis_remote_dir'],
                '--device', c['device'],
                '--hidraw', c['hidraw'], '--seconds', str(self.seconds),
                '--parent-pid', str(os.getpid())]

    def start(self):
        if self.child is not None:raise ValueError('chassis session already started')
        self.child = JsonProcess(self.command(), self.errors, self.env)
        return self.child

    def observe(self, row):
        if row.get('type') == 'final':
            self.stop_confirmed = row.get('status') == 'STOP_CONFIRMED' and row.get('stop_confirmed') is True

    def stop(self, record):
        if self.child is None:return {'started':False, 'stop_confirmed':None}
        self.child.terminate()
        deadline = time.monotonic()+7
        try:
            while time.monotonic()<deadline:
                for row in self.child.receive():
                    self.observe(row)
                    try:record(dict(component='chassis', **row))
                    except OSError:pass  # A full log disk must not abort the stop handshake.
                if self.child.eof:break
                time.sleep(.01)
        finally:
            self.child.close()
        return {'started':True, 'stop_confirmed':self.stop_confirmed}


class NativeObserver(JsonProcess):
    """One outstanding challenge; queued pre-disconnection samples cannot renew a gate."""
    def __init__(self, config, errors, env=None):
        super().__init__(native_command(config), errors, env, input_pipe=True)
        self.config = config
        self.nonce = None
        self.sent_at = 0.0
        self.next_poll = 0.0
        self.first = True
        os.set_blocking(self.process.stdin.fileno(), False)

    def receive(self):
        import uuid
        now = time.monotonic()
        timeout = self.config['startup_timeout_s'] if self.first else self.config['observer_timeout_s']
        if self.nonce is not None and now-self.sent_at >= timeout:
            raise RuntimeError('native observer challenge timed out')
        rows = super().receive()
        for row in rows:
            if self.nonce is None or row.get('nonce') != self.nonce:
                raise ValueError('unexpected/replayed native observer response')
            round_trip = now-self.sent_at
            for sample in row.get('samples', {}).values():
                if type(sample.get('age_s')) in (int, float):sample['age_s'] += round_trip
            if type(row.get('backend_age_s')) in (int, float):row['backend_age_s'] += round_trip
            self.nonce = None
            self.first = False
            self.next_poll = now+.1
        if not self.eof and self.nonce is None and now >= self.next_poll:
            self.nonce = uuid.uuid4().hex
            raw = (json.dumps({'type':'poll','nonce':self.nonce})+'\n').encode()
            if os.write(self.process.stdin.fileno(), raw) != len(raw):
                raise RuntimeError('native observer request write failed')
            self.sent_at = now
        return rows


class GripperSlew:
    """Normalized position ramp; full-range travel takes the configured seconds."""
    def __init__(self, initial=0.0, close_seconds=3.0, open_seconds=3.0, max_dt=.05):
        for value in (close_seconds, open_seconds, max_dt):
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError('positive finite slew timing required')
        self.close_seconds = close_seconds
        self.open_seconds = open_seconds
        self.max_dt = max_dt
        self.last_at = None
        self.value = self._unit(initial)

    @staticmethod
    def _unit(value):
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= 1:
            raise ValueError('normalized gripper target must be finite and within [0, 1]')
        return float(value)

    def hold(self, value, now):
        value = self._unit(value)
        if not math.isfinite(now):
            raise ValueError('finite monotonic time required')
        self.value, self.last_at = value, now
        return self.value

    def advance(self, goal, now):
        goal = self._unit(goal)
        if not math.isfinite(now) or (self.last_at is not None and now < self.last_at):
            raise ValueError('monotonic time required')
        if self.last_at is None:
            self.last_at = now
            return self.value
        dt = min(now - self.last_at, self.max_dt)
        self.last_at = now
        seconds = self.close_seconds if goal > self.value else self.open_seconds
        delta = dt / seconds
        if goal > self.value:
            self.value = min(goal, self.value + delta)
        else:
            self.value = max(goal, self.value - delta)
        return self.value


class GripperChannel:
    """Gate teleop targets with measured feedback; optionally apply a position ramp."""
    def __init__(self, minimum=0.0, maximum=1.6, close_seconds=3.0, open_seconds=3.0,
                 timeout=.3, press_threshold=.1, release_threshold=.05, teleop_mode='slew'):
        if not all(math.isfinite(x) for x in (minimum, maximum, timeout,
                                              press_threshold, release_threshold)):
            raise ValueError('finite channel settings required')
        if maximum <= minimum or timeout <= 0 or not 0 <= release_threshold < press_threshold <= 1:
            raise ValueError('invalid gripper channel range or thresholds')
        if teleop_mode not in ('slew', 'direct'):
            raise ValueError('unknown gripper teleop mode')
        self.teleop_mode = teleop_mode
        self.minimum, self.maximum = minimum, maximum
        self.timeout = timeout
        self.press_threshold, self.release_threshold = press_threshold, release_threshold
        self.slew = GripperSlew(close_seconds=close_seconds, open_seconds=open_seconds)
        self.input = self.feedback = self.motor = None
        self.armed = self.pressed = False
        self.reason = 'waiting_feedback'

    def receive_target(self, value, now):
        self.input = (GripperSlew._unit(value), now)

    def receive_feedback(self, position, now):
        if not math.isfinite(position) or not self.minimum-.1 <= position <= self.maximum+.1:
            raise ValueError('invalid measured gripper position')
        value = max(0.0, min(1.0, (position-self.minimum)/(self.maximum-self.minimum)))
        self.feedback = (value, now)

    def receive_motor(self, state, now):
        self.motor = (state, now)

    def tick(self, now, vr_ready=True):
        def fresh(row):
            return row is not None and 0 <= now-row[1] <= self.timeout
        if not fresh(self.feedback) or not fresh(self.motor) or self.motor[0] != 1:
            self.armed = False
            self.reason = 'waiting_enabled_motor_feedback'
            self.slew.hold(self.slew.value, now)
            return None
        measured = self.feedback[0]
        if not vr_ready or not fresh(self.input):
            self.armed = self.pressed = False
            self.reason = 'holding_input_unavailable'
            return self.slew.hold(measured, now)
        if not self.armed:
            self.reason = 'release_before_enable'
            value = self.slew.hold(measured, now)
            if self.input[0] <= self.release_threshold:
                self.armed = True
                self.reason = 'ready'
            return value
        if self.teleop_mode == 'direct':
            self.reason = 'following_trigger'
            return self.slew.hold(self.input[0], now)
        if self.input[0] >= self.press_threshold:
            self.pressed = True
        elif self.input[0] <= self.release_threshold:
            self.pressed = False
        self.reason = 'closing' if self.pressed else 'opening'
        return self.slew.advance(1.0 if self.pressed else 0.0, now)


def gripper_slow_main(config_path):
    """Explicit hardware entry point. Importing this module never starts it."""
    import fcntl
    import socket
    import struct
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float32, Float32MultiArray, Bool
    config = json.loads(Path(config_path).read_text())
    if int(os.environ.get('ROS_DOMAIN_ID', '24')) != config['domain_id']:
        raise ValueError('gripper domain must match the prepared configuration')
    sides = config['sides']
    for side in ('L', 'R'):
        item = sides[side]
        if item['source_topic'] == item['target_topic'] or not item['target_topic'].startswith('/tj/control/gripper_slow_target'):
            raise ValueError('isolated gripper command topics required')
    if sides['L']['target_topic'] == sides['R']['target_topic']:
        raise ValueError('left and right outputs must be separate')
    rate = config['rate_hz']
    if not 20 <= rate <= 250:
        raise ValueError('gripper update rate must be within 20..250 Hz')
    folder = ROOT/'logs/teleop'
    folder.mkdir(parents=True, exist_ok=True)
    lock = (folder/'gripper-slow.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    channels = {side:GripperChannel(
        minimum=item['minimum'], maximum=item['maximum'],
        close_seconds=config['close_seconds'], open_seconds=config['open_seconds'],
        timeout=config['timeout_s'], press_threshold=config['press_threshold'],
        release_threshold=config['release_threshold'],
        teleop_mode=config.get('teleop_mode', 'slew')) for side,item in sides.items()}
    rclpy.init(domain_id=config['domain_id'], args=['--ros-args','--disable-external-lib-logs'])
    node = rclpy.create_node('gento_gripper_slow', enable_rosout=False, start_parameter_services=False)
    pubs, subs, sockets = {}, [], {}
    vr = [False, None]
    def guarded(callback):
        def receive(message):
            try:
                callback(message, time.monotonic())
            except (ValueError, TypeError, IndexError):
                pass  # Invalid data does not refresh input or feedback age.
        return receive
    for side,item in sides.items():
        channel = channels[side]
        pubs[side] = node.create_publisher(Float32, item['target_topic'],
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        subs.append(node.create_subscription(Float32, item['source_topic'],
            guarded(lambda m,t,c=channel:c.receive_target(float(m.data),t)), qos_profile_sensor_data))
        subs.append(node.create_subscription(Float32MultiArray, item['feedback_topic'],
            guarded(lambda m,t,c=channel:c.receive_feedback(float(m.data[0]),t)), qos_profile_sensor_data))
        sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
            struct.pack('=IIII',item['motor_id']+0x10,0xC00007FF,0,0xC00007FF))
        sock.bind((item['can_interface'],))
        sock.setblocking(False)
        sockets[side] = sock
    subs.append(node.create_subscription(Bool, '/tj/info/vr_connected',
        lambda m:vr.__setitem__(slice(None),[bool(m.data),time.monotonic()]), qos_profile_sensor_data))
    # Keep one permanent driver publisher; replay submits openings to this relay.
    from std_srvs.srv import SetBool
    replay = {'active':False,'heartbeat':0.0,'targets':{}}
    def release_replay():
        replay['active']=False
        replay['targets'].clear()
        for channel in channels.values():channel.armed=channel.pressed=False
    def handoff(req,res):
        if req.data and replay['active']:
            res.success=False;res.message='Another gripper replay is active';return res
        if req.data:
            replay['active']=True;replay['heartbeat']=time.monotonic();replay['targets'].clear()
        else:release_replay()
        res.success=True;res.message='replay' if req.data else 'teleop';return res
    service=node.create_service(SetBool,'/tj/control/gripper_replay/enable',handoff)
    def replay_target(side,m):
        value=float(m.data)
        if replay['active'] and math.isfinite(value) and 0<=value<=1:
            replay['targets'][side]=(value,time.monotonic())
    for side in sides:
        subs.append(node.create_subscription(Float32,'/tj/control/gripper_replay_target'+side,
            lambda m,s=side:replay_target(s,m),qos_profile_sensor_data))
    subs.append(node.create_subscription(Bool,'/tj/control/gripper_replay/heartbeat',
        lambda m:replay.update(heartbeat=time.monotonic()) if m.data and replay['active'] else None,
        qos_profile_sensor_data))
    deadline = time.monotonic()+3
    try:
        # Discover the driver mapping before producing any target.
        while time.monotonic() < deadline:
            rclpy.spin_once(node,timeout_sec=.02)
        ownership = GripperOwnership(node, sides)
        ownership.check(time.monotonic())
        next_tick = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.002)
            now = time.monotonic()
            if ownership.check(now) is False:
                continue
            for side,sock in sockets.items():
                motor = sides[side]['motor_id']
                for _ in range(256):
                    try:
                        frame = sock.recv(16)
                    except BlockingIOError:
                        break
                    if len(frame)!=16:
                        continue
                    cid,length = struct.unpack('=IB',frame[:5])
                    data = frame[8:]
                    if cid in (0,motor+0x10) and length==8 and data[0]&15==motor:
                        channels[side].receive_motor(data[0]>>4,now)
            if now < next_tick:
                continue
            next_tick = now+1.0/rate
            vr_ready = vr[0] and vr[1] is not None and 0 <= now-vr[1] <= 2.5
            for side,channel in channels.items():
                if replay['active'] and now-replay['heartbeat']>.5:
                    release_replay()
                if replay['active']:
                    target=replay['targets'].get(side)
                    value=target[0] if target and now-target[1]<=.3 else None
                else:
                    value = channel.tick(now,vr_ready)
                if value is not None:
                    msg = Float32()
                    msg.data = value
                    pubs[side].publish(msg)
    except KeyboardInterrupt:
        pass
    finally:
        # No open/close command on shutdown; stop advancing the last target.
        for sock in sockets.values():
            sock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock.close()


def _dm_driver_pids():
    found = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            args = (proc/'cmdline').read_bytes().split(b'\0')
            if args and Path(os.fsdecode(args[0])).name == 'dm_motor_test':
                found.append(int(proc.name))
        except (OSError, ProcessLookupError):
            continue
    return found


def claim_gripper_launch(path):
    """Claim the whole tool launch before either native process can start."""
    import fcntl
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lock = path.open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        existing = _dm_driver_pids()
        if existing:
            raise RuntimeError('native DM driver already running: '+str(existing))
        return lock
    except BaseException:
        lock.close()
        raise


class GripperOwnership:
    """Keep checking that raw commands cannot bypass the sole slow output."""
    def __init__(self, node, sides, interval=.1):
        self.node, self.sides, self.interval = node, sides, interval
        self.last_checked = None
        self.ready = True
        self.resume_until = 0.0

    def check(self, now):
        if self.last_checked is not None and now-self.last_checked > .5:
            # A replay session pauses this process. DDS discovery catches up on resume.
            self.resume_until = now+2.0
        if self.last_checked is not None and now-self.last_checked < self.interval:
            return self.ready
        self.last_checked = now
        self.ready = False
        for item in self.sides.values():
            raw = self.node.get_subscriptions_info_by_topic(item['source_topic'])
            private = self.node.get_subscriptions_info_by_topic(item['target_topic'])
            if any(e.node_name=='dm_gripper_motor_node' for e in raw):
                raise RuntimeError('DM driver consumes the unfiltered Pico topic')
            if sum(e.node_name=='dm_gripper_motor_node' for e in private) != 1:
                raise RuntimeError('exactly one private DM subscriber required')
            if self.node.count_publishers(item['target_topic']) != 1:
                if now < self.resume_until:
                    return False  # Hold output until the replay publisher disappears.
                raise RuntimeError('exactly one slow target publisher required')
        self.ready = True
        return True
