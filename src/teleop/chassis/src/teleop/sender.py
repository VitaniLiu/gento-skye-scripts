"""User-run host sender. Import/help never connects or opens hardware."""
from pathlib import Path as _ProjectPath
import sys as _project_sys
_project_sys.path.insert(0,str(_ProjectPath(__file__).resolve().parents[1]/"common"))
from project_paths import configure_imports, output_directory
configure_imports()
import re
import shlex

def build_ssh_command(mode,session,robot="user@6.6.7.100",remote_dir="/home/user/tianjidipan",reset_button_index=None):
    from pathlib import PurePosixPath
    if mode not in ("check","execute") or not re.fullmatch(r"[0-9a-f]{32}",session):
        raise ValueError("Invalid mode/session")
    if not re.fullmatch(r"[A-Za-z0-9_.@:-]+",robot) or robot.startswith("-"):
        raise ValueError("Invalid SSH destination")
    path=PurePosixPath(remote_dir)
    if (not path.is_absolute() or ".." in path.parts or not (re.fullmatch(r"/home/user/tianjidipan(?:[-_A-Za-z0-9]*)",remote_dir) or remote_dir=="/home/user/gento-skye_scripts/src/teleop/chassis")):
        raise ValueError("Remote directory must be a separate /home/user/tianjidipan directory")
    remote="bash "+shlex.quote(remote_dir+"/scripts/run_robot.sh")+" --"+mode+" --session "+session
    if reset_button_index is not None:
        if type(reset_button_index) is not int or not 0<=reset_button_index<13:
            raise ValueError("Invalid reset button index")
        remote+=" --gamepad-reset --reset-button-index "+str(reset_button_index)
    return ["ssh","-T","-o","BatchMode=yes","-o","StrictHostKeyChecking=yes",
            "-o","ConnectTimeout=5","-o","ServerAliveInterval=1","-o","ServerAliveCountMax=3",robot,remote]


import argparse
import json
from dataclasses import asdict
from datetime import datetime,timedelta
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
from zoneinfo import ZoneInfo
from controller import DpadController,InputFrame,SafetyState
from capture_control import CaptureMailbox
from live_io import JsonChannel,ChannelError,HidTracker
from live_protocol import MAX_SESSION_SECONDS
from preview import Joystick

ROOT=Path(__file__).resolve().parents[2]
TZ=ZoneInfo("Asia/Shanghai")


def output(row,fd=None):
    """One bounded console write; blockage triggers orderly stop, never a queue."""
    descriptor=sys.stdout.fileno() if fd is None else fd
    previous=None
    try:
        raw=((row if isinstance(row,str) else json.dumps(row,ensure_ascii=False,allow_nan=False))+"\n").encode()
        if len(raw)>4096:return False
        previous=os.get_blocking(descriptor)
        os.set_blocking(descriptor,False)
        return os.write(descriptor,raw)==len(raw)
    except (OSError,ValueError,TypeError):return False
    finally:
        if previous is not None:
            try:os.set_blocking(descriptor,previous)
            except OSError:pass


class ConsoleSummary:
    """Human console only; full transport events remain in the session log."""
    def __init__(self):self.last=-float("inf");self.last_error=None;self.origin=None;self.session_ready=False
    def render(self,row,now):
        kind=row.get("type");reason=row.get("reason","")
        if kind in ("error","failed","reset_rejected") or (kind=="stopped" and row.get("confirmed") is not True) or (kind=="final" and row.get("status") not in ("STOP_CONFIRMED",)):
            return "底盘：异常 | "+json.dumps(row,ensure_ascii=False)
        if kind=="starting":
            self.origin=None;self.session_ready=False;self.last=-float("inf")
            return "底盘：正在检查"
        if kind=="keyboard_waist_reset_available":
            return "腰部：键盘 A 回固定高度（无需回车）" if row.get("enabled") else "腰部：当前非交互终端，键盘 A 不可用；手柄按键保留"
        if kind=="keyboard_waist_reset_requested":return "腰部：收到键盘 A，正在请求回固定高度"
        if kind=="keyboard_waist_reset_rejected":return "腰部：尚未就绪或已急停，键盘 A 未执行"
        if kind=="parking_release_requested":return "底盘：驻车中，正在请求解除"
        if kind=="parking_release_confirmed":return "底盘：驻车已解除"
        if kind=="ready":
            self.session_ready=True
            return "底盘：已就绪 | 松开 LB、方向键回中，再按住 LB 操作"
        if kind=="stopped":return "底盘：已停止（停车确认成功）"
        if kind not in ("target","checked"):return None
        pose=row.get("pose") or {}
        valid_pose=all(type(pose.get(k)) in (int,float) and math.isfinite(pose[k]) for k in ("x","y","yaw"))
        # Capture before output throttling; never re-anchor after an A-button return.
        if self.session_ready and kind=="target" and valid_pose and self.origin is None:
            self.origin=tuple(pose[k] for k in ("x","y","yaw"))
        relative="相对起点：未知"
        if valid_pose and self.origin is not None:
            x0,y0,yaw0=self.origin;dx=pose["x"]-x0;dy=pose["y"]-y0
            x=math.cos(yaw0)*dx+math.sin(yaw0)*dy
            y=-math.sin(yaw0)*dx+math.cos(yaw0)*dy
            yaw=math.atan2(math.sin(pose["yaw"]-yaw0),math.cos(pose["yaw"]-yaw0))
            relative=f"相对起点：x={x:+.3f} m, y={y:+.3f} m, 航向={math.degrees(yaw):+.1f}°"
        normal={"enabled","neutral","l1_released","stopping","input_not_ready",
                "release_l1_first","center_dpad_first","diagonal_blocked","startup_parking_release",
                "settling_at_start","reset_rejected"}
        if reason and reason not in normal:
            key=(kind,reason)
            if key!=self.last_error:
                self.last_error=key
                return "底盘：状态详情 | "+json.dumps(row,ensure_ascii=False)
        else:self.last_error=None
        if now-self.last<1.0:return None
        self.last=now
        fb=row.get("feedback") or {}
        state={"release_l1_first":"等待松开 LB","center_dpad_first":"等待方向键回中",
               "diagonal_blocked":"斜向输入不支持","input_not_ready":"等待有效输入",
               "stopping":"正在停止","startup_parking_release":"驻车解除中"}.get(reason,"正常")
        if fb.get("is_parking_mode") is True:state="驻车"
        def speed(key):
            v=fb.get(key)
            return f"{v:+.3f}" if type(v) in (int,float) and math.isfinite(v) else "未知"
        return f"底盘：{state} | 前后速度：{speed('vx')} m/s | 角速度：{speed('wz')} rad/s | {relative}"


class KeyboardResetKey:
    """Read a/A from this foreground terminal; preserve Ctrl+C and restore it."""
    def __init__(self,fd=None):
        self.fd=fd;self.enabled=False;self.saved=None;self.blocking=None
        self.escape=False;self.csi=False
    def __enter__(self):
        import termios
        if self.fd is None:self.fd=sys.stdin.fileno()
        if not os.isatty(self.fd):return self
        self.saved=termios.tcgetattr(self.fd);self.blocking=os.get_blocking(self.fd)
        mode=self.saved[:];mode[6]=self.saved[6][:]
        mode[3]&=~(termios.ICANON|termios.ECHO)
        mode[6][termios.VMIN]=0;mode[6][termios.VTIME]=0
        try:
            termios.tcsetattr(self.fd,termios.TCSANOW,mode)
            os.set_blocking(self.fd,False);self.enabled=True
        except BaseException:
            self.__exit__();raise
        return self
    def feed(self,data):
        pressed=False
        for value in data:
            if self.escape:
                if not self.csi:
                    if value in (ord('['),ord('O')):self.csi=True;continue
                    self.escape=False;continue
                if 0x40<=value<=0x7e:self.escape=False;self.csi=False
                continue
            if value==27:self.escape=True;continue
            if value in (ord('a'),ord('A')):pressed=True
        return pressed
    def poll(self):
        if not self.enabled:return False
        pressed=False
        for _ in range(4):
            try:data=os.read(self.fd,256)
            except BlockingIOError:break
            if not data:break
            pressed=self.feed(data) or pressed
        return pressed
    def __exit__(self,*exc):
        import termios
        self.enabled=False
        if self.saved is not None:
            try:termios.tcsetattr(self.fd,termios.TCSANOW,self.saved)
            except OSError:pass
            try:os.set_blocking(self.fd,self.blocking)
            except OSError:pass
            self.saved=None


class HostInputState:
    def __init__(self,session):
        self.session=session;self.hid=HidTracker()
        self.latest=None;self.sequence=0;self.release_epoch=0
        self.recorder=DpadController();self.last_record_time=0.0
        self.faulted=True;self.emergency_requested=False;self.waist_reset_seq=0;self.waist_reset_at=None

    def request_waist_return(self,now):
        if type(now) not in (int,float) or not math.isfinite(now) or now<0:raise ValueError("invalid keyboard request time")
        if self.waist_reset_seq>=2**31-1:raise ChannelError("keyboard reset sequence exhausted; restart teleop")
        self.waist_reset_seq+=1;self.waist_reset_at=now

    def observe(self,frame,event_time,timestamp):
        if frame.buttons[3]:self.emergency_requested=True
        if self.latest is not None and self.latest.buttons[4] and not frame.buttons[4]:
            self.release_epoch+=1
        self.latest=frame;self.faulted=False
        self.last_record_time=max(event_time,self.last_record_time)
        return self.recorder.update(frame,SafetyState(),self.last_record_time,timestamp).events

    def invalidate(self,now,timestamp):
        if not self.faulted:self.release_epoch+=1
        self.faulted=True;self.latest=None
        self.last_record_time=max(now,self.last_record_time)
        return self.recorder.update(InputFrame((0.0,)*8,(0,)*13,False),SafetyState(),self.last_record_time,timestamp).events

    def reply(self,poll,now):
        if (poll.get("type")!="poll" or poll.get("session")!=self.session
                or not re.fullmatch(r"[0-9a-f]{16}",str(poll.get("nonce","")))):
            raise ChannelError("Invalid request/session")
        self.sequence+=1
        age=self.hid.age(now)
        connected=self.latest is not None and not self.faulted and age<.15
        frame=self.latest or InputFrame((0.0,)*8,(0,)*13,False)
        return dict(type="input",session=self.session,nonce=poll["nonce"],seq=self.sequence,
                    axes=list(frame.axes),buttons=list(frame.buttons),connected=connected,
                    usb_age_s=age if math.isfinite(age) else 1e9,release_epoch=self.release_epoch,
                    waist_reset_seq=self.waist_reset_seq,
                    waist_reset_age_s=min(now-self.waist_reset_at,1e9) if self.waist_reset_at is not None and now>=self.waist_reset_at else 1e9)

    def finish(self,now,timestamp):
        return self.recorder.finish(max(now,self.last_record_time),timestamp).events


class UsbInput:
    def __init__(self,device,hidraw):
        self.joystick=Joystick(device);self.hidraw=hidraw;self.fd=None
    def __enter__(self):
        self.joystick.__enter__()
        try:
            js=Path(self.joystick.device).resolve().name
            hid=Path(self.hidraw).resolve().name
            jsroot=(Path("/sys/class/input")/js/"device").resolve(strict=True)
            hidroot=(Path("/sys/class/hidraw")/hid/"device").resolve(strict=True)
            if hidroot not in jsroot.parents:
                raise ValueError("HID liveness source does not match joystick")
            self.fd=os.open(self.hidraw,os.O_RDONLY|os.O_NONBLOCK)
            return self
        except BaseException:
            self.joystick.__exit__();raise
    def poll(self,state,now,wall_start,log_press):
        for _ in range(512):
            try:packet=os.read(self.fd,128)
            except BlockingIOError:break
            if not packet:raise OSError("USB HID disconnected")
            state.hid.observe(packet,now)
        for event_time,frame in self.joystick.poll(now):
            stamp=(wall_start+timedelta(seconds=event_time)).isoformat()
            for event in state.observe(frame,event_time,stamp):log_press(event)
        # Before the first HID report, retain the joystick baseline. reply() and
        # the startup gate still reject it until HID is fresh. Once HID has been
        # observed, the existing timeout/invalidation path remains unchanged.
        if state.hid.last_change is not None and state.hid.age(now)>=.15:
            for event in state.invalidate(now,datetime.now(TZ).isoformat()):log_press(event)
    def __exit__(self,*exc):
        if self.fd is not None:os.close(self.fd);self.fd=None
        self.joystick.__exit__(*exc)


def a_button_index(fd):
    """Resolve A/Cross (Linux BTN_SOUTH), without assuming a joystick index."""
    import array,fcntl
    codes=array.array('H',[0]*512)
    fcntl.ioctl(fd,0x84006a34,codes)
    matches=[i for i,code in enumerate(codes[:13]) if code==0x130]
    if len(matches)!=1:raise ValueError('A button mapping unavailable or ambiguous')
    return matches[0]


def open_local_file(prefix):
    if prefix not in ("live","ssh-stderr"):raise ValueError("Invalid log prefix")
    base=output_directory("logs", ROOT)
    if base.is_symlink():raise ValueError("Log directory cannot be a symlink")
    folder=base/("sessions" if prefix=="live" else "transport")/datetime.now(TZ).strftime("%Y-%m-%d")
    if not folder.resolve().is_relative_to(base.resolve()):raise ValueError("Log path escapes project")
    if folder.is_symlink():raise ValueError("Log directory cannot be a symlink")
    folder.mkdir(parents=True,exist_ok=True)
    directory=os.open(folder,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW)
    try:
        name=f"{prefix}-{datetime.now(TZ):%Y%m%d-%H%M%S}-{uuid.uuid4().hex[:8]}.jsonl"
        fd=os.open(name,os.O_CREAT|os.O_EXCL|os.O_WRONLY|os.O_NOFOLLOW,0o600,dir_fd=directory)
        return folder/name,os.fdopen(fd,"w",encoding="utf-8")
    finally:os.close(directory)


def open_route_file():
    folder=output_directory("routes", ROOT)/datetime.now(TZ).strftime("%Y-%m-%d")
    folder.mkdir(parents=True,exist_ok=True)
    if folder.is_symlink():raise ValueError("Route directory cannot be a symlink")
    path=folder/("route-"+datetime.now(TZ).strftime("%Y%m%d-%H%M%S")+"-"+uuid.uuid4().hex[:8]+".jsonl")
    fd=os.open(path,os.O_CREAT|os.O_EXCL|os.O_WRONLY|os.O_NOFOLLOW,0o600)
    return path,os.fdopen(fd,"w",encoding="utf-8")


def main(argv=None):
    parser=argparse.ArgumentParser(description="Independent real chassis sender: operator runs explicitly.")
    mode=parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check",action="store_true",help="Robot read-only preflight; no velocity publisher")
    mode.add_argument("--execute",action="store_true",help="Real output, subject to L1 and safety gates")
    parser.add_argument("--robot",default="user@6.6.7.100")
    parser.add_argument("--remote-dir",default="/home/user/gento-skye_scripts/src/teleop/chassis")
    parser.add_argument("--parent-pid",type=int,default=None,help="Optional coordinator PID; parent loss requests existing stop handshake")
    parser.add_argument("--device",default="/dev/input/js0")
    parser.add_argument("--hidraw",default="/dev/gamepad-control-hid")
    parser.add_argument("--no-reset-button",action="store_true",help="Manual-only mode without A return")
    parser.add_argument("--seconds",type=float,default=None,help="Optional session limit in seconds; omitted means run until operator closes")
    parser.add_argument("--json-output",action="store_true",help="Full JSON events for programmatic consumers")
    args=parser.parse_args(argv)
    if args.seconds is not None and (not math.isfinite(args.seconds) or args.seconds<=0):
        parser.error("--seconds must be positive when supplied")
    if args.parent_pid is not None and (args.parent_pid<=1 or os.getppid()!=args.parent_pid):
        parser.error("coordinator parent missing or mismatched")
    session=uuid.uuid4().hex
    try:command=build_ssh_command("execute" if args.execute else "check",session,args.robot,args.remote_dir,reset_button_index=None if args.no_reset_button else 0)
    except ValueError as exc:parser.error(str(exc))
    path,log=open_local_file("live");error_path,errors=open_local_file("ssh-stderr")
    state=HostInputState(session)
    process=None;channel=None;device=None;keyboard=None;responses=0;ready=False;stopped=None;checked=False
    started=time.monotonic();wall_start=datetime.now(TZ);exit_code=1;last_console=None
    emergency_sent=False
    console_summary=ConsoleSummary()
    def record(row,console=True):
        row=dict(row,host_time=datetime.now(TZ).isoformat())
        log.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+"\n");log.flush()
        display=(row if console else None) if args.json_output else console_summary.render(row,time.monotonic())
        if display is not None and not output(display):raise RuntimeError("Operator console closed")
    def press(event):record(dict(type="press",**asdict(event),input_source="real_joystick",motion_duration=False))
    from route_recording import RouteRecorder
    routes=RouteRecorder(record,open_route_file)
    capture_mailbox=None
    def handle(message):
        nonlocal ready,checked,stopped,last_console,capture_mailbox
        if message.get("session")!=session:raise ChannelError("Wrong endpoint session")
        kind=message.get("type")
        if kind=="poll":return
        if kind=="ready":
            ready=True
            if args.execute and message.get("capture_scoped") is True:
                capture_mailbox=CaptureMailbox(path,session)
            message=dict(message,operator_hint="先松开 L1 并让方向键回中，再按住 L1，随后按十字方向键移动；松开 L1 停车，Ctrl+C 退出。")
        if kind=="checked":checked=message.get("ok") is True
        if kind=="stopped":stopped=message.get("confirmed") is True
        show=True
        if kind=="target":
            signature=tuple(message.get(k) for k in ("vx","vy","wz","reason"))
            show=signature!=last_console;last_console=signature
            hints={"release_l1_first":"先松开 L1，再按住 L1；启动时持续按住不会使能。",
                   "center_dpad_first":"保持 L1 按住，先让十字方向键回中，再按方向键移动。"}
            if message.get("reason") in hints:
                message=dict(message,operator_hint=hints[message["reason"]])
        record(dict(message,source="robot_endpoint"),show)
        routes.observe(message)
    try:
        record(dict(type="starting",mode="execute" if args.execute else "check",log_path=str(path),ssh_stderr=str(error_path)))
        if args.execute:
            # Open and verify local hardware before making any robot connection.
            device=UsbInput(args.device,args.hidraw).__enter__()
            if not args.no_reset_button:
                index=a_button_index(device.joystick.fd)
                command=build_ssh_command("execute",session,args.robot,args.remote_dir,reset_button_index=index)
                record(dict(type="reset_button_mapped",button="A",joystick_index=index,linux_code="BTN_SOUTH"))
            deadline=time.monotonic()+.5
            while time.monotonic()<deadline and (state.latest is None or state.hid.age(time.monotonic()-started)>=.15):
                device.poll(state,time.monotonic()-started,wall_start,press);time.sleep(.01)
            if state.latest is None or state.hid.age(time.monotonic()-started)>=.15:
                raise RuntimeError("No fresh USB joystick reports; no SSH session started")
            if not args.no_reset_button:
                keyboard=KeyboardResetKey().__enter__()
                record(dict(type="keyboard_waist_reset_available",enabled=keyboard.enabled))
        process=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=errors,start_new_session=True,bufsize=0)
        channel=JsonChannel(process.stdout.fileno(),process.stdin.fileno())
        for sig in (signal.SIGTERM,signal.SIGHUP):signal.signal(sig,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt()))
        while True:
            now=time.monotonic()-started
            if args.parent_pid is not None and os.getppid()!=args.parent_pid:
                raise RuntimeError("Coordinator exited; requesting chassis stop")
            if args.execute:
                device.poll(state,now,wall_start,press)
                if keyboard is not None and keyboard.poll():
                    if ready and responses>0 and not emergency_sent and not state.emergency_requested:
                        state.request_waist_return(now)
                        record(dict(type="keyboard_waist_reset_requested",request_seq=state.waist_reset_seq,waist_only=True))
                    else:record(dict(type="keyboard_waist_reset_rejected"))
            if capture_mailbox is not None:
                for request in capture_mailbox.receive():channel.send(request)
            if args.execute and not emergency_sent and state.emergency_requested:
                channel.send(dict(type="emergency_stop",session=session))
                emergency_sent=True
                record(dict(type="error",error="Y 键急停请求已发送；全身断电及夹爪停止尚未确认，请查看机器人状态。"))
            for message in channel.receive():
                if message.get("type")=="poll":
                    if not args.execute:raise ChannelError("Check mode unexpectedly requested input")
                    response=state.reply(message,now)
                    channel.send(response);responses+=1
                    record(dict(type="input_reply",seq=response["seq"],release_epoch=response["release_epoch"],
                                l1=response["buttons"][4],dpad=response["axes"][6:8],
                                usb_age_s=response["usb_age_s"],queued_to_ssh=True,input=response,
                                host_queue_complete_ns=time.time_ns()),False)
                else:handle(message)
            if args.check and checked:exit_code=0;break
            if stopped is not None:exit_code=0 if stopped else 2;break
            if channel.eof:break
            if not ready and now>15:raise RuntimeError("Endpoint preflight timed out")
            if args.execute and args.seconds is not None and now>=args.seconds:break
            time.sleep(.005)
    except KeyboardInterrupt:
        record(dict(type="stop_requested",reason="operator_interrupt"))
    except (OSError,ValueError,RuntimeError,ChannelError) as exc:
        try:record(dict(type="error",error=str(exc)))
        except (OSError,RuntimeError):pass
    finally:
        if keyboard is not None:keyboard.__exit__()
        if process is not None and args.execute:
            if stopped is None:
                try:channel.send(dict(type="stop",session=session))
                except (ChannelError,OSError):
                    try:process.stdin.close()
                    except OSError:pass
                deadline=time.monotonic()+4
                while time.monotonic()<deadline and stopped is None:
                    try:
                        for message in channel.receive():handle(message)
                    except (ChannelError,OSError,RuntimeError):break
                    if channel.eof:break
                    time.sleep(.01)
            exit_code=0 if stopped is True else (2 if ready or responses else 1)
            try:
                record(dict(type="final",status="STOP_CONFIRMED" if stopped is True else
                            "STOP_UNCONFIRMED" if ready or responses else "MOTION_NOT_STARTED",
                            stop_confirmed=stopped,motion_session_started=ready or responses>0))
            except (OSError,RuntimeError):pass
        try:routes.close("session_closed",stop_confirmed=stopped is True)
        except (OSError,ValueError):pass
        if device is not None:
            try:
                for event in state.finish(time.monotonic()-started,datetime.now(TZ).isoformat()):press(event)
            except (OSError,RuntimeError):pass
            try:device.__exit__()
            except OSError:pass
        if process is not None:
            try:process.stdin.close()
            except OSError:pass
            try:process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:process.wait(timeout=1)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=1)
            process.stdout.close()
        if capture_mailbox is not None:capture_mailbox.close()
        log.close();errors.close()
    return exit_code


if __name__=="__main__":raise SystemExit(main())
