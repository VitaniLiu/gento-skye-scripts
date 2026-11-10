"""Host recorder launcher. Robot raw bytes are never returned to host."""
import argparse,json,subprocess,shlex,sys,time,threading,re,os,uuid,hashlib
from recording.transport import BoundedPipe,clock_estimate
from pathlib import Path
from datetime import datetime,timezone
from processing.runtime import load_config

def command(config):
 robot=config['robot_ssh']
 if not re.fullmatch(r'[A-Za-z0-9_.@:-]+',robot) or robot.startswith('-'):raise ValueError('invalid SSH destination')
 protocol=Path(__file__).with_name('protocol.py').read_text()
 source=protocol+'\n'+Path(__file__).with_name('transport.py').read_text()+'\n'+Path(__file__).with_name('worker.py').read_text()
 config['recorder_bundle_sha256']=hashlib.sha256(source.encode()).hexdigest()
 script='source /etc/apex/apex_ros_env.sh >&2; export PYTHONDONTWRITEBYTECODE=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4; exec python3 -B -u -c '+shlex.quote(source)+' '+shlex.quote(json.dumps(config))
 return ['ssh','-T','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes','-o','ConnectTimeout=5','-o','ServerAliveInterval=2','-o','ServerAliveCountMax=3',robot,'bash -e -c '+shlex.quote(script)]

def countdown(cancel,emit):
 for value in (3,2,1):
  if cancel.is_set():return False
  emit(value)
  if cancel.wait(1):return False
 return not cancel.is_set()

def main(feature='recording'):
 p=argparse.ArgumentParser(description='Read-only multi-domain recording; no motion or service changes')
 mode=p.add_mutually_exclusive_group();mode.add_argument('--interactive',action='store_true',help='Persistent s/start-stop, y/keep, n/discard session');mode.add_argument('--check',action='store_true');mode.add_argument('--record',action='store_true')
 p.add_argument('--allow-incomplete',action='store_true',help='Capture partial data into pending area; never certifies quality')
 p.add_argument('--task');p.add_argument('--passive',action='store_true',help='Allow absent host event source; missing actual-send channel is reported')
 mode.add_argument('--list-pending',action='store_true');mode.add_argument('--keep',metavar='RECORDING_ID');mode.add_argument('--discard',metavar='RECORDING_ID');mode.add_argument('--inspect',metavar='RECORDING_ID')
 p.add_argument('--control-stdin',action='store_true',help=argparse.SUPPRESS)
 p.add_argument('--chassis-origin',help=argparse.SUPPRESS)
 p.add_argument('--seconds',type=float,default=60)
 p.add_argument('--event-log',help='Existing active chassis sender JSONL; forwards only new complete lines')
 args=p.parse_args();cfg=load_config(feature)
 if args.chassis_origin:
  from recording.chassis_origin import validate_origin
  if not args.record:p.error('--chassis-origin requires --record')
  cfg['chassis_capture']=validate_origin(json.loads(args.chassis_origin))
 if args.interactive:
  from recording.interactive import run_session
  return run_session(args,cfg,command)
 operation='keep' if args.keep else 'discard' if args.discard else 'inspect' if args.inspect else 'list_pending' if args.list_pending else None
 if operation:
  cfg.update(op=operation,recording_id=args.keep or args.discard or args.inspect)
  return subprocess.run(command(cfg)).returncode
 if not args.check and not args.record:
  print(json.dumps(dict(configuration='valid',motion_enabled=False,domains=cfg['domains'])));return 0
 if not 0<args.seconds<=3600:p.error('seconds must be in (0,3600]')
 if args.record and not args.task:p.error('--record requires --task')
 event_file=None
 if args.event_log:
  path=Path(args.event_log).resolve(strict=True)
  allowed=Path(cfg['project_root'])/'logs'
  # Existing chassis logging locations can be nested under the preserved chassis tree.
  chassis=Path(cfg['project_root'])/'src/teleop/chassis/logs'
  if not (path.is_relative_to(allowed) or path.is_relative_to(chassis)):p.error('event log must be an existing project teleop log')
  event_file=path.open();event_file.seek(0,2)
  cfg['event_log']=str(path)
  provenance=path.with_suffix('.source.json')
  if provenance.is_file():cfg['event_log_provenance']=json.loads(provenance.read_text())
 if args.record and event_file is None and not args.passive:p.error('--record requires --event-log for actual chassis send evidence')
 cfg['allow_incomplete']=args.allow_incomplete
 cfg.update(op='check' if args.check else 'record',task=args.task,seconds=args.seconds,request_id=uuid.uuid4().hex,start_requested_host_ns=time.time_ns(),passive=args.passive,start_handshake=True)
 proc=subprocess.Popen(command(cfg),stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1)
 def collect_stderr():
  for line in proc.stderr:print(line,end="",file=sys.stderr,flush=True)
 stderr_thread=threading.Thread(target=collect_stderr,daemon=True);stderr_thread.start()
 pipe=BoundedPipe(proc.stdin.fileno(),cfg['transport']['write_timeout_seconds']);cutoff=[None];recording=[None]
 done=threading.Event();ready=threading.Event();cancel_start=threading.Event();errors=[];write_lock=threading.Lock();last_clock=[0];sent_count=[0];ack_count=[0];event_identity=None
 if event_file:event_identity=(path.stat().st_dev,path.stat().st_ino)
 def send(row):
  row['recording_id']=recording[0]
  pipe.send((json.dumps(row,allow_nan=False)+'\n').encode())
 def forward():
  pending=''
  try:
   while not done.is_set() or (event_file is not None and cutoff[0] is not None and event_file.tell()<cutoff[0]):
    if not ready.is_set():done.wait(.01);continue
    if time.monotonic()-last_clock[0]>1:
     send(dict(op='clock',host_send_ns=time.time_ns(),host_send_monotonic_ns=time.monotonic_ns()));last_clock[0]=time.monotonic()
    if event_file is None:done.wait(.01);continue
    st=path.stat()
    if (st.st_dev,st.st_ino)!=event_identity or st.st_size<event_file.tell():raise ValueError('event log replaced or truncated')
    if st.st_size-event_file.tell()>cfg['transport']['max_log_backlog_bytes']:raise ValueError('recorder source log backlog exceeded')
    if sent_count[0]-ack_count[0]>=cfg['transport']['max_unacknowledged']:raise ValueError('recorder acknowledgement backlog exceeded')
    chunk=event_file.readline(cfg['transport']['max_message_bytes']+1)
    if not chunk:done.wait(.005);continue
    pending+=chunk
    if len(pending)>1048576:raise ValueError('event exceeds maximum size')
    if not pending.endswith('\n'):continue
    row=json.loads(pending);pending=''
    sent_count[0]+=1
    send(dict(event_sequence=sent_count[0],host_forward_ns=time.time_ns(),source=cfg['event_log'],payload=row))
   if pending:raise ValueError('incomplete source log line at stop')
  except Exception as exc:
   errors.append(str(exc))
   try:proc.stdin.close()
   except OSError:pass
 forward_thread=threading.Thread(target=forward,daemon=True);forward_thread.start()
 def control_input():
  try:
   for line in sys.stdin:
    if line.startswith('abort:'):errors.append(line.strip())
    elif line.strip()!='stop':continue
    cancel_start.set()
    while not ready.wait(.05):
     if done.is_set():return
    send(dict(op='request_stop',host_stop_ns=time.time_ns()));return
  except (OSError,ValueError,TimeoutError) as exc:
   errors.append('interactive stop: '+str(exc))
 if args.control_stdin:threading.Thread(target=control_input,daemon=True).start()
 try:
  for line in proc.stdout:
   print(line,end='',flush=True)
   try:
    message=json.loads(line)
    if message.get('type')=='preflight' and args.record:
     proceed=countdown(cancel_start,lambda value:print(json.dumps(dict(type='countdown',remaining=value,text='预检通过，'+str(value)),ensure_ascii=False),flush=True))
     send(dict(op='begin' if proceed else 'cancel'))
    if message.get('type')=='ready':
     recording[0]=message['recording_id']
     if event_file:event_file.seek(0,2)
     ready.set()
    if message.get('type')=='draining':
     cutoff[0]=path.stat().st_size if event_file else None
     done.set();forward_thread.join(timeout=3)
     if forward_thread.is_alive():raise TimeoutError('forwarder did not drain')
     send(dict(op='stop',last_event_sequence=sent_count[0],host_stop_ns=time.time_ns(),source_cutoff_bytes=cutoff[0],transport_errors=errors))
    if message.get('type')=='ack':ack_count[0]=message['event_sequence']
    if message.get('type')=='clock' and not done.is_set():
     send(dict(op='clock_result',**clock_estimate(message,time.time_ns(),time.monotonic_ns())))
   except (ValueError,TimeoutError,OSError) as exc:
    errors.append(str(exc));done.set()
    try:proc.stdin.close()
    except OSError:pass
  result=proc.wait(timeout=10)
 except KeyboardInterrupt:
  done.set()
  try:
   forward_thread.join(timeout=3);send(dict(op='stop',last_event_sequence=sent_count[0],host_stop_ns=time.time_ns(),transport_errors=['host interrupted']))
  except (OSError,ValueError):pass
  try:result=proc.wait(timeout=15)
  except subprocess.TimeoutExpired:proc.terminate();result=2
 finally:
  done.set();forward_thread.join(timeout=3);stderr_thread.join(timeout=2)
  if event_file and not forward_thread.is_alive():event_file.close()
  if not forward_thread.is_alive():
   for stream in (proc.stdin,proc.stdout,proc.stderr):stream.close()
 if sent_count[0]!=ack_count[0]:errors.append('unacknowledged host events')
 if errors:print(json.dumps(dict(type='host_error',errors=errors)));return 2
 return result

def logged_main():
 root=Path(__file__).resolve().parents[2]
 logdir=Path(os.environ.get('GENTO_RECORDING_TEST_LOG_DIR',str(root/'logs/recording'))).resolve()
 if 'GENTO_RECORDING_TEST_LOG_DIR' in os.environ and not logdir.is_relative_to(root/'tests'):raise ValueError('test logs must be under project tests')
 logdir.mkdir(parents=True,exist_ok=True)
 path=logdir/('session_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'_'+uuid.uuid4().hex[:8]+'.log')
 class Tee:
  def __init__(self,stream,file):self.stream=stream;self.file=file
  def write(self,text):self.stream.write(text);self.file.write(text);self.file.flush()
  def flush(self):self.stream.flush();self.file.flush()
 with path.open('x') as output:
  old=sys.stdout,sys.stderr;sys.stdout=Tee(old[0],output);sys.stderr=Tee(old[1],output)
  print(json.dumps(dict(type='host_log',path=str(path))))
  try:return main()
  finally:sys.stdout,sys.stderr=old
if __name__=='__main__':sys.exit(logged_main())
