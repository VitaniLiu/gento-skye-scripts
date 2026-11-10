"""Robot-only read-only subscriptions; raw output never crosses SSH."""
import json,os,sys,time,threading,queue,signal,shutil,re,socket,struct,fcntl,hashlib
from pathlib import Path

def run(cfg):
 if cfg['op'] in ('keep','discard','inspect','list_pending','prepare_task'):
  store=Lifecycle(cfg['robot_raw_root']);op=cfg['op']
  result=getattr(store,op)() if op=='list_pending' else getattr(store,op)(cfg['task'] if op=='prepare_task' else cfg['recording_id'])
  print(json.dumps(result,ensure_ascii=False),flush=True);return 0
 output_lock=threading.Lock()
 def emit(**v):
  with output_lock:print(json.dumps(v,allow_nan=False),flush=True)
 root=Path(cfg['robot_raw_root'])
 if not root.is_absolute() or root.is_symlink() or not root.is_dir():raise ValueError('raw root must be an existing real directory')
 if not os.access(root,os.W_OK|os.X_OK):raise ValueError('raw root not writable')
 if shutil.disk_usage(root).free<cfg['minimum_free_bytes']:raise ValueError('insufficient storage')
 fd=os.open(root/'.recording.lock',os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
 lock=os.fdopen(fd,'a')
 try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 except BaseException:lock.close();raise
 emit(type='preflight',directory_writable=True,recording_available=True,free_bytes=shutil.disk_usage(root).free,minimum_free_bytes=cfg['minimum_free_bytes'],motion_enabled=False)
 if cfg['op']=='check':lock.close();return 0
 if cfg.get('start_handshake'):
  try:
   row=sys.stdin.readline(4096)
   if not row or json.loads(row).get('op')!='begin':
    emit(type='cancelled',reason='cancelled before capture');lock.close();return 0
  except BaseException:lock.close();raise
 import rclpy,rosbag2_py
 from rclpy.context import Context
 from rclpy.executors import SingleThreadedExecutor
 from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy
 from rclpy.event_handler import SubscriptionEventCallbacks
 from rclpy.serialization import serialize_message,deserialize_message
 from rosidl_runtime_py.utilities import get_message
 from std_msgs.msg import String
 contexts=[];nodes=[];executors=[];subscriptions=[];counts={};last={};writers={}
 failures=[];stopping=threading.Event();events=queue.Queue(maxsize=cfg['transport']['robot_queue_messages'])
 ranges={};domain_lifecycle={};stop_request={};requested_stop=threading.Event();guard=SequenceGuard();last_input=[time.monotonic()];queue_high=[0];health_issues=set();process_started_ns=time.time_ns();dds_sequences={};skipped_video={}
 def issue(reason):
  if reason not in health_issues:health_issues.add(reason);failures.append(reason)
 video_started=set();video_types={t:get_message(typ) for d in cfg['domains'] for t,typ in d['topics'].items() if typ=='foxglove_msgs/msg/CompressedVideo'}
 published_events=0;files_closed=False;active_lock=None;recording_id=None;active=[False];episode=None;start=time.monotonic();can_sockets={};can_last={};can_counts={};clock_samples=[];readback={};file_hashes={}
 for iface,motor in [('vcan0',1),('vcan1',2)]:
  sock=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
  try:sock.bind((iface,));sock.setblocking(False);can_sockets[sock]=(iface,motor)
  except OSError as exc:sock.close();issue('CAN unavailable '+iface+': '+str(exc))
 def can_read():
  for sock,(iface,motor) in can_sockets.items():
   for _ in range(512):
    try:raw=sock.recv(16)
    except BlockingIOError:break
    cid,dlc=struct.unpack('=IB',raw[:5]);data=raw[8:]
    if cid not in (0,motor+16) or dlc!=8 or data[0]&15!=motor:continue
    row=dict(recording_id=recording_id,episode_index=None,source='socketcan',stage='measured_feedback',clock_domain='robot_unix',sequence=can_counts.get(iface,0)+1,interface=iface,can_id=cid,motor_id=motor,state=data[0]>>4,data_hex=data.hex(),robot_receive_ns=time.time_ns())
    can_last[iface]=(time.monotonic(),row);can_counts[iface]=can_counts.get(iface,0)+1
    if active[0]:
     writers['events'].write('/recording/gripper/can',serialize_message(String(data=json.dumps(row))),row['robot_receive_ns']);observe_range(ranges,'can:'+iface,row['robot_receive_ns'])
 def stop(signum,*_):
  failures.append('capture interrupted by signal '+str(signum));stopping.set()
 for sig in (signal.SIGTERM,signal.SIGINT,signal.SIGHUP):signal.signal(sig,stop)
 def incoming():
  try:
   for line in iter(lambda:sys.stdin.readline(cfg['transport']['max_message_bytes']+1),''):
    last_input[0]=time.monotonic()
    if len(line)>cfg['transport']['max_message_bytes']:raise ValueError('event too large')
    row=json.loads(line)
    if row.get('recording_id')!=recording_id:raise ValueError('recording_id transport mismatch')
    if row.get('op')=='clock':
     emit(type='clock',host_send_ns=row['host_send_ns'],host_send_monotonic_ns=row['host_send_monotonic_ns'],robot_receive_ns=time.time_ns(),robot_send_ns=time.time_ns());continue
    if row.get('op')=='clock_result':
     row.update(recording_id=recording_id,host_clock_domain='host_unix',robot_clock_domain='robot_unix');clock_samples.append(row);continue
    if row.get('op')=='request_stop':
     stop_request['user_requested_host_ns']=row.get('host_stop_ns');requested_stop.set();continue
    if row.get('op')=='stop':
     stop_request.update(row,robot_receive_ns=time.time_ns());failures.extend(row.get('transport_errors',[]));stopping.set();return
    events.put_nowait((time.time_ns(),row));queue_high[0]=max(queue_high[0],events.qsize())
  except Exception as exc:failures.append('event transport: '+type(exc).__name__+': '+str(exc))
  finally:
   if not stopping.is_set():failures.append('host transport EOF')
   stopping.set()
 def callback(domain,topic):
  def receive(raw,info):
   now=time.time_ns();key=str(domain)+':'+topic
   last[key]=time.monotonic()
   if active[0] and topic in video_types and topic not in video_started:
    packet=bytes(deserialize_message(raw,video_types[topic]).data)
    nals={part[0]&31 for part in re.split(b'\x00\x00(?:\x00)?\x01',packet)[1:] if part}
    if not {5,7,8}.issubset(nals):skipped_video[key]=skipped_video.get(key,0)+1;return
    video_started.add(topic)
   counts[key]=counts.get(key,0)+1
   if active[0]:
    try:
     writers[domain].write(topic,raw,now);observe_range(ranges,key,now)
     groups=[name for name,spec in cfg.get('logical_streams',{}).items() if topic in spec.get('topics',[])]
     roles=[role for role,spec in cfg.get('images',{}).items() if topic in spec['topics']]
     timing=dict(schema_version='2.0',recording_id=recording_id,episode_index=None,logical_streams=groups,image_roles=roles,stage='image_sample' if roles else 'measured_feedback' if any(g.endswith('state') for g in groups) else 'control_target' if 'pico_action' in groups else 'source_observation',domain=domain,topic=topic,sequence=counts[key],robot_receive_ns=now,robot_monotonic_ns=time.monotonic_ns())
     for field in ('source_timestamp','received_timestamp','publication_sequence_number'):
      value=info.get(field) if isinstance(info,dict) else getattr(info,field,None)
      if type(value) is int:timing[field]=value
     publisher=info.get('publisher_gid') if isinstance(info,dict) else getattr(info,'publisher_gid',None)
     if publisher is not None:timing['publisher_gid']=list(publisher)
     seq=timing.get('publication_sequence_number');publisher_key=(key,str(publisher))
     if seq is not None and seq>0:
      previous=dds_sequences.get(publisher_key)
      if previous is not None and seq!=previous+1:issue('DDS sequence discontinuity: '+key)
      dds_sequences[publisher_key]=seq
     timing.update(source=dict(kind='ros_dds',domain=domain,topic=topic),clock_domains=dict(robot_receive_ns='robot_unix',robot_monotonic_ns='robot_monotonic',source_timestamp='dds_publisher_unverified',received_timestamp='dds_receive_unverified'),raw_reference=dict(bag='domain_'+str(domain),topic=topic,sequence=counts[key]),original_timestamp_storage='unchanged native CDR header/stamp/timestamp when present',missing=[dict(field='device_clock_mapping',reason='not calibrated',impact='physical sample/exposure alignment unverified')])
     for field in ('source_timestamp','received_timestamp','publication_sequence_number'):
      if field not in timing:timing['missing'].append(dict(field=field,reason='DDS interface did not provide value',impact='source timing/sequence unavailable'))
     writers['events'].write('/recording/source_timing',serialize_message(String(data=json.dumps(timing))),now)
    except Exception as exc:failures.append(str(exc));stopping.set()
  return receive
 try:
  for spec in cfg['domains']:
   domain=spec['id'];domain_lifecycle[str(domain)]=dict(pid=os.getpid(),context_started_robot_ns=time.time_ns());c=Context();rclpy.init(context=c,domain_id=domain);contexts.append(c)
   n=rclpy.create_node('gento_raw_recorder',context=c,enable_rosout=False);nodes.append(n)
   e=SingleThreadedExecutor(context=c);e.add_node(n);executors.append(e)
   for topic,typ in spec['topics'].items():
    def lost(event,key=str(domain)+':'+topic):
     issue('DDS reported message loss: '+key+' count='+str(event.total_count))
     if not cfg.get('allow_incomplete'):stopping.set()
    qos=QoSProfile(depth=cfg.get('dds_queue_depth',256),reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST)
    subscriptions.append(n.create_subscription(get_message(typ),topic,callback(domain,topic),qos,raw=True,event_callbacks=SubscriptionEventCallbacks(message_lost=lost)))
  store=Lifecycle(root)
  with locked(root/'.lifecycle.lock'):
   episode=store.begin(cfg['task'],cfg)
   active_lock=locked(episode/'.active.lock');active_lock.__enter__()
  initial_manifest=store.read(episode);recording_id=initial_manifest['recording_id']
  def manifest(status):
   quality_failures=[x for x in failures if x.startswith(('missing topic:','no chassis actual','explicit partial'))]
   capture_failures=[x for x in failures if x not in quality_failures]
   result=dict(schema_version='2.0',recording_id=recording_id,episode_index=None,lifecycle='pending',task_name=cfg['task'],configuration=cfg,
     file_status='closed' if files_closed else 'incomplete' if status=='incomplete' else 'writing',
     capture_status='completed' if files_closed and not capture_failures else 'abnormal' if status=='incomplete' else 'in_progress',
     quality_status='incomplete' if failures or any(not any(counts.get(str(d['id'])+':'+t) for d in cfg['domains'] for t in spec['topics']) for spec in cfg.get('images',{}).values()) else 'unverified',status=status,counts=counts,failures=failures,
     domains=cfg['domains'],readback=readback,file_sha256=file_hashes,source_clock_mapping='unverified',saved_ranges=ranges,domain_lifecycle=domain_lifecycle,stop_request=stop_request,process=dict(pid=os.getpid(),started_robot_ns=process_started_ns,recording_id=recording_id),transport=dict(queue_high_water=queue_high[0],accepted_sequence=guard.last,limits=cfg['transport']),video_start_discarded_packets=skipped_video,
     camera_exposure_mapping='unverified',gripper_motor_validity=dict(status='untrusted_or_missing' if any('CAN' in f or 'gripper' in f or 'motor_feedback' in f for f in failures) else 'observed_valid_frames_not_calibrated_opening',counts=can_counts,last={k:v[1] for k,v in can_last.items()}),clock_samples=clock_samples,
     missing_topics=[str(d['id'])+':'+t for d in cfg['domains'] for t in d['topics'] if not counts.get(str(d['id'])+':'+t)],
     host_event_source=cfg.get('event_log'),started_robot_unix_ns=started_ns,
     finished_robot_unix_ns=time.time_ns() if status!='in_progress' else None,
     stream_statistics={name:dict(message_count=published_events if name=='gamesir_action' else sum(counts.get(str(d['id'])+':'+t,0) for d in cfg['domains'] for t in spec.get('topics',[])),source_topics=spec.get('topics',[spec.get('topic')])) for name,spec in cfg.get('logical_streams',{}).items()},
     image_statistics={role:sum(counts.get(str(d['id'])+':'+t,0) for d in cfg['domains'] for t in spec['topics']) for role,spec in cfg.get('images',{}).items()},
     logical_streams=cfg.get('logical_streams',{}),images=cfg.get('images',{}),
     missing_information=[dict(field='images.'+role,reason='configured image stream not observed',impact='image role unavailable') for role,spec in cfg.get('images',{}).items() if not any(counts.get(str(d['id'])+':'+t) for d in cfg['domains'] for t in spec['topics'])],
     file_index=[dict(path=str(f.relative_to(episode)),size=f.stat().st_size,sha256=file_hashes.get(str(f.relative_to(episode)))) for f in episode.rglob('*') if f.is_file() and f.suffix in ('.mcap','.yaml')],motion_enabled=False)
   atomic_json(episode/'recording.json',result)
  started_ns=time.time_ns();manifest('in_progress')
  for spec in cfg['domains']:
   domain=spec['id'];w=rosbag2_py.SequentialWriter()
   w.open(rosbag2_py.StorageOptions(uri=str(episode/('domain_'+str(domain))),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
   for i,(topic,typ) in enumerate(spec['topics'].items()):
    w.create_topic(rosbag2_py.TopicMetadata(id=i,name=topic,type=typ,serialization_format='cdr'))
   writers[domain]=w
  ew=rosbag2_py.SequentialWriter()
  ew.open(rosbag2_py.StorageOptions(uri=str(episode/'host_events'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
  ew.create_topic(rosbag2_py.TopicMetadata(id=0,name='/recording/host/events',type='std_msgs/msg/String',serialization_format='cdr'))
  ew.create_topic(rosbag2_py.TopicMetadata(id=1,name='/recording/gripper/can',type='std_msgs/msg/String',serialization_format='cdr'))
  ew.create_topic(rosbag2_py.TopicMetadata(id=2,name='/recording/source_timing',type='std_msgs/msg/String',serialization_format='cdr'))
  writers['events']=ew
  counts.clear();can_counts.clear();active[0]=True;start=time.monotonic();last_input[0]=start
  for state in domain_lifecycle.values():state.update(recording_id=recording_id,capture_started_robot_ns=time.time_ns())
  threading.Thread(target=incoming,daemon=True).start()
  emit(type='ready',recording_id=recording_id,pending_path=str(episode),episode_index=None)
  sequence=0;published_events=0;event_session=None
  def drain_events():
   nonlocal sequence,published_events,event_session
   while True:
    try:received,row=events.get_nowait()
    except queue.Empty:break
    sequence+=1
    guard.accept(row.get('event_sequence'))
    payload=row.get('payload',{})
    incoming_session=payload.get('session') or payload.get('input',{}).get('session')
    if incoming_session:
     if event_session is not None and incoming_session!=event_session:raise ValueError('host source session changed')
     event_session=incoming_session
    if payload.get('type')=='target' and payload.get('published') is True:
     if payload.get('source')!='robot_endpoint' or not payload.get('command_id') or payload.get('send_result')!='ros_publish_returned' or not all(k in payload for k in ('owner','enable_epoch')):raise ValueError('target lacks actual publication provenance')
     published_events+=1
    msg=String(data=json.dumps(dict(schema_version='2.0',recording_id=recording_id,episode_index=None,sequence=sequence,stage='robot_receive',source_stage='chassis_publish' if payload.get('type')=='target' and payload.get('published') is True else 'host_observation',clock_domains=dict(robot_receive_ns='robot_unix',host_forward_ns='host_unix'),robot_receive_ns=received,event=row),allow_nan=False))
    ew.write('/recording/host/events',serialize_message(msg),received);observe_range(ranges,'host_events',received)
    emit(type='ack',event_sequence=sequence)

  draining_at=None
  while not stopping.is_set():
   if (requested_stop.is_set() or time.monotonic()-start>=cfg['seconds']) and draining_at is None:
    draining_at=time.monotonic();stop_request['coordinator_request_robot_ns']=time.time_ns();emit(type='draining',recording_id=recording_id)
   if draining_at is not None and time.monotonic()-draining_at>cfg['transport']['drain_timeout_seconds']:
    failures.append('host drain timeout');break
   if time.monotonic()-last_input[0]>cfg['transport']['heartbeat_timeout_seconds']:
    issue('host heartbeat timeout');break
   for e in executors:e.spin_once(timeout_sec=.001)
   can_read()
   if time.monotonic()-start>1 and any(i not in can_last or time.monotonic()-can_last[i][0]>1 or can_last[i][1]['state']!=1 for i in ('vcan0','vcan1')):
    issue('gripper motor feedback invalid')
    if not cfg.get('allow_incomplete'):stopping.set()
   drain_events()
   if shutil.disk_usage(root).free<cfg['minimum_free_bytes']:
    failures.append('free space threshold reached');break
   for spec in cfg['domains']:
    for topic in spec['required']:
     key=str(spec['id'])+':'+topic
     if time.monotonic()-last.get(key,start)>cfg['stream_timeout_seconds']:
      issue('stream stale: '+key)
      if not cfg.get('allow_incomplete'):stopping.set()
  drain_events()
  if stop_request.get('last_event_sequence')!=guard.last:issue('stop sequence acknowledgement mismatch or absent stop')
  active[0]=False
  for state in domain_lifecycle.values():state['capture_stopped_robot_ns']=time.time_ns()
  if not events.empty():failures.append('unflushed host events')
  if not published_events:failures.append('no chassis actual publish events received')
  for d in cfg['domains']:
   for t in d['topics'] if cfg.get('require_all_topics',True) else d['required']:
    if not counts.get(str(d['id'])+':'+t):failures.append('missing topic: '+t)
  if any(not can_counts.get(i) for i in ('vcan0','vcan1')):issue('gripper motor feedback missing during capture')
  for w in writers.values():w.close()
  writers.clear();files_closed=True
  for state in domain_lifecycle.values():state['writer_closed_robot_ns']=time.time_ns()
  # Completion of files does not certify device feedback or camera synchronization.
  for f in episode.rglob('*'):
   if f.is_file():
    with f.open('rb') as h:os.fsync(h.fileno())
  readback={}
  for bag in episode.iterdir():
   if not bag.is_dir():continue
   reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
   nread=0;measured={}
   while reader.has_next():
    topic,raw,stamp=reader.read_next();nread+=1;measured[topic]=measured.get(topic,0)+1
   readback[bag.name]=dict(total=nread,topics=measured)
   if bag.name.startswith('domain_'):
    domain=bag.name.removeprefix('domain_')
    expected={k.split(':',1)[1]:v for k,v in counts.items() if k.startswith(domain+':')}
    if measured!=expected:failures.append('readback count mismatch: '+bag.name)
   elif measured.get('/recording/host/events',0)!=sequence:failures.append('host event readback mismatch')
  for f in episode.rglob('*'):
   if f.is_file() and f.suffix in ('.mcap','.yaml'):
    h=hashlib.sha256()
    with f.open('rb') as inp:
     for chunk in iter(lambda:inp.read(1024*1024),b''):h.update(chunk)
    file_hashes[str(f.relative_to(episode))]=h.hexdigest()
  emit(type='readback',counts=readback)
  if cfg.get('allow_incomplete'):failures.append('explicit partial capture')
  manifest('incomplete' if failures else 'closed_uncertified')
  emit(type='closed',recording_id=recording_id,pending_path=str(episode),episode_index=None,failures=failures,host_events=sequence)
  return 2 if failures else 0
 except BaseException as exc:
  failures.append(type(exc).__name__+': '+str(exc))
  if episode is not None:
   try:manifest('incomplete')
   except Exception:pass
  raise
 finally:
  active[0]=False
  unfinished=bool(writers);close_ok=True
  for state in domain_lifecycle.values():state.setdefault('capture_stopped_robot_ns',time.time_ns())
  for w in writers.values():
   try:w.close()
   except Exception as exc:close_ok=False;failures.append('writer close: '+str(exc))
  if unfinished:
   files_closed=close_ok
   for state in domain_lifecycle.values():state['writer_closed_robot_ns']=time.time_ns() if close_ok else None
   if episode is not None:
    try:manifest('incomplete')
    except Exception as exc:emit(type='manifest_error',reason=str(exc),recording_id=recording_id)
  for e in executors:e.shutdown()
  for n in nodes:n.destroy_node()
  for sock in can_sockets:sock.close()
  if active_lock is not None:active_lock.__exit__(None,None,None)
  if lock is not None:lock.close()
  for c in contexts:
   if c.ok():c.shutdown()

if __name__=='__main__':
 try:sys.exit(run(json.loads(sys.argv[1])))
 except Exception as exc:
  print(json.dumps(dict(type='error',reason=str(exc))),flush=True);sys.exit(1)
