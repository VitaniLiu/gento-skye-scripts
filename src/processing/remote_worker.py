"""Read-only robot worker, sent over SSH stdin; never installed on robot."""
import sys,json,os,math,time,collections,numbers,base64
from pathlib import Path

def emit(kind,**data):
 print('GENTO_JSON '+json.dumps(dict(kind=kind,**data),ensure_ascii=False,allow_nan=False),flush=True)

def safe(root,path):
 root=Path(root); path=Path(path)
 if not root.is_absolute() or not path.is_absolute():raise ValueError('absolute source paths required')
 if root.is_symlink() or path.is_symlink():raise ValueError('symbolic link source forbidden')
 if not path.resolve().is_relative_to(root.resolve()):raise ValueError('source outside selected root')
 return path

def inventory(path):
 files=[]
 for base,dirs,names in os.walk(path,followlinks=False):
  dirs[:]=[n for n in dirs if not (Path(base)/n).is_symlink()]
  for name in names:
   f=Path(base)/name
   if not f.is_symlink() and f.is_file():files.append(f)
 return sorted(files)

def empty_fields(msg,prefix=''):
 if hasattr(msg,'get_fields_and_field_types'):
  for name in msg.get_fields_and_field_types():
   value=getattr(msg,name);key=prefix+'.'+name if prefix else name
   if value is None:yield key
   elif not isinstance(value,str) and hasattr(value,'__len__') and len(value)==0:yield key
   elif hasattr(value,'get_fields_and_field_types'):yield from empty_fields(value,key)

def numeric(msg,prefix=''):
 if hasattr(msg,'get_fields_and_field_types'):
  for k in msg.get_fields_and_field_types():
   if k=='validity':continue
   if hasattr(msg,'validity') and k in ('state','action','values','positions'):
    values=getattr(msg,k);validity=msg.validity
    if hasattr(validity,'__len__') and hasattr(values,'__len__') and len(validity)==len(values):
     for i,v in enumerate(values):
      if validity[i]:yield from numeric(v,f'{prefix}.{k}[{i}]' if prefix else f'{k}[{i}]')
     continue
   if k in ('data',) and ('CompressedImage' in type(msg).__name__ or type(msg).__name__ in ('Image','CompressedVideo')):continue
   yield from numeric(getattr(msg,k),prefix+'.'+k if prefix else k)
 elif isinstance(msg,numbers.Real) and not isinstance(msg,bool):yield prefix,float(msg)
 elif not isinstance(msg,(str,bytes,bytearray)) and hasattr(msg,'__iter__'):
  for i,v in enumerate(msg):yield from numeric(v,f'{prefix}[{i}]')

def inspect_episode(req):
 import yaml,rosbag2_py
 from rclpy.serialization import deserialize_message
 from rosidl_runtime_py.utilities import get_message
 root=Path(req['root']);path=safe(root,req['path']);files=inventory(path)
 issues=[];topics={}; stats={};counts=collections.Counter();files_report=[];recording_manifest=None
 def warn(code,location,**context):
  issues.append(dict(severity='WARNING',code=code,location=str(location),function='inspect_episode',reason=context.pop('reason',code),impact=context.pop('impact','requires review'),handling=context.pop('handling','preserve and continue'),**context))
 if not files:warn('EMPTY_EPISODE',path)
 manifest_path=path/'recording.json'
 if manifest_path.is_file():
  try:
   m=json.loads(manifest_path.read_text())
   if not isinstance(m,dict):raise ValueError('recording manifest root must be an object')
   recording_manifest={k:m.get(k) for k in ('schema_version','recording_id','lifecycle','file_status','capture_status','quality_status','status','failures','source_clock_mapping','camera_exposure_mapping')}
   recording_manifest['path']=str(manifest_path)
  except (ValueError,OSError) as exc:warn('RECORDING_MANIFEST_UNREADABLE',manifest_path,reason=str(exc))
 for asset in files:
  if asset.suffix.lower() in ('.mp4','.avi','.jpg','.jpeg','.png','.db3'):
   warn('ADDITIONAL_ASSET_NOT_DECODED',asset,reason='this revision reads MCAP messages; standalone asset parsing not implemented')
 metadata={}
 for f in files:
  if f.name=='metadata.yaml':
   try:metadata[f.parent]=yaml.safe_load(f.read_text())['rosbag2_bagfile_information']
   except Exception as exc:warn('METADATA_UNREADABLE',f,reason=str(exc))
 # Evidence comes from actual message channels, not assumed mappings.
 for f in files:
  if f.suffix!='.mcap':continue
  before=f.stat();actual=collections.Counter();decoded=0;read_count=0;footer=False
  try:
   with f.open('rb') as h:
    first=h.read(8);h.seek(-8,2);last=h.read(8)
   footer=first==last==b'\x89MCAP0\r\n'
   if not footer:warn('MCAP_MAGIC_OR_FOOTER_INVALID',f)
  except Exception as exc:warn('MCAP_HEADER_UNREADABLE',f,reason=str(exc))
  try:
   reader=rosbag2_py.SequentialReader()
   reader.open(rosbag2_py.StorageOptions(uri=str(f),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
   types={t.name:t.type for t in reader.get_all_topics_and_types()};classes={}
   for topic,typ in types.items():
    topics.setdefault(topic,dict(type=typ,count=0,decoded=0,first_ns=None,last_ns=None,backward=0,duplicate=0,max_gap_ns=0,frames={},image_decode_ok=0,image_decode_failed=0,image_unverified=0))
    try:classes[topic]=get_message(typ)
    except Exception as exc:warn('MESSAGE_TYPE_UNAVAILABLE',f,topic=topic,reason=str(exc))
   last_by_topic={}
   while reader.has_next():
    topic,data,t=reader.read_next();read_count+=1;actual[topic]+=1;counts[topic]+=1;st=topics[topic];st['count']+=1
    prev=last_by_topic.get(topic);last_by_topic[topic]=t
    if prev is not None:
     st['backward']+=int(t<prev);st['duplicate']+=int(t==prev);st['max_gap_ns']=max(st['max_gap_ns'],t-prev)
    st['first_ns']=t if st['first_ns'] is None else min(st['first_ns'],t);st['last_ns']=t if st['last_ns'] is None else max(st['last_ns'],t)
    if topic not in classes:continue
    try:
     msg=deserialize_message(data,classes[topic]);decoded+=1;st['decoded']+=1
     for field in empty_fields(msg):
      empty=st.setdefault('empty_fields',{});empty[field]=empty.get(field,0)+1
     stamp=None
     if hasattr(msg,'frame_id'):
      frame=str(msg.frame_id);st['frames'][frame]=st['frames'].get(frame,0)+1
     if hasattr(msg,'timestamp'):
      stamp=msg.timestamp.sec*1000000000+msg.timestamp.nanosec
      st['source_timestamp_ns']=stamp
     if hasattr(msg,'header'):
      frame=msg.header.frame_id;st['frames'][frame]=st['frames'].get(frame,0)+1
      stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
      # Domain unknown: report numerical difference, never label it calibrated skew.
      previous_header=st.get('last_header_ns');st['last_header_ns']=stamp
      if previous_header is not None:
       st['header_backward']=st.get('header_backward',0)+int(stamp<previous_header);st['header_duplicate']=st.get('header_duplicate',0)+int(stamp==previous_header)
      st.setdefault('header_minus_bag_ns',{'min':stamp-t,'max':stamp-t})
      delta=st['header_minus_bag_ns'];delta['min']=min(delta['min'],stamp-t);delta['max']=max(delta['max'],stamp-t)
     if 'CompressedImage' in types[topic] or 'CompressedVideo' in types[topic]:
      fmt=str(msg.format);st['image_format']=fmt
      if 'jpeg' in fmt.lower() or 'jpg' in fmt.lower() or 'png' in fmt.lower():
       import cv2,numpy as np
       im=cv2.imdecode(np.frombuffer(bytes(msg.data),dtype=np.uint8),cv2.IMREAD_UNCHANGED)
       if im is None:st['image_decode_failed']+=1
       else:st['image_decode_ok']+=1;st['image_shape']=list(im.shape)
      else:
       st['image_unverified']+=1
       if 'h264' in fmt.lower():emit('image_packet',file=str(f),topic=topic,bag_time_ns=t,header_time_ns=stamp,data=base64.b64encode(bytes(msg.data)).decode('ascii'))
      continue
     for field,value in numeric(msg):
      if field.startswith('header.') or field.endswith('validity'):continue
      key=(topic,field);v=stats.setdefault(key,dict(count=0,nonfinite=0,zero_count=0,zero_start=None,max_zero_duration_ns=0,min=None,max=None))
      v['count']+=1
      previous_field_time=v.get('last_bag_time_ns')
      if previous_field_time is not None and (t<previous_field_time or t-previous_field_time>req['gap_s']*1e9):v['zero_start']=None
      v.setdefault('first_bag_time_ns',t);v['last_bag_time_ns']=t
      if not math.isfinite(value):v['nonfinite']+=1;v['zero_start']=None;continue
      v['min']=value if v['min'] is None else min(v['min'],value);v['max']=value if v['max'] is None else max(v['max'],value)
      if abs(value)<=req['zero_tolerance']:
       v['zero_count']+=1
       if v['zero_start'] is None:v['zero_start']=t
       v['max_zero_duration_ns']=max(v['max_zero_duration_ns'],t-v['zero_start'])
      else:v['zero_start']=None
    except Exception as exc:
     if sum(i['code']=='RECORD_DECODE_FAILED' for i in issues)<20:warn('RECORD_DECODE_FAILED',f,topic=topic,record_index=read_count,bag_time_ns=t,reason=str(exc),handling='record skipped')
    if read_count%10000==0:emit('progress',episode=str(path),file=str(f),messages=read_count)
  except Exception as exc:warn('MCAP_READ_FAILED',f,reason=str(exc),messages_read=read_count,handling='remaining file skipped; next file continues')
  after=f.stat();stable=before.st_size==after.st_size and before.st_mtime_ns==after.st_mtime_ns
  if not stable:warn('SOURCE_CHANGED_DURING_READ',f)
  files_report.append(dict(path=str(f),size=before.st_size,mtime_ns=before.st_mtime_ns,messages_read=read_count,decoded=decoded,counts=dict(actual),footer_valid=footer,stable_during_read=stable))
 for folder,meta in metadata.items():
  actual=sum(f['messages_read'] for f in files_report if Path(f['path']).parent==folder)
  if actual!=meta.get('message_count'):warn('METADATA_COUNT_MISMATCH',folder/'metadata.yaml',expected=meta.get('message_count'),actual=actual)
  for entry in meta.get('topics_with_message_count',[]):
   topic=entry['topic_metadata']['name'];expected=entry['message_count']
   measured=sum(f['counts'].get(topic,0) for f in files_report if Path(f['path']).parent==folder)
   if expected!=measured:warn('TOPIC_COUNT_MISMATCH',folder/'metadata.yaml',topic=topic,expected=expected,actual=measured)
  for info in meta.get('files',[]):
   measured=sum(f['messages_read'] for f in files_report if Path(f['path'])==folder/info['path'])
   if info.get('message_count')!=measured:warn('METADATA_FILE_COUNT_MISMATCH',folder/info['path'],expected=info.get('message_count'),actual=measured)
  for rel in meta.get('relative_file_paths',[]):
   target=safe(root,folder/rel)
   if not target.is_file():warn('REFERENCED_FILE_MISSING',target)
 if not metadata:warn('METADATA_MISSING',path)
 warn('CRC_VERIFICATION_UNAVAILABLE',path,reason='native reader open/read and footer checks do not provide an explicit full CRC audit')
 warn('RECORDING_COMPLETENESS_UNPROVEN',path,reason='footer and stable file do not prove capture completion')
 warn('CLOCK_MAPPING_UNKNOWN',path,reason='bag receive time and header time retained separately; clock domains/calibration not proven')
 warn('SEMANTIC_MAPPING_UNVERIFIED',path,reason='units, ranges, transforms and 78-D validity mapping are not established; no zero padding performed')
 zero=[]
 for (topic,field),v in stats.items():
  v.pop('zero_start');v['zero_ratio']=v['zero_count']/v['count']
  if v['nonfinite']:warn('NONFINITE_VALUES',path,topic=topic,field=field,count=v['nonfinite'])
  if v['zero_ratio']>=req['zero_ratio'] and v['max_zero_duration_ns']>=req['zero_duration_s']*1e9:
   zero.append(dict(topic=topic,field=field,**v,interpretation='possible stationary/closed gripper/uncollected/abnormal feedback; no automatic fault classification',validity='unknown: source observation only, not valid Action78 slots'))
 # Only reviewed, observed topic candidates; presence is not full semantic proof.
 groups=req['logical_candidates'];coverage={}
 for group,candidates in groups.items():
  present=[t for t in candidates if topics.get(t,{}).get('decoded',0)>0]
  coverage[group]={'present_candidates':present,'semantics_verified':False,'bag_time_ranges':{t:[topics[t]['first_ns'],topics[t]['last_ns']] for t in present}}
  if not present:warn('LOGICAL_STREAM_NOT_OBSERVED',path,stream=group)
  else:warn('LOGICAL_STREAM_SEMANTICS_UNVERIFIED',path,stream=group,reason='candidate topic presence does not prove full body/gripper coverage or actual chassis send')
 for topic,st in topics.items():
  if st.get('empty_fields'):warn('EMPTY_FIELDS',path,topic=topic,fields=st['empty_fields'])
  for flag in ('backward','duplicate','image_decode_failed','image_unverified','header_backward','header_duplicate'):
   if st.get(flag):warn(flag.upper(),path,topic=topic,count=st[flag])
  if st['max_gap_ns']>req['gap_s']*1e9:warn('SAMPLING_GAP',path,topic=topic,max_gap_ns=st['max_gap_ns'])
 warn('CAMERA_IDENTITY_AND_CROP_UNVERIFIED',path,reason='head identity/time require evidence; wrist cameras not required by current conversion policy')
 return dict(source_episode=path.name,source_path=str(path),recording_manifest=recording_manifest,files=files_report,topics=topics,logical_streams=coverage,zero_observations=zero,numeric_statistics=[dict(topic=t,field=f,**v) for (t,f),v in stats.items()],issues=issues,completion='unproven',status=('partial_or_unreadable' if any(i['code'] in ('MCAP_READ_FAILED','SOURCE_CHANGED_DURING_READ','MCAP_MAGIC_OR_FOOTER_INVALID') for i in issues) else 'inspected_with_warnings') if any(f['messages_read'] for f in files_report) else 'no_readable_mcap',messages_read=sum(f['messages_read'] for f in files_report))

def extract_episode(req):
 import rosbag2_py
 from rclpy.serialization import deserialize_message
 from rosidl_runtime_py.utilities import get_message
 from rosidl_runtime_py.convert import message_to_ordereddict
 path=safe(Path(req['root']),req['path']);count=0;errors=[];fingerprints=[];batch=[]
 def flush():
  if batch:emit('records',items=batch);batch.clear()
 wanted=set(req['extract_topics']);image_topics=set(req.get('extract_image_topics',[]));timeline_min=None;timeline_max=None
 subdir=req.get('extract_bag_subdir')
 if subdir is not None and subdir!='domain_24':raise ValueError('unsupported recorder clock domain')
 if subdir and not (path/subdir).is_dir():raise ValueError('missing domain_24 recorder directory')
 for f in inventory(path):
  if f.suffix!='.mcap':continue
  if subdir and f.parent!=path/subdir:continue
  before=f.stat();read_count=0
  try:
   reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(f),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
   types={t.name:t.type for t in reader.get_all_topics_and_types()};classes={t:get_message(v) for t,v in types.items() if t in wanted|image_topics}
   while reader.has_next():
    topic,data,stamp=reader.read_next();read_count+=1
    timeline_min=stamp if timeline_min is None else min(timeline_min,stamp)
    timeline_max=stamp if timeline_max is None else max(timeline_max,stamp)
    if topic not in classes:continue
    try:
     msg=deserialize_message(data,classes[topic]);header_ns=None
     if hasattr(msg,'header'):header_ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
     if hasattr(msg,'timestamp'):header_ns=msg.timestamp.sec*1000000000+msg.timestamp.nanosec
     if topic in image_topics:
      if 'h264' not in msg.format.lower():raise ValueError('conversion currently supports evidenced H264 access-unit streams only')
      flush()
      emit('export_image',file=str(f),topic=topic,bag_ns=stamp,header_ns=header_ns,data=base64.b64encode(bytes(msg.data)).decode('ascii'))
     else:
      batch.append(dict(kind='record',file=str(f),topic=topic,type=types[topic],bag_ns=stamp,header_ns=header_ns,message=message_to_ordereddict(msg)))
      if len(batch)>=256:flush()
     count+=1
    except Exception as exc:
     if len(errors)<100:errors.append(dict(file=str(f),topic=topic,bag_ns=stamp,reason=str(exc)))
    if count%10000==0:emit('progress',file=str(f),extracted_records=count)
  except Exception as exc:errors.append(dict(file=str(f),reason=str(exc),read_count=read_count))
  after=f.stat();stable=(before.st_size,before.st_mtime_ns)==(after.st_size,after.st_mtime_ns)
  fingerprints.append(dict(path=str(f),size=before.st_size,mtime_ns=before.st_mtime_ns,stable=stable))
  if not stable:errors.append(dict(file=str(f),reason='source changed during extraction'))
 flush()
 emit('result',extracted_records=count,errors=errors,source_files=fingerprints,timeline=dict(origin_ns=timeline_min,end_ns=timeline_max,bag_subdir=subdir))

def committed_or_legacy(path):
 manifest=path/'recording.json'
 if not manifest.exists():return True
 try:m=json.loads(manifest.read_text())
 except (ValueError,OSError):return False
 if m.get('schema_version')!='2.0':return True
 return m.get('lifecycle')=='retained' and m.get('file_status')=='closed' and m.get('capture_status')=='completed'

def main(req):
 root=Path(req['root'])
 if not root.is_absolute() or root.is_symlink():raise ValueError('absolute non-symlink source root required')
 if not root.is_dir():raise ValueError('source root does not exist or is inaccessible: '+str(root))
 if req['op']=='tasks':
  emit('result',items=[dict(name=p.name,path=str(p)) for p in sorted(root.iterdir()) if p.name not in ('_pending','_discarded') and p.is_dir() and not p.is_symlink()]);return
 path=safe(root,req['path'])
 if req['op']=='episodes':
  items=[]
  # A task may directly contain one episode, or have episode child directories.
  if list(path.glob('*.mcap')):items=[path]
  else:items=[p for p in sorted(path.iterdir()) if p.is_dir() and not p.is_symlink() and any(f.suffix=='.mcap' for f in inventory(p))]
  emit('result',items=[dict(name=p.name,path=str(p)) for p in items if committed_or_legacy(p)]);return
 if req['op']=='fingerprint':
  files=[]
  for f in inventory(path):
   st=f.stat();files.append(dict(path=str(f),size=st.st_size,mtime_ns=st.st_mtime_ns))
  emit('result',fingerprint=dict(files=files,committed=committed_or_legacy(path),method='file-list-size-mtime-v1'));return
 if req['op']=='extract':
  if not committed_or_legacy(path):raise ValueError('v2 capture must be completed and explicitly retained before extraction')
  extract_episode(req);return
 if req['op']=='inspect':emit('result',report=inspect_episode(req));return
 raise ValueError('unknown read-only operation')
