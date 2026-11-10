"""Task conversion orchestration; remote raw inputs remain read-only."""
import base64,json,hashlib,copy,os,threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from collections import Counter
from fractions import Fraction
import numpy as np
from PIL import Image
from processing.runtime import ROOT,data_root,new_id,exclusive_json
from processing.remote import request
from processing.schema78 import preflight,mapped_time,field,pose_values,aligned_segments

class Collector:
    def __init__(self,schema,stage,gpu_decode=False):
        self.schema=copy.deepcopy(schema);self.receive=schema.get('time_alignment',{}).get('kind')=='replay_receive_time_v1';self.packet_meta={};self.stage=Path(stage);self.stage.mkdir(parents=True,exist_ok=True)
        self.streams={i:{'times':[],'items':[]} for i in range(len(schema['bindings']))}
        self.images={c['role']:{'times':[],'items':[]} for c in schema['cameras']}
        self.codecs={};self.counts=Counter();self.errors=Counter();self.received=0;self.saturation={};self.gpu_decode=gpu_decode
        self.encoder=ThreadPoolExecutor(max_workers=min(24,os.cpu_count() or 4));self.pending=[];self.slots=threading.Semaphore(48)
        self.commands=None
        if schema.get('command_joint_bundle',{}).get('enabled'):
            from processing.command_targets import CommandTargets
            self.commands=CommandTargets(schema['command_joint_bundle'])
    def image(self,topic,frame):
        if frame.pts is None:raise ValueError('decoded frame has no source PTS; no inferred ordering timestamps')
        stamp=round(Fraction(frame.pts)*frame.time_base*1_000_000_000)
        array=frame.to_ndarray(format='rgb24')
        for c in self.schema['cameras']:
            if c['topic']!=topic:continue
            t=mapped_time(stamp,c['clock']);meta=self.packet_meta.pop((topic,stamp),None) if self.receive else None
            if self.receive and meta is None:raise ValueError('decoded image PTS has no exact receive/header correspondence')
            crop=c.get('crop')
            if crop is not None:
                x,y,w,h=crop
                if min(x,y)<0 or min(w,h)<=0 or y+h>array.shape[0] or x+w>array.shape[1]:raise ValueError('invalid/evidence-inconsistent image crop')
                image=array[y:y+h,x:x+w]
            else:image=array
            path=self.stage/(c['role']+'_'+str(self.counts[c['role']])+'.png')
            if path.exists():raise FileExistsError(str(path))
            self.slots.acquire()
            future=self.encoder.submit(Image.fromarray(image).save,path)
            future.add_done_callback(lambda _:self.slots.release());self.pending.append(future);self.counts[c['role']]+=1
            if self.images[c['role']]['times'] and t<=self.images[c['role']]['times'][-1]:self.errors['image nonmonotonic/duplicate PTS']+=1;continue
            self.images[c['role']]['times'].append(t);self.images[c['role']]['items'].append({'image':{'path':str(path),'shape':[3,image.shape[0],image.shape[1]]},'source_ns':(meta['header_ns'] if meta['header_ns'] is not None else -1) if self.receive else stamp,'receive_ns':stamp})
    def __call__(self,record):
        self.received+=1
        if self.received>2_000_000:raise ValueError('episode record processing limit exceeded')
        if record['kind']=='export_image':
            try:
                import av
                topic=record['topic'];key=(record['file'],topic)
                if key not in self.codecs:self.codecs[key]=av.CodecContext.create('h264_cuvid' if self.gpu_decode else 'h264','r')
                stamp=record['bag_ns'] if self.receive else record['header_ns']
                if self.receive:
                    if Path(record['file']).parent.name!='domain_24':raise ValueError('image from different recorder clock domain')
                    if (topic,stamp) in self.packet_meta:raise ValueError('duplicate image receive timestamp')
                    self.packet_meta[(topic,stamp)]={'header_ns':record['header_ns']}
                if stamp is None:raise ValueError('image header timestamp missing')
                packet=av.Packet(base64.b64decode(record['data']));packet.pts=stamp;packet.time_base=Fraction(1,1_000_000_000)
                for frame in self.codecs[key].decode(packet):self.image(topic,frame)
            except Exception as exc:self.errors['image: '+str(exc)]+=1
            return
        if self.commands:
            try:
                if self.receive and Path(record['file']).parent.name!='domain_24':raise ValueError('command group from different receive clock domain')
                self.commands.push(record)
            except Exception as exc:self.errors['command targets: '+str(exc)]+=1
        for i,b in enumerate(self.schema['bindings']):
            if b.get('kind')=='fk_command':continue
            if b['topic']!=record['topic']:continue
            if self.schema.get('schema')=='gento_skye_action78_base_rows_v2' and (not b.get('verified') or not b.get('clock',{}).get('verified')):continue
            try:
                if self.receive and Path(record['file']).parent.name!='domain_24':raise ValueError('record from different recorder clock domain')
                if b.get('message_type') and record['type']!=b['message_type']:raise ValueError('source message type differs from mapping')
                raw=record['header_ns'] if b['clock']['source']=='header' else record['bag_ns'];t=mapped_time(raw,b['clock']);message=record['message']
                if 'available' in message and not b.get('availability_path'):raise ValueError('source availability mask has no verified field mapping')
                if b.get('availability_path') and not field(message,b['availability_path']):raise ValueError('source value marked unavailable')
                if b['kind']=='fk':
                    from processing.kinematics import Kinematics
                    k=self.schema['kinematics']
                    if not k.get('historical_model_match_verified'):raise ValueError('historical model not verified')
                    if not hasattr(self,'fk'):self.fk=Kinematics(k['urdf'],k['mjcf'],k.get('hashes'))
                    values=self.fk.poses78(field(message,b['names_path']),field(message,b['path']))[b['eef']]
                elif b['kind']=='pose':
                    values=pose_values(field(message,b['path']),b,message,t)
                    if self.schema.get('schema')=='gento_skye_action78_base_rows_v2':
                        col1,col2=values[3:6],values[6:9]
                        values[3:]=np.column_stack((col1,col2,np.cross(col1,col2)))[:2].reshape(6)
                else:
                    if b.get('source_frame') and message.get('header',{}).get('frame_id')!=b['source_frame']:raise ValueError('scalar source frame differs from mapping')
                    if b.get('driver_input_transform'):
                        native_input=float(field(message,b['path']))
                        if not np.isfinite(native_input):raise ValueError('nonfinite driver gripper target')
                        message={**message,'data':float(np.clip(native_input,0.,1.))}
                    paths=b['path'] if isinstance(b['path'],list) else [b['path']]
                    values=np.asarray([field(message,path) for path in paths],dtype=float)*b.get('scale',1.)+b.get('offset',0.)
                saturation=[]
                if b.get('calibration'):
                    cal=b['calibration']
                    if cal.get('out_of_range')=='saturate_with_report' and cal.get('verified') and cal.get('open')!=cal.get('closed'):
                        for slot,x in zip(b['indices'],values):
                            fraction=(float(x)-cal['closed'])/(cal['open']-cal['closed'])
                            if np.isfinite(float(x)) and (fraction<0 or fraction>1):saturation.append((slot,float(x),'closed' if fraction<0 else 'open'))
                    from processing.action78_mapping import gripper01
                    values=np.asarray([gripper01(x,b['calibration']) for x in values])
                if len(values)!=len(b['indices']) or not np.isfinite(values).all():raise ValueError('invalid mapped values')
                stream=self.streams[i]
                if stream['times'] and t<=stream['times'][-1]:raise ValueError('duplicate/backward mapped timestamp; sample skipped')
                stream['times'].append(t);stream['items'].append({'values':values,'source_ns':raw,'header_ns':record['header_ns'] if record['header_ns'] is not None else -1})
                for slot,native,endpoint in saturation:
                    key=f'{b["target"]}:{slot}'
                    st=self.saturation.setdefault(key,dict(file=str(Path(__file__).resolve()),function='Collector.__call__',source_file=record['file'],topic=b['topic'],field=b['path'],dimension=slot,target=b['target'],count=0,closed_count=0,open_count=0,native_min=native,native_max=native,first_receive_ns=record['bag_ns'],last_receive_ns=record['bag_ns'],handling='user-authorized saturation to endpoint; remains valid if timestamp/sample checks pass',examples=[]))
                    st['count']+=1;st[endpoint+'_count']+=1
                    st['native_min']=min(st['native_min'],native);st['native_max']=max(st['native_max'],native);st['last_receive_ns']=record['bag_ns']
                    if len(st['examples'])<5:st['examples'].append(dict(receive_ns=record['bag_ns'],header_ns=record['header_ns'],native_value=native,endpoint=endpoint))

            except Exception as exc:self.errors[str(i)+': '+str(exc)]+=1
    def finish(self,origin_ns=None):
        if self.receive:
            if type(origin_ns) is not int:raise ValueError("missing domain_24 origin")
            self.schema["time_alignment"]["origin_ns"]=origin_ns
        for (_,topic),codec in self.codecs.items():
            try:
                for frame in codec.decode(None):self.image(topic,frame)
            except Exception as exc:self.errors['image flush: '+str(exc)]+=1
        for future in self.pending:future.result()
        self.encoder.shutdown()
        if self.receive and self.packet_meta:self.errors['H264 packets without exact decoded PTS correspondence']+=len(self.packet_meta)
        for role,stream in self.images.items():
            pairs=sorted(zip(stream['times'],stream['items']),key=lambda x:x[0]);stream['times']=[x[0] for x in pairs];stream['items']=[x[1] for x in pairs]
        if self.commands:self.streams['command_targets']=self.commands
        segments,report=aligned_segments(self.schema,self.streams,self.images)
        report['gripper_saturation']=self.saturation
        return segments,report

def convert_task(config,task,episodes,event,test_dir=None,mapping_path=None,verify_output=True):
    schema_path=Path(mapping_path or ROOT/'configs/processing/action78_posttrain.json')
    schema=json.loads(schema_path.read_text());conversion_id=new_id('conversion');lerobot_id=new_id('lerobot')
    base=Path(test_dir) if test_dir else data_root(config)
    report_dir=base/'conversions'/conversion_id;report_dir.mkdir(parents=True,exist_ok=False)
    stage=base/'temporary/conversions'/conversion_id;stage.mkdir(parents=True,exist_ok=False)
    is_v21=schema.get('schema')=='gento_skye_action78_base_rows_v2'
    if is_v21:
        from processing.task_v21 import policy_for
        policy_for(task['name'])
        if Path(task['name']).name!=task['name'] or task['name'] in ('.','..'):raise ValueError('unsafe task name')
    destination=base/'lerobot'/(task['name'] if is_v21 else lerobot_id)
    report=dict(schema=schema['schema'],conversion_id=conversion_id,lerobot_id=lerobot_id,source_robot=config['robot_ssh'],source_task=task,configuration=config,mapping=schema,mapping_sha256=hashlib.sha256(schema_path.read_bytes()).hexdigest(),status='running',episodes=[],validation='not_run',output=None)
    if schema.get('time_alignment',{}).get('kind')=='replay_receive_time_v1':
        report['time_alignment']=schema['time_alignment']
        report['warnings']=['Receive-time alignment only; device acquisition/exposure synchronization remains unverified']
        report['unverified_bindings']=[dict(target=b['target'],indices=b['indices'],topic=b['topic']) for b in schema['bindings'] if not b.get('verified')]
        event('WARNING',report['warnings'][0],unverified_bindings=report['unverified_bindings'])
    exclusive_json(report_dir/'started.json',report)
    mapping_errors=preflight(schema);segments=[];provenance=[];expected_shape=None
    for ordinal,episode in enumerate(episodes):
        entry=dict(source_episode=episode['name'],source_path=episode['path'],source_ordinal=ordinal,output_episode_indices=[],status='not_converted',warnings=[])
        report['episodes'].append(entry)
        if mapping_errors:
            entry['warnings']=mapping_errors;entry['reason']='physical/time/image mapping is insufficient to construct valid frames'
            event('WARNING','无法转换 '+episode['name']+'：'+entry['reason'],details=mapping_errors);continue
        try:
            collector=Collector(schema,stage/('episode_'+str(ordinal)),config.get('gpu_decode',False))
            cfg={**config,'extract_topics':sorted({b['topic'] for b in schema['bindings'] if b.get('topic') and b.get('verified')}),'extract_image_topics':sorted({c['topic'] for c in schema['cameras']})}
            if schema.get('command_joint_bundle',{}).get('enabled'):cfg['extract_topics']=sorted(set(cfg['extract_topics'])|{g['topic'] for g in schema['command_joint_bundle']['groups']})
            if schema.get('time_alignment',{}).get('kind')=='replay_receive_time_v1':cfg['extract_bag_subdir']='domain_24'
            extraction=request(cfg,'extract',episode['path'],progress=lambda s:event('INFO',s),record_handler=collector)
            entry['source_files']=extraction['source_files'];entry['warnings']=extraction['errors'];entry['record_errors']=dict(collector.errors)
            missing=[b['topic'] for i,b in enumerate(schema['bindings']) if b.get('verified') and b.get('kind')!='fk_command' and not collector.streams[i]['times']]
            entry['missing_source_topics']=sorted(set(missing))
            if missing:event('WARNING','录制中没有可用的直接控制源；对应维度保留无效，不使用上游输入或反馈代替',episode=episode['path'],topics=sorted(set(missing)))
            ready,alignment=collector.finish(extraction.get('timeline',{}).get('origin_ns'));entry['alignment']=alignment
            entry['gripper_saturation']=collector.saturation
            for key,stats in collector.saturation.items():event('WARNING','夹爪越界值已按用户规则截断到端点',episode=episode['path'],binding=key,details=stats)
            if collector.receive:entry['receive_timeline']=extraction.get('timeline')
            if extraction['errors'] or collector.errors:entry['warnings'].append('partial extraction; only independently valid contiguous frames eligible')
            # A changed recording is not exported as a stable capture.
            if any(not f['stable'] for f in extraction['source_files']):ready=[];entry['warnings'].append('source changed; episode skipped')
            for segment in ready:
                shape_list=[]
                for role in [c['role'] for c in schema['cameras']]:
                    value=segment[0]['observation.images.'+role]
                    if isinstance(value,dict) and value.get('shape'):shape_list.append((value['shape'][2],value['shape'][1]))
                    else:
                        with Image.open(value) as im:shape_list.append(im.size)
                shapes=tuple(shape_list)
                if expected_shape is not None and shapes!=expected_shape:entry['warnings'].append('camera sizes differ from dataset schema; segment skipped');continue
                expected_shape=shapes;index=len(segments);segments.append(segment);entry['output_episode_indices'].append(index)
                provenance.append(dict(output_episode_index=index,mapping_sha256=report['mapping_sha256'],source_task_path=task['path'],source_episode=episode['name'],source_episode_path=episode['path'],source_start_ns=int(segment[0]['source.grid_time_ns'][0]),source_end_ns=int(segment[-1]['source.grid_time_ns'][0]),capture_complete='unproven',receive_timeline=extraction.get('timeline'),physical_sync_verified=False))
            entry['status']='converted_partial' if entry['output_episode_indices'] else 'not_converted'
        except Exception as exc:
            entry['warnings'].append(dict(file=str(Path(__file__).resolve()),function='convert_task',reason=str(exc),handling='episode skipped; continue'));event('WARNING',str(exc),episode=episode['path'])
    if segments:
        try:
            if schema.get('schema')=='gento_skye_action78_base_rows_v2':
                from processing.task_v21 import write_dataset
            else:
                from processing.exporter import write_dataset
            staging_dataset=stage/'dataset'
            if is_v21:
                report['validation']=write_dataset(staging_dataset,segments,schema,task['name'],provenance,
                                                   existing=destination if destination.exists() else None,
                                                   verify=verify_output)
                manifest=json.loads((staging_dataset/'meta/export.json').read_text())
                for entry in report['episodes']:
                    entry['output_episode_indices']=[p['output_episode_index'] for p in manifest['episodes'] if p['source_episode_path']==entry['source_path']]
                    entry['status']='converted_partial' if entry['output_episode_indices'] else 'not_converted'
                report['format']='v2.1'
            else:
                report['validation']=write_dataset(staging_dataset,segments,schema,task['name'],provenance)
            destination.parent.mkdir(parents=True,exist_ok=True)
            if destination.exists():
                if not is_v21:raise FileExistsError(str(destination))
                archive=report_dir/'replaced_datasets';archive.mkdir()
                previous=archive/destination.name
                destination.rename(previous)
                try:staging_dataset.rename(destination)
                except Exception:
                    previous.rename(destination)
                    raise
            else:staging_dataset.rename(destination)
            report['output']=str(destination);report['status']='converted_partial'
            # Recording completeness is unproven even when every selected frame was exportable.
        except Exception as exc:
            report['status']='export_failed';report['export_error']=dict(file=str(Path(__file__).resolve()),function='convert_task',reason=str(exc));event('WARNING','数据集未交付：'+str(exc))
    else:report['status']='no_convertible_frames'
    exclusive_json(report_dir/'report.json',report)
    event('INFO','转换状态 '+report['status']+'；报告 '+str(report_dir))
    return report
