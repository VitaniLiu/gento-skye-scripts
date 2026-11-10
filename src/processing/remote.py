"""SSH read-only execution, with bounded connection and operation deadlines."""
import json,subprocess,shlex,threading,queue,time,base64
from pathlib import Path

def request(config,op,path=None,progress=print,record_handler=None):
    payload={**config,'op':op,'root':config['robot_raw_root'],'path':path}
    worker=Path(__file__).with_name('remote_worker.py').read_text()
    code=worker+'\ntry:\n main('+repr(payload)+')\nexcept Exception as exc:\n emit("error",file="<remote-worker-stdin>",function="main",reason=str(exc))\n'
    command='source /etc/apex/apex_ros_env.sh; PYTHONDONTWRITEBYTECODE=1 python3 -'
    args=['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=10','-o','ServerAliveInterval=10','-o','ServerAliveCountMax=3',config['robot_ssh'],'bash -c '+shlex.quote(command)]
    proc=subprocess.Popen(args,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    messages=queue.Queue(maxsize=64);stderr=[];result=None;errors=[];last_progress=None;decoders={};video={}
    def collect(stream,kind):
        for line in stream:messages.put((kind,line))
        messages.put((kind,None))
    for stream,kind in ((proc.stdout,'out'),(proc.stderr,'err')):
        threading.Thread(target=collect,args=(stream,kind),daemon=True).start()
    def feed():
        try:proc.stdin.write(code);proc.stdin.close()
        except (OSError,ValueError):pass
    threading.Thread(target=feed,daemon=True).start()
    deadline=time.monotonic()+config.get('remote_timeout_s',1800);finished=set()
    try:
        while len(finished)<2:
            if time.monotonic()>deadline:raise TimeoutError('SSH read deadline exceeded')
            try:kind,line=messages.get(timeout=.2)
            except queue.Empty:continue
            if line is None:finished.add(kind);continue
            if kind=='err':stderr.append(line);stderr=stderr[-30:];continue
            if not line.startswith('GENTO_JSON '):continue
            item=json.loads(line[len('GENTO_JSON '):])
            if item['kind']=='records':
                if record_handler is not None:
                    for record in item['items']:record_handler(record)
            elif item['kind'] in ('record','export_image'):
                if record_handler is not None:record_handler(item)
            elif item['kind']=='image_packet':
                key=(item['file'],item['topic'])
                stats=video.setdefault(key,dict(packets=0,decoded_frames=0,errors=0,first_bag_time_ns=item['bag_time_ns'],last_bag_time_ns=item['bag_time_ns'],dimensions=[]))
                stats['packets']+=1;stats['last_bag_time_ns']=item['bag_time_ns']
                try:
                    import av
                    if key not in decoders:decoders[key]=av.CodecContext.create('h264','r')
                    for packet in decoders[key].parse(base64.b64decode(item['data'])):
                        for frame in decoders[key].decode(packet):
                            stats['decoded_frames']+=1
                            size=[frame.width,frame.height]
                            if size not in stats['dimensions']:stats['dimensions'].append(size)
                except Exception as exc:stats['errors']+=1;stats['last_error']=str(exc)
            elif item['kind']=='progress':last_progress=item;progress(json.dumps(item,ensure_ascii=False))
            elif item['kind']=='result':result=item
            elif item['kind']=='error':errors.append(item)
        status=proc.wait(timeout=5)
        if status or errors or result is None:raise RuntimeError('no complete remote result')
        for key,codec in decoders.items():
            try:
                for packet in codec.parse(b''):
                    video[key]['decoded_frames']+=len(codec.decode(packet))
                video[key]['decoded_frames']+=len(codec.decode(None))
            except Exception as exc:video[key]['errors']+=1;video[key]['last_error']=str(exc)
        if 'report' in result:
            result['report']['video_decode']=[dict(file=k[0],topic=k[1],**v,frame_time_mapping='unverified; bag packet time is not decoder frame exposure time') for k,v in video.items()]
            for item in result['report']['video_decode']:
                if not item['errors'] and item['decoded_frames']:
                    state=result['report']['topics'].get(item['topic'],{})
                    state['host_bitstream_decode_verified']=True
                    # This only supersedes remote decoder availability, not camera/timestamp validation.
                    for issue in result['report']['issues']:
                        if issue.get('code')=='IMAGE_UNVERIFIED' and issue.get('topic')==item['topic']:
                            issue['handling']='host H264 bitstream decode succeeded; per-frame camera/time mapping still unverified'
                result['report']['issues'].append(dict(severity='WARNING',code='VIDEO_TIME_MAPPING_UNVERIFIED' if not item['errors'] else 'VIDEO_DECODE_ERRORS',file=str(Path(__file__).resolve()),function='request',location=item['file'],topic=item['topic'],reason='H264 decoded on host from streamed packets; physical camera and per-frame timing remain unverified',impact='do not assert image/action synchronization',handling='retain packet time range and decode counts',decoded_frames=item['decoded_frames'],errors=item['errors']))
        return result
    except Exception as exc:
        proc.kill();proc.wait()
        raise RuntimeError(json.dumps(dict(file=str(Path(__file__).resolve()),function='request',source_path=path,remote_errors=errors,last_confirmed_progress=last_progress,stderr=''.join(stderr)[-4000:],reason=str(exc),handling='do not claim successful complete read'),ensure_ascii=False)) from exc
