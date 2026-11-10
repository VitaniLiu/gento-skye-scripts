"""Raw protocol v2 and robot-local, explicit-retention lifecycle. Stdlib only."""
import contextlib,ctypes,errno,fcntl,json,math,os,re,time,uuid
from pathlib import Path
STREAMS=('pico_action','gamesir_action','body_state','chassis_state','image')
STAGES=('control_target','host_send','robot_receive','chassis_publish','measured_feedback','image_sample')
RESERVED=('_pending','_discarded')
def fail(reason,**context):raise ValueError(json.dumps(dict(reason=reason,**context),ensure_ascii=False))
def validate_record(r):
    for k in ('schema_version','recording_id','episode_index','stream','sequence','source','timestamps','payload','missing'):
        if k not in r:fail('required field missing',field=k)
    version=r['schema_version']
    if version not in ('1.0','2.0') or r['stream'] not in STREAMS:fail('unsupported schema/stream')
    if type(r['sequence']) is not int or r['sequence']<0:fail('invalid sequence')
    index=r['episode_index']
    if not (version=='2.0' and index is None) and (type(index) is not int or index<0):fail('invalid episode index')
    if version=='2.0':
        if r.get('stage') not in STAGES:fail('explicit stage required')
        if r.get('lifecycle') not in ('pending','retained'):fail('invalid record lifecycle')
        if r['lifecycle']=='pending' and index is not None:fail('pending cannot reserve episode number')
        if r['lifecycle']=='retained' and index is None:fail('retained requires episode number')
    if not r['recording_id'] or not r['source']:fail('identity and source required')
    if not isinstance(r['payload'],dict) or not isinstance(r['missing'],list):fail('invalid payload/missing')
    for m in r['missing']:
        if not isinstance(m,dict) or not all(m.get(k) for k in ('field','reason','impact')):fail('invalid missing evidence')
    if not isinstance(r['timestamps'],list) or not r['timestamps']:fail('timestamps required')
    kinds=('device_sample','host_receive','robot_receive','send','bag_receive','robot_publish','dds_source')
    for t in r['timestamps']:
        if not isinstance(t,dict) or t.get('unit') not in ('ns','us','ms','s','tick') or not t.get('clock_domain') or type(t.get('value')) not in (int,float) or not math.isfinite(t['value']) or t.get('kind') not in kinds:fail('invalid timestamp')
    if version=='2.0' and not any(t['kind'] in ('host_receive','robot_receive','bag_receive') for t in r['timestamps']):fail('receive time required')
    if r['stream']=='gamesir_action':
        p=r['payload']
        if p.get('sent') is not True or not all(k in p for k in ('command_id','send_result','owner','enable_epoch')):fail('actual send evidence required')
        if version=='2.0' and (r['stage'] not in ('host_send','robot_receive','chassis_publish') or not p['command_id'] or not p['send_result']):fail('invalid send semantics')
    return r

def fsync_dir(p):
    fd=os.open(p,os.O_RDONLY|os.O_DIRECTORY)
    try:os.fsync(fd)
    finally:os.close(fd)
def atomic_json(path,data):
    path=Path(path);tmp=path.with_name(path.name+'.'+uuid.uuid4().hex+'.next')
    try:
        with tmp.open('x') as f:json.dump(data,f,indent=2,ensure_ascii=False,allow_nan=False);f.flush();os.fsync(f.fileno())
        os.replace(tmp,path);fsync_dir(path.parent)
    finally:
        if tmp.exists():tmp.unlink()
@contextlib.contextmanager
def locked(path):
    fd=os.open(path,os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
    try:
        fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB);yield
    finally:os.close(fd)
def rename_noreplace(src,dst):
    libc=ctypes.CDLL(None,use_errno=True)
    fn=getattr(libc,'renameat2',None)
    if fn is None:raise RuntimeError('atomic no-replace rename unavailable; refusing fallback')
    fn.argtypes=[ctypes.c_int,ctypes.c_char_p,ctypes.c_int,ctypes.c_char_p,ctypes.c_uint]
    if fn(-100,os.fsencode(src),-100,os.fsencode(dst),1):
        e=ctypes.get_errno();raise OSError(e,os.strerror(e),str(dst))
    fsync_dir(Path(src).parent);fsync_dir(Path(dst).parent)

class Lifecycle:
    def __init__(self,root):
        self.root=Path(root)
        if not self.root.is_absolute() or self.root.resolve()!=self.root or not self.root.is_dir():fail('existing real absolute raw root required')
    def area(self,name):
        p=self.root/name
        if p.is_symlink():fail('symlink area forbidden')
        p.mkdir(exist_ok=True);fsync_dir(self.root);return p
    def rid(self,rid):
        if not re.fullmatch(r'recording_[0-9a-f]{32}',rid):fail('invalid recording_id')
        return rid
    def task(self,name):
        if not isinstance(name,str) or not name.strip() or name in ('.','..') or name.startswith(('.','_')) or any(c in name for c in ('/','\\','\x00')) or len(name)>100:fail('invalid/reserved task name')
        return name
    def prepare_task(self,name):
        name=self.task(name)
        with locked(self.root/'.lifecycle.lock'):
            path=self.root/name
            if path.is_symlink():fail('task symlink forbidden')
            existed=path.exists();path.mkdir(exist_ok=True);fsync_dir(self.root)
            return dict(path=str(path),task_name=name,existed=existed)
    def begin(self,task,config):
        task=self.task(task);rid='recording_'+uuid.uuid4().hex
        p=self.area('_pending')/rid;p.mkdir()
        atomic_json(p/'recording.json',dict(schema_version='2.0',recording_id=rid,task_name=task,episode_index=None,lifecycle='pending',file_status='writing',capture_status='in_progress',quality_status='unassessed',started_robot_unix_ns=time.time_ns(),finished_robot_unix_ns=None,configuration=config,failures=[],file_index=[]))
        fsync_dir(p.parent);return p
    def read(self,p):
        if p.resolve()!=p or not p.is_relative_to(self.root) or not p.is_dir():fail('recording directory missing, escaped or symlink')
        f=p/'recording.json'
        if f.is_symlink():fail('manifest symlink forbidden')
        m=json.loads(f.read_text())
        if m.get('schema_version')!='2.0':fail('legacy data is read-only to lifecycle operations')
        return m
    def locate(self,rid):
        rid=self.rid(rid)
        for name in RESERVED:
            p=self.root/name/rid
            if p.exists():return p
        for task in self.root.iterdir():
            if task.name.startswith(('.','_')) or not task.is_dir() or task.is_symlink():continue
            for p in task.iterdir():
                if re.fullmatch(r'episode[0-9]{8}',p.name) and p.is_dir() and not p.is_symlink():
                    f=p/'recording.json'
                    if f.exists() and not f.is_symlink() and json.loads(f.read_text()).get('recording_id')==rid:return p
        fail('unknown recording_id')
    def inspect(self,rid):
        p=self.locate(rid);return dict(path=str(p),manifest=self.read(p))
    def list_pending(self):
        p=self.root/'_pending'
        if not p.exists():return []
        if p.is_symlink():fail('symlink pending forbidden')
        result=[]
        for x in sorted(p.iterdir()):
            if not x.is_dir() or x.is_symlink():continue
            try:result.append(dict(path=str(x),manifest=self.read(x)))
            except (ValueError,OSError) as exc:result.append(dict(path=str(x),recording_id=x.name,file_status='incomplete',capture_status='unknown',error=str(exc)))
        return result
    def keep(self,rid):
        with locked(self.root/'.lifecycle.lock'):
            p=self.locate(rid);m=self.read(p)
            with locked(p/'.active.lock'):
                if p.parent.name=='_discarded' or m.get('lifecycle')=='discarded':fail('discarded capture cannot be retained')
                if p.parent!=self.root/'_pending':
                    if m['lifecycle']=='retaining':
                        m['lifecycle']='retained';atomic_json(p/'recording.json',m)
                    return dict(path=str(p),manifest=m)
                if m.get('file_status')!='closed' or m.get('capture_status')!='completed':fail('unfinished/abnormal capture cannot be committed')
                task=self.root/self.task(m['task_name'])
                if task.is_symlink():fail('task symlink forbidden')
                task.mkdir(exist_ok=True);fsync_dir(self.root)
                numbers=[int(x.name[7:]) for x in task.iterdir() if re.fullmatch(r'episode[0-9]{8}',x.name)]
                index=max(numbers,default=-1)+1
                if index>99999999:fail('episode index exhausted')
                dst=task/f'episode{index:08d}'
                m.update(lifecycle='retaining',episode_index=index,retention_requested_ns=time.time_ns(),retained_path=str(dst))
                atomic_json(p/'recording.json',m)
                rename_noreplace(p,dst)
                m['lifecycle']='retained';atomic_json(dst/'recording.json',m)
                return dict(path=str(dst),manifest=m)
    def discard(self,rid):
        with locked(self.root/'.lifecycle.lock'):
            p=self.locate(rid);m=self.read(p)
            with locked(p/'.active.lock'):
                if p.parent.name=='_discarded':return dict(path=str(p),manifest=m)
                if p.parent!=self.root/'_pending':fail('retained episode cannot be discarded here')
                m.update(lifecycle='discarded',episode_index=None,discarded_ns=time.time_ns())
                atomic_json(p/'recording.json',m)
                dst=self.area('_discarded')/rid;rename_noreplace(p,dst)
                return dict(path=str(dst),manifest=m)
