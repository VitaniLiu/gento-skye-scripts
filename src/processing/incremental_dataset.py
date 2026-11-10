"""Bounded-image-memory task assembly and recoverable per-episode publication."""
from pathlib import Path
import copy,json,os,shutil,uuid
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from processing.task_v21 import COLUMNS,digest,dump,jsonl

def atomic_json(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_name(path.name+'.new')
    with tmp.open('w',encoding='utf-8') as f:
        json.dump(value,f,ensure_ascii=False,allow_nan=False,indent=2);f.write('\n');f.flush();os.fsync(f.fileno())
    os.replace(tmp,path);sync_dir(path.parent)

def sync_dir(path):
    if os.name=='nt':return  # Production host is Linux; directory fsync is unavailable on Windows.
    fd=os.open(path,os.O_RDONLY)
    try:os.fsync(fd)
    finally:os.close(fd)

def load(path):return json.loads(Path(path).read_text(encoding='utf-8'))

def seal(folder):
    folder=Path(folder)
    # File signatures are only a fast-path; changed files must be hashed again.
    return {str(p.relative_to(folder)):{'sha256':digest(p),'size':p.stat().st_size,
             'mtime_ns':p.stat().st_mtime_ns} for p in sorted(folder.rglob('*')) if p.is_file()}

def intact(folder,files):
    folder=Path(folder)
    if not files:return False
    if {str(p.relative_to(folder)) for p in folder.rglob('*') if p.is_file()}!=set(files):return False
    for rel,item in files.items():
        p=folder/rel
        if p.is_symlink() or not p.is_file():return False
        st=p.stat()
        if st.st_size!=item['size']:return False
        if st.st_mtime_ns!=item['mtime_ns'] and digest(p)!=item['sha256']:return False
    return True

def recover_transaction(journal):
    """Finish a prepared snapshot after crashes before/between/after renames."""
    journal=Path(journal)
    if not journal.exists():return
    j=load(journal);target=Path(j['target']);stage=Path(j['stage']);old=Path(j['old'])
    if j['status']=='done':return
    # Paths are written by this module and constrained again before recovery.
    base=journal.parent.parent.parent.resolve()
    for p in (target,stage,old):
        if not p.resolve().is_relative_to(base) or p.is_symlink():raise ValueError('Unsafe transaction path')
    if stage.exists():
        if target.exists():
            if old.exists():raise ValueError('Ambiguous task commit; preserve both versions')
            target.rename(old);sync_dir(target.parent);sync_dir(old.parent)
        stage.rename(target);sync_dir(target.parent)
    elif not target.exists():
        if old.exists():old.rename(target)
        raise ValueError('Prepared snapshot missing; previous version restored')
    elif load(target/'meta/export.json').get('commit_id')!=j['commit_id']:
        raise ValueError('Commit cannot be identified; preserve files for recovery')
    atomic_json(journal,{**j,'status':'done'})

def rewrite_data(source,dest,ep,offset):
    dest.parent.mkdir(parents=True,exist_ok=True)
    writer=None;cursor=0
    try:
        for batch in pq.ParquetFile(source).iter_batches(batch_size=8):
            t=pa.Table.from_batches([batch]);n=len(t)
            for name,values in [('episode_index',np.full(n,ep)),('index',np.arange(cursor+offset,cursor+offset+n))]:
                k=t.schema.get_field_index(name);t=t.set_column(k,t.schema.field(k),pa.array(values,type=pa.int64()))
            if writer is None:writer=pq.ParquetWriter(dest,t.schema,compression='snappy')
            writer.write_table(t);cursor+=n
    finally:
        if writer:writer.close()
    return cursor

class SideReader:
    def __init__(self,path):
        self.batches=iter(pq.ParquetFile(path).iter_batches(batch_size=256));self.buffer=None
    def episode(self,ep):
        import pyarrow.compute as pc
        while True:
            if self.buffer is None:
                try:self.buffer=pa.Table.from_batches([next(self.batches)])
                except StopIteration:return
            part=self.buffer.filter(pc.equal(self.buffer['episode_index'],ep))
            future=self.buffer.filter(pc.greater(self.buffer['episode_index'],ep))
            self.buffer=future if len(future) else None
            if len(part):yield part
            if len(future):return

def copy_sidecars(items,path,key):
    writer=None;offset=0;readers={}
    try:
        for ep,item in enumerate(items):
            src=item['root']/'meta'/key
            if src not in readers:readers[src]=SideReader(src)
            for t in readers[src].episode(item['original_ep']):
                if not len(t):continue
                n=len(t);start=item.get('_side_cursor',0)
                for name,values in [('episode_index',np.full(n,ep)),('index',np.arange(offset+start,offset+start+n))]:
                    k=t.schema.get_field_index(name);t=t.set_column(k,t.schema.field(k),pa.array(values,type=pa.int64()))
                if writer is None:writer=pq.ParquetWriter(path,t.schema,compression='snappy')
                writer.write_table(t);item['_side_cursor']=start+n
            if item.pop('_side_cursor',0)!=item['prov']['frames']:raise ValueError('Sidecar frame count mismatch')
            offset+=item['prov']['frames']
    finally:
        if writer:writer.close()

def task_statistics(stage,items):
    """Exact numeric quantiles on disk-backed scratch, never collect image rows."""
    n=sum(i['prov']['frames'] for i in items);stats={}
    scratch=stage/'.numeric_scratch'
    widths=[78,78,78,1,1,1,1,1];keys=COLUMNS[:3]+COLUMNS[6:]
    data=np.memmap(scratch,mode='w+',dtype=np.float64,shape=(n,sum(widths)));cursor=0;v=None
    try:
        for ep in range(len(items)):
            p=stage/f'data/chunk-{ep//1000:03d}/episode_{ep:06d}.parquet'
            for batch in pq.ParquetFile(p).iter_batches(batch_size=256,columns=keys):
                t=pa.Table.from_batches([batch]);arrays=[np.asarray(t[k].to_pylist()).reshape(len(t),w) for k,w in zip(keys,widths)]
                data[cursor:cursor+len(t)]=np.concatenate(arrays,axis=1);cursor+=len(t)
        if cursor!=n:raise ValueError('Numeric row count mismatch')
        j=0
        for key,w in zip(keys,widths):
            # Only one channel-sized vector is resident when computing quantiles.
            v=data[:,j:j+w];out={name:[float(fn(v[:,k])) for k in range(w)] for name,fn in [('min',np.min),('max',np.max),('mean',np.mean),('std',np.std)]}
            if w==78:
                for name,q in [('q01',.01),('q10',.1),('q50',.5),('q90',.9),('q99',.99)]:out[name]=[float(np.quantile(v[:,k],q)) for k in range(w)]
            out['count']=[n];stats[key]=out;j+=w
        k=COLUMNS[3];es=[i['stats'][k] for i in items];weights=np.array([x['count'][0] for x in es],float)
        means=np.array([x['mean'] for x in es]);stds=np.array([x['std'] for x in es])
        mean=np.average(means,axis=0,weights=weights);variance=np.average(stds**2+means**2,axis=0,weights=weights)-mean**2
        stats[k]={'min':np.min([x['min'] for x in es],axis=0).tolist(),'max':np.max([x['max'] for x in es],axis=0).tolist(),
                  'mean':mean.tolist(),'std':np.sqrt(np.maximum(variance,0)).tolist(),'count':[int(weights.sum())]}
    finally:
        v=None
        del data
        scratch.unlink(missing_ok=True)
    return stats

def assemble(base,task,artifact,journal,failpoint=lambda _:None):
    """Append/replace checkpoint episodes in one task commit."""
    base=Path(base).resolve();artifacts=[Path(x).resolve() for x in (artifact if isinstance(artifact,(list,tuple)) else [artifact])];target=base/'lerobot'/task
    if Path(task).name!=task or task in ('.','..') or target.is_symlink():raise ValueError('Unsafe task')
    recover_transaction(journal)
    fresh=[]
    for root in artifacts:
        exp=load(root/'meta/export.json');info=load(root/'meta/info.json')
        if len(exp['episodes'])!=1:raise ValueError('One source episode per checkpoint required')
        fresh.append((root,exp,info,exp['episodes'][0],load(root/'meta/episodes_stats.jsonl')['stats']))
    newexp,newinfo=fresh[-1][1:3];items=[];template=fresh[-1][0]
    if target.exists():
        oldexp=load(target/'meta/export.json');oldinfo=load(target/'meta/info.json')
        if oldexp['policy']!=newexp['policy']:raise ValueError('Existing task policy changed: explicit migration required; no partial overwrite')
        if oldinfo['features']!=newinfo['features']:raise ValueError('Incompatible task schema or camera size')
        oldstats=[json.loads(x)['stats'] for x in (target/'meta/episodes_stats.jsonl').read_text().splitlines()]
        for ep,prov in enumerate(oldexp['episodes']):
            items.append({'root':target,'old':True,'original_ep':ep,'prov':prov,'stats':oldstats[ep]})
    for root,exp,info,prov,stats in fresh:
        if exp['policy']!=newexp['policy'] or info['features']!=newinfo['features']:raise ValueError('Checkpoint schema or policy mismatch')
        replacement=next((i for i,x in enumerate(items) if x['prov']['source_episode_path']==prov['source_episode_path']),len(items))
        item={'root':root,'old':False,'original_ep':0,'prov':prov,'stats':stats}
        if replacement==len(items):items.append(item)
        else:items[replacement]=item
    commit=uuid.uuid4().hex;parent=base/'temporary/conversions'/('commit_'+commit);stage=parent/'dataset';stage.mkdir(parents=True)
    (stage/'meta').mkdir();eps=[];exports=[];epstats=[];offset=0
    for ep,item in enumerate(items):
        prov=copy.deepcopy(item['prov']);src=item['root']/prov['parquet'];dst=stage/f'data/chunk-{ep//1000:03d}/episode_{ep:06d}.parquet'
        dst.parent.mkdir(parents=True,exist_ok=True)
        # Existing indices remain stable for append and for same-length replacements.
        can_link=item['old'] and ep==item['original_ep'] and item.get('old_offset',sum(x['prov']['frames'] for x in items[:ep]))==offset
        if item['old']:
            old_start=sum(p['frames'] for p in oldexp['episodes'][:item['original_ep']])
            can_link=ep==item['original_ep'] and old_start==offset
        if can_link:
            signature={'size':src.stat().st_size,'mtime_ns':src.stat().st_mtime_ns}
            if signature!=prov.get('verified_file_signature') and digest(src)!=prov['sha256']:raise ValueError('Historical data changed; no commit')
            os.link(src,dst);sha=prov['sha256']
        else:
            if rewrite_data(src,dst,ep,offset)!=prov['frames']:raise ValueError('Data frame count mismatch')
            sha=digest(dst)
        st=copy.deepcopy(item['stats'])
        for key,value in [('episode_index',ep)]:st[key]={'min':[value],'max':[value],'mean':[value],'std':[0.],'count':[prov['frames']]}
        # Local checkpoint index stats become task-global index stats.
        local=st['frame_index'];st['index']={k:([v[0]+offset] if k in ('min','max','mean') else v) for k,v in local.items()}
        eps.append({'episode_index':ep,'tasks':[task],'length':prov['frames']});epstats.append({'episode_index':ep,'stats':st})
        exports.append({**prov,'output_episode_index':ep,'parquet':str(dst.relative_to(stage)),'sha256':sha,'parquet_bytes':dst.stat().st_size,
                        'verified_file_signature':{'size':dst.stat().st_size,'mtime_ns':dst.stat().st_mtime_ns}})
        offset+=prov['frames']
    for key in ['frame_quality.parquet','source_frame_mapping.parquet']:copy_sidecars(items,stage/'meta'/key,key)
    info=copy.deepcopy(newinfo);info.update(total_episodes=len(items),total_frames=offset,total_chunks=(len(items)+999)//1000,splits={'train':f'0:{len(items)}'})
    dump(stage/'meta/info.json',info);jsonl(stage/'meta/episodes.jsonl',eps);jsonl(stage/'meta/episodes_stats.jsonl',epstats)
    for name in ['tasks.jsonl','schema78.json','training_schema.json']:shutil.copyfile(template/'meta'/name,stage/'meta'/name)
    dump(stage/'meta/stats.json',task_statistics(stage,items))
    dump(stage/'meta/export.json',{**newexp,'episodes':exports,'frames':offset,'commit_id':commit,'incremental':True})
    # Check metadata/numeric pairing in batches; already verified image bytes are never decoded again.
    from processing.incremental_verify import verify_metadata
    verify_metadata(stage)
    for p in stage.rglob('*'):
        if p.is_file():
            with p.open('rb') as f:os.fsync(f.fileno())
    for directory in sorted([p for p in stage.rglob('*') if p.is_dir()],key=lambda p:len(p.parts),reverse=True):sync_dir(directory)
    sync_dir(stage);sync_dir(stage.parent);sync_dir(stage.parent.parent)
    old=base/'conversions'/'commits'/('previous_'+commit);old.parent.mkdir(parents=True,exist_ok=True);target.parent.mkdir(exist_ok=True)
    atomic_json(journal,{'status':'prepared','target':str(target),'stage':str(stage),'old':str(old),'commit_id':commit})
    failpoint('prepared')
    if target.exists():target.rename(old);sync_dir(target.parent);sync_dir(old.parent)
    failpoint('old_moved')
    stage.rename(target);sync_dir(target.parent)
    failpoint('installed')
    recover_transaction(journal)
    return {'status':'verified','episodes':len(items),'frames':offset,'format':'v2.1','columns':11,
            'verification':'new episode full verification + task metadata/indices in batches; unchanged historical images reused'}
