"""Convert, persist and release one source episode at a time; validate outputs on request."""
from pathlib import Path
import gc,hashlib,json,shutil,time
from datetime import datetime,timezone
from processing.runtime import ROOT,data_root,new_id
from processing.incremental_dataset import atomic_json,load,seal,intact,assemble,recover_transaction
from processing.remote import request
from processing.task_v21 import policy_for,digest

# Audited migration baselines, filled once during this upgrade, never on normal startup.
BASELINE_DATA_HASH='8eea00a491e89bb7fc1b65495c8a0cff5c8ccf13fe4b4cb90762320bc2e13a7e'
DATA_FILES=('conversion.py','task_v21.py','base78.py','schema78.py','kinematics.py','command_targets.py','action78_mapping.py')
PERFORMANCE_COMPATIBLE_RULES={'8a962d15ff30be7232f556dd1dcdb801d588ff076c09c59b68de841ecec17040','49716ab57b74549c89d4f69541f147d3c6944336f1ea295f265bcb1939453cb2'}

def data_baseline():return key({name:digest(ROOT/'src/processing'/name) for name in DATA_FILES})

def key(value):return hashlib.sha256(json.dumps(value,sort_keys=True,ensure_ascii=False).encode()).hexdigest()

def rule_versions(config,mapping,task,check_only=False):
    conversion=key({'version':'action78-frozen-v1','mapping':digest(mapping),'policy':None if check_only else policy_for(task)})
    return None,conversion

def existing_output(target,episode,mapping_hash,policy):
    if not (target/'meta/export.json').exists():return None
    exp=load(target/'meta/export.json')
    if exp['policy']!=policy:return None
    from processing.task_v21 import structural
    if load(target/'meta/info.json')['action_mask']!=structural(policy).astype(bool).tolist():return None
    for p in exp['episodes']:
        if p['source_episode_path']==episode['path'] and p.get('mapping_sha256')==mapping_hash:
            f=target/p['parquet']
            if f.exists() and f.stat().st_size==p['parquet_bytes']:return p
    return None

def import_delivered(base,target,episode,source,mapping_hash,policy,record,conversion_rule):
    prov=existing_output(target,episode,mapping_hash,policy)
    if prov is None:return None
    if prov.get('conversion_fingerprint') not in PERFORMANCE_COMPATIBLE_RULES|{conversion_rule}:return None
    elif not BASELINE_DATA_HASH or data_baseline()!=BASELINE_DATA_HASH:return None
    # Only import old delivery if its original extraction source file fingerprints match.
    expected={x['path']:(x['size'],x['mtime_ns']) for x in source['files'] if x['path'].endswith('.mcap') and Path(x['path']).parent.name=='domain_24'}
    for path in sorted((base/'conversions').glob('conversion_*/report.json'),reverse=True):
        try:r=load(path)
        except (OSError,ValueError):continue
        if r.get('mapping_sha256')!=mapping_hash or not isinstance(r.get('validation'),dict) or r['validation'].get('status')!='verified':continue
        entry=next((e for e in r.get('episodes',[]) if e.get('source_path')==episode['path'] and e.get('output_episode_indices')),None)
        if not entry:continue
        got={x['path']:(x['size'],x['mtime_ns']) for x in entry.get('source_files',[]) if x.get('stable')}
        if expected!=got:continue
        # Legacy policy/mapping must match the manifest; full output verification once.
        from processing.incremental_verify import verify_metadata
        verify_metadata(target)
        return {**record,'status':'committed','conversion_rule':conversion_rule,'imported_from':str(path),
                'output_parquet':prov['parquet'],'output_sha256':prov['sha256'],
                'output_signature':file_signature(target/prov['parquet'])}
    return None

def compatible_checkpoint(store,target,episode,source,mapping_hash,policy,record,conversion_rule):
    prov=existing_output(target,episode,mapping_hash,policy)
    if prov is None or prov.get('conversion_fingerprint') not in PERFORMANCE_COMPATIBLE_RULES:return None
    for path in sorted(Path(store).glob('*/record.json'),reverse=True):
        try:
            old=load(path);artifact=path.parent/'dataset'
            if old.get('source_path')==episode['path'] and old.get('source')==source and old.get('status')=='committed' and artifact.is_dir():
                return {**record,'status':'committed','conversion_rule':conversion_rule,'imported_from':str(path),
                        'output_parquet':prov['parquet'],'output_sha256':prov['sha256'],'output_signature':file_signature(target/prov['parquet'])}
        except (OSError,ValueError,KeyError):continue
    return None

def file_signature(path):
    st=Path(path).stat();return {'size':st.st_size,'mtime_ns':st.st_mtime_ns}

def release_episode_memory():
    """Drop Python/Arrow objects and return unused glibc heap pages on the host."""
    gc.collect()
    import pyarrow as pa
    pa.default_memory_pool().release_unused()
    try:
        import ctypes
        trim=getattr(ctypes.CDLL(None),'malloc_trim',None)
        if trim is not None:
            trim.argtypes=[ctypes.c_size_t];trim.restype=ctypes.c_int;trim(0)
    except (OSError,AttributeError):pass

def output_valid(target,record):
    try:
        exp=load(target/'meta/export.json')
        prov=next(p for p in exp['episodes'] if p['source_episode_path']==record['source_path'])
        f=target/prov['parquet']
        if prov.get('checkpoint_key')!=record['checkpoint_key'] and 'imported_from' not in record:return False
        if prov['sha256']!=record['output_sha256']:return False
        return file_signature(f)==record['output_signature'] or digest(f)==record['output_sha256']
    except (OSError,KeyError,StopIteration,ValueError):return False

def checkpoint_artifact(store,source_path):
    """Newest durable single-episode artifact for explicit output-only checking."""
    candidates=[]
    for recordpath in Path(store).glob('*/record.json'):
        try:
            record=load(recordpath);artifact=recordpath.parent/'dataset'
            if record.get('source_path')==source_path and record.get('status') in ('ready','committed') and artifact.is_dir():
                candidates.append((recordpath.stat().st_mtime_ns,artifact))
        except (OSError,ValueError,KeyError):continue
    return max(candidates,default=(None,None))[1]

def run(config,task,episodes,event,test_dir=None,mapping_path=None,check_only=False,
        request_fn=None,convert_fn=None,failpoint=lambda _:None):
    request_fn=request_fn or request
    if convert_fn is None:
        from processing.conversion import convert_task
        convert_fn=convert_task
    base=Path(test_dir).resolve() if test_dir else data_root(config)
    staging=(base/'temporary/conversions') if test_dir else Path('/var/tmp/gento-skye-convert')
    staging.mkdir(parents=True,exist_ok=True)
    mapping=Path(mapping_path or ROOT/'configs/processing/action78_posttrain.json')
    taskname=task['name']
    if Path(taskname).name!=taskname or taskname in ('.','..'):raise ValueError('Unsafe task')
    store=base/'conversions/checkpoints'/taskname;store.mkdir(parents=True,exist_ok=True)
    journal=base/'conversions/task_transactions'/(taskname+'.json');recover_transaction(journal)
    _,convert_rule=rule_versions(config,mapping,taskname,check_only);mapping_hash=digest(mapping);policy=None if check_only else policy_for(taskname)
    identifier=new_id('conversion');folder=base/'conversions'/identifier;folder.mkdir()
    target=base/'lerobot'/taskname
    if target.exists():
        from processing.incremental_verify import verify_metadata
        verify_metadata(target)
    summary={'conversion_id':identifier,'lerobot_id':taskname,'source_robot':config['robot_ssh'],'source_task':task,
             'mapping_sha256':mapping_hash,'status':'running','episodes':[],'output':str(target) if target.exists() else None,
             'validation':{'status':'not_run'},'mode':'incremental_v1','started_utc':datetime.now(timezone.utc).isoformat()}
    atomic_json(folder/'started.json',summary)
    atomic_json(folder/'report.json',summary)
    try:
        if check_only:
            from processing.task_v21 import verify_dataset
            for ep in episodes:
                started=time.monotonic();entry={'source_episode':ep['name'],'source_path':ep['path'],
                    'output_episode_indices':[],'status':'pending'};summary['episodes'].append(entry)
                try:
                    artifact=checkpoint_artifact(store,ep['path'])
                    if artifact is None:raise ValueError('No converted single-episode checkpoint is available')
                    validation=verify_dataset(artifact);entry.update(status='output_verified',validation=validation,
                        checkpoint=str(artifact),elapsed_s=round(time.monotonic()-started,3))
                    event('INFO',ep['name']+'：单episode输出完整性检查通过')
                except Exception as exc:
                    entry.update(status='failed',reason=str(exc));event('WARNING',ep['name']+'：'+str(exc))
                finally:atomic_json(folder/'report.json',summary)
            failed=any(e['status']=='failed' for e in summary['episodes'])
            summary['status']='partial_failure' if failed else 'output_check_finished'
            summary['validation']={'status':'partial_failure' if failed else 'verified','scope':'selected single-episode checkpoints'}
            summary['finished_utc']=datetime.now(timezone.utc).isoformat();atomic_json(folder/'report.json',summary)
            return summary
        pending=[]
        for ordinal,ep in enumerate(episodes):
            started=time.monotonic();event('INFO',f"任务 {taskname} [{ordinal+1}/{len(episodes)}] {ep['name']}：核对历史记录")
            entry={'source_episode':ep['name'],'source_path':ep['path'],'output_episode_indices':[],'status':'pending'}
            summary['episodes'].append(entry)
            try:
                source=request_fn(config,'fingerprint',ep['path'])['fingerprint']
                if not source['committed']:raise ValueError('Source is not a committed/legacy capture')
                identity=key({'robot':config['robot_ssh'],'path':ep['path'],'source':source,'rule':convert_rule})
                epdir=store/identity;epdir.mkdir(exist_ok=True);recordpath=epdir/'record.json'
                record=load(recordpath) if recordpath.exists() else {'source_path':ep['path'],'source':source,'checkpoint_key':identity,'conversion_rule':convert_rule,'status':'new'}
                if record['status']=='new' and (migrated:=compatible_checkpoint(store,target,ep,source,mapping_hash,policy,record,convert_rule)):
                    record=migrated;atomic_json(recordpath,record)
                # Dataset rename may have succeeded immediately before record update.
                if not check_only and record['status'] in ('ready','committed') and target.exists():
                    p=next((p for p in load(target/'meta/export.json')['episodes'] if p['source_episode_path']==ep['path'] and
                            (p.get('checkpoint_key')==identity or 'imported_from' in record)),None)
                    try:recoverable=p and (file_signature(target/p['parquet'])==p.get('verified_file_signature') or digest(target/p['parquet'])==p['sha256'])
                    except OSError:recoverable=False
                    if recoverable:
                        record.update(status='committed',output_parquet=p['parquet'],output_sha256=p['sha256'],output_signature=file_signature(target/p['parquet']))
                        atomic_json(recordpath,record)
                if not check_only and record['status']=='committed' and output_valid(target,record):
                    entry.update(status='skipped_committed');event('INFO',ep['name']+'：已保存且来源/规则未变，跳过检查和转换')
                elif not check_only and record['status']=='new' and (adopted:=import_delivered(base,target,ep,source,mapping_hash,policy,record,convert_rule)):
                    record=adopted;atomic_json(recordpath,record);entry.update(status='skipped_imported');event('INFO',ep['name']+'：已有正式结果验证通过，已登记复用')
                else:
                    artifact=epdir/'dataset'
                    ready=record['status'] in ('ready','committed') and artifact.exists() and intact(artifact,record.get('files'))
                    if not ready:
                        # A fresh unique work folder means incomplete files are never treated as complete.
                        work=staging/new_id('conversion');work.mkdir()
                        result=convert_fn(config,task,[ep],event,test_dir=work,mapping_path=mapping,verify_output=False)
                        if result.get('validation',{}).get('status')!='not_run' or not result.get('output'):
                            raise ValueError('Episode export did not complete with deferred verification')
                        after=request_fn(config,'fingerprint',ep['path'])['fingerprint']
                        if after!=source:raise ValueError('Source changed during conversion; do not commit')
                        produced=Path(result['output']);manifest=load(produced/'meta/export.json')
                        if len(manifest['episodes'])!=1:raise ValueError('Unexpected split episode result')
                        manifest['episodes'][0]['checkpoint_key']=identity;manifest['episodes'][0]['conversion_fingerprint']=convert_rule
                        atomic_json(produced/'meta/export.json',manifest)
                        if artifact.exists():artifact.rename(epdir/('invalid_dataset_'+new_id('run')))
                        if produced.stat().st_dev==epdir.stat().st_dev:produced.rename(artifact)
                        else:
                            incoming=epdir/('incoming_'+new_id('run'));shutil.copytree(produced,incoming);incoming.rename(artifact)
                        # Publish a ready record only after all checkpoint contents are durable.
                        import os
                        from processing.incremental_dataset import sync_dir
                        for p in artifact.rglob('*'):
                            if p.is_file():
                                with p.open('rb') as f:os.fsync(f.fileno())
                        for directory in sorted([p for p in artifact.rglob('*') if p.is_dir()],key=lambda p:len(p.parts),reverse=True):sync_dir(directory)
                        sync_dir(artifact);sync_dir(epdir);sync_dir(epdir.parent)
                        record.update(status='ready',files=seal(artifact),episode_report=result['episodes'][0],work_dir=str(work))
                        atomic_json(recordpath,record);failpoint('ready')
                        shutil.rmtree(work,ignore_errors=True);record.pop('work_dir',None);atomic_json(recordpath,record)
                    entry.update(status='checkpoint_ready');pending.append((ep,entry,artifact,record,recordpath,started))
                if target.exists() and entry['status']!='checkpoint_ready':
                    ex=load(target/'meta/export.json');entry['output_episode_indices']=[p['output_episode_index'] for p in ex['episodes'] if p['source_episode_path']==ep['path']]
                entry['elapsed_s']=round(time.monotonic()-started,3)
            except Exception as exc:
                entry.update(status='failed',reason=str(exc));event('WARNING',ep['name']+'：'+str(exc))
            finally:
                # No cross-episode collector/decoded image/Arrow row lists survive.
                release_episode_memory()
                try:
                    status=Path('/proc/self/status').read_text()
                    entry['resident_memory_mib']=round(int(status.split('VmRSS:')[1].split()[0])/1024,2)
                except (OSError,IndexError,ValueError):pass
                atomic_json(folder/'report.json',summary)
        if pending:
            try:
                validation=assemble(base,taskname,[x[2] for x in pending],journal,failpoint);export=load(target/'meta/export.json')
                for ep,entry,artifact,record,recordpath,started in pending:
                    prov=next(p for p in export['episodes'] if p['source_episode_path']==ep['path'])
                    record.update(status='committed',output_parquet=prov['parquet'],output_sha256=prov['sha256'],output_signature=file_signature(target/prov['parquet']))
                    atomic_json(recordpath,record);entry.update(status='converted_saved',output_episode_indices=[prov['output_episode_index']],elapsed_s=round(time.monotonic()-started,3))
                    if record.get('work_dir'):
                        work=Path(record['work_dir']).resolve()
                        if work.is_relative_to(staging.resolve()) and not work.is_symlink():shutil.rmtree(work,ignore_errors=True)
                    event('INFO',f"{ep['name']}：已保存 {prov['frames']} 帧，可从此处续跑")
                summary['validation']=validation;summary['output']=str(target);atomic_json(folder/'report.json',summary)
            except Exception as exc:
                for _,entry,_,_,_,_ in pending:entry.update(status='failed',reason=str(exc))
                event('WARNING','批量发布失败：'+str(exc));atomic_json(folder/'report.json',summary)
        failed=any(e['status']=='failed' for e in summary['episodes'])
        summary['status']='partial_failure' if failed else 'converted_partial'
        if target.exists():
            from processing.incremental_verify import verify_metadata
            validation=verify_metadata(target);summary['validation']={**validation,'format':'v2.1','columns':11}
        summary['finished_utc']=datetime.now(timezone.utc).isoformat();atomic_json(folder/'report.json',summary)
        return summary
    except BaseException:
        summary['status']='interrupted';atomic_json(folder/'report.json',summary);raise
