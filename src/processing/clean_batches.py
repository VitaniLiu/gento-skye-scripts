"""Ten-episode durable clean checkpoints; only one episode of images in memory."""
from pathlib import Path
import copy,gc,hashlib,json,os,shutil,time
import pyarrow as pa
import pyarrow.parquet as pq
from processing.runtime import ROOT
from processing.task_v21 import write_prepared,digest,dump,jsonl
from processing.static_plan import make_plan,check_plan_source
from processing.incremental_dataset import (atomic_json,seal,intact,load,rewrite_data,copy_sidecars,task_statistics,sync_dir,recover_transaction)
from processing.incremental_verify import verify_metadata

def source_signature(source):
    return {str(p.relative_to(source)):(p.stat().st_size,p.stat().st_mtime_ns)
            for p in sorted(source.rglob('*')) if p.is_file()}

def check_signature(source,signature):
    if source_signature(source)!=signature:raise ValueError('Source file list/size/mtime changed; regenerate preview')


def clean_batched(base,task,selected=None,plan=None,reportdir=None,failpoint=lambda _:None):
    pa.set_cpu_count(min(24,os.cpu_count() or 4))
    base=Path(base).resolve()
    if Path(task).name!=task or task in ('.','..'):raise ValueError('Unsafe task name')
    source=base/'lerobot'/task;target=base/'lerobot_clean'/task
    if source.is_symlink() or target.is_symlink():raise ValueError('Symlink dataset forbidden')
    if plan is None:
        from processing.clean_cli import create_preview
        plan,reportdir=create_preview(base,task,selected)
    reportdir=Path(reportdir).resolve()
    if reportdir.parent!=(base/'conversions').resolve():raise ValueError('Unsafe report directory')
    print('[clean] 校验来源与清洗计划……',flush=True)
    automatic=plan.get('review_status')=='auto_approved'
    if not automatic:check_plan_source(source,plan)
    signature=source_signature(source)
    expected=set(range(len(plan['episodes']))) if selected is None else set(selected)
    if plan['task']!=task or set(plan['selected_episodes'])!=expected:raise ValueError('Preview selection mismatch')
    if not automatic:
        fresh=make_plan(source,selected)
        for a,b in zip(fresh['episodes'],plan['episodes'],strict=True):
            if any(v!=b.get(k) for k,v in a.items()):raise ValueError('Preview plan changed; regenerate it')
    info=load(source/'meta/info.json');exp=load(source/'meta/export.json')
    if info['fps']!=20:raise ValueError('Expected 20Hz')
    code={n:digest(ROOT/'src/processing'/n) for n in ['clean_batches.py','static_plan.py','task_v21.py','incremental_dataset.py','incremental_verify.py']}
    key=hashlib.sha256(json.dumps({'source':plan['metadata_sha256'],'selected':sorted(expected),'code':code,'keep':[e['keep_indices'] for e in plan['episodes']]},sort_keys=True).encode()).hexdigest()
    work=base/'conversions/clean_checkpoints'/task/key;work.mkdir(parents=True,exist_ok=True)
    journal=base/'conversions/task_transactions'/('clean_'+task+'.json')
    recover_transaction(journal)
    records=[];total=len(plan['episodes']);start_time=time.monotonic();offsets=[];cursor=0
    for e in plan['episodes']:offsets.append(cursor);cursor+=e['output_frames']
    for batch_start in range(0,total,10):
        end=min(total,batch_start+10);marker=work/f'batch_{batch_start//10:06d}.json'
        if marker.exists():
            saved=load(marker)
            if len(saved)!=end-batch_start or any(not intact(work/x['artifact'],x['files']) for x in saved):raise ValueError('Checkpoint damaged; preserve files and investigate')
            records.extend(saved);print(f'[已保存 {end}/{total}] 复用第 {batch_start//10+1} 批，无需重新清洗',flush=True);continue
        pending=[]
        for ep in range(batch_start,end):
            print(f'[处理 {ep+1}/{total}；已保存 {batch_start}/{total}] episode_{ep:06d} 读取/清洗/校验',flush=True)
            pe=plan['episodes'][ep];path=source/info['data_path'].format(episode_chunk=ep//1000,episode_index=ep)
            if not automatic and digest(path)!=plan['metadata_sha256'][str(path.relative_to(source))]:raise ValueError('Source episode changed')
            rows=pq.read_table(path).to_pylist()
            q=pq.read_table(source/'meta/frame_quality.parquet',filters=[('episode_index','=',ep)]).to_pylist()
            m=pq.read_table(source/'meta/source_frame_mapping.parquet',filters=[('episode_index','=',ep)]).to_pylist()
            if not len(rows)==len(q)==len(m)==pe['input_frames']:raise ValueError('Source sidecar count mismatch')
            for i,(row,qr,mr) in enumerate(zip(rows,q,m,strict=True)):
                if any(x['frame_index']!=i or x['episode_index']!=ep for x in (row,qr,mr)):raise ValueError('Source pairing mismatch')
                row['_quality']={k:v for k,v in qr.items() if k not in ('episode_index','frame_index','index')}
                row['_source']={k:v for k,v in mr.items() if k not in ('episode_index','frame_index','index')}
            retained=[rows[i] for i in pe['keep_indices']]
            image_shape=info['features']['observation.images.head_color']['shape']
            for row in retained:
                row['observation.images.head_color']={**row['observation.images.head_color'],'shape':image_shape}
            artifact=work/f'episode_{ep:06d}'
            if artifact.exists():shutil.rmtree(artifact)  # uncommitted episode only, beneath fixed checkpoint root
            prov={**exp['episodes'][ep],'clean_source_episode_index':ep,'clean_removed_frames':pe['removed_frames'],'clean_split':False,
                  'source_start_ns':retained[0]['_source']['source_grid_time_ns'],'source_end_ns':retained[-1]['_source']['source_grid_time_ns']}
            write_prepared(artifact,[retained],load(source/'meta/schema78.json')['source_mapping'],task,[prov],exp['policy'],verify=False,
                           episode_index_offset=ep,index_offset=offsets[ep])
            if not automatic:
                # Full retained-value verification remains in the explicit/manual clean path.
                out=pq.read_table(artifact/'data/chunk-000/episode_000000.parquet').to_pylist()
                for before,after in zip(retained,out,strict=True):
                    for k,v in before.items():
                        if k.startswith('_') or k in ('timestamp','frame_index','index','episode_index'):continue
                        if k.startswith('observation.images.') and v is not None:
                            if after[k]['bytes']!=v['bytes']:raise ValueError('Retained image bytes changed: '+k)
                        elif after[k]!=v:raise ValueError('Retained content changed: '+k)
                for name,field in [('frame_quality.parquet','_quality'),('source_frame_mapping.parquet','_source')]:
                    side=pq.read_table(artifact/'meta'/name).to_pylist()
                    for before,after in zip(retained,side,strict=True):
                        if before[field]!={k:v for k,v in after.items() if k not in ('episode_index','frame_index','index')}:raise ValueError('Retained sidecar changed')
            record={'artifact':artifact.name,'files':seal(artifact),'report':{'source_episode_index':ep,'output_episode_index':ep,'selected':ep in expected,
                    'input_frames':len(rows),'removed_frames':pe['removed_frames'],'output_frames':len(retained),
                    'removed_runs':[{'start_frame':r['start'],'end_frame':r['end']-1,'frames':r['end']-r['start'],'duration_s':(r['end']-r['start'])/20} for r in pe['candidate_ranges']]}}
            pending.append(record)
            print(f'[处理 {ep+1}/{total}] {len(rows)} → {len(retained)} 帧，删除 {pe["removed_frames"]} 帧；耗时 {time.monotonic()-start_time:.1f}s',flush=True)
            del rows,q,m,retained,row
            if not automatic:del out,side,before,after
            gc.collect();pa.default_memory_pool().release_unused()
        for rec in pending:
            folder=work/rec['artifact']
            for p in folder.rglob('*'):
                if p.is_file():
                    with p.open('rb') as f:os.fsync(f.fileno())
            sync_dir(folder)
        atomic_json(marker,pending);records.extend(pending)
        print(f'[已保存 {end}/{total}] 第 {batch_start//10+1} 批持久保存完成',flush=True)
        failpoint('batch_saved')
    print('[汇总] 所有批次已保存；快速检查来源文件属性……',flush=True)
    check_signature(source,signature)
    stage=work/'final_dataset';legacy=(work/'legacy_format_v1').exists()
    if stage.exists() and not legacy:shutil.rmtree(stage)
    (stage/'meta').mkdir(parents=True,exist_ok=True);items=[];exports=[];eps=[];epstats=[];offset=0
    for ep,rec in enumerate(records):
        root=work/rec['artifact'];prov=load(root/'meta/export.json')['episodes'][0];st=load(root/'meta/episodes_stats.jsonl')['stats']
        src=root/prov['parquet'];dst=stage/f'data/chunk-{ep//1000:03d}/episode_{ep:06d}.parquet'
        if legacy:
            if not dst.exists():rewrite_data(src,dst,ep,offset)
            sha=digest(dst);original_ep=0
        else:
            dst.parent.mkdir(parents=True,exist_ok=True);os.link(src,dst)
            sha=prov['sha256'];original_ep=ep
        items.append({'root':root,'original_ep':original_ep,'prov':prov,'stats':st})
        st=copy.deepcopy(st);st['episode_index']={k:([ep] if k in ('min','max','mean') else v) for k,v in st['episode_index'].items()}
        st['index']={k:([v[0]+offset] if k in ('min','max','mean') else v) for k,v in st['frame_index'].items()}
        eps.append({'episode_index':ep,'tasks':[task],'length':prov['frames']});epstats.append({'episode_index':ep,'stats':st})
        exports.append({**prov,'output_episode_index':ep,'parquet':str(dst.relative_to(stage)),'sha256':sha,'parquet_bytes':dst.stat().st_size});offset+=prov['frames']
        print(f'[汇总 {ep+1}/{total}] episode_{ep:06d}',flush=True)
    for name in ['frame_quality.parquet','source_frame_mapping.parquet']:
        print('[汇总] 合并 '+name,flush=True)
        copy_sidecars(items,stage/'meta'/name,name)
    template=work/records[0]['artifact']/'meta'
    for name in ['info.json','tasks.jsonl','schema78.json','training_schema.json']:shutil.copyfile(template/name,stage/'meta'/name)
    newinfo=load(stage/'meta/info.json');newinfo.update(total_episodes=total,total_frames=offset,total_chunks=(total+999)//1000,splits={'train':f'0:{total}'})
    dump(stage/'meta/info.json',newinfo);jsonl(stage/'meta/episodes.jsonl',eps);jsonl(stage/'meta/episodes_stats.jsonl',epstats)
    print('[汇总] 计算全局统计（磁盘临时数组）……',flush=True)
    dump(stage/'meta/stats.json',task_statistics(stage,items));dump(stage/'meta/export.json',{**exp,'episodes':exports,'frames':offset,'commit_id':key})
    print('[汇总] 校验输出索引与元数据……',flush=True)
    validation=verify_metadata(stage)
    summary={'clean_id':reportdir.name,'status':'verified','output':str(target),'source':str(source),'method':plan['method'],'review_status':plan.get('review_status','unapproved_candidate'),
             'input_frames':plan['input_frames'],'removed_frames':plan['removed_frames'],'output_frames':offset,'retained_fields_verified':True,
             'selected_episodes':sorted(expected),'episodes':[r['report'] for r in records],'batch_size':10,'checkpoint':str(work),'validation':validation,
             'visualization':str(reportdir/'static_frames.html') if (reportdir/'static_frames.html').exists() else None,'source_sha256':plan['metadata_sha256']}
    dump(stage/'meta/clean_report.json',summary);dump(stage/'meta/clean_plan.json',plan)
    dump(stage/'meta/review_location.json',{'url_path':summary['visualization'],'review_status':summary['review_status']})
    print('[发布] 快速检查来源文件属性；不再全量读取源文件计算哈希……',flush=True)
    check_signature(source,signature)
    print('[发布] 正在将候选文件刷盘……',flush=True)
    for p in stage.rglob('*'):
        if p.is_file():
            with p.open('rb') as f:os.fsync(f.fileno())
    old=base/'conversions/commits'/('previous_clean_'+reportdir.name);old.parent.mkdir(parents=True,exist_ok=True);target.parent.mkdir(exist_ok=True)
    atomic_json(journal,{'status':'prepared','target':str(target),'stage':str(stage),'old':str(old),'commit_id':key})
    recover_transaction(journal);dump(reportdir/'clean_report.json',summary)
    print(f'[完成 {total}/{total}] 已发布 {target}',flush=True)
    return summary
