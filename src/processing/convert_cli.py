"""Interactive conversion front end; raw data remains read-only."""
import argparse,fcntl,hashlib,json,sys
from pathlib import Path
from processing.runtime import ROOT,load_config,data_root,new_id,exclusive_json
from processing.remote import request
from processing.main import main as run_pipeline,choose

def delivered(base,config,task,mapping_hash=None,warn=print):
    """Use successful trace records AND existing standard datasets, never directory names alone."""
    found=[]
    for path in sorted((base/'conversions').glob('*/report.json')):
        try:
            r=json.loads(path.read_text())
            if r.get('source_robot')!=config['robot_ssh'] or r.get('source_task',{}).get('path')!=task['path']:continue
            if mapping_hash and r.get('mapping_sha256')!=mapping_hash:continue
            if not r.get('output') or not isinstance(r.get('validation'),dict) or r['validation'].get('status')!='verified':continue
            output=Path(r['output'])
            if output.is_symlink() or output.parent.resolve()!=(base/'lerobot').resolve():continue
            if not (output/'meta/info.json').is_file():continue
            manifest=output/'meta/export.json'
            if manifest.is_file() and json.loads((output/'meta/info.json').read_text()).get('codebase_version')=='v2.1':
                export=json.loads(manifest.read_text())
                if not (output/'meta/frame_quality.parquet').is_file():continue
                paths={e['source_episode_path'] for e in export['episodes']}
                if mapping_hash and any(e.get('mapping_sha256')!=mapping_hash for e in export['episodes']):continue
            else:
                paths={e['source_path'] for e in r['episodes'] if e.get('output_episode_indices') and e.get('status','').startswith('converted')}
            if paths:found.append((path,r,output,paths))
        except (ValueError,OSError,KeyError,TypeError) as exc:
            warn(f'WARNING: {Path(__file__).resolve()} delivered: 无法读取追溯 {path}: {exc}')
    return found

def select_mode(prompt,options,input_fn=input):
    while True:
        print(prompt)
        for key,label in options.items():print(f'{key}. {label}')
        value=input_fn('请选择（q退出）：').strip()
        if value.lower()=='q':return None
        if value in options:return value
        print('输入无效，请重新选择。')

def replace_previous(base,config,task,selected,previous,report):
    """Retire old task datasets only after verified delivery covers every selected source."""
    covered={e['source_path'] for e in report.get('episodes',[]) if e.get('output_episode_indices')}
    output=Path(report['output']) if report.get('output') else None
    if not output or not isinstance(report.get('validation'),dict) or report['validation'].get('status')!='verified' or not {e['path'] for e in selected}<=covered:
        return {'status':'kept_previous','reason':'new verified dataset does not cover every selected episode','retired':[]}
    archive=base/'conversions'/report['conversion_id']/'replaced_datasets'
    retired=[];errors=[];seen=set()
    for _,old,path,_ in previous:
        if path in seen or path==output:continue
        seen.add(path)
        try:
            if path.is_symlink() or path.parent.resolve()!=(base/'lerobot').resolve():raise ValueError('unsafe dataset location')
            if not path.exists():continue
            archive.mkdir(parents=True,exist_ok=True)
            target=archive/path.name
            if target.exists():raise FileExistsError(str(target))
            path.rename(target)
            retired.append({'old_output':str(path),'archive':str(target),'conversion_id':old['conversion_id']})
        except (OSError,ValueError) as exc:
            errors.append({'file':str(Path(__file__).resolve()),'function':'replace_previous','path':str(path),'reason':str(exc)})
    return {'status':'replaced' if not errors else 'replacement_partial','retired':retired,'errors':errors,'output':str(output)}

def main(argv=None):
    parser=argparse.ArgumentParser(description='convert：交互选择机器人任务、操作及episode范围')
    parser.add_argument('--test-output',help='离线/实数据测试输出，必须在项目tests内')
    args=parser.parse_args(argv)
    config=load_config('processing')
    base=Path(args.test_output).resolve() if args.test_output else data_root(config)
    if args.test_output and not base.is_relative_to((ROOT/'tests').resolve()):parser.error('测试输出必须位于项目tests/')
    lockpath=(base if args.test_output else ROOT/'logs/processing')/'convert.lock'
    lockpath.parent.mkdir(parents=True,exist_ok=True)
    with lockpath.open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            print(f'FAULT: {Path(__file__).resolve()} main: 已有convert任务运行；锁={lockpath}');return 2
        return session(config,base,args.test_output)

def session(config,base,test_output):
    while True:
        try:
            tasks=request(config,'tasks')['items']
            print('机器人任务根目录：'+str(config['robot_raw_root']))
            for i,t in enumerate(tasks,1):print(f"{i}. {t['name']}  {t['path']}")
            if not tasks:raise ValueError('任务根目录为空')
            task=choose(tasks)
            if task is None:return 0
            episodes=sorted(request(config,'episodes',task['path'])['items'],key=lambda e:e['name'])
            if not episodes:raise ValueError('所选任务没有可处理MCAP episode')
            break
        except (RuntimeError,ValueError,OSError) as exc:
            print(f'WARNING: {Path(__file__).resolve()} session: {exc}')
            choice=input('r重新选择，输入机器人根目录，或q退出：').strip()
            if choice.lower()=='q':return 0
            if choice.startswith('/'):config['robot_raw_root']=choice
    print(f"任务：{task['path']}；共{len(episodes)}条")
    for i,e in enumerate(episodes,1):print(f"{i}. {e['name']}")
    op=select_mode('操作',{'1':'仅检查已转换的单episode输出','2':'直接转换为LeRobot'})
    if op is None:return 0
    mode=select_mode('处理范围',{'1':'整个目录（逐条提交，已完成且未变化的自动跳过）','2':'仅未转换的数据','3':'单条数据'})
    if mode is None:return 0
    mapping=ROOT/'configs/processing/action78_posttrain.json'
    mapping_hash=hashlib.sha256(mapping.read_bytes()).hexdigest()
    previous=delivered(base,config,task)
    if mode=='2':
        selected=episodes
        print('逐条核对来源与规则指纹；已完成的自动跳过，未完成的从检查点续跑。')
    elif mode=='3':
        print('选择episode：')
        item=choose(episodes)
        if item is None:return 0
        selected=[item]
    else:selected=episodes
    if not selected:print('没有未转换的数据。');return 0
    print(f"当前任务 {task['name']}；选中{len(selected)}条；模式={'检查已转换输出' if op=='1' else '直接转换'}")
    params=['--task',task['name'],'--root',config['robot_raw_root'],'--mapping',str(mapping)]
    for e in selected:params+=['--selected-episode',e['name']]
    if op=='1':params+=['--check-only']
    if test_output:params+=['--test-output',test_output]
    result={}
    code=run_pipeline(params,result,lock_held=True)
    report=result.get('conversion')
    if op=='2' and mode=='1' and report:
        replacement=replace_previous(base,config,task,selected,previous,report)
        exclusive_json(base/'conversions'/report['conversion_id']/'replacement.json',replacement)
        print('替换状态：'+replacement['status'])
        if replacement['status']=='kept_previous':print('本轮未完整覆盖所选episode，已有数据集保留。')
        if replacement.get('errors'):print(json.dumps(replacement['errors'],ensure_ascii=False));code=2
    if op=='2' and code==0 and report:
        from processing.clean_cli import auto_clean
        clean=auto_clean(base,task['name'])
        exclusive_json(base/'conversions'/report['conversion_id']/'auto_clean.json',clean)
        print(f"自动清理完成：删除 {clean['removed_frames']} 帧，保留 {clean['output_frames']} 帧；输出 {clean['output']}")
    return code

if __name__=='__main__':
    try:raise SystemExit(main())
    except (EOFError,KeyboardInterrupt):print('\n已退出。');raise SystemExit(130)
    except Exception as exc:
        print(f'FAULT: {Path(__file__).resolve()} main: {type(exc).__name__}: {exc}',file=sys.stderr);raise SystemExit(2)
