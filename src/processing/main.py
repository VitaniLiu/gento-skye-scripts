"""Interactive task selection and independent raw-data validation."""
import argparse,json,sys,traceback
from pathlib import Path
from datetime import datetime,timezone
from processing.runtime import load_config,data_root,new_id,exclusive_json,log
from processing.remote import request
from processing.validation_review import review_report,describe_issue

def choose(items,input_fn=input):
    while True:
        choice=input_fn('任务编号或完整名称（q退出）：').strip()
        if choice.lower()=='q':return None
        named=[x for x in items if x['name']==choice]
        if named:return named[0]
        if choice.isdigit() and 1<=int(choice)<=len(items):return items[int(choice)-1]
        print('任务不存在，请重新选择。')

def main(argv=None,result=None,lock_held=False):
    parser=argparse.ArgumentParser(description='机器人任务只读检查与20Hz Action78转换（base_link、头部单路、逐维有效掩码）；不下载原始MCAP')
    parser.add_argument('--check-only',action='store_true',help='只检查，不进入转换')
    parser.add_argument('--mapping',help='版本化映射配置；默认action78_posttrain.json，旧版需显式指定')
    parser.add_argument('--root',help='本轮机器人任务根目录，不保存上次选择')
    parser.add_argument('--list-only',action='store_true')
    parser.add_argument('--episode',help='只读验证一个已知episode，跳过交互任务选择')
    parser.add_argument('--test-output',help='测试输出目录，必须位于项目tests内；测试日志也写此处')
    parser.add_argument('--task',help='本轮任务名称')
    parser.add_argument('--selected-episode',action='append',help='本轮选中episode名称，可重复')
    args=parser.parse_args(argv);config=load_config('processing')
    if args.root:config['robot_raw_root']=args.root
    project=Path(config['project_root']);test_dir=None
    if args.test_output:
        test_dir=Path(args.test_output).resolve()
        if not test_dir.is_relative_to((project/'tests').resolve()):parser.error('测试输出必须位于项目tests/')
        test_dir.mkdir(parents=True,exist_ok=True)
    def event(severity,reason,display=True,**context):
        if display:print(severity+': '+reason,flush=True)
        if test_dir:
            with (test_dir/'processing.jsonl').open('a') as f:f.write(json.dumps(dict(severity=severity,file=str(Path(__file__).resolve()),function='main',reason=reason,context=context),ensure_ascii=False)+'\n')
        else:log('processing',severity,reason,**context)
    while not config['robot_raw_root']:
        value=input('机器人任务根目录（q退出）：').strip()
        if value=='q':return 0
        if value.startswith('/'):config['robot_raw_root']=value
        else:print('请输入已核实的绝对路径。')
    while True:
        try:
            tasks=request(config,'tasks')['items']
            print('机器人：'+config['robot_ssh']+' 根目录：'+config['robot_raw_root'])
            for i,t in enumerate(tasks,1):print(f"{i}. {t['name']}  {t['path']}")
            if args.list_only:return 0
            if args.episode:task={'name':Path(args.episode).name,'path':args.episode};episodes=[task]
            else:
                if not tasks:raise ValueError('任务根目录为空')
                task=next((t for t in tasks if t['name']==args.task),None) if args.task else choose(tasks)
                if args.task and task is None:raise ValueError('指定任务不存在：'+args.task)
                if task is None:return 0
                episodes=request(config,'episodes',task['path'])['items']
                if args.selected_episode:
                    requested=set(args.selected_episode);available={e['name'] for e in episodes}
                    if not requested<=available:raise ValueError('所选episode不存在：'+repr(sorted(requested-available)))
                    episodes=sorted([e for e in episodes if e['name'] in requested],key=lambda e:e['name'])
                if not episodes:raise ValueError('任务无可处理MCAP episode，请重新选择或退出')
            break
        except (RuntimeError,ValueError,OSError) as exc:
            event('WARNING',str(exc))
            if args.episode or args.list_only or args.task:return 2
            choice=input('输入新根目录、r重选或q退出：').strip()
            if choice=='q':return 0
            if choice.startswith('/'):config['robot_raw_root']=choice
    print('任务路径：'+task['path']+'；episode数量：'+str(len(episodes)))
    for e in episodes:print('  '+e['name']+' '+e['path'])
    mapping_path=Path(args.mapping) if args.mapping else Path(config['project_root'])/'configs/processing/action78_posttrain.json'
    if json.loads(mapping_path.read_text()).get('schema')=='gento_skye_action78_base_rows_v2':
        from processing.incremental_pipeline import run
        def execute():
            report=run(config,task,episodes,event,test_dir,mapping_path,args.check_only)
            if result is not None:result['conversion']=report
            print('逐条处理完成；报告：'+str((test_dir if test_dir else data_root(config))/'conversions'/report['conversion_id']/'report.json'),flush=True)
            return 2 if report['status']=='partial_failure' else 0
        if lock_held:return execute()
        import fcntl
        lockpath=(test_dir if test_dir else project/'logs/processing')/'convert.lock'
        lockpath.parent.mkdir(parents=True,exist_ok=True)
        with lockpath.open('a') as lock:
            try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:
                print('已有 convert/clean 运行，请稍后重试。');return 2
            return execute()
    identifier=new_id('validation');destination=(test_dir if test_dir else data_root(config)/'validations')/identifier
    destination.mkdir(parents=True,exist_ok=False)
    summary=dict(schema_version='1.0',validation_id=identifier,source_robot=config['robot_ssh'],source_task=task,started_utc=datetime.now(timezone.utc).isoformat(),read_method='SSH stdin worker; native rosbag2_py on robot; no raw copy',configuration=config,episodes=[],conversion_status=('not_requested' if args.check_only else 'pending'),status='running')
    exclusive_json(destination/'started.json',summary)
    for index,episode in enumerate(episodes):
        event('INFO',f"任务 {task['name']} [{index+1}/{len(episodes)}] {episode['name']}")
        try:
            report=request(config,'inspect',episode['path'],progress=lambda s:event('INFO',s))['report']
            report=review_report(report,config,args.mapping)
            for issue in report['issues']:event(issue.get('severity','WARNING'),describe_issue(issue),display=(issue.get('severity')!='INFO' or issue.get('code') in ('CRC_NOT_VERIFIED','RECEIVE_TIME_ALIGNMENT_ONLY')),episode=episode['path'],issue=issue)
            count=report.get('review',{}).get('severity_counts',{})
            event('INFO',f"检查结果：WARNING={count.get('WARNING',0)}，INFO={count.get('INFO',0)}；完整观察与依据写入结构化报告。")
        except Exception as exc:
            report=dict(source_path=episode['path'],source_episode=episode['name'],status='read_failed',messages_read=None,issues=[dict(severity='WARNING',file=str(Path(__file__).resolve()),function='main',reason=str(exc),impact='actual read range unknown/incomplete',handling='skip episode; continue')])
            event('WARNING',str(exc),episode=episode['path'])
        report.update(source_robot=config['robot_ssh'],source_task_path=task['path'],source_episode_ordinal=index,lerobot_episode_index=None)
        name=f'episode_{index:06d}.json';exclusive_json(destination/name,report)
        summary['episodes'].append(dict(source_episode=episode['name'],source_path=episode['path'],report=name,status=report['status'],messages_read=report['messages_read'],lerobot_episode_index=None))
    summary['status']='inspection_finished_with_warnings' if any(x['status']=='inspected_with_warnings' for x in summary['episodes']) else 'inspection_finished'
    if any(x['status'] in ('read_failed','no_readable_mcap','partial_or_unreadable') for x in summary['episodes']):summary['status']='partial_or_unreadable'
    summary['finished_utc']=datetime.now(timezone.utc).isoformat()
    event('INFO','检查报告：'+str(destination))
    if not args.check_only:
        from processing.conversion import convert_task
        conversion=convert_task({**config,'validation_id':identifier},task,episodes,event,test_dir,args.mapping)
        if result is not None:result['conversion']=conversion
        summary['conversion_status']=conversion['status'];summary['conversion_id']=conversion['conversion_id'];summary['lerobot_id']=conversion['lerobot_id'];summary['output']=conversion['output']
        exclusive_json(destination/'summary.json',summary)
        if conversion['status'] in ('no_convertible_frames','export_failed'):return 2
    else:exclusive_json(destination/'summary.json',summary)
    return 2 if summary['status']=='partial_or_unreadable' else 0
if __name__=='__main__':raise SystemExit(main())
