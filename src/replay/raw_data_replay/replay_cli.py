#!/usr/bin/env python3
"""Interactive host entry point for robot-local raw recordings."""
import json,os,shlex,subprocess,sys
ROBOT='user@6.6.7.100'
ROOT='/home/user/gento-skye_raw_data'
SCRIPTS='/home/user/gento-skye_scripts/src/replay/raw_data_replay'

def choose(title,items,label):
    if not items:raise RuntimeError('没有可回放的录制文件')
    print('\n'+title)
    for i,item in enumerate(items,1):print(f'  {i}. {label(item)}')
    while True:
        value=input('输入序号（q 退出）：').strip()
        if value.lower()=='q':raise SystemExit(0)
        if value.isdigit() and 1<=int(value)<=len(items):return items[int(value)-1]
        print('请输入列表中的序号。')

def main():
    code="""import json,pathlib
root=pathlib.Path('/home/user/gento-skye_raw_data').resolve();out={}
for task in sorted(root.iterdir()):
 if not task.is_dir() or task.is_symlink() or task.name.startswith('_'):continue
 episodes=[]
 for ep in sorted(task.glob('episode[0-9]*')):
  if ep.is_symlink() or not ep.is_dir():continue
  try:m=json.loads((ep/'recording.json').read_text())
  except (OSError,ValueError):continue
  if m.get('file_status')=='closed' and (ep/'domain_24/metadata.yaml').is_file():episodes.append({'name':ep.name,'path':str(ep),'quality':m.get('quality_status','unknown')})
 if episodes:out[task.name]=episodes
print(json.dumps(out))
"""
    result=subprocess.run(['ssh',ROBOT,'python3 -B -c '+shlex.quote(code)],text=True,capture_output=True)
    if result.returncode:raise RuntimeError(result.stderr.strip() or '无法读取机器人录制目录')
    data=json.loads(result.stdout)
    if '--list' in sys.argv:
        print(json.dumps(data,ensure_ascii=False,indent=2));return
    task=choose('选择任务目录',list(data),str)
    ep=choose('选择录制文件',data[task],lambda e:e['name']+'  ['+e['quality']+']')
    full='--full' in sys.argv
    script=SCRIPTS+('/body_replay.py' if full else '/native_replay.py')
    flags=' --with-grippers --with-chassis --execute' if full else ''
    mode='完整回放' if full else '原生回放'
    print(f"\n{mode}：{task}/{ep['name']}（Ctrl+C 停止）",flush=True)
    command='source /etc/apex/apex_ros_env.sh; exec python3 -B '+shlex.quote(script)+' '+shlex.quote(ep['path'])+flags
    os.execvp('ssh',['ssh','-t',ROBOT,'bash -lc '+shlex.quote(command)])
if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:print('\n已取消。');sys.exit(130)
    except (RuntimeError,ValueError,EOFError) as e:print(str(e),file=sys.stderr);sys.exit(1)
