"""Interactive static motion candidate cleaning. Source datasets are read-only."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,fcntl,json,shutil
import numpy as np
from processing.runtime import ROOT,new_id
from processing.main import choose
from processing.task_v21 import load_existing,write_prepared,verify_dataset,digest,dump
from processing.static_plan import make_plan,check_plan_source,write_visualization,print_terminal_preview
from processing.static_viewer import generate_media

def create_preview(base,task,selected=None):
    source=Path(base)/"lerobot"/task
    plan=make_plan(source,selected)
    folder=Path(base)/"conversions"/new_id("conversion")
    folder.mkdir(parents=True,exist_ok=False)
    dump(folder/"clean_plan.json",plan)
    generate_media(folder/"static_frames.html",plan)
    check_plan_source(source,plan)
    dump(folder/"clean_plan.json",plan)
    return plan,folder

def clean_task(base,task,selected=None,plan=None,reportdir=None):
    from processing.clean_batches import clean_batched
    return clean_batched(base,task,selected,plan,reportdir)

def auto_clean(base,task):
    plan=make_plan(Path(base)/'lerobot'/task,trust_manifest=True)
    plan.update(review_status='auto_approved',approval='automatic_after_convert')
    folder=Path(base)/'conversions'/new_id('conversion');folder.mkdir(parents=True)
    dump(folder/'clean_plan.json',plan)
    return clean_task(base,task,None,plan,folder)


def session(base,input_fn=input):
    base=Path(base)
    tasks=[{"name":p.name,"path":str(p)} for p in sorted((base/"lerobot").iterdir())
           if p.is_dir() and not p.is_symlink() and (p/"meta/info.json").is_file()]
    if not tasks:raise ValueError("No LeRobot task datasets")
    print("选择清理任务（原始 lerobot 不变；输出到 lerobot_clean）：")
    for i,t in enumerate(tasks,1):print(f"{i}. {t['name']}")
    task=choose(tasks,input_fn)
    if task is None:return None
    info=json.loads((Path(task["path"])/"meta/info.json").read_text())
    print("1. 清理该任务的全部 episode\n2. 选择单条 episode")
    while True:
        mode=input_fn("请选择（q退出）：").strip()
        if mode=="q":return None
        if mode in ("1","2"):break
        print("输入无效")
    selected=None
    if mode=="2":
        eps=[{"name":f"episode_{i:06d}","index":i} for i in range(info["total_episodes"])]
        for i,e in enumerate(eps,1):print(f"{i}. {e['name']}")
        ep=choose(eps,input_fn)
        if ep is None:return None
        selected=[ep["index"]]
    print("正在扫描静止区间并生成视频和曲线预览……",flush=True)
    plan,folder=create_preview(base,task["name"],selected)
    print_terminal_preview(plan)
    print("详细 HTML（可选）："+str(folder/"static_frames.html"),flush=True)
    print(f"拟删除 {plan['removed_frames']} 帧，保留 {plan['output_frames']} 帧；输出为待审核候选，不拆 episode。",flush=True)
    while True:
        decision=input_fn("输入 y 生成独立候选数据集（不代表批准）；n/q 仅保留预览：").strip().lower()
        if decision in ("n","q"):
            print("未清洗；预览已保存。",flush=True)
            return {"status":"preview_only","visualization":str(folder/"static_frames.html")}
        if decision=="y":break
        print("请输入 y、n 或 q。")
    print("已确认 y，开始生成候选数据；无需重复输入。",flush=True)
    report=clean_task(base,task["name"],selected,plan,folder)
    print(f"清理完成：输入 {report['input_frames']} 帧，删除 {report['removed_frames']} 帧，保留 {report['output_frames']} 帧。",flush=True)
    print("状态："+report["status"]+"；输出："+report["output"],flush=True)
    return report

def main(argv=None):
    p=argparse.ArgumentParser(description="交互生成静止删帧审核候选，输出lerobot_clean")
    p.add_argument("--test-output",help="测试存储根目录，必须在项目tests下")
    p.add_argument("--preview",metavar="TASK",help="只生成网页审核包，不写清洗数据")
    p.add_argument("--list",action="store_true",help="列出可选任务")
    p.add_argument("--resume-report",help="使用已有审核目录恢复已确认的分批清洗；不重新生成视频")
    args=p.parse_args(argv)
    if args.test_output:
        base=Path(args.test_output).resolve()
        if not base.is_relative_to((ROOT/"tests").resolve()):p.error("测试输出必须在项目tests内")
        lockpath=base/"clean.lock"
    else:
        from processing.runtime import data_root,load_config
        base=data_root(load_config("processing"));lockpath=ROOT/"logs/processing/convert.lock"
    if args.list:
        print(json.dumps([x.name for x in sorted((base/"lerobot").iterdir()) if x.is_dir() and (x/"meta/info.json").exists()]))
        return 0
    if args.preview:
        if Path(args.preview).name!=args.preview or args.preview in (".",".."):p.error("Invalid task")
        plan,folder=create_preview(base,args.preview)
        print_terminal_preview(plan)
        print("REVIEW_DIR="+str(folder),flush=True)
        return 0
    with lockpath.open("a") as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:print("已有 convert/clean 运行，请稍后重试。");return 2
        if args.resume_report:
            folder=Path(args.resume_report).resolve()
            if folder.parent!=(base/'conversions').resolve():raise ValueError('Unsafe report directory')
            plan=json.loads((folder/'clean_plan.json').read_text())
            report=clean_task(base,plan['task'],plan['selected_episodes'],plan,folder)
        else:
            report=session(base)
    return 2 if report and report["status"] not in ("verified","preview_only") else 0

if __name__=="__main__":
    try:raise SystemExit(main())
    except (EOFError,KeyboardInterrupt):print("\n已退出。");raise SystemExit(130)
    except Exception as exc:print(f"FAULT: {Path(__file__).resolve()} main: {type(exc).__name__}: {exc}");raise SystemExit(2)
