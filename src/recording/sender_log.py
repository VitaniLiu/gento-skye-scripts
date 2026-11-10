"""Task-scoped live sender evidence. No control calls or historical log replay."""
import json,os,threading,time,uuid
from pathlib import Path
from recording.protocol import locked,atomic_json

def find_active_sender(project):
    project=Path(project).resolve();candidates=[]
    script=project/'src/teleop/chassis/src/teleop/sender.py'
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            argv=(proc/'cmdline').read_bytes().split(b'\0')
            if os.fsencode(script) not in argv or b'--execute' not in argv:continue
            for fd in (proc/'fd').iterdir():
                try:
                    target=Path(os.readlink(fd))
                    if target.name.startswith('live-') and target.suffix=='.jsonl' and target.is_relative_to(project/'logs/teleop') and target.is_file():
                        pair=(int(proc.name),target.resolve())
                        if pair not in candidates:candidates.append(pair)
                except (OSError,ValueError):continue
        except (OSError,ValueError):continue
    if len(candidates)!=1:raise ValueError('未找到唯一活动底盘 sender（候选数 %d）；请先正常启动独立底盘遥操。'%len(candidates))
    return candidates[0]

class TaskSenderLog:
    def __init__(self,project,task):
        if not task or task.startswith(('.','_')) or task in ('.','..') or any(c in task for c in ('/','\\','\0')):raise ValueError('invalid task name')
        self.project=Path(project).resolve();self.folder=self.project/'logs/teleop/sessions'/task
        if self.folder.resolve()!=self.folder:raise ValueError('sender task directory symlink forbidden')
        self.folder.mkdir(parents=True,exist_ok=True)
        self.lease=locked(self.folder/'.sender-task.lock');self.lease.__enter__()
        try:
            marker=self.folder/'.sender-task.json'
            if not marker.exists():
                if any(p.name!='.sender-task.lock' for p in self.folder.iterdir()):raise ValueError('existing directory is not a task sender directory')
                atomic_json(marker,dict(task_name=task,schema_version='1.0'))
            elif json.loads(marker.read_text()).get('task_name')!=task:raise ValueError('task directory identity mismatch')
        except BaseException:self.lease.__exit__(None,None,None);raise
        self.path=None;self.thread=None;self.stop_event=threading.Event();self.error=None;self.rows=0;self.task=task;self.cutoff=None;self.source_lease=None
    def start(self,source=None):
        if self.thread:return self.path
        pid,source=find_active_sender(self.project) if source is None else (None,Path(source).resolve(strict=True))
        if not source.is_relative_to(self.project/'logs') or not source.is_file():raise ValueError('sender source must be an existing project log')
        self.source_lease=locked(source.with_suffix('.task.lock'));self.source_lease.__enter__()
        self.source=source;self.input=source.open('rb');self.input.seek(0,2);stat=os.fstat(self.input.fileno());self.identity=(stat.st_dev,stat.st_ino)
        self.path=self.folder/('sender-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8]+'.jsonl')
        try:self.output=self.path.open('xb')
        except BaseException:self.input.close();raise
        self.meta=dict(task_name=self.task,source_path=str(source),source_pid=pid,source_device=stat.st_dev,source_inode=stat.st_ino,source_start_byte=self.input.tell(),started_host_ns=time.time_ns(),status='active',log_path=str(self.path))
        atomic_json(self.path.with_suffix('.source.json'),self.meta)
        def follow():
            partial=b''
            try:
                while True:
                    st=source.stat()
                    if (st.st_dev,st.st_ino)!=self.identity or st.st_size<self.input.tell():raise ValueError('sender source rotated/truncated')
                    if st.st_size-self.input.tell()>8*1024*1024:raise ValueError('task sender log backlog exceeded')
                    remaining=self.cutoff-self.input.tell() if self.cutoff is not None else 1048577
                    if remaining<=0:
                        if partial:raise ValueError('incomplete sender line on task exit')
                        break
                    chunk=self.input.readline(min(1048577,remaining))
                    if not chunk:
                        if self.stop_event.is_set():
                            if partial:raise ValueError('incomplete sender line on task exit')
                            break
                        if pid is not None and not Path('/proc',str(pid)).exists():raise ValueError('sender process exited')
                        self.stop_event.wait(.01);continue
                    partial+=chunk
                    if len(partial)>1048576:raise ValueError('sender event exceeds size limit')
                    if not partial.endswith(b'\n'):continue
                    json.loads(partial);self.output.write(partial);self.output.flush();self.rows+=1;partial=b''
            except Exception as exc:self.error=str(exc)
            finally:
                self.meta.update(status='error' if self.error else 'closed',error=self.error,rows=self.rows,source_end_byte=self.input.tell(),finished_host_ns=time.time_ns())
                self.input.close();self.output.close()
                try:atomic_json(self.path.with_suffix('.source.json'),self.meta)
                except Exception as exc:self.error=self.error or str(exc)
        self.thread=threading.Thread(target=follow,daemon=True);self.thread.start();return self.path
    def healthy(self):
        if self.error:raise RuntimeError('任务sender日志异常：'+self.error)
    def close(self):
        if self.thread:
            try:self.cutoff=self.source.stat().st_size
            except OSError:pass
        self.stop_event.set()
        if self.thread:self.thread.join(timeout=3)
        if self.thread and self.thread.is_alive():self.error='sender log drain timeout'
        if not self.thread or not self.thread.is_alive():
            for name in ('input','output'):
                stream=getattr(self,name,None)
                if stream and not stream.closed:stream.close()
            if self.source_lease:self.source_lease.__exit__(None,None,None)
        self.lease.__exit__(None,None,None)
