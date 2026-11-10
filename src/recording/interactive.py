"""Persistent keyboard UI; delegates acquisition and lifecycle to existing components."""
import contextlib,json,os,queue,select,subprocess,sys,termios,threading,time,tty
from pathlib import Path
from recording.sender_log import TaskSenderLog
from recording.chassis_origin import ChassisCapture

@contextlib.contextmanager
def keys():
    if not sys.stdin.isatty():raise ValueError('交互录制需要终端，请直接在终端运行。')
    fd=sys.stdin.fileno();saved=termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd,termios.TCSANOW)
        yield lambda timeout:os.read(fd,1).decode(errors='ignore') if select.select([fd],[],[],timeout)[0] else None
    finally:termios.tcsetattr(fd,termios.TCSADRAIN,saved)

class Session:
    def __init__(self,start,request_stop,operate,output=print):
        self.start=start;self.request_stop=request_stop;self.operate=operate;self.output=output
        self.state='idle';self.recording_id=None;self.closed=None;self.can_keep=False
    def key(self,key):
        key=key.lower()
        if key=='s' and self.state=='idle':
            self.recording_id=None;self.closed=None;self.can_keep=False;self.state='starting'
            try:self.start()
            except Exception:self.state='idle';raise
            self.output('正在预检并启动录制；再次按 s 可请求结束。')
        elif key=='s' and self.state in ('starting','recording'):
            self.state='stopping';self.request_stop();self.output('正在停止并检查文件，请等待……')
        elif key in ('y','n') and self.state=='review':
            if key=='y' and not self.can_keep:
                self.output('收尾或采集边界未确认，不能提交；可按 n 丢弃或退出后检查。');return
            try:result=self.operate('keep' if key=='y' else 'discard',self.recording_id)
            except Exception as exc:self.output('处理失败，数据仍保留待处理：'+str(exc));return
            self.output(('已保存：' if key=='y' else '已丢弃（保留在机器人丢弃区）：')+result['path'])
            if key=='y':self.output('目录名称：'+Path(result['path']).name)
            self.state='idle';self.recording_id=None;self.output('按 s 开始下一段；Ctrl+C 退出。')
    def event(self,message):
        kind=message.get('type')
        if kind=='countdown':
            self.output('录制倒计时：'+str(message['remaining']))
        elif kind=='ready':
            self.recording_id=message['recording_id']
            if self.state=='starting':self.state='recording'
            self.output('录制已开始：'+self.recording_id+('；已请求停止，正在收尾。' if self.state=='stopping' else '；按 s 结束。'))
        elif kind=='closed':
            self.closed=message;self.recording_id=message.get('recording_id',self.recording_id)
        elif kind in ('error','host_error','manifest_error'):
            self.output('录制报告：'+json.dumps(message,ensure_ascii=False))
    def finished(self,code,*,allow_keep=True):
        if self.recording_id:
            self.state='review'
            failures=(self.closed or {}).get('failures',[])
            self.can_keep=not code and self.closed is not None and not failures and allow_keep
            if self.can_keep:
                self.output('本段收尾完成，尚未分配正式编号。按 y 保存，按 n 丢弃；Ctrl+C 留待处理后退出。')
            else:
                self.output('本段异常、收尾或采集边界未确认，未提交为正式 episode。')
                self.output('本条留待处理；按 n 丢弃，或 Ctrl+C 退出后检查。')
        else:
            self.state='idle';self.output('采集未就绪。按 s 重试；Ctrl+C 退出。中断数据可用 --list-pending 检查。')

def run_session(args,cfg,command):
    def operate(op,rid=None,task=None):
        request=dict(cfg,op=op,recording_id=rid,task=task)
        result=subprocess.run(command(request),capture_output=True,text=True,timeout=30)
        if result.stderr:print(result.stderr,end='',file=sys.stderr)
        if result.returncode:raise RuntimeError(result.stdout.strip() or '机器人操作失败')
        return json.loads(result.stdout)
    task_log=None
    try:
        while True:
            task=(args.task or input('任务文件夹名称：')).strip()
            try:task_info=operate('prepare_task',task=task);break
            except Exception as exc:
                print('任务不可用：'+str(exc))
                if args.task:return 2
        print(('继续已有任务：' if task_info['existed'] else '已创建任务：')+task_info['path'])
        task_log=TaskSenderLog(cfg['project_root'],task)
        print('本任务 sender 日志目录：'+str(task_log.folder))
        event_log=args.event_log
        if event_log:
            path=Path(event_log).resolve(strict=True)
            project=Path(cfg['project_root'])
            if not path.is_file() or not any(path.is_relative_to(p) for p in (project/'logs',project/'src/teleop/chassis/logs')):
                raise ValueError('sender 日志必须是项目现有遥操日志文件')
            event_log=str(path)
        if event_log or not args.passive:
            try:print('已关联 sender 日志：'+str(task_log.start(event_log)))
            except ValueError as exc:print(str(exc)+' 按 s 时会重新检查。')
    except (KeyboardInterrupt,EOFError):
        if task_log:task_log.close()
        print('\n已退出。');return 0
    except Exception as exc:
        if task_log:task_log.close()
        print(str(exc));return 2
    child=[None];messages=queue.Queue();reader=[None];capture=[None]
    def start():
        if capture[0] is not None:
            if capture[0].cleanup_pending:capture[0].abort(force=True)
            elif capture[0].stop_requested and not capture[0].ended:capture[0].end()
            capture[0]=None
        task_log.healthy()
        selected_log=task_log.start(event_log) if event_log or not args.passive else None
        argv=[sys.executable,str(Path(__file__).with_name('main.py')),'--record','--task',task,'--seconds','3600','--control-stdin']
        if selected_log:
            capture[0]=ChassisCapture(task_log.source)
            origin=capture[0].begin()
            argv+=['--event-log',str(selected_log),'--chassis-origin',json.dumps(origin,allow_nan=False)]
        if args.passive:argv+=['--passive']
        if args.allow_incomplete:argv+=['--allow-incomplete']
        try:child[0]=subprocess.Popen(argv,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1,start_new_session=True)
        except BaseException:
            if capture[0]:capture[0].abort()
            raise
        def read():
            for line in child[0].stdout:
                try:messages.put(json.loads(line))
                except ValueError:messages.put(dict(type='diagnostic',text=line.rstrip()))
        reader[0]=threading.Thread(target=read,daemon=True);reader[0].start()
    def stop():
        if child[0] and child[0].poll() is None:
            try:
                if capture[0]:capture[0].end()
            finally:
                child[0].stdin.write('stop\n');child[0].stdin.flush()
    session=Session(start,stop,operate)
    reported_error=[None];exiting=[False]
    def pump():
        if task_log.error and reported_error[0]!=task_log.error:
            reported_error[0]=task_log.error;print('任务日志错误：'+task_log.error)
            if child[0] and child[0].poll() is None:
                try:child[0].stdin.write('abort:task sender log '+task_log.error+'\n');child[0].stdin.flush()
                except OSError:pass
        while True:
            try:message=messages.get_nowait()
            except queue.Empty:break
            if message.get("type")=="ready" and capture[0]:capture[0].recording_ready=True
            session.event(message)
            if message.get('type')=='host_log':print('本段日志：'+message['path'])
            elif message.get('type')=='diagnostic':print(message['text'])
        proc=child[0]
        if proc and proc.poll() is not None and not reader[0].is_alive():
            # Reader finished: process any final closed/error before review.
            if not messages.empty():return pump()
            pending=False
            if capture[0] and not capture[0].recording_ready:
                capture[0].cleanup_pending=True
                try:capture[0].abort(force=True)
                except Exception as exc:
                    pending=True;print('采集未开始，底盘起点清理未确认；下次开始前会重试：'+str(exc))
            elif capture[0] and not capture[0].ended:
                if capture[0].stop_requested:
                    pending=True;print('底盘结束边界状态未知；下次开始前会先重试结束，当前不能确认可复位。')
                else:
                    try:capture[0].abort()
                    except Exception as exc:
                        pending=True;capture[0].cleanup_pending=True;print('底盘采集边界未确认：'+str(exc))
            if not pending:capture[0]=None
            session.finished(proc.returncode,allow_keep=not exiting[0] and not task_log.error and not pending);proc.stdin.close();proc.stdout.close();child[0]=None
    try:
        with keys() as read_key:
            print('按 s 开始；再次按 s 结束；收尾完成后按 y 保存、n 丢弃。无需回车。Ctrl+C 退出。',flush=True)
            while True:
                pump();key=read_key(.05)
                if key=='\x03':raise KeyboardInterrupt()
                if key=='':raise EOFError()
                if key:
                    try:session.key(key)
                    except Exception as exc:print('操作失败：'+str(exc))
    except (KeyboardInterrupt,EOFError):
        exiting[0]=True
        print('\n正在退出，未确认的数据将留在机器人待处理区。')
        if child[0]:
            try:
                stop();deadline=time.monotonic()+30
                while child[0] and time.monotonic()<deadline:pump();time.sleep(.05)
            except (OSError,ValueError):pass
            if child[0]:
                child[0].terminate()
                try:child[0].wait(timeout=5)
                except subprocess.TimeoutExpired:child[0].kill();child[0].wait(timeout=5)
                print('收尾未确认完成；请用 --list-pending 检查机器人待处理数据。')
        if session.recording_id:print('待处理 recording_id：'+session.recording_id)
        return 0

    finally:
        if task_log:
            task_log.close()
            if task_log.error:print('任务sender日志收尾异常：'+task_log.error)
