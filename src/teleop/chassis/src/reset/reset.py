from pathlib import Path as _ProjectPath
import sys as _project_sys
_project_sys.path.insert(0,str(_ProjectPath(__file__).resolve().parents[1]/"common"))
from project_paths import configure_imports, output_directory
configure_imports()
#!/usr/bin/env python3
"""Small return-to-reference entry. Default is OFFLINE and never opens SSH."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import time
import uuid

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT))
from logic import (Sample, ReturnController, StraightReturnController, Lease, REFERENCE_FILE, REFERENCE_SHA, POSE_GID,
                   STATE_GID, COMMAND_GID, finite, wrap,
                   validate_reference, validate_verification, MeasurementFreshness, SourceLatch,
                   controller_for, position_error, target_problem)
from controller import Command
from live_io import JsonChannel, ChannelError
from live_protocol import Feedback, FeedbackClock
import robot_endpoint as base

REFERENCE=ROOT/"reference"/REFERENCE_FILE
VERIFICATION=ROOT/"config"/"verification.json"

def reference():
    return validate_reference(REFERENCE.read_bytes())

def verification(control_mode="straight"):
    record = json.loads(VERIFICATION.read_text())
    validate_verification(record,time.time(),straight=control_mode=="straight")
    return record

def target_of(ref):
    p=ref["pose"]
    return p["x_m"],p["y_m"],p["yaw_rad"]

def decode_odometry(msg,at,seq,fb):
    """This vendor uses raw yaw in orientation.w; reject other encodings."""
    p=msg.pose.pose.position;q=msg.pose.pose.orientation
    if (msg.header.frame_id!="odom" or msg.child_frame_id!="base_link"
        or not all(finite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w))
        or any(abs(v)>1e-8 for v in (q.x,q.y,q.z))):
        raise ValueError("pose_frame_or_encoding_changed")
    return Sample(p.x,p.y,wrap(q.w),fb.vx,fb.vy,fb.wz,
                  min(at,fb.received_at),seq,max(at,fb.received_at))


class CompletionRuntime(base.EndpointRuntime):
    """Keep zero-first stopping, but never certify a goal lost during stop ack."""
    def __init__(self,*args,final_check,continuous=False,**kwargs):
        super().__init__(*args,**kwargs)
        self.final_check=final_check
        self.continuous=continuous
    def emit(self,row,now):
        if (self.continuous and row.get("type")=="stopped" and
                row.get("confirmed") is True and self.stop_reason=="reset_complete"):
            row=dict(row,type="reset_completed",session_continues=True)
        return super().emit(row,now)

    def stopping(self,now):
        # Continuous A-return still owns the session during zero-speed ack.
        # Keep challenge/reply input alive for the independent waist controller.
        if self.continuous and self.stop_reason=="reset_complete":
            self.emit(self.guard.poll(now),now)
        if self.stop_reason=="reset_complete":
            self.apply(Command(reason="stopping"),now,None)
            try:problem=self.final_check(now)
            except Exception as exc:problem="final_inspection_failed: "+str(exc)
            if problem:self.stop_reason="completion_invalid: "+problem
        super().stopping(now)
        if (self.continuous and self.finished and self.exit_code==0 and
                self.stop_reason=="reset_complete"):
            self.guard.restart_after_return(now)
            self.finished=False;self.exit_code=None
            self.stop_at=None;self.stop_reason=None;self.stop_verifier=None


def measurement_stamp(message):
    stamp=message.header.stamp
    if type(stamp.sec) is not int or type(stamp.nanosec) is not int or not 0<=stamp.nanosec<1000000000:
        return -1
    return stamp.sec*1000000000+stamp.nanosec

class ReturnGuard:
    def __init__(self,session,target,get_sample,control_mode="straight"):
        self.controller_type=controller_for(control_mode)
        self.lease=Lease(session);self.target=target;self.get_sample=get_sample
        self.control=None;self.wait_started=None;self.finished_reason=None
    def poll(self,now):
        return self.lease.poll(now)
    def accept(self,msg,now):
        return self.lease.accept(msg,now)
    def step(self,now,feedback,armed=True):
        if self.finished_reason:return Command(reason=self.finished_reason)
        if self.wait_started is None:self.wait_started=now
        if not isinstance(feedback,Feedback) or not feedback.fresh(now) or not feedback.manual or not armed:
            self.finished_reason="manual_or_speed_feedback_lost"
        elif not self.lease.fresh(now):
            if self.control is not None:self.finished_reason="lease_lost"
            elif now-self.wait_started>2:self.finished_reason="lease_start_timeout"
            else:return Command(reason="waiting_for_host")
        if self.finished_reason:return Command(reason=self.finished_reason)
        if self.control is None:self.control=self.controller_type(self.target,now)
        out=self.control.step(self.get_sample(),now)
        self.finished_reason=self.control.finished_reason
        return Command(out.vx,0.,out.wz,out.reason)

def exact_source(node,topic,gid):
    rows=node.get_publishers_info_by_topic(topic)
    return len(rows)==1 and bytes(rows[0].endpoint_gid).hex()==gid

class ReturnSessionIdentity(base.SessionIdentity):
    """Pin current odometry for session-relative retrace, never historic returns."""
    def __init__(self,poses,states,commands,**kwargs):
        super().__init__(states,commands,**kwargs)
        self.poses=poses;self.pose_gid=None
        self.pose_pending=None;self.pose_since=None

    def inspect(self):
        if self.fault:return self.fault
        problem=super().inspect()
        rows=list(self.poses())
        gid=base.endpoint_gid(rows[0],"nav_msgs/msg/Odometry") if len(rows)==1 else None
        issue=None
        if gid is None:issue="pose_publisher_missing_or_ambiguous"
        elif self.pose_gid is not None and gid!=self.pose_gid:issue="pose_publisher_changed"
        if issue:
            if self.pose_gid is not None:self.fault=issue
            self.pose_pending=None;self.pose_since=None
            return issue
        if problem:return problem
        if self.pose_gid is None:
            now=self.clock()
            if gid!=self.pose_pending:self.pose_pending=gid;self.pose_since=now
            if not 0<=now-self.pose_since or now-self.pose_since<self.discovery_s:
                return "pose_source_discovery_pending"
            self.pose_gid=gid
        return None

    def evidence(self):
        return dict(super().evidence(),pose_gid=self.pose_gid,
                    reference_policy="current_session_origin")


def waist_intent_buttons(frame,fresh,button_index,sequence=0,request_age_s=1e9):
    """Carry only a waist request; never synthesize the chassis A button."""
    if type(sequence) is not int or not 0<=sequence<2**31:
        raise ValueError('invalid keyboard waist reset sequence')
    if type(request_age_s) not in (int,float) or not math.isfinite(request_age_s) or request_age_s<0:
        raise ValueError('invalid keyboard waist reset age')
    return [int(fresh and bool(frame.buttons[4])),
            int(fresh and bool(frame.buttons[button_index])),
            int(fresh and frame.axes[6]==0 and frame.axes[7]==0),
            sequence,int(bool(fresh)),int(fresh and sequence>0 and request_age_s<.15)]


def robot_main(mode,session,control_mode="straight",gamepad=False,button_index=0):
    # Check all local gates before ROS imports or a command publisher.
    ref=reference()
    if gamepad:
        control_mode = "retrace"
    else:
        controller_for(control_mode)
    if mode=="execute" and not gamepad:verification(control_mode)
    import fcntl
    import robot_endpoint as base
    from live_protocol import ownership_problem
    lock=None
    try:
        if mode=="execute":
            lock=os.open(ROOT/".sender.lock",os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        from geometry_msgs.msg import TwistStamped
        from move.msg import State
        from nav_msgs.msg import Odometry
        rclpy.init(args=["--ros-args","--disable-rosout-logs","--disable-external-lib-logs"],
                    domain_id=int(os.environ.get("ROS_DOMAIN_ID","0")),signal_handler_options=SignalHandlerOptions.NO)
        node=rclpy.create_node("tianjidipan_return_"+session[:12],
                              enable_rosout=False,start_parameter_services=False)
        raw_channel=JsonChannel(sys.stdin.fileno(),sys.stdout.fileno())
        clock=FeedbackClock();pose_clock=FeedbackClock()
        pose_measure=MeasurementFreshness();speed_measure=MeasurementFreshness();provenance=SourceLatch()
        box=dict(feedback=None,pose=None,pose_at=None,pose_seq=0,speed_seq=0,
                 fault=None,publisher=None)
        monitor=base.CommandMonitor()
        identity=(ReturnSessionIdentity(
            lambda:node.get_publishers_info_by_topic("/controller/odom"),
            lambda:node.get_publishers_info_by_topic("/move/State"),
            lambda:node.get_publishers_info_by_topic("/move/ManualMoveCmd")) if gamepad else None)
        dormant_base=base.DormantBaseGuard()

        def valid_source(topic,historical_gid):
            if identity is None:return exact_source(node,topic,historical_gid)
            if identity.inspect() is not None:return False
            gid=identity.pose_gid if topic=="/controller/odom" else identity.state_gid
            return exact_source(node,topic,gid)
        runtime=None
        parking=(base.create_parking_release(node,lambda row:runtime.emit(row,time.monotonic()))
                 if gamepad and mode=="execute" else None)


        def sources():
            clock_fault=("pose_dds_clock_fault" if pose_clock.faulted else
                         "speed_dds_clock_fault" if clock.faulted else
                         pose_measure.fault or speed_measure.fault)
            return provenance.observe(valid_source("/controller/odom",POSE_GID),
                valid_source("/move/State",STATE_GID),
                runtime is not None and runtime.ready,clock_fault)
        def state_cb(msg,info):
            sources()
            if not valid_source("/move/State",STATE_GID):
                box["feedback"]=None
                return
            box["speed_seq"]+=1
            fb=base.state_feedback(msg,info,box["speed_seq"],
                                   time.time_ns(),time.monotonic(),clock)
            if fb is None:
                box["feedback"]=None;sources();return
            at=speed_measure.sample(measurement_stamp(msg),fb.received_at)
            sources()
            if at is None:
                box["feedback"]=None;return
            box["feedback"]=Feedback(at,fb.seq,fb.manual,fb.vx,fb.vy,fb.wz)
            if parking is not None:
                parking.observe(getattr(msg,"is_parking_mode",None),at,fb.seq)
        def pose_cb(msg,info):
            sources()
            if not valid_source("/controller/odom",POSE_GID):
                box["pose"]=None
                return
            lookup=info.get if isinstance(info,dict) else lambda k:getattr(info,k,None)
            dds_at=pose_clock.sample(lookup("source_timestamp"),lookup("received_timestamp"),
                                 time.time_ns(),time.monotonic(),lookup("publication_sequence_number"))
            sources()
            if dds_at is None:
                if pose_clock.faulted:box["pose"]=None
                return
            at=pose_measure.sample(measurement_stamp(msg),dds_at)
            sources()
            if at is None:
                if pose_measure.fault:box["pose"]=None
                return
            try:
                decode_odometry(msg,at,1,Feedback(at,1,True,0,0,0))
                box["pose_seq"]+=1
                box["pose"]=msg;box["pose_at"]=at
            except (AttributeError,ValueError,TypeError) as exc:
                box["pose"]=None;box["fault"]=str(exc)
        def feedback():
            sources()
            if not valid_source("/move/State",STATE_GID):
                box["feedback"]=None
            return box["feedback"]
        def sample_now():
            fb=feedback();at=box["pose_at"]
            if (not valid_source("/controller/odom",POSE_GID) or box["pose"] is None
                or fb is None or not fb.fresh(time.monotonic()) or at is None
                or not 0<=time.monotonic()-at<.2):
                return None
            seq=box["pose_seq"]*1000000000+fb.seq
            return decode_odometry(box["pose"],at,seq,fb)
        def active_target():
            if gamepad and runtime is not None:
                path=getattr(runtime.guard,"path",None)
                if path is not None and path.origin is not None:
                    return path.origin.x,path.origin.y,path.origin.yaw
            return target_of(ref)
        def ownership():
            problem=sources()
            if problem:return problem
            if box["fault"]:return box["fault"]
            if monitor.problem():return monitor.problem()
            if sample_now() is None:return "pose_missing_or_stale"
            if runtime is not None and not runtime.ready:
                fb=feedback()
                if fb is None or abs(fb.vx)>.01 or abs(fb.vy)>.01:return "base_not_stationary"
                s=sample_now()
                if s is None:return "pose_missing_or_stale"
                if not gamepad and math.hypot(s.x-ref["pose"]["x_m"],s.y-ref["pose"]["y_m"])>1:
                    return "target_out_of_range"
            rows=node.get_publishers_info_by_topic("/move/ManualMoveCmd")
            if identity is not None:
                problem=identity.inspect() or dormant_base.inspect(rows)
                if problem:return problem
                names=base.publisher_names(rows,identity.command_gid)
                if dormant_base.gid:names=[name for name in names if name!="base_teleop"]
                if not identity.announced:
                    raw_channel.send(dict(type="source_identified",session=session,**identity.evidence()))
                    identity.announced=True
            else:
                names=[e.node_name for e in rows if bytes(e.endpoint_gid).hex()!=COMMAND_GID]
            legacy_root=Path("/home/user/gamepad_control")
            legacy=base.legacy_snapshot(legacy_root,time.monotonic(),time.time())
            if legacy_root.exists() and "host_gamepad_control" not in names:names.append("host_gamepad_control")
            return ownership_problem(names,node.get_name(),legacy)

        def final_check(now):
            problem=ownership()
            if problem:return problem
            s=sample_now();fb=feedback()
            if s is None or fb is None or not fb.fresh(now) or not fb.manual:
                return "final_feedback_missing"
            issue=target_problem(s,active_target(),control_mode)
            if issue:return issue
            if abs(fb.vx)>.01 or abs(fb.vy)>.01 or abs(fb.wz)>.02:
                return "final_base_not_stationary"
            return None

        def enable():
            if not gamepad:verification(control_mode)
            box["publisher"]=node.create_publisher(TwistStamped,"/move/ManualMoveCmd",
                                  QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
        def publish(command):
            if command.vy!=0 or (control_mode=="straight" and command.wz!=0 and (not gamepad or getattr(runtime.guard,"control",None) is not None)):
                raise ValueError("invalid_command_for_control_mode")
            msg=TwistStamped();stamp=node.get_clock().now()
            msg.header.stamp=stamp.to_msg();msg.header.frame_id="base_link"
            msg.twist.linear.x=command.vx;msg.twist.angular.z=command.wz
            monitor.expect(msg,time.monotonic())
            box["publisher"].publish(msg)
            return stamp.nanoseconds
        class Telemetry:
            @property
            def eof(self):return raw_channel.eof
            def receive(self):return raw_channel.receive()
            def send(self,row):
                row=dict(row,reference_sha256=REFERENCE_SHA,control_mode=control_mode)
                if row.get("type")=="ready" and gamepad:
                    row["reset_button"]="A"
                    row["reset_available"]=runtime.guard.seed_manual_origin(time.monotonic())
                    row["capture_scoped"]=True
                if row.get("type") in ("target","checked","stopped","reset_completed"):
                    try:s=sample_now()
                    except Exception:s=None
                    if s is not None:
                        row["pose"]=asdict(s)
                        if not gamepad:
                            target=active_target()
                            row["position_error"]=position_error(s,target)
                            row["error_m"]=math.hypot(s.x-target[0],s.y-target[1])
                            row["error_yaw_rad"]=wrap(target[2]-s.yaw)
                    if runtime is not None:
                        control=getattr(runtime.guard,"control",None)
                        row["phase"]=control.phase if control else "PREFLIGHT"
                        if gamepad:
                            row["capture_id"]=getattr(runtime.guard,"capture_id",None)
                            row["capture_state"]=getattr(runtime.guard,"capture_state","idle")
                raw_channel.send(row)
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(State,"/move/State",state_cb,qos)
        node.create_subscription(Odometry,"/controller/odom",pose_cb,qos)
        node.create_subscription(TwistStamped,"/move/ManualMoveCmd",
            lambda m:monitor.observe(m,time.monotonic()),
            QoSProfile(depth=128,reliability=ReliabilityPolicy.RELIABLE))
        runtime=CompletionRuntime(mode,session,time.monotonic(),Telemetry(),
                                  enable,publish,feedback,ownership,final_check=final_check,continuous=gamepad)
        runtime.clock=time.monotonic
        if parking is not None:runtime.startup_prepare=parking.step
        if gamepad:
            from button import ButtonReturnGuard
            runtime.guard=ButtonReturnGuard(session,target_of(ref),sample_now,
                lambda:None,lambda row:runtime.emit(row,time.monotonic()),
                control_mode=control_mode,button_index=button_index,capture_scoped=True)
            if mode=="execute":
                from sensor_msgs.msg import Joy
                lift_intent=node.create_publisher(Joy,"/tj/control/gamesir_waist_intent",1)
                original_accept=runtime.guard.accept
                def accept_with_waist(message,now):
                    accepted=original_accept(message,now)
                    if accepted:
                        frame,issued,received=runtime.guard.manual.latest
                        fresh=frame.connected and 0<=frame.usb_age_s+now-issued<.15
                        intent=Joy();intent.header.stamp=node.get_clock().now().to_msg()
                        intent.axes=[float(-frame.axes[4])]
                        intent.buttons=waist_intent_buttons(frame,fresh,button_index,message.get("waist_reset_seq",0),
                                                           message.get("waist_reset_age_s",1e9)+max(0.,now-issued))
                        lift_intent.publish(intent)
                    return accepted
                runtime.guard.accept=accept_with_waist
        else:
            runtime.guard=ReturnGuard(session,target_of(ref),sample_now,control_mode)
        def interrupt(signum,_):runtime.request_stop("signal_"+str(signum),time.monotonic())
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,interrupt)
        def tick():
            now=time.monotonic()
            try:runtime.tick(now)
            except Exception as exc:
                runtime.request_stop("runtime_error: "+str(exc),now)
                if runtime.ready:
                    try:publish(Command(reason="exception_zero"))
                    except Exception:pass
                if not runtime.ready or now-runtime.stop_at>=2:
                    runtime.finished=True;runtime.exit_code=2
        node.create_timer(.02,tick)
        try:
            while rclpy.ok() and not runtime.finished:rclpy.spin_once(node,timeout_sec=.02)
        finally:
            if box["publisher"] is not None:
                try:publish(Command(reason="final_zero"))
                except Exception:pass
            node.destroy_node();rclpy.shutdown()
        return runtime.exit_code if runtime.exit_code is not None else 2
    finally:
        if lock is not None:os.close(lock)

def host_run(mode,control_mode="straight"):
    session=uuid.uuid4().hex
    logdir=output_directory("logs", ROOT)/"sessions"/datetime.now(TZ).strftime("%Y-%m-%d");logdir.mkdir(parents=True,exist_ok=True)
    stamp=datetime.now().astimezone().strftime("%Y%m%d-%H%M%S")+"-"+session[:8]
    args=["ssh","-T","-o","BatchMode=yes","-o","StrictHostKeyChecking=yes",
          "-o","ConnectTimeout=5","user@6.6.7.100"]
    shell=("source /etc/apex/apex_ros_env.sh >&2; "
           "export PYTHONDONTWRITEBYTECODE=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4; "
           "exec python3 -u -B /home/user/tianjidipan/src/reset/reset.py --robot-side "
           "--session "+session+" --"+mode+" --control-mode "+shlex.quote(control_mode))
    args+=["bash -c "+shlex.quote(shell)]
    stopped_error=None
    stopped=None;reason=None;checked=False;failed=False;interrupted=False
    stop_at=None;seq=0;ready=False;last_show=-1.;previous_phase=None
    started=time.monotonic();process=None;handlers={}
    with (logdir/("return-"+stamp+".jsonl")).open("x") as log, (logdir/("ssh-"+stamp+".txt")).open("xb") as errors:
        def record(row,show=True):
            row=dict(row,host_time=datetime.now().astimezone().isoformat())
            log.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+"\n");log.flush()
            if show:print(json.dumps(row,ensure_ascii=False,allow_nan=False),flush=True)
        def interrupt(_sig,_frame):
            nonlocal interrupted
            interrupted=True
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
            handlers[sig]=signal.signal(sig,interrupt)
        try:
            record(dict(type="starting",mode=mode,control_mode=control_mode,reference_sha256=REFERENCE_SHA,
                        log_path=str(logdir/("return-"+stamp+".jsonl"))))
            process=subprocess.Popen(args,stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                                     stderr=errors,start_new_session=True,bufsize=0)
            channel=JsonChannel(process.stdout.fileno(),process.stdin.fileno())
            while True:
                now=time.monotonic()
                if stop_at is None and (interrupted or now-started>(145 if mode=="execute" else 15)):
                    reason="operator_interrupt" if interrupted else "host_timeout"
                    stop_at=now
                    try:channel.send(dict(type="stop",session=session))
                    except ChannelError:pass
                for row in channel.receive():
                    if row.get("session")!=session:raise ChannelError("wrong_session")
                    kind=row.get("type")
                    if kind=="poll":
                        if stop_at is None:
                            if not re.fullmatch(r"[0-9a-f]{16}",str(row.get("nonce",""))):raise ChannelError("bad_nonce")
                            seq+=1
                            channel.send(dict(type="input",session=session,nonce=row["nonce"],seq=seq))
                        continue
                    if kind=="ready":ready=True
                    if kind=="checked":checked=row.get("ok") is True
                    if kind=="failed":failed=True;reason=row.get("reason")
                    if kind=="stopped":
                        stopped=row.get("confirmed") is True;reason=row.get("reason")
                        stopped_error=row.get("position_error")
                    show=kind!="target" or now-last_show>=1 or row.get("phase")!=previous_phase
                    record(row,show)
                    if show:last_show=now;previous_phase=row.get("phase")
                if checked or failed or stopped is not None or channel.eof:break
                if stop_at is not None and now-stop_at>=4:break
                time.sleep(.005)
        except (OSError,ValueError,RuntimeError) as exc:
            reason="host_error: "+str(exc)
            record(dict(type="error",reason=reason))
            if process is not None:
                try:channel.send(dict(type="stop",session=session))
                except (ChannelError,UnboundLocalError):pass
                deadline=time.monotonic()+3
                while time.monotonic()<deadline:
                    try:
                        for row in channel.receive():
                            if row.get("session")==session and row.get("type")=="stopped":
                                stopped=row.get("confirmed") is True
                                record(row)
                        if stopped is not None or channel.eof:break
                    except (ChannelError,UnboundLocalError):break
                    time.sleep(.01)
        finally:
            if process is not None:
                try:process.stdin.close()
                except OSError:pass
                try:process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try:process.wait(timeout=1)
                    except subprocess.TimeoutExpired:process.kill();process.wait(timeout=1)
                process.stdout.close()
            for sig,old in handlers.items():signal.signal(sig,old)
            completed=stopped is True and reason=="reset_complete" and not interrupted
            status=("CHECKED" if checked else "MOTION_NOT_STARTED" if not ready
                    else "STOP_CONFIRMED" if stopped is True else "STOP_UNCONFIRMED")
            record(dict(type="final",completed=completed,status=status,reason=reason,
                        stop_confirmed=stopped,control_mode=control_mode,position_error=stopped_error))
    return 0 if checked or completed else 130 if interrupted and stopped is True else 2

def main(argv=None):
    parser=argparse.ArgumentParser(description="Small chassis return; default OFFLINE preview.")
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument("--check",action="store_true",help="Read-only live check; requires manual deployment first")
    modes.add_argument("--execute",action="store_true",help="Real return; requires separately verified prerequisites")
    parser.add_argument("--control-mode",choices=("straight","pose"),default="straight",
                        help="Current test stage: straight only, or future full position and heading")
    parser.add_argument("--robot-side",action="store_true",help=argparse.SUPPRESS)
    parser.add_argument("--session",help=argparse.SUPPRESS)
    args=parser.parse_args(argv)
    try:
        ref=reference()
        if args.robot_side:
            if not re.fullmatch(r"[0-9a-f]{32}",str(args.session or "")):raise ValueError("invalid_session")
            if not (args.check or args.execute):raise ValueError("explicit_mode_required")
            return robot_main("execute" if args.execute else "check",args.session,args.control_mode)
        issue=None
        try:verification(args.control_mode)
        except (OSError,ValueError,TypeError,KeyError) as exc:issue=str(exc)
        if args.execute:
            if issue:raise ValueError("execute_locked: "+issue)
            return host_run("execute",args.control_mode)
        if args.check:return host_run("check",args.control_mode)
        print(json.dumps(dict(mode="offline_preview",reference_at=ref["captured_at"],
            target=dict(x_m=target_of(ref)[0],y_m=target_of(ref)[1],yaw_rad=target_of(ref)[2]),
            reference_sha256=REFERENCE_SHA,control_mode=args.control_mode,restore_heading=args.control_mode=="pose",
            robot_connected=False,publisher_enabled=False,
            execute_unlocked=issue is None,lock_reason=issue,
            limits=dict(linear_mps=.05,angular_radps=0 if args.control_mode=="straight" else .1,position_tolerance_m=.02)),
            ensure_ascii=False))
        return 0
    except (OSError,ValueError,TypeError,KeyError) as exc:
        print(json.dumps(dict(type="blocked",reason=str(exc),publisher_enabled=False),ensure_ascii=False))
        return 2

if __name__=="__main__":raise SystemExit(main())
