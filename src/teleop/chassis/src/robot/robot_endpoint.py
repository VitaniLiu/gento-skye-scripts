"""Standalone endpoint; ROS is imported only inside explicitly invoked main()."""
from pathlib import Path as _ProjectPath
import sys as _project_sys
_project_sys.path.insert(0,str(_ProjectPath(__file__).resolve().parents[1]/"common"))
from project_paths import configure_imports
configure_imports()
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import signal
import sys
import time
from controller import Command
from acceleration_limit import AccelerationLimiter
from parking_release import create_parking_release
from live_io import JsonChannel,ChannelError
from live_protocol import Feedback,SessionGuard,StopVerifier,ownership_problem,STOP_TIMEOUT,MAX_SESSION_SECONDS,feedback_sample_time,FeedbackClock,number

TOPIC = "/move/ManualMoveCmd"
STATE_TOPIC = "/move/State"


def state_feedback(message,info,sequence,wall_ns,now,clock=None):
    """Jazzy supplies a dictionary; accept attribute metadata as well."""
    lookup=info.get if isinstance(info,dict) else lambda key:getattr(info,key,None)
    sampled=(clock.sample(lookup("source_timestamp"),lookup("received_timestamp"),wall_ns,now,
                          lookup("publication_sequence_number")) if clock is not None else
             feedback_sample_time(lookup("source_timestamp"),lookup("received_timestamp"),wall_ns,now))
    if sampled is None:return None
    try:
        return Feedback(sampled,sequence,message.is_soft_auto is False,
                        float(message.speed.linear.x),float(message.speed.linear.y),
                        float(message.speed.angular.z))
    except (AttributeError,TypeError,ValueError,OverflowError):return None


# DDS endpoint IDs belong to a process instance, not to a permanent device.
# The robot LAN is trusted, as in the original chassis client: this checks
# graph consistency and command conflicts, not cryptographic device identity.
STATE_TYPE = "move/msg/State"
COMMAND_TYPE = "geometry_msgs/msg/TwistStamped"
NATIVE_NAMES = {"_NODE_NAME_UNKNOWN_", "robot_server"}


def endpoint_gid(row, expected_type):
    try:
        raw=bytes(row.endpoint_gid)
        if len(raw)!=16 or raw==bytes(16) or row.topic_type!=expected_type:return None
        return raw.hex()
    except (AttributeError,TypeError,ValueError):return None


def publisher_names(endpoints,native_gid=None):
    names=[];native_count=0
    for endpoint in endpoints:
        if native_gid is not None and endpoint_gid(endpoint,COMMAND_TYPE)==native_gid:
            native_count+=1
        else:names.append(endpoint.node_name)
    if native_count>1:names.append("duplicate_native_command_endpoint")
    return names


def state_publisher_problem(endpoints,expected_gid=None):
    rows=list(endpoints)
    if len(rows)!=1:return "state_publisher_missing_or_ambiguous"
    gid=endpoint_gid(rows[0],STATE_TYPE)
    if gid is None:return "invalid_state_publisher"
    if expected_gid is not None and gid!=expected_gid:return "state_publisher_changed"
    return None


class SessionIdentity:
    """Learn a unique source at startup; never change it within a session.

    Dormant native publishers are learned separately from live command traffic.
    CommandMonitor still rejects every competing command, including zero.
    A restarted source is accepted only by a new operator-started session.
    """
    def __init__(self,states,commands,clock=time.monotonic,discovery_s=1.0):
        self.states,self.commands=states,commands
        self.clock,self.discovery_s=clock,discovery_s
        self.pending=None;self.pending_since=None
        self.state_gid=None;self.command_gid=None;self.fault=None;self.announced=False

    def inspect(self):
        if self.fault:return self.fault
        states=list(self.states());commands=list(self.commands())
        problem=state_publisher_problem(states,self.state_gid)
        candidates=[r for r in commands if r.node_name in NATIVE_NAMES]
        if problem is None:
            if len(candidates)!=1:problem="native_command_publisher_missing_or_ambiguous"
            else:
                command=endpoint_gid(candidates[0],COMMAND_TYPE)
                if command is None:problem="invalid_native_command_publisher"
                elif self.command_gid is not None and command!=self.command_gid:
                    problem="native_command_publisher_changed"
                elif self.state_gid is None:
                    now=self.clock()
                    pending=(endpoint_gid(states[0],STATE_TYPE),command,
                             tuple(sorted((r.node_name,bytes(r.endpoint_gid).hex(),r.topic_type) for r in commands)))
                    if pending!=self.pending:
                        self.pending=pending;self.pending_since=now
                    if not 0<=now-self.pending_since or now-self.pending_since<self.discovery_s:
                        return "source_discovery_pending"
                    self.state_gid=pending[0]
                    self.command_gid=command
        if problem is not None:
            if self.state_gid is not None:self.fault=problem
            else:self.pending=None;self.pending_since=None
        return problem

    def evidence(self):
        return dict(state_gid=self.state_gid,native_command_gid=self.command_gid,
                    identity_policy="unique_source_pinned_for_this_session")


class StateFeedbackGuard:
    """Verify state provenance on receipt AND every read, including stopping.

    A graph change discards cached samples and clock alignment. Command and
    legacy-controller errors cannot prevent zero publication or this check.
    """
    def __init__(self,endpoints,identity=None):
        self.endpoints=endpoints;self.identity=identity;self.clock=FeedbackClock();self.feedback=None;self.seq=0

    def verified(self):
        try:valid=(self.identity is not None and self.identity.inspect() is None
                   and state_publisher_problem(self.endpoints(),self.identity.state_gid) is None)
        except Exception:valid=False
        if not valid:self.feedback=None;self.clock=FeedbackClock()
        return valid

    def receive(self,message,info,wall_ns,now):
        if not self.verified():return
        self.seq+=1
        self.feedback=state_feedback(message,info,self.seq,wall_ns,now,clock=self.clock)

    def get(self):
        return self.feedback if self.verified() else None


class DormantBaseGuard:
    """Pin the known Apex no-base publisher; any actual traffic still faults."""
    def __init__(self):self.gid=None
    def inspect(self,rows):
        base=[r for r in rows if r.node_name=="base_teleop"]
        if len(base)>1:return "duplicate_apex_base_publishers"
        if not base:return "apex_base_publisher_disappeared" if self.gid else None
        gid=endpoint_gid(base[0],COMMAND_TYPE)
        if gid is None:return "invalid_apex_base_publisher"
        if self.gid is not None and self.gid!=gid:return "apex_base_publisher_changed"
        self.gid=gid
        return None

class CommandMonitor:
    """Reject other command traffic, including zero. Jazzy omits publisher GID.

    Match our own loopback by exact stamp/frame/all six velocity fields. This is
    a collision check on a trusted DDS network, not cryptographic arbitration.
    """
    def __init__(self):self.expected={};self.fault=None

    @staticmethod
    def signature(message):
        stamp=message.header.stamp;twist=message.twist
        values=tuple(getattr(vector,axis) for vector in (twist.linear,twist.angular) for axis in ('x','y','z'))
        if (type(stamp.sec) is not int or type(stamp.nanosec) is not int
                or not 0<=stamp.nanosec<10**9 or not all(number(v) for v in values)):
            raise ValueError("Invalid command fields")
        return (stamp.sec,stamp.nanosec,message.header.frame_id)+values

    def expect(self,message,now):
        self.expected={key:t for key,t in self.expected.items() if 0<=now-t<.5}
        self.expected[self.signature(message)]=now
        if len(self.expected)>128:self.expected.pop(next(iter(self.expected)))

    def observe(self,message,now):
        try:
            sent=self.expected.pop(self.signature(message),None)
            if sent is None or not 0<=now-sent<.5:self.fault="external_command_received"
        except (AttributeError,TypeError,ValueError,OverflowError):self.fault="invalid_external_command"

    def problem(self):return self.fault


class EndpointRuntime:
    def __init__(self,mode,session,started,channel,enable_publish,publish,feedback,ownership,discovery_s=1.0,max_session_s=None):
        self.mode,self.session,self.started=mode,session,started
        self.channel,self.enable_publish,self.publish=channel,enable_publish,publish
        self.feedback,self.ownership=feedback,ownership
        self.guard=SessionGuard(session)
        self.ready=False;self.finished=False;self.exit_code=None
        self.stop_at=None;self.stop_reason=None;self.stop_verifier=None
        self.ready_at=None
        self.clean_since=None;self.discovery_s=discovery_s
        self.clock=None
        self.max_session_s=max_session_s
        self.acceleration_limiter=AccelerationLimiter(started=started)
        self.capture_requests={}
        self.startup_prepare=None;self.preparing=False

    def request_stop(self,reason,now):
        if self.stop_at is None:
            self.stop_at=now;self.stop_reason=reason;self.stop_verifier=StopVerifier(now)

    def emit(self,row,now):
        try:
            self.channel.send(dict(row,session=self.session))
            return True
        except ChannelError:
            self.request_stop("output_channel_failed",now)
            return False

    def apply(self,command,now,feedback):
        requested=command
        if command.reason=="replay":self.acceleration_limiter.reset(now)
        else:command=self.acceleration_limiter.update(command,now)
        stamp=self.publish(command)
        self.command_sequence=getattr(self,"command_sequence",0)+1
        info=asdict(feedback) if isinstance(feedback,Feedback) and feedback.fresh(now) else None
        self.emit(dict(type="target",**asdict(command),robot_monotonic_s=now,
                       requested_target=asdict(requested),
                       acceleration_limited=(command.vx,command.vy,command.wz)!=(requested.vx,requested.vy,requested.wz),
                       acceleration_limits=dict(linear_m_s2=self.acceleration_limiter.linear_accel,angular_rad_s2=self.acceleration_limiter.angular_accel,braking="immediate",legacy_replay="bypass"),
                       ros_stamp_ns=stamp,published=True,feedback=info,
                       command_id=self.session+":"+str(self.command_sequence),
                       send_result="ros_publish_returned",owner=self.session,
                       enable_epoch=getattr(self.guard,"release_epoch",None)),now)

    def capture_request(self,message,now,feedback):
        from capture_control import valid_request
        request_id=message.get("request_id")
        signature=tuple(message.get(k) for k in ("session","operation","capture_id"))
        if request_id in self.capture_requests:
            old_signature,ack=self.capture_requests[request_id]
            if old_signature==signature:self.emit(ack,now);return
        ack=dict(type="capture_boundary",request_id=request_id,capture_id=message.get("capture_id"),
                 operation=message.get("operation"),ok=False)
        try:
            if not valid_request(message) or message.get("session")!=self.session:
                raise ValueError("invalid_capture_request")
            if not self.ready or self.stop_at is not None:raise ValueError("endpoint_not_ready")
            if not isinstance(feedback,Feedback) or not feedback.fresh(now) or not feedback.manual:
                raise ValueError("feedback_not_ready")
            handler=getattr(self.guard,"capture_boundary",None)
            if handler is None:raise ValueError("capture_scope_unavailable")
            ack.update(handler(message["operation"],message["capture_id"],now),ok=True)
        except (ValueError,TypeError,AttributeError) as exc:ack["reason"]=str(exc)
        self.capture_requests[request_id]=(signature,ack)
        if len(self.capture_requests)>64:self.capture_requests.pop(next(iter(self.capture_requests)))
        self.emit(ack,now)

    def tick(self,now):
        if self.finished:return
        boundaries=[]
        try:messages=self.channel.receive()
        except ChannelError:
            messages=[];self.request_stop("input_channel_failed",now)
        emergency=any(m.get("session")==self.session and m.get("type")=="emergency_stop" for m in messages)
        if emergency and not getattr(self,"emergency_requested",False):
            self.emergency_requested=True
            self.request_stop("native_emergency_stop",now)
            self.stop_reason="native_emergency_stop"
            try:
                import subprocess
                from project_paths import output_directory
                logdir=output_directory("logs");logdir.mkdir(parents=True,exist_ok=True)
                logfile=logdir/("native-emergency-"+self.session+".log")
                helper=Path(__file__).resolve().parents[1]/"common"/"emergency_stop.py"
                with logfile.open("ab",buffering=0) as stream:
                    subprocess.Popen([sys.executable,"-B",str(helper),"--execute"],
                        stdin=subprocess.DEVNULL,stdout=stream,stderr=stream,cwd=logdir,start_new_session=True)
                self.emit(dict(type="emergency_stop_requested",confirmed=False,details_log=str(logfile)),now)
            except Exception as exc:
                self.emit(dict(type="emergency_stop_failed",reason=str(exc),confirmed=False),now)
        for message in messages:
            if message.get("type")=="emergency_stop" and message.get("session")==self.session:continue
            if message.get("session")!=self.session:
                self.request_stop("wrong_session",now)
            elif message.get("type")=="stop":
                self.request_stop("host_stop",now)
            elif message.get("type")=="input":
                self.guard.accept(message,now)
            elif message.get("type")=="capture":boundaries.append(message)
            else:self.request_stop("invalid_message_type",now)
        if self.channel.eof:self.request_stop("stdin_eof",now)
        if self.ready and self.stop_at is not None:
            self.stopping(now);return
        try:
            fb=self.feedback();problem=self.ownership()
        except Exception as exc:
            fb=None;problem="inspection_failed: "+str(exc)
            if self.ready:
                self.request_stop(problem,now);self.stopping(now);return
        if self.clock is not None:now=self.clock()
        fresh=isinstance(fb,Feedback) and fb.fresh(now)
        if not self.ready:
            for message in boundaries:self.capture_request(message,now,fb)
            reason=(problem or ("feedback_missing_or_stale" if not fresh else
                    "software_manual_required" if not fb.manual else
                    "base_not_stationary" if max(abs(fb.vx),abs(fb.vy),abs(fb.wz))>.02 else None))
            if self.stop_at is not None:
                reason=self.stop_reason
            if reason is None:
                if self.clean_since is None:self.clean_since=now
                if now-self.clean_since<self.discovery_s:return
                if self.mode=="check":
                    ok=self.emit(dict(type="checked",ok=True,publisher_enabled=False,feedback=asdict(fb)),now)
                    self.finished=True;self.exit_code=0 if ok else 1;return
                if self.startup_prepare is not None:
                    if not self.preparing:
                        self.enable_publish();self.preparing=True
                    self.apply(Command(reason="startup_parking_release"),now,fb)
                    if self.stop_at is not None:
                        self.finished=True;self.exit_code=1;return
                    try:
                        prepared=self.startup_prepare(now)
                    except ValueError as exc:
                        self.emit(dict(type="failed",reason=str(exc),publisher_enabled=True),now)
                        self.finished=True;self.exit_code=1;return
                    if self.stop_at is not None:
                        self.finished=True;self.exit_code=1;return
                    if not prepared:return
                else:self.enable_publish()
                self.ready=True;self.ready_at=now
                self.emit(dict(type="ready",publisher_enabled=True,max_session_s=self.max_session_s),now)
            else:
                self.clean_since=None
            if reason is not None and (self.stop_at is not None or now-self.started>=5):
                self.emit(dict(type="failed",reason=reason,publisher_enabled=False),now)
                self.finished=True;self.exit_code=1
            return
        if problem:self.request_stop(problem,now)
        if self.max_session_s is not None and now-self.ready_at>=self.max_session_s:self.request_stop("session_expired",now)
        if self.stop_at is not None:
            self.stopping(now);return
        for message in boundaries:self.capture_request(message,now,fb)
        command=self.guard.step(now,fb,armed=True)
        finished_reason=getattr(self.guard,"finished_reason",None)
        if finished_reason:
            self.request_stop(finished_reason,now);self.stopping(now);return
        self.apply(command,now,fb)
        self.emit(self.guard.poll(now),now)


    def stopping(self,now):
        # Zero must not depend on graph/file/feedback inspection succeeding.
        self.apply(Command(reason="stopping"),now,None)
        try:fb=self.feedback()
        except Exception:fb=None
        if self.clock is not None:now=self.clock()
        expired=now-self.stop_at>=STOP_TIMEOUT
        confirmed=not expired and self.stop_verifier.observe(fb,now)
        if confirmed or expired:
            self.emit(dict(type="stopped",confirmed=confirmed,reason=self.stop_reason,
                           status="STOP_CONFIRMED" if confirmed else "STOP_UNCONFIRMED"),now)
            self.finished=True;self.exit_code=0 if confirmed else 2


def legacy_snapshot(root, now_mono, now_wall):
    """Reads old readiness files; never changes an old file or service."""
    try:
        path=root/"status.json"
        status=json.loads(path.read_text())
        if not isinstance(status,dict):return None
        permission=root/"armed_until"
        remaining=float(permission.read_text())-now_mono if permission.exists() else 0.0
        return dict(armed=status.get("armed"),permission_remaining=remaining,
                    age_s=now_wall-path.stat().st_mtime)
    except (OSError,ValueError,TypeError):return None


def main(argv=None,guard_factory=None):
    parser=argparse.ArgumentParser(description="Independent chassis endpoint; check never publishes.")
    mode=parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check",action="store_true")
    mode.add_argument("--execute",action="store_true")
    parser.add_argument("--gamepad-reset",action="store_true")
    parser.add_argument("--reset-button-index",type=int,default=0)
    parser.add_argument("--session",required=True)
    args=parser.parse_args(argv)
    import re
    if not re.fullmatch(r"[0-9a-f]{32}",args.session):parser.error("Invalid session")
    mode="execute" if args.execute else "check"
    if args.gamepad_reset:
        if guard_factory is not None:raise ValueError("conflicting_control_guards")
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"reset"))
        from reset import robot_main
        return robot_main(mode,args.session,gamepad=True,button_index=args.reset_button_index)
    # Everything above this point works on the host without ROS or robot access.
    import fcntl
    lock_fd=None
    if args.execute:
        lock_fd=os.open(Path(__file__).resolve().parents[2]/".sender.lock",os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
        try:fcntl.flock(lock_fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            print(json.dumps(dict(type="failed",session=args.session,reason="another_new_sender_running",publisher_enabled=False)),flush=True)
            os.close(lock_fd);return 1
    try:
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        from geometry_msgs.msg import TwistStamped
        from move.msg import State
        rclpy.init(args=["--ros-args","--disable-rosout-logs","--disable-external-lib-logs"],
                    domain_id=int(os.environ.get("ROS_DOMAIN_ID","0")),signal_handler_options=SignalHandlerOptions.NO)
        node=rclpy.create_node("tianjidipan_dpad_"+args.session[:16],enable_rosout=False)
        channel=JsonChannel(sys.stdin.fileno(),sys.stdout.fileno())
        holder={"publisher":None,"feedback":None,"seq":0}
        identity=SessionIdentity(lambda:node.get_publishers_info_by_topic(STATE_TOPIC),
                                 lambda:node.get_publishers_info_by_topic(TOPIC))
        feedback_reader=StateFeedbackGuard(lambda:node.get_publishers_info_by_topic(STATE_TOPIC),identity)
        command_monitor=CommandMonitor();dormant_base=DormantBaseGuard()
        parking=(create_parking_release(node,lambda row:runtime.emit(row,time.monotonic()))
                 if mode=="execute" else None)
        def receive_state(message,info):
            feedback_reader.receive(message,info,time.time_ns(),time.monotonic())
            current=feedback_reader.get()
            if parking is not None and current is not None:
                parking.observe(getattr(message,"is_parking_mode",None),current.received_at,current.seq)
        node.create_subscription(State,STATE_TOPIC,receive_state,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        def receive_command(message):command_monitor.observe(message,time.monotonic())
        node.create_subscription(TwistStamped,TOPIC,receive_command,
            QoSProfile(depth=128,reliability=ReliabilityPolicy.RELIABLE))
        def enable():
            holder["publisher"]=node.create_publisher(TwistStamped,TOPIC,
                QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
        def publish(command):
            message=TwistStamped();stamp=node.get_clock().now()
            message.header.stamp=stamp.to_msg();message.header.frame_id="base_link"
            message.twist.linear.x=command.vx;message.twist.linear.y=command.vy
            message.twist.angular.z=command.wz
            command_monitor.expect(message,time.monotonic())
            holder["publisher"].publish(message)
            return stamp.nanoseconds
        def ownership():
            rows=node.get_publishers_info_by_topic(TOPIC)
            problem=identity.inspect() or command_monitor.problem() or dormant_base.inspect(rows)
            names=publisher_names(node.get_publishers_info_by_topic(TOPIC),identity.command_gid)
            if dormant_base.gid:names=[name for name in names if name!="base_teleop"]
            if problem:return problem
            if not identity.announced:
                channel.send(dict(type="source_identified",session=args.session,**identity.evidence()))
                identity.announced=True
            root=Path("/home/user/gamepad_control")
            old=legacy_snapshot(root,time.monotonic(),time.time())
            # Known legacy readiness is checked even before graph discovery sees its node.
            observed=list(names)
            if root.exists() and "host_gamepad_control" not in observed:
                observed.append("host_gamepad_control")
            return ownership_problem(observed,node.get_name(),old)
        runtime=EndpointRuntime(mode,args.session,time.monotonic(),channel,enable,publish,
                                feedback_reader.get,ownership)
        runtime.clock=time.monotonic
        if parking is not None:runtime.startup_prepare=parking.step
        if guard_factory is not None:runtime.guard=guard_factory(args.session)
        def stop_signal(signum,_):runtime.request_stop("signal_"+str(signum),time.monotonic())
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,stop_signal)
        def tick():
            now=time.monotonic()
            try:runtime.tick(now)
            except Exception as exc:
                runtime.request_stop("runtime_error: "+str(exc),now)
                if not runtime.ready:
                    runtime.emit(dict(type="failed",reason=str(exc),publisher_enabled=False),now)
                    runtime.finished=True;runtime.exit_code=1
                elif now-runtime.stop_at>=STOP_TIMEOUT:
                    runtime.emit(dict(type="stopped",confirmed=False,status="STOP_UNCONFIRMED",reason=str(exc)),now)
                    runtime.finished=True;runtime.exit_code=2
        node.create_timer(.02,tick)
        try:
            while rclpy.ok() and not runtime.finished:rclpy.spin_once(node,timeout_sec=.02)
        finally:
            if holder["publisher"] is not None:
                try:publish(Command(reason="final_zero"))
                except Exception:pass
            node.destroy_node();rclpy.shutdown()
        return runtime.exit_code if runtime.exit_code is not None else 2
    finally:
        if lock_fd is not None:os.close(lock_fd)


if __name__=="__main__":raise SystemExit(main())
