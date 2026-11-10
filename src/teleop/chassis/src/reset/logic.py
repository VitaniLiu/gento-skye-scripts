"""Small planar return controller. No ROS, files, networking or motion side effects."""
from dataclasses import dataclass
from datetime import datetime
import hashlib
import math
import secrets

REFERENCE_FILE = "reset-reference-20261102-101804.json"
REFERENCE_SHA = "86eb0b4c1da2ca00c722eb8f0087669e3f7ceaa974d34cd68bf254307644f566"
POSE_GID = "0110c5e19a9a977a7ecf10c300002903"
STATE_GID = "0110c5e19a9a977a7ecf10c300002b03"
COMMAND_GID = "011017e1cda14686cbd2e4df00002203"
RETRACE_POS_TOL = .007
RETRACE_YAW_TOL = math.radians(.5)
POS_TOL = .02
YAW_TOL = math.radians(2)

def finite(v):
    return type(v) in (int, float) and math.isfinite(v)

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def clip(v, limit):
    return max(-limit, min(limit, v))

def validate_reference(raw):
    import json
    if len(raw)>16384 or hashlib.sha256(raw).hexdigest()!=REFERENCE_SHA:
        raise ValueError("reference_hash_mismatch")
    r=json.loads(raw);p=r["pose"];source=r["source"]
    if (not all(finite(p[k]) for k in ("x_m","y_m","yaw_rad"))
        or source["frame_id"]!="odom" or source["child_frame_id"]!="base_link"
        or source["publisher_gid"]!=POSE_GID
        or p["yaw_encoding"]!="legacy raw yaw (radians) in orientation.w; NOT a quaternion"):
        raise ValueError("invalid_reference")
    return r

def validate_verification(record, now, straight=False):
    if not isinstance(record,dict):
        raise ValueError("verification_missing")
    direction="straight_direction_and_scale_verified" if straight else "direction_and_scale_verified"
    for name in (direction,"independent_stop_verified","no_odometry_reset_confirmed"):
        if record.get(name) is not True:
            raise ValueError("not_verified: "+name)
    if record.get("reference_sha256")!=REFERENCE_SHA or record.get("controller_prefix")!=POSE_GID[:24]:
        raise ValueError("verification_identity_mismatch")
    timeout=record.get("stop_timeout_s")
    if not finite(timeout) or not 0<timeout<=.2:
        raise ValueError("independent_stop_timeout_not_verified")
    if not isinstance(record.get("evidence_note"),str) or len(record["evidence_note"].strip())<10:
        raise ValueError("verification_evidence_missing")
    try:
        d=datetime.fromisoformat(record["verified_at"])
        if d.tzinfo is None:raise ValueError()
        age=now-d.timestamp()
        if not 0<=age<=86400:raise ValueError()
    except (KeyError,TypeError,ValueError,OverflowError):
        raise ValueError("verification_expired_or_invalid") from None
    return record

@dataclass(frozen=True)
class Sample:
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float
    at: float
    seq: int
    observed_at: float | None = None

@dataclass(frozen=True)
class Motion:
    vx: float = 0.
    vy: float = 0.
    wz: float = 0.
    reason: str = ""

class Lease:
    """Host replies to unpredictable robot challenges; stale replies cannot renew."""
    def __init__(self,session):
        self.session=session;self.pending={};self.last_seq=-1;self.issued=None
    def poll(self,now):
        self.pending={k:t for k,t in self.pending.items() if 0<=now-t<.15}
        nonce=secrets.token_hex(8);self.pending[nonce]=now
        return dict(type="poll",session=self.session,nonce=nonce)
    def accept(self,msg,now):
        if msg.get("type")!="input" or msg.get("session")!=self.session:return False
        issued=self.pending.pop(msg.get("nonce"),None)
        seq=msg.get("seq")
        if (issued is None or not 0<=now-issued<.15 or type(seq) is not int
            or seq<=self.last_seq or (self.issued is not None and issued<=self.issued)):
            return False
        self.last_seq=seq;self.issued=issued;return True
    def fresh(self,now):
        return self.issued is not None and 0<=now-self.issued<.15


class MeasurementFreshness:
    """Check sensor/header time progression as well as fresh DDS transport."""
    def __init__(self):
        self.header=None;self.anchor_header=None;self.anchor_at=None
        self.last_at=None;self.fault=None
    def sample(self,header_ns,dds_at):
        if self.fault:return None
        if type(header_ns) is not int or header_ns<=0 or not finite(dds_at):
            self.fault="invalid_measurement_stamp";return None
        if self.header is not None:
            if header_ns<self.header:
                self.fault="measurement_clock_regression";return None
            if header_ns==self.header:return None
        if self.anchor_header is None:
            self.anchor_header=header_ns;self.anchor_at=dds_at
        at=self.anchor_at+(header_ns-self.anchor_header)/1e9
        lag=dds_at-at
        if lag<-.05:
            self.fault="measurement_clock_jump";return None
        if lag>=.2:
            self.fault="measurement_stale";return None
        if lag<0:self.anchor_at+=lag;at=dds_at
        self.header=header_ns;self.last_at=at
        return at

class SourceLatch:
    """An observed execution-time identity/clock fault cannot recover silently."""
    def __init__(self):self.fault=None
    def observe(self,pose_ok,state_ok,active,clock_fault=None):
        problem=("unverified_pose_publisher" if not pose_ok else
                 "unverified_state_publisher" if not state_ok else clock_fault)
        if active and problem and self.fault is None:self.fault=problem
        return self.fault or problem

class ReturnController:
    """Turn -> approach -> settle -> restore yaw -> settle; bounded corrections."""
    def __init__(self,target,started):
        if len(target)!=3 or not all(finite(v) for v in target) or not finite(started):
            raise ValueError("invalid_target")
        self.target=tuple(target);self.started=started;self.phase=None
        self.finished_reason=None;self.previous=None;self.tick_at=None
        self.stable_at=None;self.best=None;self.progress_at=started
        self.bearing=0.;self.next_phase=None;self.corrections=0;self.drive_sign=0
    def abort(self,reason):
        self.finished_reason=reason
        return Motion(reason=reason)
    def transition(self,phase,now):
        self.phase=phase;self.stable_at=None;self.best=None;self.progress_at=now
    def stopped(self,s):
        return abs(s.vx)<=.01 and abs(s.vy)<=.01 and abs(s.wz)<=.02
    def hold(self,s,seconds):
        if not self.stopped(s):self.stable_at=None;return False
        if self.stable_at is None:self.stable_at=s.at if s.observed_at is None else s.observed_at
        return s.at-self.stable_at>=seconds-1e-9
    def replan(self,s,now):
        self.corrections+=1
        if self.corrections>3:return self.abort("correction_limit")
        self.next_phase="ALIGN";self.transition("BRAKE",now)
        return Motion(reason="replan_braking")
    def step(self,s,now):
        if self.finished_reason:return Motion(reason=self.finished_reason)
        if (not isinstance(s,Sample) or not finite(now)
            or not all(finite(getattr(s,k)) for k in ("x","y","yaw","vx","vy","wz","at"))
            or type(s.seq) is not int or s.seq<0):
            return self.abort("invalid_feedback")
        if s.observed_at is not None and (not finite(s.observed_at) or not s.at<=s.observed_at<=now):
            return self.abort("invalid_feedback")
        if not 0<=now-s.at<.2:return self.abort("feedback_stale")
        if now-self.started>=120:return self.abort("session_timeout")
        if self.tick_at is not None and not 0<=now-self.tick_at<=.1+1e-9:
            return self.abort("scheduler_late")
        self.tick_at=now
        if self.previous is not None:
            p=self.previous
            if s.seq<p.seq or s.at<p.at:return self.abort("feedback_regression")
            if s.seq==p.seq and s!=p:return self.abort("feedback_sequence_changed")
            if s.seq!=p.seq:
                dt=s.at-p.at
                if math.hypot(s.x-p.x,s.y-p.y)>.08+.2*dt or abs(wrap(s.yaw-p.yaw))>.15+.3*dt:
                    return self.abort("pose_jump")
        self.previous=s
        dx=self.target[0]-s.x;dy=self.target[1]-s.y
        distance=math.hypot(dx,dy);final_angle=wrap(self.target[2]-s.yaw)
        if self.phase is None:
            if distance>1:return self.abort("target_out_of_range")
            self.bearing=math.atan2(dy,dx)
            self.transition("FINAL" if distance<=POS_TOL else "ALIGN",now)
        if self.phase=="BRAKE":
            if self.hold(s,.3):
                next_phase=self.next_phase
                if next_phase=="ALIGN":self.bearing=math.atan2(dy,dx)
                self.transition(next_phase,now)
            return Motion(reason="braking")
        if self.phase=="VERIFY":
            if distance>POS_TOL:return self.replan(s,now)
            if abs(final_angle)>YAW_TOL:
                self.next_phase="FINAL";self.transition("BRAKE",now)
                return Motion(reason="yaw_drift_braking")
            if self.hold(s,.5):return self.abort("reset_complete")
            return Motion(reason="verifying_stopped")
        if self.phase=="ALIGN":
            if distance<=POS_TOL:
                self.next_phase="FINAL";self.transition("BRAKE",now)
                return Motion(reason="position_reached")
            angle=wrap(self.bearing-s.yaw);metric=abs(angle)
            if abs(angle)<=math.radians(3):
                self.drive_sign=0;self.transition("DRIVE",now);return Motion(reason="aligned")
            out=Motion(wz=clip(angle,.1),reason="align")
        elif self.phase=="DRIVE":
            if distance<=POS_TOL:
                self.next_phase="FINAL";self.transition("BRAKE",now)
                return Motion(reason="position_reached")
            angle=wrap(self.bearing-s.yaw)
            cross=-dx*math.sin(self.bearing)+dy*math.cos(self.bearing)
            if abs(angle)>.35 or abs(cross)>.035:return self.replan(s,now)
            along=dx*math.cos(self.bearing)+dy*math.sin(self.bearing)
            if abs(along)<POS_TOL and distance>POS_TOL:return self.replan(s,now)
            sign=1 if along>0 else -1
            if self.drive_sign and sign!=self.drive_sign:
                self.corrections+=1
                if self.corrections>3:return self.abort("correction_limit")
                self.drive_sign=sign;self.next_phase="DRIVE";self.transition("BRAKE",now)
                return Motion(reason="reversal_braking")
            self.drive_sign=sign
            out=Motion(vx=clip(.5*along,.02 if distance<=.1 else .05),
                       wz=clip(angle,.1),reason="approach")
            metric=distance
        elif self.phase=="FINAL":
            if distance>POS_TOL:return self.replan(s,now)
            if abs(final_angle)<=YAW_TOL:
                self.transition("VERIFY",now);return Motion(reason="pose_reached")
            out=Motion(wz=clip(final_angle,.1),reason="restore_heading");metric=abs(final_angle)
        else:return self.abort("invalid_phase")
        if self.best is None or metric<self.best-.001:
            self.best=metric;self.progress_at=now
        if now-self.progress_at>=8:return self.abort("no_progress")
        if self.best is not None and metric>self.best+.12:return self.abort("error_growing")
        return out


class StraightReturnController(ReturnController):
    """Return along the initial body axis. Never command lateral motion or yaw."""
    def step(self,s,now):
        if self.finished_reason:return Motion(reason=self.finished_reason)
        if (not isinstance(s,Sample) or not finite(now)
            or not all(finite(getattr(s,k)) for k in ("x","y","yaw","vx","vy","wz","at"))
            or type(s.seq) is not int or s.seq<0):
            return self.abort("invalid_feedback")
        if s.observed_at is not None and (not finite(s.observed_at) or not s.at<=s.observed_at<=now):
            return self.abort("invalid_feedback")
        if not 0<=now-s.at<.2:return self.abort("feedback_stale")
        if now-self.started>=120:return self.abort("session_timeout")
        if self.tick_at is not None and not 0<=now-self.tick_at<=.1+1e-9:
            return self.abort("scheduler_late")
        self.tick_at=now
        if self.previous is not None:
            p=self.previous
            if s.seq<p.seq or s.at<p.at:return self.abort("feedback_regression")
            if s.seq==p.seq and s!=p:return self.abort("feedback_sequence_changed")
            if s.seq!=p.seq:
                dt=s.at-p.at
                if math.hypot(s.x-p.x,s.y-p.y)>.08+.2*dt or abs(wrap(s.yaw-p.yaw))>.15+.3*dt:
                    return self.abort("pose_jump")
        self.previous=s
        dx=self.target[0]-s.x;dy=self.target[1]-s.y
        distance=math.hypot(dx,dy)
        if self.phase is None:
            if distance>1:return self.abort('target_out_of_range')
            self.bearing=s.yaw
            self.transition('DRIVE',now)
        along=dx*math.cos(self.bearing)+dy*math.sin(self.bearing)
        cross=-dx*math.sin(self.bearing)+dy*math.cos(self.bearing)
        if abs(cross)>POS_TOL:return self.abort('straight_target_off_axis')
        if self.phase=='BRAKE':
            if self.hold(s,.3):self.transition('DRIVE',now)
            return Motion(reason='straight_braking')
        if distance<=POS_TOL:
            if self.phase!='VERIFY':self.transition('VERIFY',now)
            if self.hold(s,.5):return self.abort('reset_complete')
            return Motion(reason='straight_verifying_stopped')
        if self.phase=='VERIFY':
            self.transition('BRAKE',now)
            return Motion(reason='straight_settle_braking')
        sign=1 if along>0 else -1
        if self.drive_sign and sign!=self.drive_sign:
            self.corrections+=1
            if self.corrections>3:return self.abort('correction_limit')
            self.drive_sign=sign;self.transition('BRAKE',now)
            return Motion(reason='straight_reversal_braking')
        self.drive_sign=sign
        if self.best is None or distance<self.best-.001:
            self.best=distance;self.progress_at=now
        if now-self.progress_at>=8:return self.abort('no_progress')
        if distance>self.best+.12:return self.abort('error_growing')
        return Motion(vx=clip(.5*along,.02 if distance<=.1 else .05),
                      reason='straight_approach')


def controller_for(control_mode):
    """Choose a control stage without changing target, feedback or transport."""
    choices={'straight':StraightReturnController,'pose':ReturnController}
    try:return choices[control_mode]
    except (KeyError,TypeError):raise ValueError('invalid_control_mode') from None

def position_error(sample,target):
    """Residual in the saved start's forward/left axes; not a ground measurement."""
    dx=sample.x-target[0];dy=sample.y-target[1];yaw=target[2]
    return dict(longitudinal_m=dx*math.cos(yaw)+dy*math.sin(yaw),
                lateral_m=-dx*math.sin(yaw)+dy*math.cos(yaw),
                distance_m=math.hypot(dx,dy),heading_rad=wrap(sample.yaw-yaw))

def target_problem(sample,target,control_mode):
    if control_mode != "retrace":
        controller_for(control_mode)
    tolerance = RETRACE_POS_TOL if control_mode == "retrace" else POS_TOL
    error=position_error(sample,target)
    if error['distance_m']>tolerance:return 'position_outside_tolerance'
    yaw_tolerance = RETRACE_YAW_TOL if control_mode == 'retrace' else YAW_TOL
    if control_mode in ('retrace','pose') and abs(error['heading_rad'])>yaw_tolerance:
        return 'heading_outside_tolerance'
    return None
