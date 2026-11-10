"""Pure request/reply freshness, D-pad gates and stop confirmation."""
from dataclasses import dataclass
import math
import secrets
from controller import Command, DpadController, InputFrame, SafetyState

INPUT_TIMEOUT = .15
FEEDBACK_TIMEOUT = .2
STOP_TIMEOUT = 2.0
MAX_SESSION_SECONDS = 1800.0


def number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def feedback_sample_time(source_ns,received_ns,wall_ns,now):
    """Fail closed if DDS timestamp support/clock compatibility is unavailable."""
    if any(type(v) is not int or v<=0 for v in (source_ns,received_ns,wall_ns)):
        return None
    ages=[(wall_ns-v)/1e9 for v in (source_ns,received_ns)]
    if not all(0<=age<FEEDBACK_TIMEOUT for age in ages):return None
    return now-max(ages)


class FeedbackClock:
    """Align a fixed remote clock offset; never treat source time as local wall time.

    This bounds local queue age and source lag relative to the session baseline.
    An unknown constant transport delay cannot be measured without clock sync.
    """
    def __init__(self):
        self.anchor_source=None;self.anchor_local=None
        self.first_local=None;self.last_source=None;self.last_seq=None
        self.count=0;self.faulted=False

    def sample(self,source_ns,received_ns,wall_ns,now,sequence=None):
        if self.faulted:return None
        if (not number(now) or any(type(v) is not int or v<=0
                for v in (source_ns,received_ns,wall_ns))):return None
        age=(wall_ns-received_ns)/1e9
        if not 0<=age<FEEDBACK_TIMEOUT:return None
        received=now-age
        seq=sequence if type(sequence) is int and 0<=sequence<2**64-1 else None
        if self.last_source is not None:
            if source_ns<self.last_source:
                self.faulted=True;return None
            if source_ns==self.last_source or (seq is not None and self.last_seq is not None and seq<=self.last_seq):
                return None
        if self.anchor_source is None:
            self.anchor_source=source_ns;self.anchor_local=received;self.first_local=received
        sampled=self.anchor_local+(source_ns-self.anchor_source)/1e9
        delay=received-sampled
        if delay < -.05:
            self.faulted=True;return None
        if delay<0:
            self.anchor_local+=delay;sampled=received
        if not 0<=now-sampled<FEEDBACK_TIMEOUT:return None
        self.last_source=source_ns
        if seq is not None:self.last_seq=seq
        self.count+=1
        if self.count<3 or received-self.first_local<.1:return None
        return sampled


@dataclass(frozen=True)
class Feedback:
    received_at: float
    seq: int
    manual: bool
    vx: float
    vy: float
    wz: float

    def fresh(self, now):
        return (number(self.received_at) and 0 <= now-self.received_at < FEEDBACK_TIMEOUT
                and type(self.manual) is bool
                and all(number(v) for v in (self.vx,self.vy,self.wz)))


class SessionGuard:
    def __init__(self, session):
        self.session = session
        self.pending = {}
        self.last_seq = -1
        self.last_poll_at = -1.0
        self.release_epoch = None
        self.latest = None
        self.reset_pending = False
        self.controller = DpadController()
        self.previous_feedback_at = None

    def poll(self, now):
        self.pending = {key:t for key,t in self.pending.items() if 0 <= now-t < INPUT_TIMEOUT}
        nonce = secrets.token_hex(8)
        self.pending[nonce] = now
        return dict(type="poll", session=self.session, nonce=nonce)

    def accept(self, msg, now):
        if not isinstance(msg,dict) or msg.get("type") != "input" or msg.get("session") != self.session:
            return False
        nonce = msg.get("nonce")
        if not isinstance(nonce,str) or nonce not in self.pending:
            return False
        issued = self.pending.pop(nonce)
        if not number(now) or not 0 <= now-issued < INPUT_TIMEOUT or issued <= self.last_poll_at:
            return False
        seq, epoch = msg.get("seq"), msg.get("release_epoch")
        if (type(seq) is not int or seq <= self.last_seq or type(epoch) is not int
                or epoch < 0 or (self.release_epoch is not None and epoch < self.release_epoch)):
            return False
        try:
            frame = InputFrame(msg["axes"],msg["buttons"],msg["connected"],msg["usb_age_s"],0.0)
        except KeyError:
            return False
        if not DpadController._valid_input(frame):
            self.latest = None
            self.reset_pending = True
            return False
        if self.latest is not None:
            previous, previous_issued, previous_received = self.latest
            if (now-previous_issued+previous.usb_age_s >= INPUT_TIMEOUT
                    or now-previous_received >= INPUT_TIMEOUT):
                self.reset_pending = True
        if self.release_epoch is not None and epoch != self.release_epoch:
            self.reset_pending = True
        self.release_epoch = epoch
        self.last_seq, self.last_poll_at = seq, issued
        self.latest = (frame, issued, now)
        return True

    def step(self, now, feedback, armed=True):
        if self.previous_feedback_at is not None and now-self.previous_feedback_at>=FEEDBACK_TIMEOUT:
            self.reset_pending=True
        self.previous_feedback_at=feedback.received_at if isinstance(feedback,Feedback) and feedback.fresh(now) else None
        if self.reset_pending:
            self.controller.update(InputFrame((0.0,)*8,(0,)*13,False),SafetyState(),now,str(now))
            self.reset_pending = False
        if self.latest is None:
            frame = InputFrame((0.0,)*8,(0,)*13,False)
        else:
            raw, issued, received = self.latest
            frame = InputFrame(raw.axes,raw.buttons,raw.connected,
                               raw.usb_age_s + max(0.0,now-issued),max(0.0,now-received))
        fresh = isinstance(feedback,Feedback) and feedback.fresh(now)
        safety = SafetyState(armed is True, fresh and feedback.manual,
                             now-feedback.received_at if fresh else float("inf"))
        return self.controller.update(frame,safety,now,str(now)).command


class StopVerifier:
    """Require three distinct, fresh zero-speed samples spanning >=100ms."""
    def __init__(self, requested_at):
        self.requested_at = requested_at
        self.last_seq = -1
        self.first_at = None
        self.count = 0
        self.last_sample_at = None

    def observe(self, feedback, now):
        if not isinstance(feedback,Feedback) or not feedback.fresh(now) or feedback.received_at < self.requested_at:
            self.count = 0; self.first_at = None
            return False
        if feedback.seq <= self.last_seq:
            return False
        self.last_seq = feedback.seq
        if self.last_sample_at is not None and feedback.received_at-self.last_sample_at>=FEEDBACK_TIMEOUT:
            self.count=0;self.first_at=None
        self.last_sample_at=feedback.received_at
        if any(abs(v) > .02 for v in (feedback.vx,feedback.vy,feedback.wz)):
            self.count = 0; self.first_at = None
            return False
        if self.first_at is None:
            self.first_at = feedback.received_at
        self.count += 1
        return self.count >= 3 and feedback.received_at-self.first_at >= .1


def ownership_problem(publishers, own_name, _legacy):
    unknown = set(publishers) - {own_name,"host_gamepad_control"}
    if unknown:
        return "unknown_command_publishers: " + ",".join(sorted(unknown))
    if publishers.count("host_gamepad_control") > 1:
        return "duplicate_legacy_publishers"
    return None
