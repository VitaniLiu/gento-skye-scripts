"""Session-local measured path and closed-loop reverse traversal. No I/O."""
from dataclasses import dataclass
import math

from logic import (Motion, Sample, RETRACE_POS_TOL, RETRACE_YAW_TOL,
                   clip, finite, wrap)


RETRACE_WAYPOINT_TOL = .015
STRAIGHT_CORRECTION_MAX = .02
PIVOT_LENGTH_MAX = .02
FINAL_ENTRY_TOL = RETRACE_POS_TOL
WHEEL_STRAIGHTEN_SPEED = .008
WHEEL_STRAIGHTEN_SECONDS = .25


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float
    turning: bool = False

    def distance(self, other):
        return math.hypot(self.x - other.x, self.y - other.y)


def sample_problem(sample, previous, now):
    if not isinstance(sample, Sample) or not all(finite(v) for v in
            (sample.x, sample.y, sample.yaw, sample.vx, sample.vy, sample.wz, sample.at, now)):
        return 'invalid_pose'
    if not 0 <= now - sample.at < .2:
        return 'stale_pose'
    if previous is not None:
        dt = sample.at - previous.at
        if dt < 0 or dt >= .2 or sample.seq < previous.seq:
            return 'broken_pose_history'
        distance = math.hypot(sample.x - previous.x, sample.y - previous.y)
        turn = abs(wrap(sample.yaw - previous.yaw))
        if distance > .006 + .6 * dt or turn > .02 + 2 * dt:
            return 'pose_jump'
    return None


class RecordedPath:
    def __init__(self, target):
        self.origin = None
        self.points = []
        self.previous = None
        self.fault = None
        self.segment_turning = False
        self.commands = []
        self.command_at = None
        self.last_command = (0.,0.)

    def observe(self, sample, now, commanded_wz=0., commanded_vx=None):
        if self.fault:
            return
        self.fault = sample_problem(sample, self.previous, now)
        if self.fault:
            return
        if commanded_vx is not None:
            if (not all(finite(v) for v in (commanded_vx, commanded_wz)) or
                    abs(commanded_vx) > .2 or abs(commanded_wz) > .2):
                self.fault = 'invalid_recorded_command'
                return
            if self.command_at is not None:
                dt = now-self.command_at
                if not 0 <= dt <= .100001:
                    self.fault = 'broken_command_history'
                    return
                if dt:
                    v,w = self.last_command
                    if self.commands and self.commands[-1][:2] == (v,w):
                        self.commands[-1] = (v,w,self.commands[-1][2]+dt)
                    else:
                        if len(self.commands) >= 4000:
                            self.fault = 'path_capacity_exceeded'
                            return
                        self.commands.append((v,w,dt))
            self.command_at = now
            self.last_command = (commanded_vx,commanded_wz)
        pose = Pose(sample.x, sample.y, sample.yaw)
        if not self.points:
            if abs(sample.vx) > .01 or abs(sample.vy) > .01 or abs(sample.wz) > .02:
                self.fault = 'recording_did_not_start_stationary'
                return
            self.origin = pose
            self.points.append(pose)
            self.previous = sample
            return
        if pose.distance(self.origin) > 1.:
            self.fault = "target_out_of_range"
            return
        self.segment_turning |= abs(commanded_wz) > 1e-9
        last = self.points[-1]
        if pose.distance(last) >= .025 or abs(wrap(pose.yaw-last.yaw)) >= .075:
            if len(self.points) >= 4000:
                self.fault = 'path_capacity_exceeded'
                return
            self.points.append(Pose(pose.x, pose.y, pose.yaw, self.segment_turning))
            self.segment_turning = False
        self.previous = sample

    def controller(self, _target, now):
        if self.fault or self.previous is None:
            raise ValueError(self.fault or 'path_missing')
        if self.command_at is not None:
            return ReverseCommandPlayer(self.commands, now)
        sample = self.previous
        end = Pose(sample.x, sample.y, sample.yaw,
                   self.segment_turning)
        points = self.points.copy()
        if end != points[-1]:
            points.append(end)
        return ReverseFollower(points, now)


class ReverseFollower:
    """Visit consecutive segments in reverse order, including on-the-spot turns."""
    phase = 'RETRACE'

    def __init__(self, points, now):
        # Rotations recorded before first leaving the start belong to departure.
        # Keep their outgoing heading for drive direction, but omit those pivots
        # from the end of the return. Later visits to the start stay in sequence.
        origin = points[0]
        self.origin = origin
        first_outside = next((i for i,p in enumerate(points)
                              if p.distance(origin) > RETRACE_POS_TOL), len(points))
        departure = points[first_outside-1]
        self.points = [Pose(origin.x, origin.y, departure.yaw)] + points[first_outside:]
        self.index = len(self.points)-1
        self.started = self.tick = self.progress_at = now
        self.best = math.inf
        self.previous = None
        self.settled = None
        self.straighten_started = None
        self.straightened = False
        self.terminal = False
        self.recovering_path_heading = False
        self.finished_reason = None

    def end(self, reason):
        self.finished_reason = reason
        return Motion(reason=reason)

    def moving(self, command, metric, now):
        if metric < self.best-.0001:
            self.best, self.progress_at = metric, now
        if now-self.progress_at >= 8:
            return self.end('no_progress')
        return command

    def step(self, sample, now):
        if self.finished_reason:
            return Motion(reason=self.finished_reason)
        first_tick = self.previous is None
        problem = sample_problem(sample, self.previous, now)
        if problem or not 0 <= now-self.tick <= .100001 or now-self.started >= 120:
            return self.end(problem or 'retrace_timing_limit')
        self.previous, self.tick = sample, now
        pose = Pose(sample.x, sample.y, sample.yaw)
        end = self.points[self.index]
        goal = self.points[max(0, self.index-1)]
        dx, dy = goal.x-pose.x, goal.y-pose.y
        sx, sy = goal.x-end.x, goal.y-end.y
        length = math.hypot(sx, sy)
        projection = (((pose.x-end.x)*sx + (pose.y-end.y)*sy)/length**2
                      if length else 0.)
        u = max(0., min(1., projection))
        if math.hypot(pose.x-end.x-u*sx, pose.y-end.y-u*sy) > .035:
            return self.end('left_recorded_path')
        distance = math.hypot(dx, dy)
        yaw_error = wrap(goal.yaw-pose.yaw)
        start_yaw_error = wrap(self.origin.yaw-pose.yaw)
        terminal_active = (self.index <= 1 and
                           (distance <= FINAL_ENTRY_TOL or
                            self.straighten_started is not None or
                            self.straightened))
        if terminal_active and not self.terminal:
            self.terminal = True
            self.recovering_path_heading = False
            self.best, self.progress_at = math.inf, now
        if terminal_active:
            if abs(start_yaw_error) > RETRACE_YAW_TOL:
                self.settled = None
                self.straighten_started = None
                self.straightened = False
                return self.moving(Motion(wz=clip(2*start_yaw_error, .05),
                    reason='restore_start_heading'), abs(start_yaw_error), now)
            longitudinal = (dx*math.cos(self.origin.yaw) +
                            dy*math.sin(self.origin.yaw))
            if not self.straightened:
                if self.straighten_started is None:
                    self.straighten_started = now
                    self.best, self.progress_at = math.inf, now
                if now-self.straighten_started < WHEEL_STRAIGHTEN_SECONDS:
                    self.settled = None
                    direction = 1. if longitudinal >= 0. else -1.
                    return Motion(vx=direction*WHEEL_STRAIGHTEN_SPEED,
                                  reason='straighten_wheels')
                self.straightened = True
                self.best, self.progress_at = math.inf, now
            if abs(longitudinal) > .001:
                self.settled = None
                return self.moving(Motion(vx=clip(2*longitudinal,
                    WHEEL_STRAIGHTEN_SPEED), reason='final_straight_position'),
                    abs(longitudinal), now)
            if distance <= RETRACE_POS_TOL:
                if (abs(sample.vx) <= .01 and abs(sample.vy) <= .01 and
                        abs(sample.wz) <= .02):
                    if self.settled is None:
                        self.settled = sample.at
                    if sample.at-self.settled >= .5:
                        return self.end('reset_complete')
                else:
                    self.settled = None
                return Motion(reason='settling_at_start')
            self.straighten_started = None
            self.straightened = False
            self.terminal = False
            self.recovering_path_heading = True
            self.best, self.progress_at = math.inf, now
        if self.recovering_path_heading:
            path_yaw_error = wrap(goal.yaw-pose.yaw)
            if abs(path_yaw_error) > .035:
                return self.moving(Motion(wz=clip(2*path_yaw_error, .05),
                    reason='restore_path_heading'), abs(path_yaw_error), now)
            self.recovering_path_heading = False
            self.best, self.progress_at = math.inf, now
            return Motion(reason='path_heading_restored')
        recorded_turn = end.turning or abs(wrap(end.yaw-goal.yaw)) >= .035
        pivot = recorded_turn and length < PIVOT_LENGTH_MAX
        if pivot and self.index > 1 and abs(yaw_error) <= .035:
            self.index -= 1
            self.best, self.progress_at = math.inf, now
            return Motion(reason='next_recorded_point')
        after_goal = self.points[max(0, self.index-2)]
        upcoming_turn = goal.turning or abs(wrap(goal.yaw-after_goal.yaw)) >= .035
        capture_tolerance = (RETRACE_POS_TOL if self.index <= 1 or recorded_turn
                             or upcoming_turn or first_tick else RETRACE_WAYPOINT_TOL)
        passed_straight = self.index > 1 and not recorded_turn and projection >= 1.
        if (self.index > 1 and (distance <= capture_tolerance or passed_straight)
                and (not recorded_turn or abs(yaw_error) <= .035)):
            self.index -= 1
            self.best, self.progress_at = math.inf, now
            return Motion(reason='next_recorded_point')
        self.settled = None
        if pivot:
            metric = abs(yaw_error)*.01
            cmd = Motion(wz=clip(2*yaw_error, .1), reason='restore_recorded_yaw')
        else:
            reverse = sx*math.cos(goal.yaw) + sy*math.sin(goal.yaw) < 0
            speed = min(.05, 2*distance) if not recorded_turn or abs(yaw_error) < .35 else 0.
            if recorded_turn:
                turn = clip(2*yaw_error, .1)
            else:
                bearing = math.atan2(dy, dx)
                steer = wrap(bearing + (math.pi if reverse else 0.) -
                             pose.yaw)
                turn = clip(.7*steer, STRAIGHT_CORRECTION_MAX)
            cmd = Motion(vx=-speed if reverse else speed, wz=turn,
                         reason='follow_recorded_segment')
            metric = distance + (abs(yaw_error)*.01 if recorded_turn else 0.)
        return self.moving(cmd, metric, now)

class ReverseCommandPlayer:
    """Reverse manual command order and sign, with uniform time scaling."""
    phase = 'RETRACE'
    def __init__(self, commands, now):
        commands = list(commands)
        while commands and commands[0][:2] == (0.,0.):commands.pop(0)
        while commands and commands[-1][:2] == (0.,0.):commands.pop()
        self.scale = min([1.] + [.05/abs(v) for v,w,d in commands if v]
                         + [.1/abs(w) for v,w,d in commands if w])
        self.commands = [(-v*self.scale,-w*self.scale,d/self.scale)
                         for v,w,d in reversed(commands)]
        self.duration = sum(d for v,w,d in self.commands)
        if self.duration + .5 >= 120:
            raise ValueError('retrace_timing_limit')
        self.started = self.tick = now
        self.index = 0
        self.segment_end = self.commands[0][2] if self.commands else 0.
        self.previous = None
        self.settled = None
        self.progress_pose = None
        self.progress_at = now
        self.finished_reason = None

    def end(self, reason):
        self.finished_reason = reason
        return Motion(reason=reason)

    def step(self, sample, now):
        if self.finished_reason:return Motion(reason=self.finished_reason)
        problem = sample_problem(sample,self.previous,now)
        if problem or not 0 <= now-self.tick <= .100001 or now-self.started >= 120:
            return self.end(problem or 'retrace_timing_limit')
        self.previous,self.tick = sample,now
        elapsed = now-self.started
        while self.index < len(self.commands) and elapsed+1e-9 >= self.segment_end:
            self.index += 1
            if self.index < len(self.commands):
                self.segment_end += self.commands[self.index][2]
        if self.index < len(self.commands):
            v,w,_ = self.commands[self.index]
            if v or w:
                p = self.progress_pose
                if (p is None or math.hypot(sample.x-p.x,sample.y-p.y) >= .001 or
                        abs(wrap(sample.yaw-p.yaw)) >= .005):
                    self.progress_pose,self.progress_at = sample,now
                if now-self.progress_at >= 8:return self.end('no_progress')
            else:self.progress_pose,self.progress_at = sample,now
            return Motion(vx=v,wz=w,reason='replay_recorded_command')
        if abs(sample.vx)<=.01 and abs(sample.vy)<=.01 and abs(sample.wz)<=.02:
            if self.settled is None:self.settled = sample.at
            if sample.at-self.settled >= .5-1e-9:return self.end('reset_complete')
        else:self.settled = None
        return Motion(reason='settling_after_replay')
