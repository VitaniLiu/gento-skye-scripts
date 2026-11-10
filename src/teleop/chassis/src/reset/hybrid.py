"""Recorded command order with measured-pose completion. No I/O or ROS."""
from dataclasses import dataclass, field
import math
from logic import Motion, Sample, wrap, clip, finite
from retrace import Pose, sample_problem

@dataclass
class Leg:
    vx: float
    wz: float
    points: list = field(default_factory=list)

class RecordedTrip:
    def __init__(self, target):
        self.origin=None
        self.previous=None
        self.fault=None
        self.legs=[]
        self.last_intent=None
        self.count=0

    def observe(self, sample, now, commanded_wz=0., commanded_vx=0.):
        if self.fault:return
        self.fault=sample_problem(sample,self.previous,now)
        if self.fault:return
        if not all(finite(v) for v in (commanded_vx,commanded_wz)):
            self.fault='invalid_recorded_command';return
        if abs(commanded_vx)>.2 or abs(commanded_wz)>.2:
            self.fault='invalid_recorded_command';return
        pose=Pose(sample.x,sample.y,sample.yaw)
        if self.origin is None:
            if abs(sample.vx)>.01 or abs(sample.vy)>.01 or abs(sample.wz)>.02:
                self.fault='recording_did_not_start_stationary';return
            self.origin=pose
        if pose.distance(self.origin)>1.:
            self.fault='target_out_of_range';return
        # The new pose is the result of the PREVIOUS command. Zero-command
        # coast remains part of that same leg, up to the next moving command.
        if self.legs:
            points=self.legs[-1].points
            if len(points)==1:points.append(pose);self.count+=1
            elif (pose.distance(points[-2])>=.025 or
                  abs(wrap(pose.yaw-points[-2].yaw))>=.05):
                points.append(pose);self.count+=1
            else:points[-1]=pose
        intent=(commanded_vx,commanded_wz)
        if any(intent) and intent!=self.last_intent:
            self.legs.append(Leg(*intent,[pose]))
            self.last_intent=intent
            self.count+=1
        if self.count>4000:self.fault='path_capacity_exceeded'
        self.previous=sample

    def controller(self,target,now):
        if self.fault or self.origin is None:
            raise ValueError(self.fault or 'path_missing')
        return HybridFollower(self.origin,self.legs,now)

class HybridFollower:
    """Reverse recorded intent; change legs only after measured arrival."""
    phase='RETRACE'
    def __init__(self,origin,legs,now):
        self.origin=origin
        # Drop zero-length input taps; retain actual initial rotations.
        self.legs=[Leg(l.vx,l.wz,list(l.points)) for l in reversed(legs)
                   if len(l.points)>1 and
                   (l.points[0].distance(l.points[-1])>.001 or
                    abs(wrap(l.points[-1].yaw-l.points[0].yaw))>.003)]
        self.index=0
        self.point_index=len(self.legs[0].points)-2 if self.legs else 0
        self.previous=None
        self.started=self.tick=self.progress_at=now
        self.best=math.inf
        self.settled=None
        self.finished_reason=None

    def end(self,reason):
        self.finished_reason=reason
        return Motion(reason=reason)

    def advance(self,now):
        self.index+=1
        self.settled=None
        self.best=math.inf;self.progress_at=now
        if self.index<len(self.legs):
            self.point_index=len(self.legs[self.index].points)-2
        return Motion(reason='next_measured_leg')

    def step(self,s,now):
        if self.finished_reason:return Motion(reason=self.finished_reason)
        problem=sample_problem(s,self.previous,now)
        if problem or not 0<=now-self.tick<=.100001 or now-self.started>=120:
            return self.end(problem or 'retrace_timing_limit')
        self.previous,self.tick=s,now
        stopped=abs(s.vx)<=.01 and abs(s.vy)<=.01 and abs(s.wz)<=.02
        if self.index>=len(self.legs):
            if stopped:
                if self.settled is None:self.settled=s.at
                if s.at-self.settled>=.5-1e-9:return self.end('reset_complete')
            else:self.settled=None
            return Motion(reason='settling_at_start')
        leg=self.legs[self.index];goal=leg.points[0]
        pose=Pose(s.x,s.y,s.yaw)
        distance=pose.distance(goal)
        yaw_error=wrap(goal.yaw-s.yaw)
        pivot=abs(leg.vx)<1e-9
        metric=abs(yaw_error) if pivot else distance
        if metric<self.best-(.001 if pivot else .0005):
            self.best=metric;self.progress_at=now
        if now-self.progress_at>=8:return self.end('no_progress')
        # A rotation leg stays where it was measured; never translate to
        # compensate for rotary odometry drift.
        if pivot:
            if pose.distance(leg.points[-1])>.035:
                return self.end('left_recorded_path')
            arrived=abs(yaw_error)<=math.radians(.4)
        else:
            # Preserve the measured path, including an excursion crossing start.
            def segment_distance(a,b):
                dx=b.x-a.x;dy=b.y-a.y;d=dx*dx+dy*dy
                u=clip(((s.x-a.x)*dx+(s.y-a.y)*dy)/d,1.) if d else 0.
                u=max(0.,u)
                return math.hypot(s.x-a.x-u*dx,s.y-a.y-u*dy)
            i=self.point_index
            # Keep local ordering; do not jump to an earlier crossing.
            corridor=min(segment_distance(leg.points[j],leg.points[j+1])
                         for j in range(max(0,i-1),min(len(leg.points)-1,i+2)))
            if corridor>.035:return self.end('left_recorded_path')
            while self.point_index>0 and pose.distance(leg.points[self.point_index])<=.018:
                self.point_index-=1
            arrived=distance<=.006 and self.point_index==0
        if arrived:
            if stopped:
                if self.settled is None:self.settled=s.at
                if s.at-self.settled>=.3-1e-9:return self.advance(now)
            else:self.settled=None
            return Motion(reason='settling_measured_leg')
        self.settled=None
        if pivot:
            # The old tiny proportional yaw command stalled on the real base.
            # Use a bounded nonzero command above the observed dead zone.
            predicted=yaw_error-s.wz*.12
            if yaw_error*predicted<=0:return Motion(reason='braking_recorded_turn')
            w=math.copysign(min(.1,max(.03,1.2*abs(predicted))),yaw_error)
            return Motion(wz=w,reason='reverse_recorded_turn')
        sign=-1. if leg.vx>0 else 1.
        # Follow the next 10 cm of this same leg rather than chasing a noisy
        # individual point. Waypoint progress is still strictly in reverse.
        carrot=leg.points[self.point_index]
        ahead=pose.distance(carrot)
        j=self.point_index
        while j>0 and ahead<.1:
            ahead+=carrot.distance(leg.points[j-1]);j-=1
            carrot=leg.points[j]
        bearing=math.atan2(carrot.y-s.y,carrot.x-s.x)
        heading=wrap(bearing+(math.pi if sign<0 else 0.)-s.yaw)
        if not leg.wz and distance<.12:
            dx=goal.x-s.x;dy=goal.y-s.y
            cross=-dx*math.sin(goal.yaw)+dy*math.cos(goal.yaw)
            fade=max(0.,(distance-.04)/.08)
            desired=goal.yaw+sign*math.atan2(cross,.1)*fade
            heading=wrap(desired-s.yaw)
        # Recorded direction is authoritative. Never turn in place during
        # a recorded drive or drive away chasing a waypoint already passed.
        if abs(heading)>math.pi/2:return self.end('recorded_direction_mismatch')
        speed=min(.05,max(.01,.7*distance))
        if abs(heading)>.35:speed=min(speed,.02)
        turn=clip(1.2*heading,.1 if leg.wz else .05)
        if abs(turn)<1e-9:turn=0.
        if not leg.wz and distance<.12 and abs(heading)>.005:
            turn=math.copysign(max(.03,abs(turn)),turn)
        elif not leg.wz and distance<.12:
            turn=0.
        # Finish an originally straight leg with longitudinal wheel command.
        if not leg.wz and distance<.04:turn=0.
        return Motion(vx=sign*speed,wz=turn,reason='reverse_measured_drive')
