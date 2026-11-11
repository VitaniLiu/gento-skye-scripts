"""Offline independent vehicle model: gain/lag differ between go and return."""
import math
import unittest
from hybrid import RecordedTrip,HybridFollower
from retrace import Pose
from logic import Sample,wrap

class Vehicle:
    def __init__(self,gain=1.,tau=.15):
        self.x=self.y=self.a=self.v=self.w=0.
        self.gain=gain;self.tau=tau
    def step(self,v,w):
        v=v*self.gain if abs(v)>=.008 else 0.
        w=w if abs(w)>=.02 else 0.
        k=min(1.,.02/self.tau)
        self.v+=(v-self.v)*k;self.w+=(w-self.w)*k
        self.x+=.02*self.v*math.cos(self.a)
        self.y+=.02*self.v*math.sin(self.a)
        self.a=wrap(self.a+.02*self.w)
    def sample(self,t,i):
        return Sample(self.x,self.y,self.a,self.v,0.,self.w,t,i)

def record(route):
    trip=RecordedTrip(None);vehicle=Vehicle();t=0.;i=0
    for v,w,seconds in [(0.,0.,.5)]+route+[(0.,0.,1.)]:
        for _ in range(round(seconds/.02)):
            trip.observe(vehicle.sample(t,i),t,w,commanded_vx=v)
            vehicle.step(v,w);t+=.02;i+=1
    assert trip.fault is None,trip.fault
    return trip,vehicle,t,i

class HybridTests(unittest.TestCase):
    def run_route(self,route,gain=1.08,tau=.15):
        trip,v,t,i=record(route)
        follower=trip.controller(None,t);v.gain=gain;v.tau=tau
        outputs=[]
        for n in range(6000):
            cmd=follower.step(v.sample(t,i),t)
            outputs.append(cmd)
            self.assertLessEqual(abs(cmd.vx),.05)
            self.assertLessEqual(abs(cmd.wz),.1)
            if follower.finished_reason:break
            v.step(cmd.vx,cmd.wz);t+=.02;i+=1
        self.assertEqual(follower.finished_reason,'reset_complete',
                         (v.x,v.y,math.degrees(v.a),follower.index,outputs[-1]))
        self.assertLessEqual(math.hypot(v.x,v.y),.007)
        return trip,v,outputs

    def test_straight_actual_arrival_with_changed_speed_gain(self):
        for gain in (.85,1.08,1.2):
            with self.subTest(gain=gain):
                _,_,out=self.run_route([(.2,0.,2.68)],gain)
                self.assertTrue(all(c.wz==0 for c in out))

    def test_straight_then_turn_reverses_turn_before_driving(self):
        _,_,out=self.run_route([(.2,0.,2.),(0.,0.,.8),(0.,.2,4.),(0.,0.,.5)])
        moving=[c for c in out if c.vx or c.wz]
        self.assertEqual(moving[0].vx,0.)
        self.assertLess(moving[0].wz,0.)
        self.assertLess(moving[-1].vx,0.)
        self.assertEqual(moving[-1].wz,0.)

    def test_L_route_and_initial_pivot(self):
        self.run_route([(0.,.2,3.),(0.,0.,.8),(.2,0.,1.),
                        (0.,0.,.8),(0.,-.2,3.),(0.,0.,.8),(.2,0.,1.)])

    def test_cross_start_retains_excursion(self):
        trip,v,out=self.run_route([(.2,0.,1.),(0.,0.,.8),(-.2,0.,1.5)])
        moving=[c.vx for c in out if c.vx]
        self.assertGreater(moving[0],0.)
        self.assertLess(moving[-1],0.)

    def test_coasting_is_included_in_measured_leg(self):
        trip,_,_,_=record([(.2,0.,1.),(0.,0.,1.),(0.,.2,1.)])
        self.assertGreater(trip.legs[0].points[-1].x,.19)
        self.assertEqual(len(trip.legs),2)

    def test_stationary_and_stale_and_stall_stop(self):
        trip=RecordedTrip(None)
        trip.observe(Sample(0.,0.,0.,0.,0.,0.,0.,0),0.)
        f=trip.controller(None,0.)
        for i in range(30):cmd=f.step(Sample(0.,0.,0.,0.,0.,0.,i*.02,i),i*.02)
        self.assertEqual(f.finished_reason,'reset_complete')
        self.assertEqual((cmd.vx,cmd.wz),(0.,0.))
        trip,v,t,i=record([(.2,0.,1.)])
        f=trip.controller(None,t)
        cmd=f.step(v.sample(t-1.,i),t)
        self.assertEqual(f.finished_reason,'stale_pose')
        f=trip.controller(None,t)
        for j in range(410):
            now=t+j*.02;cmd=f.step(v.sample(now,i+j),now)
            if f.finished_reason:break
        self.assertEqual(f.finished_reason,'no_progress')
        self.assertEqual((cmd.vx,cmd.wz),(0.,0.))

if __name__=='__main__':unittest.main()
