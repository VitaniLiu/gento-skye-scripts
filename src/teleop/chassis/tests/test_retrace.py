import hashlib
import json
import math
from pathlib import Path
import sys
import unittest

from button import ButtonReturnGuard
from logic import Sample, validate_reference, wrap, REFERENCE_FILE
from live_protocol import Feedback

class Replay:
    def __init__(self, origin, verify=lambda: None):
        self.t = 0.
        self.n = 0
        self.events = []
        self.guard = ButtonReturnGuard('a'*32, origin, lambda: self.sample, verify,
                                      self.events.append, control_mode='retrace')

    def frame(self, t, pose, a=0, velocity=(0.,0.,0.), epoch=0, l1=0, x=0., y=0.):
        self.t, self.n = t, self.n+1
        self.sample = Sample(*pose, *velocity, t, self.n)
        poll = self.guard.poll(t)
        buttons = [a]+[0]*12
        buttons[4] = l1
        axes = [0.]*6+[x,y]
        accepted = self.guard.accept(dict(type='input', session='a'*32, nonce=poll['nonce'],
            seq=self.n, axes=axes, buttons=buttons, connected=True, usb_age_s=0.,
            release_epoch=epoch), t+.001)
        assert accepted
        return self.guard.step(t+.002, Feedback(t, self.n, True, *velocity))

    def drive_model(self, pose, epoch=0):
        t = self.t + .02
        self.frame(t,pose,epoch=epoch)
        t += .02
        self.frame(t,pose,epoch=epoch)
        t += .02
        cmd = self.frame(t,pose,a=1,epoch=epoch)
        assert self.events[-1]['type']=='reset_started', self.events
        x,y,yaw = pose
        path = []
        for i in range(6001):
            assert abs(cmd.vx)<=.05 and cmd.vy==0 and abs(cmd.wz)<=.1
            if self.guard.finished_reason:
                assert self.guard.finished_reason=='reset_complete', self.guard.finished_reason
                assert cmd.vx==cmd.wz==0
                return (x,y,yaw),path
            path.append((x,y,yaw,cmd.vx,cmd.wz))
            x += .02 * cmd.vx * math.cos(yaw)
            y += .02 * cmd.vx * math.sin(yaw)
            yaw = wrap(yaw+.02*cmd.wz)
            t += .02
            cmd = self.frame(t,(x,y,yaw),velocity=(cmd.vx,0.,cmd.wz),epoch=epoch)
        raise AssertionError('no completion')

class RetraceTests(unittest.TestCase):
    def test_straight_recorded_segment_uses_bounded_moving_correction(self):
        from retrace import Pose, ReverseFollower

        # Measured geometry immediately before the unexpected rotation in
        # live-20261102-121126: the outbound command had wz=0 throughout.
        points = [
            Pose(.344475, .003343, -.026180),
            Pose(.318199, .004051, -.026180),
        ]
        f = ReverseFollower(points, 0.)
        sample = Sample(.312097, .001140, -.027926, 0., 0., 0., 0., 1)
        cmd = f.step(sample, 0.)
        self.assertGreater(cmd.vx, 0.)
        self.assertTrue(0. < abs(cmd.wz) <= .02)

    def test_intermediate_waypoint_accepts_real_nine_mm_lateral_miss(self):
        from retrace import Pose, ReverseFollower

        origin = Pose(-.018607, .012256, .012217)
        waypoint = Pose(.602856, .020771, .012217)
        end = Pose(.630155, .021106, .012217)
        f = ReverseFollower([origin, waypoint, end], 0.)
        first = Sample(end.x, end.y, end.yaw, 0., 0., 0., 0., 1)
        self.assertEqual(f.step(first, 0.).reason, 'follow_recorded_segment')
        sample = Sample(waypoint.x, .011493, .005235, 0., 0., 0., .05, 2)
        cmd = f.step(sample, .05)
        self.assertEqual(cmd.reason, 'next_recorded_point')
        self.assertEqual(f.index, 1)

    def test_short_recorded_turn_rotates_without_translation(self):
        from retrace import Pose, ReverseFollower

        goal = Pose(.60, .02, .01)
        end = Pose(.615, .03, .09, turning=True)
        f = ReverseFollower([Pose(0., 0., 0.), goal, end], 0.)
        cmd = f.step(Sample(end.x, end.y, end.yaw, 0., 0., 0., 0., 1), 0.)
        self.assertEqual(cmd.vx, 0.)
        self.assertNotEqual(cmd.wz, 0.)
        cmd = f.step(Sample(end.x, end.y, goal.yaw, 0., 0., 0., .05, 2), .05)
        self.assertEqual(cmd.reason, 'next_recorded_point')

    def test_passed_intermediate_straight_point_advances_inside_corridor(self):
        from retrace import Pose, ReverseFollower

        origin = Pose(0., 0., 0.)
        goal = Pose(.10, 0., 0.)
        end = Pose(.13, 0., 0.)
        f = ReverseFollower([origin, goal, end], 0.)
        f.step(Sample(end.x, end.y, 0., 0., 0., 0., 0., 1), 0.)
        cmd = f.step(Sample(.08, .01, 0., 0., 0., 0., .08, 2), .08)
        self.assertEqual(cmd.reason, 'next_recorded_point')
        self.assertEqual(f.index, 1)

    def test_turn_metadata_uses_manual_command_not_feedback_noise(self):
        from retrace import RecordedPath

        path = RecordedPath(None)
        path.observe(Sample(0., 0., 0., 0., 0., 0., 0., 1), 0., commanded_wz=0.)
        path.observe(Sample(.03, 0., 0., 0., 0., -.05, .1, 2), .1,
                     commanded_wz=0.)
        self.assertFalse(path.points[-1].turning)

        path.observe(Sample(.06, 0., .05, 0., 0., 0., .2, 3), .2,
                     commanded_wz=.1)
        self.assertTrue(path.points[-1].turning)

    def test_final_pose_restores_yaw_then_finishes_straight(self):
        from retrace import Pose, ReverseFollower
        f = ReverseFollower([Pose(0,0,0),Pose(.03,0,0)],0)
        x,y,yaw = .003,0.,.04
        vx=wz=0.
        moving=[]
        for i in range(200):
            t = i*.02
            cmd=f.step(Sample(x,y,yaw,vx,0.,wz,t,i),t)
            if abs(cmd.vx)+abs(cmd.wz)>1e-9:moving.append((cmd.vx,cmd.wz))
            if f.finished_reason:break
            vx,wz=cmd.vx,cmd.wz
            x += .02*vx*math.cos(yaw)
            y += .02*vx*math.sin(yaw)
            yaw=wrap(yaw+.02*wz)
        self.assertEqual(f.finished_reason,'reset_complete')
        self.assertLessEqual(math.hypot(x,y),.007)
        self.assertLessEqual(abs(yaw),math.radians(.5))
        self.assertTrue(moving)
        self.assertNotEqual(moving[-1][0],0.)
        self.assertEqual(moving[-1][1],0.)

    def test_terminal_yaw_stall_stops_after_eight_seconds(self):
        from retrace import Pose, ReverseFollower
        f=ReverseFollower([Pose(0,0,0),Pose(.03,0,0)],0.)
        for i in range(406):
            t=i*.02
            cmd=f.step(Sample(.003,0,.04,0.,0.,0.,t,i),t)
            if f.finished_reason:break
        self.assertEqual(f.finished_reason,'no_progress')
        self.assertEqual((cmd.vx,cmd.wz),(0.,0.))

    def test_terminal_lateral_drift_realigns_before_reacquiring_path(self):
        from retrace import Pose, ReverseFollower
        points=[Pose(0,0,0),Pose(0,0,math.pi/2),
                Pose(0,.03,math.pi/2)]
        f=ReverseFollower(points,0.)
        x,y,yaw=0.,.03,math.pi/2
        vx=wz=0.
        shifted=False
        reasons=[]
        for i in range(6000):
            t=i*.02
            cmd=f.step(Sample(x,y,yaw,vx,0.,wz,t,i),t)
            reasons.append(cmd.reason)
            if cmd.reason=='settling_at_start' and not shifted:
                y += .0003
                shifted=True
            if f.finished_reason:break
            vx,wz=cmd.vx,cmd.wz
            x += .02*vx*math.cos(yaw)
            y += .02*vx*math.sin(yaw)
            yaw=wrap(yaw+.02*wz)
        self.assertTrue(shifted)
        self.assertEqual(f.finished_reason,'reset_complete')
        self.assertIn('restore_path_heading',reasons)
        self.assertLessEqual(math.hypot(x,y),.007)
        self.assertLessEqual(abs(yaw),math.radians(.5))

    def test_final_validation_requires_start_heading(self):
        from logic import target_problem
        aligned=Sample(.006,0,math.radians(.4),0.,0.,0.,0.,1)
        self.assertIsNone(target_problem(aligned,(0,0,0),'retrace'))
        yaw_drift=Sample(.006,0,math.radians(.6),0.,0.,0.,0.,2)
        self.assertEqual(target_problem(yaw_drift,(0,0,0),'retrace'),
                         'heading_outside_tolerance')
        position_drift=Sample(.012,0,0.,0.,0.,0.,0.,3)
        self.assertEqual(target_problem(position_drift,(0,0,0),'retrace'),
                         'position_outside_tolerance')

    def test_real_verification_stays_locked(self):
        from reset import verification
        r = Replay((0,0,0),lambda: verification("retrace"))
        for i in range(31):
            r.frame(i*.04,(i*.004,0,0))
        cmd = r.frame(1.24,(.12,0,0),a=1)
        self.assertEqual((cmd.vx,cmd.wz),(0.,0.))
        self.assertEqual(r.events[-1]['type'],'reset_rejected')
        self.assertIn('not_verified',r.events[-1]['reason'])

    def test_session_starts_at_current_stationary_pose(self):
        r=Replay((0,0,0))
        start=(.3,-.2,.4)
        r.frame(0,start)
        self.assertEqual(r.guard.path.origin.x,start[0])
        self.assertEqual(r.guard.path.origin.y,start[1])
        self.assertEqual(r.guard.path.origin.yaw,start[2])

    def test_gap_rejects(self):
        r=Replay((0,0,0))
        start=(.3,0,0)
        r.frame(0,start)
        r.frame(.3,start)
        r.frame(.32,start)
        cmd=r.frame(.34,start,a=1)
        self.assertEqual((cmd.vx,cmd.wz),(0,0))
        self.assertEqual(r.events[-1]['type'],'reset_rejected')

    def test_second_press_cancels_retrace(self):
        r=Replay((0,0,0))
        r.frame(0.,(0,0,0))
        r.frame(.02,(0,0,0),l1=1)
        cmd=r.frame(.04,(0,0,0),l1=1,y=-1.)
        self.assertGreater(cmd.vx,0.)
        r.frame(.06,(.004,0,0),l1=1,y=-1.)
        r.frame(.08,(.008,0,0),epoch=1)
        r.frame(.10,(.008,0,0),epoch=1)
        cmd=r.frame(.12,(.008,0,0),a=1,epoch=1)
        self.assertEqual(cmd.reason,'reverse_measured_drive')
        self.assertLess(cmd.vx,0.)
        self.assertGreaterEqual(cmd.vx,-.05)
        self.assertEqual(cmd.wz,0.)
        r.frame(.14,(.008,0,0),epoch=1)
        cmd=r.frame(.16,(.008,0,0),a=1,epoch=1)
        self.assertEqual((cmd.vx,cmd.wz),(0,0))
        self.assertEqual(r.guard.finished_reason,'reset_cancelled_by_operator')

def replay_log(filename):
    root=Path(__file__).resolve().parent.parent
    ref=validate_reference((root/'reference'/REFERENCE_FILE).read_bytes())
    data=Path(filename).read_bytes()
    assert hashlib.sha256(data).hexdigest()==ref['capture_evidence']['sha256']
    keys=('x_m','y_m','yaw_rad')
    origin=tuple(ref['pose'][k] for k in keys)
    r=Replay(origin)
    count=0
    for line in data.splitlines():
        row=json.loads(line)
        if row['type']!='pose_sample': continue
        p=tuple(row['pose'][k] for k in keys)
        v=tuple(row['feedback'][k] for k in ('vx_mps','vy_mps','wz_radps'))
        r.frame(row['robot_monotonic_s'],p,velocity=v)
        count+=1
    end,path=r.drive_model(p)
    error=math.dist(end[:2],origin[:2])
    assert error <= .007
    print(f'{count} samples; {len(r.guard.control.points)} points; model return {len(path)*.02:.2f}s; residual {error*100:.2f}cm')

if __name__=='__main__':
    if len(sys.argv)==2 and sys.argv[1].endswith('.jsonl'):
        replay_log(sys.argv[1])
    else:
        unittest.main()
