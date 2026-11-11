"""Offline command reversal regression tests."""
import unittest
import math
from logic import Sample
from retrace import RecordedPath

class CommandReverseTests(unittest.TestCase):
    def sample(self,t):
        return Sample(0.,0.,0.,0.,0.,0.,t,round(t*1000))

    def test_reverse_order_sign_duration_and_no_terminal_correction(self):
        path=RecordedPath(None)
        for t,v,w in [(0.,0.,0.),(.1,.2,0.),(.2,0.,.2),(.3,0.,0.),(.4,0.,0.)]:
            path.observe(self.sample(t),t,commanded_wz=w,commanded_vx=v)
        player=path.controller(None,1.)
        rows=[]
        for i in range(80):
            t=1.+i*.02
            cmd=player.step(self.sample(t),t)
            rows.append(cmd)
            if player.finished_reason:break
        moving=[c for c in rows if c.vx or c.wz]
        self.assertEqual(moving[0].reason,'replay_recorded_command')
        self.assertAlmostEqual(moving[0].wz,-.05)
        self.assertEqual(moving[0].vx,0.)
        self.assertAlmostEqual(moving[-1].vx,-.05)
        self.assertEqual(moving[-1].wz,0.)
        self.assertEqual(len(moving),40)
        self.assertEqual(player.finished_reason,'reset_complete')
        self.assertTrue(all(abs(c.vx)<=.05 and abs(c.wz)<=.1 for c in rows))
        self.assertTrue(all(c.reason in ('replay_recorded_command','settling_after_replay','reset_complete') for c in rows))

    def test_straight_reverse_never_adds_yaw(self):
        path=RecordedPath(None)
        for i in range(11):
            t=i*.02
            path.observe(self.sample(t),t,commanded_vx=.2 if i<10 else 0.)
        player=path.controller(None,1.)
        cmd=player.step(Sample(0.,0.,.04,0.,0.,0.,1.,1000),1.)
        self.assertAlmostEqual(cmd.vx,-.05)
        self.assertEqual(cmd.wz,0.)


    def test_forward_then_reverse_model_returns_to_start(self):
        from retrace import ReverseCommandPlayer
        def integrate(p,v,w,d):
            x,y,a=p
            if abs(w)>1e-12:
                x+=v/w*(math.sin(a+w*d)-math.sin(a))
                y+=v/w*(math.cos(a)-math.cos(a+w*d))
            else:
                x+=v*d*math.cos(a);y+=v*d*math.sin(a)
            return x,y,a+w*d
        routes=[
            [(.2,0.,1.)],
            [(.2,0.,1.),(0.,.2,2.),(.2,0.,1.)],
            [(0.,.2,2.),(.2,0.,1.)],
            [(.2,0.,1.),(-.2,0.,1.5)],
            [(.2,.2,2.)],
        ]
        for route in routes:
            with self.subTest(route=route):
                p=(0.,0.,0.)
                for v,w,d in route:p=integrate(p,v,w,d)
                player=ReverseCommandPlayer(route,0.)
                for i in range(6000):
                    t=i*.02
                    cmd=player.step(Sample(*p,0.,0.,0.,t,i),t)
                    if player.finished_reason:break
                    p=integrate(p,cmd.vx,cmd.wz,.02)
                self.assertEqual(player.finished_reason,'reset_complete')
                self.assertLess(math.hypot(*p[:2]),1e-9)
                self.assertLess(abs(p[2]),1e-9)

    def test_stale_feedback_stops_replay(self):
        from retrace import ReverseCommandPlayer
        player=ReverseCommandPlayer([(.2,0.,1.)],0.)
        cmd=player.step(self.sample(-1.),0.)
        self.assertEqual((cmd.vx,cmd.wz),(0.,0.))
        self.assertEqual(player.finished_reason,'stale_pose')

if __name__=='__main__':unittest.main()
