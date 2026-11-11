import unittest
from live_protocol import Feedback,SessionGuard,StopVerifier,ownership_problem


def feedback(now=0.0,seq=1,manual=True,vx=0.0,vy=0.0,wz=0.0):
    return Feedback(now,seq,manual,vx,vy,wz)


def reply(poll,seq=1,x=0.0,y=0.0,l1=0,epoch=0,usb_age_s=0.0):
    axes=[0.0]*8;axes[6:8]=[x,y]
    buttons=[0]*13;buttons[4]=l1
    return dict(type="input",session=poll['session'],nonce=poll['nonce'],seq=seq,
                axes=axes,buttons=buttons,connected=True,usb_age_s=usb_age_s,release_epoch=epoch)


class ProtocolTests(unittest.TestCase):
    def setUp(self):self.g=SessionGuard("a"*32);self.seq=0
    def send(self,t,**kw):
        self.seq+=1;p=self.g.poll(t)
        self.assertTrue(self.g.accept(reply(p,seq=self.seq,**kw),t+.001))
        return self.g.step(t+.002,feedback(t+.002,self.seq))
    def arm(self):
        self.send(0)
        self.send(.02,l1=1)
    def test_four_directions_reuse_fixed_mapping(self):
        for x,y,vx,wz in [(0,-1,.2,0),(0,1,-.2,0),(-1,0,0,.2),(1,0,0,-.2)]:
            self.setUp();self.arm();c=self.send(.04,x=x,y=y,l1=1)
            self.assertEqual((c.vx,c.vy,c.wz),(vx,0,wz))
    def test_delayed_poll_reply_is_rejected_and_target_stops(self):
        self.arm();self.send(.04,y=-1,l1=1)
        old=self.g.poll(.06)
        self.assertFalse(self.g.accept(reply(old,seq=99,y=-1,l1=1),1.0))
        self.assertEqual(self.g.step(1.0,feedback(1.0,100)).vx,0)
    def test_nonce_is_single_use_and_seq_monotonic(self):
        p=self.g.poll(0);r=reply(p)
        self.assertTrue(self.g.accept(r,.001))
        self.assertFalse(self.g.accept(r,.002))
        p=self.g.poll(.01)
        self.assertFalse(self.g.accept(reply(p,seq=1),.011))
    def test_wrong_session_and_bad_values_are_rejected(self):
        for patch in [dict(session='b'*32),dict(axes=[0]*7),dict(usb_age_s=-1),dict(seq=True),dict(release_epoch=-1),dict(connected='true')]:
            p=self.g.poll(0);r=reply(p);r.update(patch)
            self.assertFalse(self.g.accept(r,.001))
    def test_total_usb_age_includes_request_round_trip(self):
        self.arm();p=self.g.poll(.04)
        self.assertTrue(self.g.accept(reply(p,seq=3,y=-1,l1=1,usb_age_s=.10),.14))
        self.assertEqual(self.g.step(.141,feedback(.141,3)).vx,0)
    def test_timeout_recovery_cannot_resume_while_l1_held(self):
        self.arm();self.send(.04,y=-1,l1=1)
        self.assertEqual(self.g.step(.3,feedback(.3,3)).vx,0)
        self.assertEqual(self.send(.31,y=-1,l1=1).vx,0)
        self.send(.33,l1=0,epoch=1);self.send(.35,l1=1,epoch=1)
        self.assertEqual(self.send(.37,y=-1,l1=1,epoch=1).vx,.2)
    def test_queued_l1_release_epoch_resets_gate(self):
        self.arm();self.send(.04,y=-1,l1=1)
        self.assertEqual(self.send(.06,y=-1,l1=1,epoch=1).vx,0)
    def test_feedback_age_and_manual_mode_stop_output(self):
        self.arm();self.send(.04,y=-1,l1=1)
        self.assertEqual(self.g.step(.05,feedback(.05,5,manual=False)).vx,0)
        self.assertEqual(self.g.step(.5,feedback(.1,6)).vx,0)
    def test_partial_and_diagonal_input_are_zero(self):
        self.arm()
        for i,(x,y) in enumerate([(0,-.9),(1,1),(-1,-1)]):
            c=self.send(.04+i*.02,x=x,y=y,l1=1)
            self.assertEqual((c.vx,c.vy,c.wz),(0,0,0))
    def test_stop_confirmation_requires_new_distinct_stationary_feedback(self):
        v=StopVerifier(1.0)
        self.assertFalse(v.observe(feedback(.9,1),1.01))
        self.assertFalse(v.observe(feedback(1.01,2),1.01))
        self.assertFalse(v.observe(feedback(1.01,2),1.12))
        self.assertFalse(v.observe(feedback(1.06,3),1.12))
        self.assertTrue(v.observe(feedback(1.12,4),1.12))
    def test_motion_or_stale_feedback_resets_stop_count(self):
        v=StopVerifier(1.0)
        v.observe(feedback(1.01,1),1.01)
        self.assertFalse(v.observe(feedback(1.06,2,vx=.1),1.06))
        self.assertFalse(v.observe(feedback(1.1,3),1.5))
    def test_known_legacy_publisher_does_not_block_but_unknown_does(self):
        self.assertIsNotNone(ownership_problem(['other_controller'],'ours',None))
        self.assertIsNone(ownership_problem(
            ['host_gamepad_control'],'ours',
            dict(armed=True,permission_remaining=30,age_s=.1)))
        self.assertIsNone(ownership_problem(['ours','host_gamepad_control'],'ours',dict(armed=False,permission_remaining=0,age_s=.1)))
        self.assertIsNone(ownership_problem(['host_gamepad_control'],'ours',None))

if __name__=='__main__':unittest.main()
