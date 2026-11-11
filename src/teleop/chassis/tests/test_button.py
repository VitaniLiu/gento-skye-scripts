import unittest
from button import ButtonReturnGuard
from logic import Sample
from live_protocol import Feedback


class ButtonTests(unittest.TestCase):
    def setUp(self):
        self.events=[];self.now=0;self.x=.8;self.seq=0
        self.g=ButtonReturnGuard('a'*32,(0,0,1),self.sample,lambda:None,self.events.append)
    def sample(self):return Sample(self.x,0,0,0,0,0,self.now,self.seq)
    def send(self,t,a=0,l1=0,dpad=0,connected=True):
        self.seq+=1;self.now=t+.002
        p=self.g.poll(t);buttons=[0]*13;buttons[0]=a;buttons[4]=l1
        axes=[0.]*8;axes[7]=dpad
        msg=dict(type='input',session='a'*32,nonce=p['nonce'],seq=self.seq,
                 axes=axes,buttons=buttons,connected=connected,usb_age_s=0,release_epoch=0)
        self.assertTrue(self.g.accept(msg,t+.001))
        return self.g.step(self.now,Feedback(self.now,self.seq,True,0,0,0))
    def start(self):self.send(0);return self.send(.02,a=1)
    def test_single_A_starts_existing_straight_controller(self):
        out=self.start()
        self.assertLess(out.vx,0);self.assertEqual((out.vy,out.wz),(0,0))
        self.assertEqual(self.events[-1]['type'],'reset_started')
    def test_A_held_on_startup_does_not_start(self):
        self.assertEqual(self.send(0,a=1).vx,0)
        self.assertIsNone(self.g.control)
        self.send(.02,a=0)
        self.assertLess(self.send(.04,a=1).vx,0)
    def test_holding_A_does_not_cancel_or_retrigger(self):
        self.start()
        for t in [.04,.06,.08]:self.assertLess(self.send(t,a=1).vx,0)
        self.assertEqual(len(self.events),1);self.assertIsNone(self.g.finished_reason)
    def test_second_A_press_cancels_with_zero(self):
        self.start();self.send(.04,a=0)
        out=self.send(.06,a=1)
        self.assertEqual((out.vx,out.vy,out.wz),(0,0,0))
        self.assertEqual(self.g.finished_reason,'reset_cancelled_by_operator')
    def test_L1_cancels_and_does_not_mix_manual_output(self):
        self.start();out=self.send(.04,l1=1,dpad=-1)
        self.assertEqual((out.vx,out.wz),(0,0))
        self.assertEqual(self.g.finished_reason,'reset_cancelled_by_operator')
    def test_manual_dpad_mapping_before_A_is_unchanged(self):
        self.send(0);self.send(.02,l1=1)
        self.assertEqual(self.send(.04,l1=1,dpad=-1).vx,.2)
    def test_validation_failure_rejects_A_without_disabling_manual(self):
        def locked():raise ValueError('not_verified: independent_stop_verified')
        self.g.verify=locked
        self.assertEqual(self.start().vx,0);self.assertIsNone(self.g.control)
        self.assertEqual(self.events[-1]['type'],'reset_rejected')
        self.assertIn('independent_stop_verified',self.events[-1]['reason'])
        self.send(.04,l1=1);self.assertEqual(self.send(.06,l1=1,dpad=-1).vx,.2)
    def test_no_A_trigger_while_manual_controls_active(self):
        self.send(0);out=self.send(.02,a=1,l1=1,dpad=-1)
        self.assertEqual(out.vx,0);self.assertIsNone(self.g.control)
    def test_disconnection_cancels_and_does_not_resume(self):
        self.start();self.assertEqual(self.send(.04,connected=False).vx,0)
        self.assertEqual(self.g.finished_reason,'reset_input_lost')
        self.assertEqual(self.send(.06).vx,0)
    def test_lease_loss_stops_before_recovery(self):
        self.start();self.now=.3
        out=self.g.step(.3,Feedback(.3,100,True,0,0,0))
        self.assertEqual(out.vx,0);self.assertEqual(self.g.finished_reason,'reset_input_lost')
    def test_software_mode_loss_stops(self):
        self.start();self.now=.04
        out=self.g.step(.04,Feedback(.04,100,False,0,0,0))
        self.assertEqual(out.vx,0);self.assertEqual(self.g.finished_reason,'reset_feedback_lost')
    def test_completion_does_not_restore_heading(self):
        self.x=.01;self.start()
        for i in range(2,35):
            out=self.send(i*.02)
            self.assertEqual((out.vy,out.wz),(0,0))
        self.assertEqual(self.g.finished_reason,'reset_complete')
    def test_event_channel_failure_cannot_start_motion(self):
        self.g.notify=lambda row:False
        out=self.start()
        self.assertEqual(out.vx,0);self.assertEqual(self.g.finished_reason,'output_channel_failed')

if __name__=='__main__':unittest.main()
