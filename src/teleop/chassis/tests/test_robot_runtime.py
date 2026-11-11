import unittest
from live_protocol import Feedback
from robot_endpoint import EndpointRuntime

class Inbox:
    def __init__(self):self.messages=[];self.sent=[];self.eof=False
    def receive(self):values=self.messages;self.messages=[];return values
    def send(self,value):self.sent.append(value)

class RuntimeTests(unittest.TestCase):
    def setup_runtime(self,mode='execute'):
        self.channel=Inbox();self.published=[];self.enabled=[]
        self.fb=Feedback(0,1,True,0,0,0);self.problem=None
        self.r=EndpointRuntime(mode,'a'*32,0,self.channel,lambda:self.enabled.append(True),self.published.append,lambda:self.fb,lambda:self.problem,discovery_s=0)
    def test_check_never_enables_or_publishes(self):
        self.setup_runtime('check');self.r.tick(0)
        self.assertTrue(self.r.finished)
        self.assertEqual(self.r.exit_code,0)
        self.assertEqual(self.enabled,[]);self.assertEqual(self.published,[])
        self.assertEqual(self.channel.sent[-1]['type'],'checked')
    def test_execute_requires_preflight_and_publishes_only_zero_without_input(self):
        self.setup_runtime();self.r.tick(0);self.r.tick(.02)
        self.assertEqual(len(self.enabled),1)
        self.assertTrue(self.published)
        self.assertTrue(all(c.vx==c.vy==c.wz==0 for c in self.published))
    def test_conflicting_publisher_blocks_preflight_without_zero_or_motion(self):
        self.setup_runtime();self.problem='other_controller'
        self.r.tick(0);self.r.tick(5.1)
        self.assertTrue(self.r.finished)
        self.assertEqual(self.r.exit_code,1)
        self.assertEqual(self.published,[]);self.assertEqual(self.enabled,[])
    def test_eof_requests_zero_and_confirms_only_fresh_stopped_feedback(self):
        self.setup_runtime();self.r.tick(0)
        self.channel.eof=True;self.r.tick(.02)
        for t,seq in [(.03,2),(.08,3),(.15,4)]:
            self.fb=Feedback(t,seq,True,0,0,0);self.r.tick(t)
        self.assertTrue(self.r.finished)
        self.assertEqual(self.channel.sent[-1]['type'],'stopped')
        self.assertTrue(self.channel.sent[-1]['confirmed'])
        self.assertTrue(all(c.vx==c.vy==c.wz==0 for c in self.published))
    def test_stop_without_fresh_feedback_is_not_reported_success(self):
        self.setup_runtime();self.r.tick(0)
        self.r.request_stop('operator_stop',.02)
        self.fb=Feedback(.03,2,True,.1,0,0);self.r.tick(.03);self.r.tick(2.03)
        self.assertTrue(self.r.finished)
        self.assertEqual(self.r.exit_code,2)
        self.assertFalse(self.channel.sent[-1]['confirmed'])
    def test_ownership_change_during_execution_requests_stop(self):
        self.setup_runtime();self.r.tick(0)
        self.problem='legacy_controller_still_armed';self.r.tick(.02)
        self.assertTrue(self.published)
        self.assertEqual(self.published[-1].vx,0)
        self.assertEqual(self.published[-1].reason,'stopping')

if __name__=='__main__':unittest.main()
