import io,os,time,unittest
from unittest.mock import patch
import sender
from live_protocol import Feedback,StopVerifier
from robot_endpoint import EndpointRuntime,legacy_snapshot
import test_live_protocol as protocol_tests
from test_live_protocol import feedback,reply
from test_robot_runtime import Inbox

class ReviewTests(unittest.TestCase):
    def test_both_sender_modes_reach_controlled_failure_without_network(self):
        for mode in ['--check','--execute']:
            log=io.StringIO();err=io.StringIO()
            class Keep(io.StringIO):
                def close(self):pass
            log=Keep();err=Keep()
            with patch.object(sender,'open_local_file',side_effect=[('fake-log',log),('fake-errors',err)]), patch.object(sender,'output',return_value=True), patch.object(sender.subprocess,'Popen',side_effect=OSError('mock SSH unavailable')) as pop, patch.object(sender.UsbInput,'__enter__',side_effect=OSError('mock USB unavailable')):
                self.assertEqual(sender.main([mode]),1)
                self.assertIn('starting',log.getvalue())
                self.assertIn('mock ',log.getvalue())
                self.assertEqual(pop.call_count,1 if mode=='--check' else 0)
    def test_console_full_pipe_is_bounded(self):
        r,w=os.pipe();os.set_blocking(w,False)
        try:
            try:
                while True:os.write(w,b'x'*4096)
            except BlockingIOError:pass
            # New output implementation accepts a descriptor for a real pipe regression.
            start=time.monotonic()
            self.assertFalse(sender.output({'type':'stop'},fd=w))
            self.assertLess(time.monotonic()-start,.1)
        finally:os.close(r);os.close(w)
    def runtime(self):
        channel=Inbox();published=[]
        r=EndpointRuntime('execute','a'*32,0,channel,lambda:None,published.append,lambda:Feedback(0,1,True,0,0,0),lambda:None)
        r.ready=True;r.ready_at=0
        return r,channel,published
    def test_graph_exception_cannot_prevent_zero(self):
        r,c,p=self.runtime()
        def fail():raise ValueError('bad graph')
        r.ownership=fail
        r.tick(.02)
        self.assertEqual(p[-1].vx,0);self.assertIsNotNone(r.stop_at)
        r.tick(.04);self.assertEqual(len(p),2)
    def test_feedback_exception_cannot_prevent_zero(self):
        r,c,p=self.runtime();r.request_stop('test',.02)
        def fail():raise ValueError('bad feedback')
        r.feedback=fail;r.tick(.02)
        self.assertEqual(p[-1].vx,0)
    def test_stop_after_deadline_must_fail(self):
        r,c,p=self.runtime();r.request_stop('test',0)
        for t,seq in [(.02,2),(.08,3),(2.1,4)]:
            r.feedback=lambda t=t,seq=seq:Feedback(t,seq,True,0,0,0)
            r.tick(t)
        self.assertFalse(c.sent[-1]['confirmed']);self.assertEqual(r.exit_code,2)
    def test_feedback_gap_resets_confirmation(self):
        v=StopVerifier(0)
        for t,seq in [(.02,1),(.08,2),(.5,3)]:
            self.assertFalse(v.observe(Feedback(t,seq,True,0,0,0),t))
    def test_reply_does_not_hide_expired_previous_lease(self):
        t=protocol_tests.ProtocolTests();t.setUp();t.arm();t.send(.04,y=-1,l1=1)
        self.assertEqual(t.g.step(.18,feedback(.18,9)).vx,.2)
        p=t.g.poll(.18)
        self.assertTrue(t.g.accept(reply(p,seq=4,y=-1,l1=1),.20))
        self.assertEqual(t.g.step(.201,feedback(.201,10)).vx,0)
    def test_source_feedback_timestamp_rejects_queued_or_unsupported(self):
        from live_protocol import feedback_sample_time
        wall=1_000_000_000_000
        self.assertAlmostEqual(feedback_sample_time(wall-50_000_000,wall-10_000_000,wall,5),4.95)
        for source,received in [(0,wall),(wall-300_000_000,wall),(wall+1,wall),(wall,0)]:
            self.assertIsNone(feedback_sample_time(source,received,wall,5))
    def test_discovery_waits_and_resets_on_conflict(self):
        c=Inbox();p=[];problem=[None];fb=[Feedback(0,1,True,0,0,0)]
        r=EndpointRuntime('execute','a'*32,0,c,lambda:p.append('enabled'),lambda cmd:None,lambda:fb[0],lambda:problem[0])
        r.tick(0);self.assertEqual(p,[])
        problem[0]='other';fb[0]=Feedback(.9,2,True,0,0,0);r.tick(.9)
        problem[0]=None
        for t in [1,1.5,2.01]:fb[0]=Feedback(t,int(t*100),True,0,0,0);r.tick(t)
        self.assertEqual(p,['enabled'])

    def test_jazzy_dictionary_metadata_is_accepted(self):
        from robot_endpoint import state_feedback
        from types import SimpleNamespace as N
        wall=1_000_000_000_000
        message=N(is_soft_auto=False,speed=N(linear=N(x=0,y=0),angular=N(z=0)))
        info={'source_timestamp':wall-50_000_000,'received_timestamp':wall-10_000_000}
        fb=state_feedback(message,info,7,wall,5)
        self.assertAlmostEqual(fb.received_at,4.95)
        self.assertEqual(fb.seq,7);self.assertTrue(fb.manual)
        self.assertIsNone(state_feedback(message,{},8,wall,5))
