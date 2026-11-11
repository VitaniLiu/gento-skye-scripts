import io,json,unittest
from pathlib import Path
from route_recording import RouteRecorder
from live_protocol import Feedback
import test_robot_runtime as runtime_tests
import test_reset as reset_tests

class MemoryLog(io.StringIO):
    def close(self):self.saved=self.getvalue();super().close()

class RouteRecordingTests(unittest.TestCase):
    def setUp(self):
        self.events=[];self.files=[]
        def opened():
            stream=MemoryLog();self.files.append(stream)
            return Path('/tmp/route-'+str(len(self.files))+'.jsonl'),stream
        self.recorder=RouteRecorder(self.events.append,opened)
    def target(self,t,v=0.,actual=0.,x=0.,phase='PREFLIGHT'):
        self.recorder.observe(dict(type='target',phase=phase,reason='enabled' if v else 'neutral',
            vx=v,vy=0.,wz=0.,session='a'*32,robot_monotonic_s=t,
            pose=dict(x=x,y=0.,yaw=0.,vx=actual,vy=0.,wz=0.)))
    def test_coasting_is_recorded_before_route_is_closed(self):
        self.target(0.,.2,.2)
        self.target(.02,0.,.1,x=.004)
        self.target(.20,0.,0.,x=.014)
        self.assertEqual(len(self.events),1)
        self.target(.72,0.,0.,x=.014)
        self.assertEqual(self.events[-1]['type'],'manual_route_finished')
        self.assertTrue(self.events[-1]['stationary_confirmed'])
        self.assertEqual(self.events[-1]['end_pose']['x'],.014)
    def test_waiting_does_not_exit_and_next_motion_starts_another_route(self):
        self.target(0.,.2,.2)
        self.target(.1,0.,0.)
        self.target(.6,0.,0.)
        self.target(40000.,0.,0.)
        self.target(40000.02,.2,.1)
        self.assertEqual(len(self.files),2)
        self.assertEqual(self.recorder.number,2)
    def test_A_return_is_not_added_to_manual_route(self):
        self.target(0.,.2,.2)
        self.target(.1,0.,0.)
        self.recorder.observe(dict(type='reset_started'))
        self.target(.2,-.05,-.05,phase='RETRACE')
        self.assertEqual(len(self.files),1)
        self.assertIsNone(self.recorder.stream)
    def test_close_preserves_partial_route_and_reports_unconfirmed(self):
        self.target(0.,.2,.2)
        self.recorder.close(stop_confirmed=False)
        self.assertFalse(self.events[-1]['stationary_confirmed'])
        self.assertIn('session_closed',self.files[0].saved)

class ContinuousRuntimeTests(unittest.TestCase):
    def test_default_session_does_not_expire_after_ten_hours(self):
        helper=runtime_tests.RuntimeTests();helper.setup_runtime()
        helper.r.tick(0)
        helper.fb=Feedback(36000.,2,True,0.,0.,0.)
        helper.r.tick(36000.)
        self.assertIsNone(helper.r.stop_at)
        self.assertFalse(helper.r.finished)
    def test_requested_finite_limit_is_still_supported(self):
        helper=runtime_tests.RuntimeTests();helper.setup_runtime()
        helper.r.max_session_s=10.
        helper.r.tick(0)
        helper.fb=Feedback(10.,2,True,0.,0.,0.)
        helper.r.tick(10.)
        self.assertEqual(helper.r.stop_reason,'session_expired')
    def test_successful_A_return_confirms_stop_then_keeps_session_open(self):
        entry=reset_tests.EntryTests().entry()
        helper=runtime_tests.RuntimeTests();helper.setup_runtime()
        r=entry.CompletionRuntime('execute','a'*32,0.,helper.channel,
            lambda:None,helper.published.append,lambda:helper.fb,lambda:None,
            discovery_s=0,final_check=lambda now:None,continuous=True)
        class Guard:
            finished_reason='reset_complete'
            def poll(self,now):return dict(type='poll',session='a'*32,nonce='b'*16)
            def restart_after_return(self,now):self.finished_reason=None;self.restarted=now
        r.guard=Guard();r.ready=True;r.ready_at=0.
        r.request_stop('reset_complete',0.)
        for i,t in enumerate([.02,.06,.12,.18]):
            helper.fb=Feedback(t,i+2,True,0.,0.,0.)
            r.stopping(t)
            if r.stop_at is None:break
        self.assertFalse(r.finished)
        self.assertIsNone(r.stop_at)
        self.assertIsNone(r.guard.finished_reason)
        self.assertEqual(helper.channel.sent[-1]['type'],'reset_completed')
        self.assertTrue(helper.channel.sent[-1]['confirmed'])
    def test_invalid_return_still_stops_the_session(self):
        entry=reset_tests.EntryTests().entry()
        helper=runtime_tests.RuntimeTests();helper.setup_runtime()
        r=entry.CompletionRuntime('execute','a'*32,0.,helper.channel,
            lambda:None,helper.published.append,lambda:helper.fb,lambda:None,
            discovery_s=0,final_check=lambda now:'position_outside_tolerance',continuous=True)
        r.request_stop('reset_complete',0.)
        for i,t in enumerate([.02,.06,.12,.18]):
            helper.fb=Feedback(t,i+2,True,0.,0.,0.);r.stopping(t)
        self.assertTrue(r.finished)
        self.assertEqual(helper.channel.sent[-1]['type'],'stopped')
        self.assertIn('completion_invalid',r.stop_reason)

if __name__=='__main__':unittest.main()
