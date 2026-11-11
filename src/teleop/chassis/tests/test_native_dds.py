import unittest
from types import SimpleNamespace as N
import live_protocol as p
import robot_endpoint as e

COMMAND_GID='011079df54084638711a973a00002203'
STATE_GID='0110d3cf174484cd40a28aed00002b03'

def endpoint(name,gid,topic_type=e.STATE_TYPE):return N(node_name=name,node_namespace='/',endpoint_gid=bytes.fromhex(gid),topic_type=topic_type)
def message(vx=0,stamp=1):
    return N(header=N(stamp=N(sec=stamp,nanosec=0),frame_id='base_link'),twist=N(linear=N(x=vx,y=0,z=0),angular=N(x=0,y=0,z=0)))

class NativeTests(unittest.TestCase):
    def session(self,state_gid=STATE_GID,command_gid=COMMAND_GID):
        states=[endpoint('_NODE_NAME_UNKNOWN_',state_gid)]
        commands=[endpoint('_NODE_NAME_UNKNOWN_',command_gid,e.COMMAND_TYPE)]
        return e.SessionIdentity(lambda:states,lambda:commands,discovery_s=0),states,commands
    def test_discovery_waits_for_late_legacy_names_before_pinning(self):
        identity,states,commands=self.session()
        clock=[0.];identity.clock=lambda:clock[0];identity.discovery_s=1.
        self.assertEqual(identity.inspect(),'source_discovery_pending')
        commands.append(endpoint('_NODE_NAME_UNKNOWN_','56'*16,e.COMMAND_TYPE))
        clock[0]=.1
        self.assertIsNotNone(identity.inspect());self.assertIsNone(identity.fault)
        commands[-1].node_name='host_gamepad_control'
        clock[0]=.2
        self.assertEqual(identity.inspect(),'source_discovery_pending')
        clock[0]=1.21
        self.assertIsNone(identity.inspect())
        self.assertEqual(identity.command_gid,COMMAND_GID)
    def test_restart_before_new_session_accepts_fresh_endpoint_ids(self):
        first,_,_=self.session()
        second,_,_=self.session('12'*16,'34'*16)
        self.assertIsNone(first.inspect());self.assertIsNone(second.inspect())
        self.assertNotEqual(first.state_gid,second.state_gid)
        self.assertNotEqual(first.command_gid,second.command_gid)
    def test_state_replacement_or_disappearance_is_latched_until_new_session(self):
        for change in ('replace','remove','duplicate'):
            identity,states,commands=self.session();self.assertIsNone(identity.inspect())
            original=states[:]
            if change=='replace':states[:]=[endpoint('_NODE_NAME_UNKNOWN_','12'*16)]
            elif change=='remove':states.clear()
            else:states.extend(original)
            self.assertIsNotNone(identity.inspect())
            states[:]=original
            self.assertIsNotNone(identity.inspect())
    def test_native_command_replacement_or_disappearance_is_latched(self):
        for change in ('replace','remove','duplicate'):
            identity,states,commands=self.session();self.assertIsNone(identity.inspect())
            original=commands[:]
            if change=='replace':commands[:]=[endpoint('_NODE_NAME_UNKNOWN_','34'*16,e.COMMAND_TYPE)]
            elif change=='remove':commands.clear()
            else:commands.extend(original)
            self.assertIsNotNone(identity.inspect())
            commands[:]=original
            self.assertIsNotNone(identity.inspect())
    def test_startup_never_enrolls_ambiguous_invalid_or_wrong_type_sources(self):
        for change in ('duplicate_state','duplicate_command','state_type','command_type','zero','short'):
            identity,states,commands=self.session()
            if change=='duplicate_state':states.extend(states[:])
            elif change=='duplicate_command':commands.extend(commands[:])
            elif change=='state_type':states[0].topic_type=e.COMMAND_TYPE
            elif change=='command_type':commands[0].topic_type=e.STATE_TYPE
            elif change=='zero':states[0].endpoint_gid=bytes(16)
            else:states[0].endpoint_gid=b'short'
            self.assertIsNotNone(identity.inspect())
            self.assertIsNone(identity.state_gid)
            self.assertIsNone(identity.command_gid)
    def test_additional_named_controller_remains_a_conflict(self):
        identity,states,commands=self.session()
        commands.append(endpoint('foreign_controller','56'*16,e.COMMAND_TYPE))
        self.assertIsNone(identity.inspect())
        names=e.publisher_names(commands,identity.command_gid)
        self.assertIn('foreign_controller',names)
        self.assertIsNotNone(p.ownership_problem(names,'our_node',None))
    def test_source_change_during_runtime_stops_and_cannot_certify_stop(self):
        from test_robot_runtime import Inbox
        identity,states,commands=self.session()
        guard=e.StateFeedbackGuard(lambda:states,identity)
        self.assertIsNone(identity.inspect())
        channel=Inbox();published=[]
        feedback=p.Feedback(0,1,True,0,0,0)
        runtime=e.EndpointRuntime('execute','a'*32,0,channel,lambda:None,
                                  published.append,lambda:feedback,
                                  identity.inspect,discovery_s=0)
        runtime.tick(0)
        states[:]=[endpoint('_NODE_NAME_UNKNOWN_','12'*16)]
        runtime.feedback=guard.get
        runtime.tick(.02)
        self.assertEqual(runtime.stop_reason,'state_publisher_changed')
        self.assertEqual(published[-1].reason,'stopping')
        self.assertEqual((published[-1].vx,published[-1].vy,published[-1].wz),(0,0,0))
        runtime.tick(2.1)
        self.assertFalse(channel.sent[-1]['confirmed'])
    def test_check_discovers_new_session_without_creating_command_publisher(self):
        from test_robot_runtime import Inbox
        for state_gid,command_gid in [(STATE_GID,COMMAND_GID),('12'*16,'34'*16)]:
            identity,_,_=self.session(state_gid,command_gid)
            channel=Inbox();enabled=[];published=[]
            runtime=e.EndpointRuntime('check','a'*32,0,channel,
                                      lambda:enabled.append(True),published.append,
                                      lambda:p.Feedback(0,1,True,0,0,0),
                                      identity.inspect,discovery_s=0)
            runtime.tick(0)
            self.assertEqual(runtime.exit_code,0)
            self.assertEqual(enabled,[]);self.assertEqual(published,[])
    def test_only_this_session_native_command_endpoint_is_exempt(self):
        known=endpoint('_NODE_NAME_UNKNOWN_',COMMAND_GID,e.COMMAND_TYPE)
        self.assertEqual(e.publisher_names([known],COMMAND_GID),[])
        other=endpoint('_NODE_NAME_UNKNOWN_','01'*16,e.COMMAND_TYPE)
        self.assertIn('_NODE_NAME_UNKNOWN_',e.publisher_names([other],COMMAND_GID))
        self.assertIn('_NODE_NAME_UNKNOWN_',e.publisher_names([known]))
        self.assertEqual(e.publisher_names([endpoint('host_gamepad_control','02'*16,e.COMMAND_TYPE)],COMMAND_GID),['host_gamepad_control'])
    def test_unique_source_is_learnable_but_replacement_in_session_is_rejected(self):
        known=endpoint('_NODE_NAME_UNKNOWN_',STATE_GID)
        self.assertIsNone(e.state_publisher_problem([known]))
        self.assertIsNone(e.state_publisher_problem([endpoint('_NODE_NAME_UNKNOWN_','01'*16)]))
        for rows in [[],[known,known],[endpoint('_NODE_NAME_UNKNOWN_','01'*16)]]:
            self.assertIsNotNone(e.state_publisher_problem(rows,STATE_GID))
    def clock(self):
        cls=getattr(p,'FeedbackClock',None)
        self.assertTrue(callable(cls),'missing independent clock alignment')
        return cls()
    def sample(self,clock,t,source_t=None,age=0,seq=None):
        wall=1_800_000_000_000_000_000+int(t*1e9)
        source=wall-18_544_575_000_000_000 if source_t is None else 1_781_455_425_000_000_000+int(source_t*1e9)
        return clock.sample(source,wall-int(age*1e9),wall,t,seq)
    def warm(self,c):
        for t in [0,.05,.1,.15]:self.sample(c,t,seq=int(t*100))
    def test_large_fixed_source_clock_offset_is_accepted_after_warmup(self):
        c=self.clock();self.assertIsNone(self.sample(c,0,seq=0))
        self.warm(c)
        self.assertAlmostEqual(self.sample(c,.2,seq=20),.2)
    def test_repeated_source_or_sequence_cannot_refresh_feedback(self):
        for kwargs in [dict(source_t=.15,seq=20),dict(seq=15)]:
            c=self.clock();self.warm(c)
            self.assertIsNone(self.sample(c,.2,**kwargs))
    def test_local_stale_and_future_receive_timestamp_are_rejected(self):
        for age in [.201,-.01]:
            c=self.clock();self.warm(c)
            self.assertIsNone(self.sample(c,.2,age=age,seq=20))
    def test_source_jump_or_old_backlog_requires_restart(self):
        for source_t in [-1,3]:
            c=self.clock();self.warm(c)
            self.assertIsNone(self.sample(c,.2,source_t=source_t,seq=20))
            self.assertIsNone(self.sample(c,.25,seq=25))
    def test_pre_stop_queued_sample_keeps_its_earlier_time(self):
        c=self.clock();self.warm(c)
        sampled=self.sample(c,.3,source_t=.16,age=.1,seq=16)
        self.assertIsNotNone(sampled)
        self.assertLess(sampled,.2)
        v=p.StopVerifier(.2)
        self.assertFalse(v.observe(p.Feedback(sampled,16,True,0,0,0),.3))
    def test_foreign_commands_even_zero_are_rejected_but_own_loopback_is_allowed(self):
        cls=getattr(e,'CommandMonitor',None)
        self.assertTrue(callable(cls),'missing foreign command monitoring')
        monitor=cls();own=message(.2,stamp=10)
        monitor.expect(own,1)
        monitor.observe(own,1.01)
        self.assertIsNone(monitor.problem())
        monitor.observe(message(0,stamp=11),1.02)
        self.assertIsNotNone(monitor.problem())

    def test_second_identical_loopback_is_foreign(self):
        monitor=e.CommandMonitor();own=message(.2,stamp=10)
        monitor.expect(own,1);monitor.observe(own,1.01)
        self.assertIsNone(monitor.problem())
        monitor.observe(own,1.02)
        self.assertIsNotNone(monitor.problem())
    def test_stop_feedback_refuses_ambiguous_graph_and_clears_cached_sample(self):
        cls=getattr(e,'StateFeedbackGuard',None)
        self.assertTrue(callable(cls),'missing independently verified stop feedback')
        rows=[endpoint('_NODE_NAME_UNKNOWN_',STATE_GID)]
        commands=[endpoint('_NODE_NAME_UNKNOWN_',COMMAND_GID,e.COMMAND_TYPE)]
        identity=e.SessionIdentity(lambda:rows,lambda:commands,discovery_s=0)
        guard=cls(lambda:rows,identity)
        msg=N(is_soft_auto=False,speed=N(linear=N(x=0,y=0),angular=N(z=0)))
        for t in [0,.05,.1,.15]:
            wall=1_800_000_000_000_000_000+int(t*1e9)
            guard.receive(msg,dict(source_timestamp=wall-18_544_575_000_000_000,received_timestamp=wall,publication_sequence_number=int(t*100)),wall,t)
        self.assertIsNotNone(guard.get())
        rows.append(endpoint('unexpected','01'*16))
        self.assertIsNone(guard.get())
        rows.pop()
        self.assertIsNone(guard.get())
    def test_command_subscription_retains_foreign_command_before_own_loopback(self):
        import sys
        from collections import deque
        from unittest.mock import patch
        subscriptions=[]
        class Qos:
            def __init__(self,**kw):self.__dict__.update(kw)
        class Node:
            def create_subscription(self,typ,topic,callback,qos):subscriptions.append((topic,callback,qos))
            def create_timer(self,*args):pass
            def destroy_node(self):pass
        class Inbox:
            def __init__(self,*args):self.eof=False
            def send(self,msg):pass
            def receive(self):return []
        modules={
            'rclpy':N(init=lambda **kw:None,create_node=lambda *a,**kw:Node(),ok=lambda:False,shutdown=lambda:None),
            'rclpy.signals':N(SignalHandlerOptions=N(NO=0)),
            'rclpy.qos':N(QoSProfile=Qos,ReliabilityPolicy=N(RELIABLE='reliable',BEST_EFFORT='best_effort')),
            'geometry_msgs.msg':N(TwistStamped=object),
            'move.msg':N(State=object)}
        monitor=e.CommandMonitor();own=message(.2,stamp=10);monitor.expect(own,1)
        with patch.dict(sys.modules,modules),patch.object(e,'JsonChannel',Inbox),patch.object(e,'CommandMonitor',return_value=monitor),patch.object(e.signal,'signal',lambda *a:None):
            e.main(['--check','--session','a'*32])
        topic,callback,qos=next(row for row in subscriptions if row[0]=='/move/ManualMoveCmd')
        self.assertEqual(qos.reliability,'reliable')
        pending=deque(maxlen=qos.depth);pending.extend([message(0,stamp=11),own])
        for msg in pending:callback(msg)
        self.assertIsNotNone(monitor.problem())
