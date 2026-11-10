"""Replay measured gripper motor opening only, on the native body clock."""
import bisect,math,os,signal,time
from pathlib import Path

def prepare(plan):
    result={}
    for side in ('L','R'):
        rows=plan['streams'].get('/tj/info/gripper_feedback_'+side,[])
        if not rows:raise ValueError('Missing recorded gripper opening '+side)
        result[side]=[]
        for t,m in rows:
            if not m.data or not math.isfinite(m.data[0]):raise ValueError('Invalid gripper opening')
            # Same motor-position mapping used by the deployed slow controller.
            v=float(m.data[0])/1.6
            if not -.02<=v<=1.02:raise ValueError('Gripper feedback outside calibrated motor range')
            result[side].append((t,max(0.,min(1.,v))))
    return result

class Replay:
    def __init__(self,node,plan):
        from std_msgs.msg import Float32
        self.message=Float32;self.node=node;self.rows=prepare(plan);self.origin=None
        self.first_ik=plan['streams']['/tj/control/ik_request'][0][0]
        self.times={s:[t for t,v in rows] for s,rows in self.rows.items()}
        self.indices={s:-1 for s in self.rows};self.pubs={};self.enabled=False
        from std_msgs.msg import Bool
        from std_srvs.srv import SetBool
        self.bool_message=Bool;self.request=SetBool.Request
        self.client=node.create_client(SetBool,'/tj/control/gripper_replay/enable')
        try:
            if not self.client.wait_for_service(timeout_sec=3):
                raise RuntimeError('请重启 Tool 以加载夹爪回放交接接口')
            self.heartbeat=node.create_publisher(Bool,'/tj/control/gripper_replay/heartbeat',1)
            for side in self.rows:
                self.pubs[side]=node.create_publisher(Float32,'/tj/control/gripper_replay_target'+side,1)
            import rclpy
            deadline=time.monotonic()+5
            while not (self.heartbeat.get_subscription_count()>0 and
                       all(p.get_subscription_count()>0 for p in self.pubs.values())):
                if time.monotonic()>deadline:raise RuntimeError('Gripper relay topics did not connect')
                rclpy.spin_once(node,timeout_sec=.02)
            self.switch(True);self.enabled=True
            self.beat()
            self.heart_timer=node.create_timer(.1,self.beat)
            self.timer=node.create_timer(.005,self.tick)
        except BaseException:
            self.close();raise
    def switch(self,enabled):
        import rclpy
        req=self.request();req.data=enabled
        future=self.client.call_async(req)
        rclpy.spin_until_future_complete(self.node,future,timeout_sec=2)
        if not future.done():raise RuntimeError('Gripper handoff timed out')
        result=future.result()
        if not result.success:raise RuntimeError(result.message)
    def beat(self):
        m=self.bool_message();m.data=True;self.heartbeat.publish(m)
    def anchor(self,message):
        if self.origin is None:
            stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
            age=self.node.get_clock().now().nanoseconds-stamp
            if not 0<=age<100_000_000:raise RuntimeError('Native body timestamp unavailable')
            self.origin=time.monotonic_ns()-age-self.first_ik
    def tick(self):
        if self.origin is None:return
        elapsed=time.monotonic_ns()-self.origin
        for side,rows in self.rows.items():
            i=bisect.bisect_right(self.times[side],elapsed)-1
            if i<0:continue
            self.indices[side]=i;m=self.message();m.data=rows[i][1];self.pubs[side].publish(m)
    def close(self):
        for name in ('timer','heart_timer'):
            if hasattr(self,name):self.node.destroy_timer(getattr(self,name));delattr(self,name)
        try:
            if self.enabled:self.switch(False)
        except Exception as exc:
            self.node.get_logger().error('Gripper release failed; heartbeat expires within 0.5s: '+str(exc))
        finally:
            self.enabled=False
            for pub in self.pubs.values():self.node.destroy_publisher(pub)
            self.pubs={}
            if hasattr(self,'heartbeat'):self.node.destroy_publisher(self.heartbeat);del self.heartbeat

if __name__=='__main__':
    import sys
    from body_replay import main
    sys.argv.append('--with-grippers')
    main()
