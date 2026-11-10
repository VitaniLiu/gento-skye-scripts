"""Play recorded lift commands on the same receive-time grid as chassis replay."""
import bisect
import math
import time

class WaistReplay:
    def __init__(self,node,plan,call,initial_height):
        from std_msgs.msg import Float64
        from std_srvs.srv import SetBool
        self.msg=Float64;self.request=SetBool.Request
        self.client=node.create_client(SetBool,'/tj/control/waist_playback/enable')
        self.pub=node.create_publisher(Float64,'/tj/control/waist_playback/height',1)
        self.call=call
        rows=plan['streams'].get('/tj/control/joint_cmd_body',[])
        self.times=[t for t,_ in rows]
        self.heights=[float(m.positions[0]) for _,m in rows]
        self.first_ik=plan['streams']['/tj/control/ik_request'][0][0]
        self.duration=plan['duration_ns']
        if not self.heights or any(not math.isfinite(h) or not 0<=h<=.86 for h in self.heights):
            raise ValueError('腰部升降录制缺失或超出有效范围')
        if abs(self.heights[0]-initial_height)>.005:
            raise ValueError('腰部起点与首条录制指令相差超过 5 mm')
        self.enabled=False;self.origin=None;self.sent=0

    def enable(self,spin):
        deadline=time.monotonic()+3
        while not self.pub.get_subscription_count():
            if time.monotonic()>=deadline:raise RuntimeError('机器人未加载腰部回放接口')
            spin(.02)
        self.call(self.client,self.request(data=True))
        self.enabled=True

    def anchor(self,m,node):
        if self.origin is None:
            stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
            age=node.get_clock().now().nanoseconds-stamp
            if not 0<=age<100_000_000:raise RuntimeError('腰部回放无法对齐原生 IK 时钟')
            self.origin=time.monotonic_ns()-age-self.first_ik

    def tick(self):
        if not self.enabled or self.origin is None:return
        t=time.monotonic_ns()-self.origin
        i=bisect.bisect_right(self.times,t)-1
        if i<0:return
        if t<self.times[-1] and t-self.times[i]>200_000_000:
            raise RuntimeError('录制腰高指令中断超过 200 ms')
        self.pub.publish(self.msg(data=self.heights[i]))
        self.sent+=1

    def close(self):
        if self.enabled:
            self.enabled=False
            self.call(self.client,self.request(data=False))
