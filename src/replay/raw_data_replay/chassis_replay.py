"""Recorded ManualMoveCmd replay, anchored to native upper-body timestamps."""
import bisect,copy,math,time,sys
from pathlib import Path
TOPIC='/move/ManualMoveCmd'
def prepare(plan):
    rows=plan['streams'].get(TOPIC,[])
    if not rows:raise ValueError('Missing chassis manual command stream')
    for _,m in rows:
        values=[m.twist.linear.x,m.twist.linear.y,m.twist.linear.z,m.twist.angular.x,m.twist.angular.y,m.twist.angular.z]
        if not all(math.isfinite(v) for v in values):raise ValueError('Nonfinite chassis command')
        if any(values[i]!=0 for i in (2,3,4)):raise ValueError('Unsupported chassis axis')
    return rows

class Replay:
    def __init__(self,node,plan,return_after=False):
        from geometry_msgs.msg import TwistStamped
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        self.node=node;self.message=TwistStamped;self.rows=prepare(plan) if plan is not None else []
        self.times=[t for t,m in self.rows];self.origin=None;self.index=-1
        if plan is not None:
            self.first_ik=plan['streams']['/tj/control/ik_request'][0][0]
            self.duration=plan['duration_ns']
        self.sent=set();self.conflict=False;self.closed=False
        qos=QoSProfile(depth=128,reliability=ReliabilityPolicy.RELIABLE)
        self.sub=node.create_subscription(TwistStamped,TOPIC,self.observe,qos)
        self.pub=node.create_publisher(TwistStamped,TOPIC,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
        self.timer=node.create_timer(.01,self.tick) if plan is not None else None
        self.nonzero_sent=0
        self.path=None;self.parking=None;self.capture=False;self.paused=False
        if return_after:
            sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'teleop/chassis/src/common'))
            from project_paths import configure_imports
            configure_imports()
            from retrace import RecordedPath
            from reset import decode_odometry
            from live_protocol import Feedback
            from parking_release import create_parking_release
            from nav_msgs.msg import Odometry
            from move.msg import State
            from rclpy.qos import qos_profile_sensor_data
            self.path=RecordedPath(None);self.odom=None;self.speed=None;self.seq=0
            self.speed_seq=0;self.parking=create_parking_release(node,lambda row: print(row["type"],flush=True))
            def odom(m):
                self.seq+=1;self.odom=(m,time.monotonic(),self.seq)
            def speed(m):
                v=m.speed
                self.speed_seq+=1
                now=time.monotonic()
                self.speed=Feedback(now,self.speed_seq,not m.is_soft_auto,
                                    v.linear.x,v.linear.y,v.angular.z)
                self.parking.observe(m.is_parking_mode,now,self.speed_seq)
            self.decode=decode_odometry
            node.create_subscription(Odometry,'/controller/odom',odom,qos_profile_sensor_data)
            node.create_subscription(State,'/move/State',speed,qos_profile_sensor_data)
            self.capture_timer=node.create_timer(.02,self.record_pose)

    def sample(self):
        now=time.monotonic()
        if (self.odom is None or self.speed is None or not self.speed.manual or
                not self.speed.fresh(now) or now-self.odom[1]>=.2):
            return None
        return self.decode(*self.odom,self.speed)

    def record_pose(self):
        if self.capture:
            self.path.observe(self.sample(),time.monotonic())

    def release_parking(self,spin):
        while True:
            if self.conflict:
                raise RuntimeError('底盘有其他指令源，不能解除驻车')
            self.publish(self.message())
            if self.parking.step(time.monotonic()):return
            spin(.02)

    def begin_capture(self):
        self.capture=True
        self.record_pose()
        if self.path.fault:
            raise RuntimeError('无法记录底盘起点：'+self.path.fault)

    def pause(self):
        self.paused=True
        self.publish(self.message())

    def return_to_start(self,spin):
        from logic import target_problem
        self.capture=False
        now=time.monotonic()
        self.path.observe(self.sample(),now)
        if self.parking.latest is None or self.parking.latest[0] or now-self.parking.latest[1]>=.2:
            raise RuntimeError('底盘驻车或驻车反馈失效，取消回程')
        controller=self.path.controller(None,now)
        start=self.path.origin
        sample=self.sample()
        target=(start.x,start.y,start.yaw)
        if (sample is not None and not target_problem(sample,target,'retrace') and
                abs(sample.vx)<=.01 and abs(sample.vy)<=.01 and abs(sample.wz)<=.02):
            return
        controller.origin=start
        while True:
            if self.conflict:
                raise RuntimeError('底盘回程时检测到其他指令源')
            now=time.monotonic()
            if self.parking.latest is None or self.parking.latest[0] or now-self.parking.latest[1]>=.2:
                raise RuntimeError('底盘驻车或驻车反馈失效，停止回程')
            command=controller.step(self.sample(),now)
            msg=self.message();msg.twist.linear.x=command.vx;msg.twist.angular.z=command.wz
            self.publish(msg)
            if controller.finished_reason:
                sample=self.sample()
                if (controller.finished_reason!='reset_complete' or sample is None or
                        target_problem(sample,target,'retrace')):
                    raise RuntimeError('底盘回程未完成：'+controller.finished_reason)
                return
            spin(.02)
    def observe(self,m):
        stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        if stamp in self.sent:self.sent.discard(stamp)
        else:self.conflict=True
    def anchor(self,m):
        if self.origin is None:
            stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
            age=self.node.get_clock().now().nanoseconds-stamp
            if not 0<=age<100_000_000:raise RuntimeError('Native body timestamp unavailable')
            self.origin=time.monotonic_ns()-age-self.first_ik
    def publish(self,m):
        m=copy.deepcopy(m);stamp=self.node.get_clock().now()
        m.header.stamp=stamp.to_msg();m.header.frame_id='base_link'
        self.sent.add(stamp.nanoseconds)
        if len(self.sent)>4096:raise RuntimeError('Chassis command observation unavailable')
        self.pub.publish(m)
    def tick(self):
        if self.closed:return
        if self.conflict:raise RuntimeError('Another chassis command source is publishing; stop teleoperation before replay')
        if self.paused or self.origin is None:return
        elapsed=time.monotonic_ns()-self.origin
        if elapsed>=self.duration:
            self.publish(self.message());return
        i=bisect.bisect_right(self.times,elapsed)-1
        if i<0 or i==self.index:return
        if elapsed-self.times[i]>100_000_000:raise RuntimeError('Recorded chassis command gap exceeds 100 ms')
        self.index=i
        m=self.rows[i][1]
        moving=any(abs(v)>1e-8 for v in (m.twist.linear.x,m.twist.linear.y,m.twist.angular.z))
        if moving and self.parking is not None and (self.parking.latest is None or self.parking.latest[0] or
                       time.monotonic()-self.parking.latest[1]>=.2):
            raise RuntimeError('底盘驻车或驻车反馈失效，回放已停止')
        self.publish(m)
        self.nonzero_sent+=int(moving)
    def close(self):
        if self.closed:return
        self.closed=True
        if self.timer is not None:self.node.destroy_timer(self.timer)
        if self.path is not None:self.node.destroy_timer(self.capture_timer)
        # Repeat zero on completion and Ctrl+C; never resume old movement.
        for _ in range(10):
            self.publish(self.message());time.sleep(.02)
        self.node.destroy_subscription(self.sub);self.node.destroy_publisher(self.pub)

if __name__=='__main__':
    import sys
    from body_replay import main
    sys.argv.append('--with-chassis');main()
