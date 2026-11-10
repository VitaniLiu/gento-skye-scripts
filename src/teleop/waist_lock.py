"""Final body-command filter. Import/help/dry-run never publish robot commands."""
import argparse
import json
from pathlib import Path
import math
import time


def locked_positions(positions,angles):
    values=list(positions)
    if len(values)!=6 or not all(math.isfinite(v) for v in values):
        raise ValueError('invalid body positions')
    if len(angles)!=2 or not all(math.isfinite(v) for v in angles):
        raise ValueError('invalid lock angles')
    values[1:3]=angles
    return values


def parse_waist_buttons(buttons):
    """Accept legacy3 or sequence/transport/event-freshness-extended6 intent controls."""
    if len(buttons) not in (3,6) or any(v not in (0,1) for v in buttons[:3]):
        raise ValueError('invalid waist controls')
    if len(buttons)==3:return 0,True,False
    sequence,fresh,event_fresh=buttons[3:]
    if type(sequence) is not int or not 0<=sequence<2**31 or fresh not in (0,1) or event_fresh not in (0,1):
        raise ValueError('invalid keyboard reset sequence/freshness')
    return sequence,bool(fresh),bool(event_fresh)


class KeyboardReturnLatch:
    """Consume each request once, even if feedback makes that request unusable."""
    def __init__(self):self.gid=None;self.sequence=0
    def accept(self,gid,sequence):
        if gid!=self.gid:
            self.gid=gid;self.sequence=sequence
            return False
        if sequence<=self.sequence:return False
        self.sequence=sequence;return True


class LiftTarget:
    """Normalized UP-positive stick velocity, bounded time steps and feedback lag.

    Pure controller only. A receiver must validate transport freshness before
    calling update; reconnect requires a neutral stick before movement resumes.
    """
    def __init__(self,height,lower=0.,upper=.86,speed=.03,deadzone=.15,reset_height=None):
        if not all(math.isfinite(v) for v in (height,lower,upper,speed,deadzone)) or not lower<=height<=upper or speed<=0 or not 0<=deadzone<1:
            raise ValueError('invalid lift settings')
        self.target=height;self.lower=lower;self.upper=upper
        self.speed=speed;self.deadzone=deadzone;self.last=None;self.rearmed=False
        if reset_height is not None and (type(reset_height) not in (int,float) or not math.isfinite(reset_height) or not lower<=reset_height<=upper):
            raise ValueError('invalid fixed reset height')
        self.reset_height=reset_height
        self.origin=height if reset_height is None else reset_height
        self.returning=False;self.return_started=None;self.keyboard_returning=False
        self.replay_reset_at=None
        self.playback=False
    def begin_session(self,height):
        if not math.isfinite(height) or not self.lower<=height<=self.upper:raise ValueError('invalid session height')
        self.origin=height if self.reset_height is None else self.reset_height
        self.target=height;self.returning=False;self.keyboard_returning=False;self.last=None;self.rearmed=False
    def request_keyboard_return(self,feedback,now):
        if (type(feedback) not in (int,float) or not math.isfinite(feedback) or not self.lower<=feedback<=self.upper
                or type(now) not in (int,float) or not math.isfinite(now)):
            raise ValueError("invalid keyboard reset request")
        if self.returning and self.keyboard_returning:return False
        self.target=feedback;self.last=now;self.returning=True;self.keyboard_returning=True
        self.return_started=now;self.rearmed=False
        return True
    def request_return(self,now):
        self.keyboard_returning=False
        self.returning=not self.returning;self.return_started=now
        self.rearmed=False
    def update(self,stick,enabled,fresh,feedback,now):
        if not all(type(v) in (int,float) and math.isfinite(v) for v in (stick,feedback,now)) or not -1<=stick<=1 or not self.lower<=feedback<=self.upper:
            self.rearmed=False;self.last=None;self.returning=False;self.keyboard_returning=False
            raise ValueError('invalid lift input')
        dt=None if self.last is None else now-self.last;self.last=now
        if self.returning:
            if (not fresh or ((enabled or abs(stick)>self.deadzone) and not self.keyboard_returning)
                    or dt is None or not 0<=dt<=.1 or now-self.return_started>45):
                self.returning=False;self.keyboard_returning=False;self.target=feedback;self.rearmed=False
                return self.target
            if abs(feedback-self.origin)<=.002 and abs(self.target-self.origin)<=.002:
                self.target=self.origin;self.returning=False;self.keyboard_returning=False
                return self.target
            step=self.speed*min(dt,.02)
            delta=max(-step,min(step,self.origin-self.target))
            self.target=max(self.lower,min(self.upper,feedback+.02,max(feedback-.02,self.target+delta)))
            return self.target
        if not fresh or not enabled or dt is None or not 0<=dt<=.1:
            self.rearmed=False;self.target=feedback;return self.target
        if abs(stick)<=self.deadzone:
            self.rearmed=True;return self.target
        if not self.rearmed:return self.target
        velocity=math.copysign((abs(stick)-self.deadzone)/(1-self.deadzone)*self.speed,stick)
        candidate=self.target+velocity*min(dt,.02)
        self.target=max(self.lower,min(self.upper,feedback+.02,max(feedback-.02,candidate)))
        return self.target


def configured_reset_height(path):
    record=json.loads(Path(path).read_text())
    height=record.get('reset_height_m')
    if type(height) not in (int,float) or not math.isfinite(height) or not 0<=height<=.86:
        raise ValueError('invalid waist reset_height_m; expected metres in [0, .86]')
    return float(height)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--config',default=str(Path(__file__).resolve().parents[2]/'configs/teleop/waist.json'))
    args=parser.parse_args()
    if not args.execute:
        print('离线模式：腰部过滤器未启动，无指令发布。');return
    reset_height=configured_reset_height(args.config)
    import rclpy
    from sensor_msgs.msg import JointState, Joy
    from std_msgs.msg import Bool,Float64
    from std_srvs.srv import SetBool
    from marvin_msgs.msg import JointcmdBody, Jointfeedback
    from marvin_msgs.srv import JsonCommand
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    rclpy.init()
    node=rclpy.create_node('gento_waist_lock')
    output_topic='/tj/control/joint_cmd_body'
    anchor=None;feedback_at=None;fault=None;publisher=None;started=time.monotonic()
    lift=None;lift_feedback=None;intent_value=0.;intent_enabled=False;intent_at=None;intent_gid=None
    body_feedback=None;body_feedback_at=None;reset_released=False;feedback_paused=False
    keyboard_requests=KeyboardReturnLatch();intent_transport_fresh=True
    home_passthrough=False;home_angles=None;measured_angles=None
    playback_target=None
    def fail(reason):
        nonlocal fault
        if fault is None:node.get_logger().error(reason)
        fault=reason
    node.create_subscription(Bool,'/tj/control/gamesir_emergency_stop',lambda msg:fail('Y emergency stop latched') if msg.data else None,10)
    def feedback(msg):
        nonlocal anchor,feedback_at,lift,lift_feedback,measured_angles
        try:
            indices=[list(msg.name).index(k) for k in ('waist_joint_2','waist_joint_3')]
            values=[msg.position[i] for i in indices]
            if not all(math.isfinite(v) for v in values):raise ValueError('nonfinite feedback')
            measured_angles=values
            if anchor is None:anchor=values;node.get_logger().info('腰部俯仰及旋转保持初始角度；升降由 LB+右摇杆控制，Pico 腰部目标屏蔽。')
            lift_feedback=float(msg.position[list(msg.name).index('waist_joint_1')])
            if lift is None:
                lift=LiftTarget(lift_feedback,reset_height=reset_height)
                node.get_logger().info(f'腰部固定复位高度：{reset_height:.6f} m')
            feedback_at=time.monotonic()
        except (ValueError,IndexError):fail('missing/invalid waist feedback')
    def receive_intent(msg):
        nonlocal intent_value,intent_enabled,intent_at,intent_gid,reset_released,intent_transport_fresh
        if lift is not None and (lift.replay_reset_at is not None or lift.playback):return
        rows=node.get_publishers_info_by_topic('/tj/control/gamesir_waist_intent')
        if len(rows)!=1 or not rows[0].node_name.startswith('tianjidipan_return_'):
            intent_enabled=False;return
        gid=bytes(rows[0].endpoint_gid)
        if len(msg.axes)!=1 or not math.isfinite(msg.axes[0]) or not -1<=msg.axes[0]<=1:
            intent_enabled=False;return
        try:keyboard_sequence,transport_fresh,request_fresh=parse_waist_buttons(msg.buttons)
        except ValueError:
            intent_enabled=False;return
        stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        if not 0<=node.get_clock().now().nanoseconds-stamp<150000000:
            intent_enabled=False;return
        now=time.monotonic()
        new_keyboard=keyboard_requests.accept(gid,keyboard_sequence)
        if lift is None or feedback_at is None or now-feedback_at>.2:
            if new_keyboard:node.get_logger().warning('键盘 A 腰部复位未执行：反馈未就绪；恢复后请重新按 A')
            return
        if gid!=intent_gid:
            intent_gid=gid;lift.begin_session(lift_feedback);reset_released=False
        if intent_at is not None and now-intent_at>=.15:
            reset_released=False;lift.returning=False;lift.keyboard_returning=False;lift.rearmed=False
        intent_value=msg.axes[0];intent_enabled=bool(msg.buttons[0]);intent_at=now
        intent_transport_fresh=transport_fresh
        if not msg.buttons[1]:reset_released=True
        elif reset_released:
            reset_released=False
            if not intent_enabled and msg.buttons[2] and abs(intent_value)<=lift.deadzone:
                lift.request_return(now)
                node.get_logger().info('腰部高度复位'+('开始' if lift.returning else '取消')+f'，目标 {lift.origin:.3f} m')
            else:node.get_logger().warning('腰部复位拒绝：请松开 LB，并让右摇杆及方向键回中')
        # Keyboard A is a waist-only, idempotent request, independent of LB/stick.
        if new_keyboard:
            if fault or not transport_fresh or not request_fresh or body_feedback_at is None or now-body_feedback_at>.2:
                node.get_logger().warning('键盘 A 腰部复位未执行：制动/故障、复位请求过期、输入或反馈无效；恢复后请重新按 A')
            else:
                started_return=lift.request_keyboard_return(lift_feedback,now)
                node.get_logger().info('键盘 A 腰部固定高度复位'+('开始' if started_return else '继续')+f'，目标 {lift.origin:.6f} m')
    node.create_subscription(Joy,'/tj/control/gamesir_waist_intent',receive_intent,1)
    def receive_body_feedback(msg):
        nonlocal body_feedback,body_feedback_at
        values=list(msg.body_positions)
        if len(values)!=6 or not all(math.isfinite(v) for v in values):
            fail('invalid full body feedback');return
        body_feedback=values;body_feedback_at=time.monotonic()
    def reset_service(req,res,height=None):
        nonlocal home_passthrough,anchor
        now=time.monotonic()
        if not req.data:
            if lift is not None and lift.replay_reset_at is not None:
                lift.replay_reset_at=None;lift.origin=reset_height;home_passthrough=False;anchor=measured_angles
                lift.returning=False;lift.keyboard_returning=False;lift.rearmed=False;lift.target=lift_feedback
            res.success=True;return res
        if (fault or lift is None or lift.playback or lift.replay_reset_at is not None or feedback_at is None
                or body_feedback_at is None or now-feedback_at>.2 or now-body_feedback_at>.2
                or (intent_at is not None and now-intent_at<.15 and intent_enabled)):
            res.message='腰部未就绪、已有复位或遥操正在控制升降';return res
        lift.origin=reset_height if height is None else height
        lift.request_keyboard_return(lift_feedback,now)
        lift.replay_reset_at=now
        res.success=True;return res

    def home_service(req,res):
        nonlocal home_passthrough,home_angles,anchor
        if req.data and (lift is None or lift.replay_reset_at is None or fault or abs(lift_feedback-lift.origin)>.002):
            res.message='腰部尚未到达复位高度';return res
        if not req.data and home_passthrough:anchor=measured_angles
        home_passthrough=req.data;home_angles=None
        res.success=True;return res

    def native_body(msg):
        nonlocal home_angles
        values=list(msg.positions[1:3])
        if home_passthrough and all(math.isfinite(v) for v in values):home_angles=values

    def reset_to_height(req,res):
        try:
            height=json.loads(req.command_json)['height_m']
            if type(height) not in (float,int) or not math.isfinite(height) or not 0<=height<=.86:
                raise ValueError()
        except (ValueError,KeyError,TypeError):
            res.message='invalid waist height_m';return res
        from types import SimpleNamespace
        return reset_service(SimpleNamespace(data=True),res,height)

    def playback_enable(req,res):
        nonlocal playback_target
        now=time.monotonic()
        if req.data and (fault or lift is None or lift.playback or lift.replay_reset_at is not None
                or lift.returning or feedback_at is None or body_feedback_at is None
                or now-feedback_at>.2 or now-body_feedback_at>.2
                or (intent_at is not None and now-intent_at<.15 and intent_enabled)):
            res.message='腰部反馈未就绪或已有控制源';return res
        if lift is not None:
            lift.playback=req.data;lift.target=lift_feedback;lift.rearmed=False;lift.last=None
        playback_target=None
        res.success=True;return res

    def playback_height(msg):
        nonlocal playback_target
        if lift is not None and lift.playback and math.isfinite(msg.data) and 0<=msg.data<=.86:
            playback_target=(time.monotonic(),msg.data)

    node.create_service(SetBool,'/tj/control/waist_playback/enable',playback_enable)
    node.create_subscription(Float64,'/tj/control/waist_playback/height',playback_height,1)
    node.create_service(JsonCommand,'/tj/control/waist_reset_to_height',reset_to_height)
    node.create_service(SetBool,'/tj/control/waist_reset',reset_service)
    node.create_service(SetBool,'/tj/control/waist_home_passthrough',home_service)
    node.create_subscription(JointcmdBody,'/tj/control/waist_unlocked/joint_cmd_body',native_body,
                             QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    def tick():
        nonlocal publisher,feedback_paused,reset_released,home_passthrough,anchor
        if fault or anchor is None or lift is None:return
        now=time.monotonic()
        if now-started<2:return
        rows=node.get_publishers_info_by_topic(output_topic)
        if any(r.node_name!=node.get_name() for r in rows):fail('competing final body publisher');return
        # No Pico/IK command is needed. Full measured positions seed unused slots.
        if lift.replay_reset_at is not None and now-lift.replay_reset_at>95:
            lift.replay_reset_at=None;lift.origin=reset_height;home_passthrough=False;anchor=measured_angles
            lift.returning=False;lift.keyboard_returning=False;lift.target=lift_feedback
        if intent_at is None and lift.replay_reset_at is None and not lift.playback:return
        if feedback_at is None or now-feedback_at>.2 or body_feedback_at is None or now-body_feedback_at>.2:
            if not feedback_paused:node.get_logger().warning('腰部反馈超时：暂停输出，取消复位，等待反馈恢复及摇杆回中')
            feedback_paused=True;reset_released=False
            lift.returning=False;lift.keyboard_returning=False;lift.rearmed=False;lift.last=None
            return
        if feedback_paused:
            feedback_paused=False;lift.target=lift_feedback;lift.last=None;lift.rearmed=False
            node.get_logger().info('腰部反馈已恢复：请将右摇杆回中后再操作')
        if abs(body_feedback[0]-lift_feedback)>.005:
            fail('lift feedback layout mismatch');return
        replay_reset=lift.replay_reset_at is not None
        fresh=replay_reset or (intent_at is not None and 0<=now-intent_at<.15 and intent_transport_fresh)
        try:
            out=JointcmdBody()
            positions=locked_positions(body_feedback,home_angles if home_passthrough and home_angles is not None else anchor)
            was_keyboard=lift.returning and lift.keyboard_returning
            if lift.playback:
                # Retain the teleop feedback lead limit; on stream loss hold measured height.
                target=(playback_target[1] if playback_target is not None and
                        now-playback_target[0]<=.20 else lift_feedback)
                positions[0]=min(lift.upper,lift_feedback+.02,max(lift.lower,lift_feedback-.02,target))
            elif replay_reset and not lift.returning:
                positions[0]=lift.origin if abs(lift_feedback-lift.origin)<=.002 else lift_feedback
            else:
                positions[0]=lift.update(0. if replay_reset else intent_value,
                                         False if replay_reset else intent_enabled,fresh,lift_feedback,now)
            if was_keyboard and not lift.returning:
                if abs(lift_feedback-lift.origin)<=.002:
                    node.get_logger().info(f"键盘 A 腰部复位完成，反馈 {lift_feedback:.6f} m，目标 {lift.origin:.6f} m")
                else:node.get_logger().warning("键盘 A 腰部复位已中断，请检查输入/反馈后重新按 A")
            out.positions=positions
            out.header.stamp=node.get_clock().now().to_msg()
        except (ValueError,TypeError) as exc:fail(str(exc));return
        if publisher is None:publisher=node.create_publisher(JointcmdBody,output_topic,10)
        publisher.publish(out)
    node.create_subscription(JointState,'/tj/joint_states',feedback,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(Jointfeedback,'/tj/info/joint_feedback',receive_body_feedback,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_timer(.02,tick)
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
