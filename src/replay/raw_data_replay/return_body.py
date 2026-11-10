"""Return via native Home and the replay start snapshot."""
import json
import math
from pathlib import Path
import re
import time
import xml.etree.ElementTree as ET

WAIST='waist_joint_1'
JOINT_GROUPS=(
    tuple(f'Joint{i}_L' for i in range(1,8)),
    tuple(f'Joint{i}_R' for i in range(1,8)),
    ('Head_J1_Joint','Head_J2_Joint'),
    ('waist_joint_2','waist_joint_3'),
)

class BodyReturn:
    def __init__(self,node):
        from marvin_msgs.srv import Int,JsonCommand
        from std_srvs.srv import SetBool,Trigger
        from marvin_msgs.msg import Jointfeedback
        from rclpy.qos import qos_profile_sensor_data
        self.n=node;self.Bool=SetBool;self.Int=Int;self.Trigger=Trigger;self.JsonCommand=JsonCommand
        self.feedback=None
        self.height=node.create_client(JsonCommand,'/tj/control/waist_reset_to_height')
        self.release=node.create_client(SetBool,'/tj/control/waist_reset')
        self.angles=node.create_client(SetBool,'/tj/control/waist_home_passthrough')
        self.home=node.create_client(Trigger,'/tj/control/go_home')
        self.mux=node.create_client(Int,'/tj/control/set_input')
        node.create_subscription(Jointfeedback,'/tj/info/joint_feedback',self.feedback_cb,qos_profile_sensor_data)
        path=Path('/opt/kernelmind/apex/install/marvin_description/share/marvin_description/mjcf/Gento_Sky/Gento_Sky.xml')
        model=ET.parse(path)
        names=[j.get('name') for j in model.getroot().find('worldbody').iter('joint') if j.get('name')]
        key=model.find("./keyframe/key[@name='home']")
        home=dict(zip(names,map(float,key.get('qpos').split()),strict=True))
        self.home_target={name:home[name] for group in JOINT_GROUPS for name in group}

    def feedback_cb(self,m):
        self.feedback=(time.monotonic(),list(m.body_positions),list(m.head_positions))

    def prepare(self):
        for client in (self.height,self.release,self.angles,self.home,self.mux):
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError('回程服务不可用：'+client.srv_name)

    def run(self,start,state,spin,call):
        from marvin_msgs.msg import JointcmdArm,JointcmdBody,JointcmdHead
        def latest():
            t,m=state['joints']
            if time.monotonic()-t>.25:
                raise RuntimeError('身体回程关节反馈超时')
            result=dict(zip(m.name,m.position))
            if any(not math.isfinite(result[k]) for k in start):
                raise RuntimeError('身体回程关节反馈无效')
            return result
        def wait_at(target,timeout,not_before=0):
            end=time.monotonic()+timeout;stable=None
            while time.monotonic()<end:
                now=time.monotonic();q=latest()
                within=now>=not_before and all(abs(q[j]-v)<=(.002 if j==WAIST else .02)
                                                for j,v in target.items())
                stable=(stable or now) if within else None
                if stable is not None and now-stable>=.2:return
                spin(.02)
            raise RuntimeError('身体或腰部未在限时内到达目标')
        height=start[WAIST]
        call(self.height,self.JsonCommand.Request(command_json=json.dumps({'height_m':height})))
        try:
            wait_at({WAIST:height},46)
            call(self.mux,self.Int.Request(data=1))
            reply=call(self.home,self.Trigger.Request())
            match=re.search(r'duration=([0-9.]+)',reply.message)
            if not match:raise RuntimeError('原生 Home 未报告轨迹时长：'+reply.message)
            duration=float(match[1])
            call(self.angles,self.Bool.Request(data=True))
            try:
                wait_at({**self.home_target,WAIST:height},duration+8,time.monotonic()+duration)
                initial=latest()
                if self.feedback is None or time.monotonic()-self.feedback[0]>.25:
                    raise RuntimeError('身体回程缺少关节组反馈')
                _,body,head=self.feedback
                types=(JointcmdArm,JointcmdArm,JointcmdHead,JointcmdBody)
                pubs=[self.n.create_publisher(t,'/tj/control/user/joint_cmd_'+s,1)
                      for t,s in zip(types,('A','B','head','body'))]
                def publish(u):
                    stamp=self.n.get_clock().now().to_msg()
                    q=[[initial[k]+u*(start[k]-initial[k]) for k in group] for group in JOINT_GROUPS]
                    values=(*q[:2],q[2]+head[2:],[height]+q[3]+body[3:])
                    for pub,t,v in zip(pubs,types,values):
                        m=t(positions=v);m.header.stamp=stamp;pub.publish(m)
                active=False
                try:
                    call(self.mux,self.Int.Request(data=3));active=True
                    for _ in range(110):
                        latest();publish(0.);spin(.02)
                    total=max(1.,max(abs(start[k]-initial[k]) for k in self.home_target)/.10)
                    begin=time.monotonic()
                    while True:
                        latest()
                        t=min(1.,(time.monotonic()-begin)/total)
                        publish(t*t*(3-2*t));spin(.02)
                        if t==1.:break
                    settled=None;end=time.monotonic()+5
                    while time.monotonic()<end:
                        q=latest();now=time.monotonic();publish(1.)
                        near=all(abs(q[k]-v)<=(.002 if k==WAIST else .02) for k,v in start.items())
                        settled=(settled or now) if near else None
                        if settled is not None and now-settled>=.2:break
                        spin(.02)
                    else:raise RuntimeError('身体未回到本次回放起点')
                finally:
                    try:
                        if active:call(self.mux,self.Int.Request(data=0))
                    finally:
                        for pub in pubs:self.n.destroy_publisher(pub)
            finally:
                call(self.angles,self.Bool.Request(data=False))
        finally:
            call(self.release,self.Bool.Request(data=False))
