"""Upper-body replay via an unmodified, private native Gento player."""
import argparse,json,os,signal,subprocess,time,uuid
from pathlib import Path
import fcntl
TOPIC='/tj/control/ik_request'
def prepare(plan):
    rows=plan['streams'].get(TOPIC,[])
    if not rows:raise ValueError('Missing native IKRequest stream')
    return rows

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('episode');p.add_argument('--execute',action='store_true')
    p.add_argument('--with-grippers',action='store_true',help='Replay recorded measured gripper openings alongside native body')
    p.add_argument('--with-chassis',action='store_true',help='Replay recorded chassis commands on the native body clock')
    a=p.parse_args()
    from timeline import load
    plan=load(a.episode);rows=prepare(plan)
    initial=plan['streams']['/tj/joint_states'][0][1]
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool,String,Int32,Int16MultiArray
    from marvin_msgs.msg import IKRequest
    from marvin_msgs.srv import GentoReplay,Int
    rclpy.init(domain_id=24,signal_handler_options=SignalHandlerOptions.NO);tag='raw_upper_'+uuid.uuid4().hex[:8]
    n=rclpy.create_node(tag);state={};proc=None;started=False
    def receive(key,m):state[key]=(time.monotonic(),m)
    for typ,topic,key,qos in [(JointState,'/tj/joint_states','joints',qos_profile_sensor_data),(Bool,'/tj/info/vr_connected','vr',qos_profile_sensor_data),(Int16MultiArray,'/tj/info/robot_state','mode',qos_profile_sensor_data),(String,'/tj/info/gento_replay/native_status','native',10)]:
        n.create_subscription(typ,topic,lambda m,k=key:receive(k,m),qos)
    latch=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(Int32,'/tj/info/ik_request_mux/active_source',lambda m:receive('mux',m),latch)
    n.create_subscription(IKRequest,'/tj/control/ik_request/replay',lambda m:receive('replay_seen',m),qos_profile_sensor_data)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=min(.01,max(0,end-time.monotonic())))
    def check(initial_check=False):
        now=time.monotonic()
        for k,age in [('joints',.3),('mode',.3),('vr',2.5)]:
            if k not in state or now-state[k][0]>age:raise RuntimeError('Missing/stale '+k)
        if state['vr'][1].data:raise RuntimeError('VR connected: release live teleoperation first')
        if list(state['mode'][1].data)!=[2,2,1,1,1,0]:raise RuntimeError('Robot mode not ready: '+str(list(state['mode'][1].data)))
        source=state['mux'][1].data if 'mux' in state else None
        if source not in ([0,1,3] if initial_check else [3]):
            raise RuntimeError('Unexpected IK ownership: source='+str(source))
        if initial_check:
            if 'native' not in state or now-state['native'][0]>2:raise RuntimeError('Native status missing/stale')
            status=json.loads(state['native'][1].data)
            if status.get('playback_active') or status.get('recording'):raise RuntimeError('Native player busy')
            if source==3:
                if 'replay_seen' in state and now-state['replay_seen'][0]<1:
                    raise RuntimeError('Replay channel is actively publishing; stop the other replay first')
                allowed={'gento_replay_node',tag+'_native'}
                foreign=[e.node_name for e in n.get_publishers_info_by_topic('/tj/control/ik_request/replay') if e.node_name not in allowed]
                if foreign:raise RuntimeError('Other replay publishers: '+str(foreign))
            current=state['joints'][1];pos=dict(zip(current.name,current.position))
            if not current.velocity or max(map(abs,current.velocity))>.02:raise RuntimeError('Robot not stationary')
            errors={k:abs(pos.get(k,float('inf'))-v) for k,v in zip(initial.name,initial.position)}
            if not errors or any(v>(.005 if k=='waist_joint_1' else .05) for k,v in errors.items()):raise RuntimeError('Recorded start pose mismatch: '+json.dumps(errors))
    def call(client,req):
        if not client.wait_for_service(timeout_sec=3):raise RuntimeError('Service unavailable')
        f=client.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=5)
        if not f.done():raise RuntimeError('Service timeout')
        response=f.result()
        if hasattr(response,'success') and not response.success:raise RuntimeError(response.message)
        return response
    def request(value):
        r=GentoReplay.Request();r.data=value;r.name=str(Path(a.episode).resolve()/'domain_24');return r
    log=[];out=None;client=None;grippers=None;chassis=None;lock=None;waist=None;qp_routed=False
    try:
        spin(3);check(True)
        print(json.dumps({'preflight':'passed','duration_s':plan['duration_ns']/1e9,'execute':a.execute}),flush=True)
        if not a.execute:return
        from return_body import BodyReturn,JOINT_GROUPS,WAIST
        return_body=BodyReturn(n)
        return_body.prepare()
        if a.with_chassis:
            lockpath=Path(__file__).resolve().parents[2]/'teleop/chassis/.sender.lock'
            lock=lockpath.open('a')
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        folder=Path(__file__).resolve().parents[3]/'logs/replay';folder.mkdir(parents=True,exist_ok=True)
        out=(folder/(tag+'.json')).open('w')
        args=['/opt/kernelmind/apex/install/gento_replay/lib/gento_replay/gento_replay_node','--ros-args','-r','__ns:=/tj','-r','__node:='+tag+'_native']
        for key,value in {'ros_namespace':'tj','bag_topic':TOPIC,'input_topic':'/'+tag+'/unused_input','loop_playback':'false','record_camera':'false'}.items():args+=['-p',key+':='+value]
        # Isolate every native auxiliary output; only native IK reaches the robot.
        for old,new in [('control/gento_replay/playback_named','/'+tag+'/playback'),('control/gento_replay/playback','/'+tag+'/playback_int'),('info/gento_replay/status','/'+tag+'/status'),('control/gripperValueL','/'+tag+'/gripperL'),('control/gripperValueR','/'+tag+'/gripperR'),('/hand_left/joint_commands','/'+tag+'/handL'),('/hand_right/joint_commands','/'+tag+'/handR')]:args+=['-r',old+':='+new]
        n.create_subscription(String,'/'+tag+'/status',lambda m:receive('private',m),10)
        count=[0]
        def on_ik(m):
            count[0]+=1
            if grippers is not None:grippers.anchor(m)
            if chassis is not None:chassis.anchor(m)
            if waist is not None:waist.anchor(m,n)
        n.create_subscription(IKRequest,'/tj/control/ik_request/replay',on_ik,qos_profile_sensor_data)
        env=dict(os.environ,ROS_DOMAIN_ID='24');proc=subprocess.Popen(args,env=env)
        client=n.create_client(GentoReplay,'/'+tag+'/playback')
        if not client.wait_for_service(timeout_sec=8):raise RuntimeError('Private native service unavailable')
        spin(1);check(True)
        if a.with_grippers:
            from gripper_replay import Replay
            grippers=Replay(n,plan)
        if a.with_chassis:
            from chassis_replay import Replay as ChassisReplay
            chassis=ChassisReplay(n,plan,return_after=True)
            spin(.3)
        check(True)
        joints=state['joints'][1]
        measured=dict(zip(joints.name,joints.position))
        return_start={name:measured[name] for name in [WAIST]+[j for group in JOINT_GROUPS for j in group]}
        from waist_replay import WaistReplay
        waist=WaistReplay(n,plan,call,return_start[WAIST])
        waist.enable(spin)
        if chassis is not None:
            chassis.release_parking(spin)
            chassis.begin_capture()
        # Native replay selects the IK source; the final joint mux is separate.
        # The previous post-replay return leaves it in idle.
        call(return_body.mux,return_body.Int.Request(data=1))
        qp_routed=True
        log.append({'joint_input':'teleop_qp'})
        started=True;call(client,request(1));begin=time.monotonic();active=False
        while time.monotonic()-begin<plan['duration_ns']/1e9+8:
            spin(.02)
            waist.tick()
            if proc.poll() is not None:raise RuntimeError('Native process exited')
            if 'private' in state:
                status=json.loads(state['private'][1].data)
                if status.get('playback_active'):active=True
                if active and not status.get('playback_active'):break
            if time.monotonic()-begin>1:
                if state.get('mux',(0,None))[1] is not None and state['mux'][1].data==1 and time.monotonic()-begin>=plan['duration_ns']/1e9-.1:
                    # Native completion status and mux restoration use different DDS topics.
                    spin(.3)
                    status=json.loads(state['private'][1].data)
                    if active and not status.get('playback_active'):break
                check()
            if time.monotonic()-begin>2 and count[0]==0:raise RuntimeError('No native IK output')
        else:raise RuntimeError('Playback completion timeout')
        if not count[0] or time.monotonic()-begin<plan['duration_ns']/1e9-.5:
            raise RuntimeError('回放提前结束，取消自动复位')
        if not waist.sent:
            raise RuntimeError('腰部轨迹未发出，不能标记回放完成')
        if chassis is not None:
            has_motion=any(any(abs(v)>1e-8 for v in (m.twist.linear.x,m.twist.linear.y,m.twist.angular.z))
                           for _,m in chassis.rows)
            if has_motion and not chassis.nonzero_sent:
                raise RuntimeError('录制含底盘运动，但未发出非零速度')
        final_waist=dict(zip(state['joints'][1].name,state['joints'][1].position))[WAIST]
        log.append({'waist_playback':{'recorded_start_m':waist.heights[0],'recorded_end_m':waist.heights[-1],
                                     'measured_end_m':final_waist}})
        if time.monotonic()-state['joints'][0]>.3 or abs(final_waist-waist.heights[-1])>.02:
            raise RuntimeError('回放结束时腰高反馈无效，或与录制末帧相差超过 20 mm')
        log.append({'result':'completed','observed_ik_messages':count[0],
                    'waist_commands':waist.sent,'nonzero_chassis_commands':chassis.nonzero_sent if chassis else 0})
        if chassis is not None:chassis.pause()
        waist.close()
        if qp_routed:
            try:call(return_body.mux,return_body.Int.Request(data=0))
            except Exception as e:log.append({'joint_input_stop_error':str(e)})
        if grippers is not None:grippers.close();grippers=None
        call(client,request(0));started=False
        proc.terminate();proc.wait(timeout=3);proc=None
        print('回放完成，停顿 3 秒后复位',flush=True)
        spin(3.)
        if chassis is not None:
            sample=chassis.sample()
            if sample is None or abs(sample.vx)>.01 or abs(sample.vy)>.01 or abs(sample.wz)>.02:
                raise RuntimeError('底盘未确认停稳，取消自动复位')
        return_body.run(return_start,state,spin,call)
        qp_routed=False
        log.append({'body_and_waist_return':'confirmed'})
        if chassis is not None:
            chassis.return_to_start(spin)
            log.append({'chassis_return':'confirmed'})
        print('已回到本次回放起点',flush=True)
    except BaseException as exc:
        log.append({'replay_error':str(exc) or type(exc).__name__})
        raise
    finally:
        if waist is not None:
            try:waist.close()
            except Exception as e:log.append({'waist_stop_error':str(e)})
        if chassis is not None:
            try:chassis.close();log.append({'chassis_stop':'zero_sent'})
            except Exception as e:log.append({'chassis_stop_error':str(e)})
        if grippers is not None:
            grippers.close();grippers=None
        if started and client is not None:
            try:call(client,request(0));log.append({'native_stop':'confirmed'})
            except Exception as e:log.append({'native_stop_error':str(e)})
        if qp_routed:
            try:call(return_body.mux,return_body.Int.Request(data=0))
            except Exception as e:log.append({'joint_input_stop_error':str(e)})
        if grippers is not None:grippers.close()
        if proc is not None:
            proc.terminate()
            try:proc.wait(timeout=3)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
        if out:json.dump(log,out,indent=2);out.close()
        if lock is not None:lock.close()
        n.destroy_node();rclpy.shutdown()
if __name__=='__main__':
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
        signal.signal(sig,signal.default_int_handler)
    main()
