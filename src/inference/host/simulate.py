#!/usr/bin/env python3
"""Offline preview of saved native server replies using production IK/planning.
No RobotClient construction, ROS imports, SSH calls or command publishers.
MuJoCo forward kinematics only: no claim of dynamics, collision-free execution,
contact success, controller tracking or real-robot validation.
"""
import argparse
import html
import json
import os
from pathlib import Path
import sys
import time
import numpy as np
from kinematics import JOINTS,Kinematics
from protocol import decode_array
from trajectory import plan_rows,joint_vector

ROOT=Path(__file__).resolve().parents[3]


def write_video(model_path,times,q,base,output,fps=20):
    os.environ.setdefault("MUJOCO_GL","egl")
    import mujoco
    import imageio_ffmpeg
    from PIL import Image,ImageDraw
    model=mujoco.MjModel.from_xml_path(str(model_path));data=mujoco.MjData(model)
    key=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_KEY,"home")
    if key>=0:mujoco.mj_resetDataKeyframe(model,data,key)
    addresses=[int(model.joint(name).qposadr[0]) for name in JOINTS]
    root=model.body("base_link");root_id=root.id;origin=model.body_pos[root_id].copy()
    original_quat=model.body_quat[root_id].copy()
    renderer=mujoco.Renderer(model,height=360,width=480)
    option=mujoco.MjvOption();option.geomgroup[3]=0
    cameras=[]
    for azimuth in [135,45]:
        cam=mujoco.MjvCamera();cam.lookat[:]=[.1,0,.9];cam.distance=2.8;cam.azimuth=azimuth;cam.elevation=-15;cameras.append(cam)
    writer=imageio_ffmpeg.write_frames(str(output),(960,400),fps=fps,codec="libx264",pix_fmt_in="rgb24",output_params=["-crf","23"],ffmpeg_log_level="error")
    writer.send(None)
    samples=np.arange(0,times[-1]+.5/fps,1/fps)
    # Kinematic root motion integrated from the same planned base velocity.
    pose=np.zeros((len(times),3))
    for i in range(1,len(times)):
        dt=times[i]-times[i-1];v,w=base[i-1]
        mid=pose[i-1,2]+w*dt/2
        pose[i]=pose[i-1]+[v*np.cos(mid)*dt,v*np.sin(mid)*dt,w*dt]
    try:
        for frame,t in enumerate(samples):
            j=min(np.searchsorted(times,t,side="right")-1,len(times)-1)
            j=max(j,0)
            data.qpos[addresses]=q[j]
            model.body_pos[root_id]=origin+[pose[j,0],pose[j,1],0]
            yaw=pose[j,2];model.body_quat[root_id]=[np.cos(yaw/2),0,0,np.sin(yaw/2)]
            mujoco.mj_forward(model,data)
            canvas=Image.new("RGB",(960,400),(18,24,33));draw=ImageDraw.Draw(canvas)
            for k,cam in enumerate(cameras):
                cam.lookat[:2]=pose[j,:2]+[.1,0]
                renderer.update_scene(data,camera=cam,scene_option=option)
                canvas.paste(Image.fromarray(renderer.render()),(k*480,40))
            draw.text((14,9),f"{output.stem} | t={t:.2f}/{times[-1]:.2f}s | KINEMATIC PREVIEW ONLY",fill="white")
            draw.text((14,25),"No ROS / no robot IO / no physical contacts simulated",fill="#ffcd83")
            if frame==0 and output.stem=="planned":canvas.save(output.parent/"initial_virtual.png")
            writer.send(np.asarray(canvas).tobytes())
            if frame%200==0:print(f"render {output.stem}: {frame}/{len(samples)}",flush=True)
    finally:
        writer.close();renderer.close();model.body_pos[root_id]=origin;model.body_quat[root_id]=original_quat
    return dict(frames=len(samples),duration_s=float(times[-1]),fps=fps)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input",type=Path,required=True,help="saved client diagnostic/chunk JSON, local only")
    parser.add_argument("--model",default="ours_a1")
    parser.add_argument("--task",choices=["gouwu","toulaji"],default="toulaji")
    parser.add_argument("--output",type=Path,default=ROOT/"tests/inference/simulation_latest")
    parser.add_argument("--no-render",action="store_true")
    parser.add_argument("--query-server",action="store_true",help="optional inference from saved observation; connects only to policy server, never robot")
    args=parser.parse_args(argv)
    out=args.output.resolve();out.relative_to((ROOT/"tests").resolve());out.mkdir(parents=True,exist_ok=True)
    # Import request parsing only here. No live objects are constructed.
    from main import load_configuration,response_rows,task_spec
    config=load_configuration(ROOT/"configs/inference/default.json",args.model);settings=config["inference"]
    record=json.loads(args.input.read_text(),object_hook=decode_array)
    request=record["request"];reply=record["response"];observation=record["observation"]
    expected_task=settings["tasks"][args.task].get("server_task",args.task)
    if request["task"] not in (args.task,expected_task):raise ValueError("input task differs from selected task")
    if args.query_server:
        from main import server_address,check_info
        from protocol import PolicyClient,encode_array
        request=dict(request,task=expected_task,model=config["server"]["model"])
        request["observation"]=dict(request["observation"])
        obs=request["observation"]
        fields=["image_timestamps_ns","image_target_timestamps_ns","image_padding_mask"]
        if len(settings["image_history_offsets_s"])==1:
            obs["images"]={"head_color":obs["images"]["head_color"][-1:]}
            for field in fields:
                if field in obs:obs[field]=obs[field][-1:]
        import uuid
        request["request_id"]=uuid.uuid4().hex
        host,port,tunnel=server_address(config)
        try:
            with PolicyClient(host,port,config["server"]["timeout_s"],wire_protocol=config["server"]["wire_protocol"]) as policy:
                info=policy.request("info")
                check_info(info,config,args.task,task_spec(settings,args.task)[0],settings["execute_steps"])
                request["session_id"]=policy.request("reset")["session_id"]
                reply=policy.request_object(request)
            record=dict(request=request,response=reply,observation=observation,
                mode="saved_real_observation_fresh_policy",server_info=info,
                policy_query_time_ns=time.time_ns(),robot_commands_sent=0)
            (out/"server_chunk.json").write_text(json.dumps(record,default=encode_array))
        finally:
            if tunnel:tunnel.close()
    elif request.get("model",args.model)!=config["server"]["model"]:
        raise ValueError("saved model differs; use matching --model or explicit --query-server")
    k=Kinematics(ROOT/"src/inference/models/runtime_expanded.urdf",ROOT/"src/inference/models/robot_Gento_Sky.xml")
    count=min(settings["execute_steps"],len(reply["actions"]))
    start=time.perf_counter()
    rows=response_rows(reply,request["request_id"],task_spec(settings,args.task)[1],count,settings,observation,k)
    ik_s=time.perf_counter()-start
    execution_observation=record.get("execution_observation",observation)
    plan=plan_rows(rows,execution_observation,k,settings,config["grippers"],
                   startup_transition=record.get("plan_report",{}).get("startup_transition",True))
    initial=dict(zip(execution_observation["joint_names"],execution_observation["joint_positions"]))
    raw=np.vstack([[initial[n] for n in JOINTS],[joint_vector(row) for row in rows]])
    raw_times=np.arange(len(raw))*settings["action_dt_s"]
    base=np.array([[x["vx"],x["wz"]] for x in plan.rows])
    raw_base=np.vstack([np.zeros(2),[[x["vx"],x["wz"]] for x in rows]])
    # The viewer starts from measured q, not the MJCF home keyframe.
    (out/"observed_current.png").write_bytes(__import__("base64").b64decode(request["observation"]["images"]["head_color"][-1]))
    initial_state=np.asarray(request["observation"]["state"])
    first_action=np.asarray(reply["actions"])[0]
    initial_differences={name:{"measured_xyz_m":initial_state[offset:offset+3].tolist(),
        "first_target_xyz_m":first_action[offset:offset+3].tolist(),
        "position_delta_m":float(np.linalg.norm(first_action[offset:offset+3]-initial_state[offset:offset+3]))}
        for offset,name in [(0,"flange_L"),(9,"flange_R"),(18,"eyes_link")]}
    (out/"initial_pose_comparison.json").write_text(json.dumps(initial_differences,indent=2))
    report=dict(input=str(args.input.resolve()),input_timestamp_ns=request["observation_timestamp_ns"],
        observation_mode=record.get("mode","saved_real_observation"),initial_pose_differences=initial_differences,
        initial_joint_positions=execution_observation["joint_positions"],task=args.task,model=args.model,robot_io=False,
        simulation="MuJoCo forward kinematics, no dynamics/contact/tracking validation",ik_seconds=ik_s,
        plan=plan.report,ik_diagnostics=k.last_ik_diagnostics)
    # Quantify joint interpolation's deviation from a linear/SLERP EEF path.
    from scipy.spatial.transform import Rotation,Slerp
    waypoint_fk=[k.forward(q) for q in plan.knot_q]
    errors={name:[] for name in ["flange_L","flange_R","eyes_link"]}
    for i in range(0,len(plan.times),10):
        t=plan.times[i];segment=min(max(np.searchsorted(plan.knots,t,side="right")-1,0),len(plan.knots)-2)
        alpha=(t-plan.knots[segment])/(plan.knots[segment+1]-plan.knots[segment])
        actual=k.forward(plan.q[i])
        for name in errors:
            a,b=waypoint_fk[segment][name],waypoint_fk[segment+1][name]
            position=a[:3,3]*(1-alpha)+b[:3,3]*alpha
            rotation=Slerp([0,1],Rotation.from_matrix(np.stack([a[:3,:3],b[:3,:3]])))([alpha]).as_matrix()[0]
            errors[name].append([float(np.linalg.norm(actual[name][:3,3]-position)),
                float(np.linalg.norm(Rotation.from_matrix(rotation@actual[name][:3,:3].T).as_rotvec()))])
    report["interpolation_deviation"]={name:{"max_position_m":max(v[0] for v in values),"max_rotation_rad":max(v[1] for v in values)} for name,values in errors.items()}
    # Verify viewer's robot model agrees with production FK at measured start.
    os.environ.setdefault("MUJOCO_GL","egl")
    import mujoco
    model_path=ROOT/"src/inference/models/simulation.xml"
    model=mujoco.MjModel.from_xml_path(str(model_path));data=mujoco.MjData(model)
    for name,value in zip(JOINTS,plan.q[0]):data.qpos[int(model.joint(name).qposadr[0])]=value
    mujoco.mj_forward(model,data);fk=k.forward(plan.q[0])
    report["viewer_fk_position_error_m"]={name:float(np.linalg.norm(data.xpos[model.body(name).id]-fk[name][:3,3])) for name in errors}
    if max(report["viewer_fk_position_error_m"].values())>1e-5:raise ValueError("viewer robot differs from production FK")
    np.savez_compressed(out/"trajectory.npz",time_s=plan.times,q=plan.q,raw_time_s=raw_times,raw_q=raw,base_velocity=base,knots=plan.knots,knot_q=plan.knot_q)
    if not args.no_render:
        report["raw_video"]=write_video(model_path,raw_times,raw,raw_base,out/"raw.mp4")
        report["planned_video"]=write_video(model_path,plan.times,plan.q,base,out/"planned.mp4")
    (out/"report.json").write_text(json.dumps(report,indent=2))
    stride=max(1,len(plan.times)//1500)
    payload=json.dumps(dict(names=list(JOINTS),t=plan.times[::stride].tolist(),q=plan.q[::stride].tolist(),raw_t=raw_times.tolist(),raw_q=raw.tolist(),report=report))
    page="""<!doctype html><meta charset=utf-8><title>SKYE client offline simulation</title>
<style>body{background:#111823;color:#e0e7f1;font:16px system-ui;margin:24px}video{width:48%;background:#000}button,select{padding:8px}pre{white-space:pre-wrap}svg{background:#192435;width:100%;height:150px}section{margin:18px 0}a{color:#82c6ff}</style>
<h1>SKYE / client trajectory preview</h1><div id=initial></div><p>Kinematic preview only. No robot connection. No dynamics, contact, collision-free or tracking claim.</p>
<p>LEFT: raw 20 Hz targets (unsafe discontinuities shown only in simulation). RIGHT: production planned trajectory. Independent real-time durations; speed is adjustable.</p>
<video id=raw controls src=raw.mp4></video> <video id=planned controls src=planned.mp4></video>
<section><button onclick="raw.currentTime=planned.currentTime=0;raw.play();planned.play()">Play from start</button> Playback rate <select onchange="raw.playbackRate=planned.playbackRate=+this.value"><option>1</option><option>2</option><option>4</option><option>8</option></select></section>
<p><a href=report.json>Full metrics and IK residuals</a> | <a href=trajectory.npz>Joint trajectory arrays</a></p>
<pre id=summary></pre><div id=plots></div><script>const D=__DATA__;
initial.innerHTML=`<p>Input: ${D.report.observation_mode}; timestamp ${D.report.input_timestamp_ns} ns. Virtual start uses measured joints. One policy request; no robot execution.</p><p>Real observed head image / virtual measured start</p><img width="320" src="observed_current.png"><img width="640" src="initial_virtual.png"><pre>${JSON.stringify(D.report.initial_pose_differences,null,2)}</pre>`;
summary.textContent=JSON.stringify({plan:D.report.plan,interpolation_deviation:D.report.interpolation_deviation},null,2);
D.names.forEach((name,j)=>{let vals=D.q.map(x=>x[j]),r=D.raw_q.map(x=>x[j]),lo=Math.min(...vals,...r),hi=Math.max(...vals,...r);if(hi-lo<.01){lo-=.01;hi+=.01}let line=(t,q,end)=>t.map((x,i)=>`${30+x/end*1120},${130-(q[i][j]-lo)/(hi-lo)*100}`).join(' ');let section=document.createElement('section');section.innerHTML=`<b>${name} — planned time scale (${D.t.at(-1).toFixed(1)}s), range ${lo.toFixed(3)}..${hi.toFixed(3)}</b><svg viewBox="0 0 1200 150"><polyline fill="none" stroke="#58d7b1" stroke-width="2" points="${line(D.t,D.q,D.t.at(-1))}"/></svg>`;plots.append(section)});
</script>""".replace("__DATA__",payload.replace("</", "<" + chr(92) + "/"))
    (out/"index.html").write_text(page)
    print(json.dumps(dict(output=str(out),report=plan.report,robot_io=False)),flush=True)
    return 0

if __name__=="__main__":raise SystemExit(main())
