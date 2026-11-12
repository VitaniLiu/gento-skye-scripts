#!/usr/bin/env python3
"""Read the real robot once, query policy once, execute only in virtual preview."""
import argparse,base64,json,sys,time,uuid
from pathlib import Path
import numpy as np
from main import (RobotClient,ROOT,load_configuration,aligned_observation,
                  build_infer_request,task_spec,request_task,check_info,server_address)
from kinematics import Kinematics,JOINTS
from protocol import PolicyClient,encode_array

class ReadOnlyRobotClient(RobotClient):
    endpoint="observe_only.py"
    def send(self,command,**fields):
        if command not in ("observe","quit"):raise ValueError("read-only robot connection")
        return super().send(command,**fields)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model",default="ours_a1")
    p.add_argument("--task",choices=["toulaji","gouwu"],default="toulaji")
    p.add_argument("--output",type=Path,default=ROOT/"tests/inference/live_simulation_latest")
    p.add_argument("--no-render",action="store_true")
    args=p.parse_args(argv)
    out=args.output.resolve();out.relative_to((ROOT/"tests").resolve());out.mkdir(parents=True,exist_ok=True)
    config=load_configuration(ROOT/"configs/inference/default.json",args.model);s=config["inference"]
    k=Kinematics(ROOT/"src/inference/models/runtime_expanded.urdf",ROOT/"src/inference/models/robot_Gento_Sky.xml")
    robot=None
    try:
        print("Reading real robot: subscriptions only, no reset or command publishers",flush=True)
        robot=ReadOnlyRobotClient(config,task=args.task);robot.wait("ready",timeout=15)
        needed=-min(s["image_history_offsets_s"])*1e9;deadline=time.monotonic()+40
        while True:
            with robot.history.lock:
                frames=list(robot.history.frames)
            if frames and frames[-1][0]-frames[0][0]>=needed:break
            if time.monotonic()>deadline:raise TimeoutError("decoded camera history not ready")
            if robot.process.poll() is not None:raise RuntimeError("read-only endpoint closed")
            if not robot.messages.empty():
                message=robot.messages.get_nowait()
                if message.get("type")=="error":raise RuntimeError(message["error"])
            time.sleep(.02)
        observation,images=aligned_observation(robot)
        instruction,mask=task_spec(s,args.task)
        request=build_infer_request(s,request_task(s,args.task),instruction,mask,uuid.uuid4().hex,"pending",observation,images,k,config["grippers"])
        robot.send("quit");robot.close();robot=None
    finally:
        if robot is not None:robot.close()
    # Persist measured snapshot even if the policy service is unavailable.
    capture=dict(request=request,observation=observation,mode="live_real_observation_to_virtual_only",robot_commands_sent=0)
    (out/"measured_observation.json").write_text(json.dumps(capture,default=encode_array))
    for index,image in enumerate(images["head_color"]):
        (out/f"observed_{index}.png").write_bytes(base64.b64decode(image))
    print("Real snapshot acquired; robot connection closed. Querying policy...",flush=True)
    host,port,tunnel=server_address(config)
    try:
        with PolicyClient(host,port,config["server"]["timeout_s"],wire_protocol=config["server"]["wire_protocol"]) as policy:
            info=policy.request("info");check_info(info,config,args.task,instruction,s["execute_steps"])
            request["session_id"]=policy.request("reset")["session_id"]
            reply=policy.request_object(request)
    finally:
        if tunnel:tunnel.close()
    capture["response"]=reply;capture["server_info"]=info
    (out/"server_chunk.json").write_text(json.dumps(capture,default=encode_array))
    # Explain the initial flange mismatch using measured state vs first target.
    from scipy.spatial.transform import Rotation
    state=np.asarray(request["observation"]["state"]);action=np.asarray(reply["actions"])[0]
    differences={}
    for offset,name in [(0,"flange_L"),(9,"flange_R"),(18,"eyes_link")]:
        def rot(v):
            cols=v.reshape(2,3);x=cols[0]/np.linalg.norm(cols[0]);y=cols[1]-x*np.dot(x,cols[1]);y/=np.linalg.norm(y)
            return np.column_stack([x,y,np.cross(x,y)])
        angle=Rotation.from_matrix(rot(action[offset+3:offset+9])@rot(state[offset+3:offset+9]).T).magnitude()
        differences[name]=dict(measured_xyz_m=state[offset:offset+3].tolist(),first_target_xyz_m=action[offset:offset+3].tolist(),position_delta_m=float(np.linalg.norm(action[offset:offset+3]-state[offset:offset+3])),rotation_delta_deg=float(np.degrees(angle)))
    print("Initial target differences: "+json.dumps(differences),flush=True)
    (out/"initial_pose_comparison.json").write_text(json.dumps(differences,indent=2))
    from simulate import main as preview
    command=["--input",str(out/"server_chunk.json"),"--model",args.model,"--task",args.task,"--output",str(out)]
    if args.no_render:command.append("--no-render")
    return preview(command)

if __name__=="__main__":raise SystemExit(main())
