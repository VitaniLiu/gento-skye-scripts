"""Base-relative Action78 alignment; validity never inferred from zero values."""
from pathlib import Path
import numpy as np
from processing.schema78 import STEP_NS, previous, VALIDITY

VERSION="gento_skye_action78_base_rows_v2"
def preflight_base(s):
    errors=[]
    receive=s.get("time_alignment",{}).get("kind")=="replay_receive_time_v1"
    if receive and (s["time_alignment"].get("bag_subdir")!="domain_24" or s["time_alignment"].get("physical_sync_verified") is not False):errors.append("receive timeline must identify domain_24 and unverified physical sync")
    if s.get("fps")!=20 or s.get("schema")!=VERSION:errors.append("schema/fps mismatch")
    if s.get("pose_reference",{}).get("name")!="base_link":errors.append("EEF output must be base_link")
    if s.get("rotation6d",{}).get("order")!=["R00","R01","R02","R10","R11","R12"]:errors.append("rotation row order mismatch")
    if s.get("masks",{}).get("static_applicability")!=VALIDITY.astype(bool).tolist():errors.append("structural mask mismatch")
    if len(s.get("names",[]))!=78 or len(set(s["names"]))!=78:errors.append("78 unique fields required")
    if [c["role"] for c in s.get("cameras",[])]!=["head"]:errors.append("exactly one head view required")
    for c in s.get("cameras",[]):
        if not c.get("topic") or not c.get("verified") or not c.get("identity_evidence"):errors.append("head identity/topic unverified")
        if c.get("packet_frame_rule")!="one_access_unit_per_message" or not c.get("packet_frame_evidence"):errors.append("head H264 access unit/time correspondence unverified")
    for side in ("action","state"):
        covered=[i for b in s["bindings"] if b.get("target")==side for i in b.get("indices",[])]
        active=[i for i,x in enumerate(VALIDITY) if x]
        if sorted(covered)!=active:errors.append(side+": each applicable slot must have exactly one mapping entry, even if unverified")
    enabled=[b for b in s["bindings"] if b.get("verified")]
    if not enabled:errors.append("no evidenced physical binding enabled; masked all-zero dataset prohibited")
    for b in enabled+s["cameras"]:
        clock=b.get("clock",{})
        if not clock.get("verified") or not clock.get("evidence") or clock.get("source") not in ("bag","header") or not clock.get("source_domain"):
            errors.append("unverified clock for "+str(b.get("indices",b.get("role"))))
            continue
        for name in ("offset_ns","valid_from_ns","valid_until_ns","uncertainty_ns"):
            if type(clock.get(name)) is not int:errors.append("missing integer clock field "+name)
        if receive and (clock.get("source")!="bag" or clock.get("target_domain")!="robot_domain24_mcap_receive" or clock.get("source_domain")!="robot_domain24_mcap_receive" or clock.get("offset_ns")!=0 or clock.get("scale")!=1):errors.append("receive policy requires recorder identity mapping")
        scale=clock.get("scale")
        if type(scale) not in (int,float) or not np.isfinite(scale) or scale<=0:errors.append("clock scale must be positive finite")
        age=b.get("max_age_ns")
        if type(age) is not int or not 0<age<=1_000_000_000:errors.append("bounded sample hold age required")
        uncertainty=clock.get("uncertainty_ns")
        if type(uncertainty) is int and (uncertainty<0 or type(age) is int and uncertainty>age):errors.append("clock uncertainty outside sampling tolerance")
        lo,hi=clock.get("valid_from_ns"),clock.get("valid_until_ns")
        if type(lo) is int and type(hi) is int and lo>hi:errors.append("clock interval reversed")
        if "role" in b and clock.get("source")!="header" and not receive:errors.append("image PTS requires explicit source header clock mapping")
        if b.get("kind") in ("fk","fk_command") and not s.get("kinematics",{}).get("historical_model_match_verified"):errors.append("capture-time kinematic model unverified")
        if b.get("calibration") and not b["calibration"].get("verified"):errors.append("gripper calibration unverified")
        if b.get("target") and (not b.get("evidence") or not b.get("topic") or b.get("path") is None):errors.append("binding source missing")
    domains={b.get("clock",{}).get("target_domain") for b in enabled+s["cameras"]}
    if len(domains)!=1 or None in domains:errors.append("one evidenced clock target required")
    return [f"{Path(__file__).resolve()}:preflight_base: {e}" for e in errors]

def align_base(s,streams,images):
    head=images.get("head",{})
    if not head.get("times"):return [],{"skipped":"head frames missing"}
    enabled=[i for i,b in enumerate(s["bindings"]) if b.get("verified") and b.get("clock",{}).get("verified")]
    commands=streams.get("command_targets")
    bounds=[streams[i]["times"] for i in enabled if streams[i]["times"]]
    if commands:bounds.extend(x["times"] for x in commands.streams.values() if x["times"])
    # Numeric stream absence is represented by masks; image timing defines the export span.
    start=head["times"][0]
    end=head["times"][-1]
    receive=s.get("time_alignment",{}).get("kind")=="replay_receive_time_v1"
    origin=s.get("time_alignment",{}).get("origin_ns") if receive else 0
    if type(origin) is not int:raise ValueError("missing actual recorder timeline origin")
    start=origin+((start-origin+STEP_NS-1)//STEP_NS)*STEP_NS
    if (end-start)//STEP_NS>2_000_000:raise ValueError(f"{Path(__file__).resolve()}:align_base: timeline exceeds bound")
    static=np.asarray(s["masks"]["static_applicability"],dtype=bool)
    segments=[];segment=[];missing=[];skipped=0;fk=None
    for t in range(start,end+1,STEP_NS):
        try:im=previous(head,t,s["cameras"][0]["max_age_ns"])
        except ValueError:
            skipped+=1
            if segment:segments.append(segment);segment=[]
            continue
        frame={"source.grid_time_ns":np.array([t],np.int64),"observation.images.head":im["image"],
               "source.camera_head_time_ns":np.array([im["source_ns"]],np.int64),
               "quality.camera_head_age_ns":np.array([t-im["mapped_ns"]],np.int64),"validity":static.copy()}
        if receive:
            frame["source.replay_offset_ns"]=np.array([t-origin],np.int64)
            frame["source.camera_head_receive_time_ns"]=np.array([im["receive_ns"]],np.int64)
        sides={}
        for side in ("state","action"):
            sides[side]={"value":np.zeros(78,np.float32),"valid":np.zeros(78,bool),
                         "source":np.full(78,-1,np.int64),"age":np.full(78,-1,np.int64),
                         "reason":np.where(static,1,3).astype(np.uint8)}
        if receive:
            for row in sides.values():row["header"]=np.full(78,-1,np.int64)
        command_item=None;command_error=None
        if s.get("command_joint_bundle",{}).get("enabled"):
            n=len(s["command_joint_bundle"]["groups"])
            for key in ("source.command_group_receive_ns","source.command_group_header_ns","quality.command_group_age_ns"):frame[key]=np.full(n,-1,np.int64)
            try:
                if commands is None:raise ValueError("recorded joint target groups missing")
                if fk is None:
                    from processing.kinematics import Kinematics
                    k=s["kinematics"];fk=Kinematics(k["urdf"],k["mjcf"],k.get("hashes"))
                command_item=commands.at(t,fk)
                for key,field in (("source.command_group_receive_ns","receive_times"),("source.command_group_header_ns","header_times"),("quality.command_group_age_ns","ages")):frame[key]=command_item[field]
            except (ValueError,KeyError,IndexError) as exc:command_error=str(exc)
        for i in enabled:
            b=s["bindings"][i];row=sides[b["target"]];idx=b["indices"]
            try:
                if b.get("kind")=="fk_command":
                    if command_item is None:raise ValueError(command_error or "joint target bundle unavailable")
                    item={**command_item,"values":command_item["poses"][b["eef"]],"mapped_ns":command_item["source_ns"]}
                else:item=previous(streams[i],t,b["max_age_ns"])
                value=np.asarray(item["values"],np.float32)
                if len(value)!=len(idx) or not np.isfinite(value).all():raise ValueError("invalid mapped values")
                if receive:row["header"][idx]=item.get("header_ns",-1)
                row["value"][idx]=value;row["valid"][idx]=True
                row["source"][idx]=item["source_ns"];row["age"][idx]=t-item["mapped_ns"];row["reason"][idx]=0
            except (ValueError,IndexError,KeyError) as exc:
                row["reason"][idx]=2
                if len(missing)<1000:missing.append({"grid_ns":t,"side":b["target"],"indices":idx,"reason":str(exc)})
        for side,row in sides.items():
            if receive:frame["source."+side+"_header_time_ns"]=row["header"]
            row["valid"] &= static;row["value"][~row["valid"]]=0
            frame["observation.state" if side=="state" else "action"]=row["value"]
            frame["quality."+side+"_valid"]=row["valid"]
            frame["quality."+side+"_reason"]=row["reason"]
            frame["source."+side+"_time_ns"]=row["source"]
            frame["quality."+side+"_age_ns"]=row["age"]
        segment.append(frame)
        if len(segment)>=s.get("max_segment_frames",20000):segments.append(segment);segment=[]
    if segment:segments.append(segment)
    return segments,{"exportable_frames":sum(map(len,segments)),"skipped_frames":skipped,"missing_samples":missing,
                     "reason_codes":{"0":"valid","1":"mapping/calibration unverified","2":"missing/stale/invalid sample","3":"not applicable"},
                     "time_alignment":s.get("time_alignment"),"sampling":"20Hz causal bounded hold; no interpolation, no future shift; missing dimension masked"}
