"""Standard LeRobot writer for base-relative rows6D with head-only media."""
from pathlib import Path
import json
import numpy as np
from processing.exporter import image_array
from processing.dataset_stats import stable_episode_stats
from processing.base78 import VERSION

def physical_stats(values, valid):
    values=np.asarray(values,dtype=np.float64);valid=np.asarray(valid,dtype=bool)
    result={k:np.zeros(78) for k in ("min","max","mean","std","q01","q10","q50","q90","q99")}
    counts=valid.sum(0)
    for i in range(78):
        x=values[valid[:,i],i]
        if not len(x):continue
        for key,fn in (("min",np.min),("max",np.max),("mean",np.mean),("std",np.std)):result[key][i]=fn(x)
        for key,q in (("q01",.01),("q10",.1),("q50",.5),("q90",.9),("q99",.99)):result[key][i]=np.quantile(x,q)
    # LeRobot requires scalar frame count. Per-dimension valid counts are explicit metadata.
    result["count"]=np.array([len(values)],np.int64)
    return result,counts

def masked_episode_stats(data,features):
    stats=stable_episode_stats(data,features)
    for key,side in (("action","action"),("observation.state","state")):
        stats[key],_=physical_stats(data[key],data["quality."+side+"_valid"])
    return stats

def write_dataset(root,segments,schema,task,provenance):
    import importlib.metadata
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    import lerobot.datasets.lerobot_dataset as impl
    if schema.get("schema")!=VERSION:raise ValueError(f"{Path(__file__).resolve()}:write_dataset: incompatible schema")
    if importlib.metadata.version("lerobot")!="0.4.3":raise ValueError("LeRobot 0.4.3 required")
    root=Path(root)
    if root.exists():raise FileExistsError(str(root))
    if not segments or not all(segments):raise ValueError("empty segments")
    if len(provenance)!=len(segments):raise ValueError("provenance count mismatch")
    names=schema["names"];features={}
    for side,key in (("state","observation.state"),("action","action")):
        features[key]={"dtype":"float32","shape":(78,),"names":names}
        for suffix,dtype in (("valid","bool"),("reason","uint8"),("age_ns","int64")):
            features["quality."+side+"_"+suffix]={"dtype":dtype,"shape":(78,),"names":names}
        features["source."+side+"_time_ns"]={"dtype":"int64","shape":(78,),"names":names}
    features["validity"]={"dtype":"bool","shape":(78,),"names":names}
    for key in ("source.grid_time_ns","source.camera_head_time_ns","quality.camera_head_age_ns"):
        features[key]={"dtype":"int64","shape":(1,),"names":["time_ns"]}
    if schema.get("time_alignment",{}).get("kind")=="replay_receive_time_v1":
        for key in ("source.replay_offset_ns","source.camera_head_receive_time_ns"):
            features[key]={"dtype":"int64","shape":(1,),"names":["time_ns"]}
        for side in ("state","action"):
            features["source."+side+"_header_time_ns"]={"dtype":"int64","shape":(78,),"names":names}
    if schema.get("command_joint_bundle",{}).get("enabled"):
        groups=[g["name"] for g in schema["command_joint_bundle"]["groups"]]
        for key in ("source.command_group_receive_ns","source.command_group_header_ns","quality.command_group_age_ns"):
            features[key]={"dtype":"int64","shape":(len(groups),),"names":groups}
    features["observation.images.head"]={"dtype":"image","shape":image_array(segments[0][0]["observation.images.head"]).shape,"names":["height","width","channels"]}
    dataset=LeRobotDataset.create(repo_id="local/gento_skye",fps=20,features=features,root=root,robot_type="gento_skye",use_videos=False,video_backend="pyav")
    original=impl.compute_episode_stats
    try:
        impl.compute_episode_stats=masked_episode_stats
        for seg in segments:
            for row in seg:
                f=dict(row);f["task"]=task;f["observation.images.head"]=image_array(f["observation.images.head"]);dataset.add_frame(f)
            dataset.save_episode()
        dataset.finalize()
    finally:
        impl.compute_episode_stats=original
    allrows=[f for seg in segments for f in seg];counts={}
    path=root/"meta/stats.json";stats=json.loads(path.read_text())
    for key,side in (("action","action"),("observation.state","state")):
        st,n=physical_stats([f[key] for f in allrows],[f["quality."+side+"_valid"] for f in allrows])
        stats[key]={k:v.tolist() for k,v in st.items()};counts[key]=n.tolist()
    path.write_text(json.dumps(stats,allow_nan=False,indent=2)+"\n")
    path=root/"meta/info.json";info=json.loads(path.read_text())
    info["gento_skye"]={"schema":VERSION,"frame":"base_link","rotation6d":"first_two_rows","gripper_range":[0,1],
       "time_alignment":schema.get("time_alignment"),"source_mapping":schema,"provenance":provenance,"valid_count_per_dimension":counts,
       "statistics_contract":"physical stats exclude false mask; count is total frames as required by LeRobot; count-zero dimensions use numeric placeholders, not observations; training must apply validity masks",
       "missing_reason_codes":{"0":"valid","1":"unverified mapping","2":"sample missing/stale/invalid","3":"not applicable"},
       "status":"partial; masks and reasons authoritative"}
    path.write_text(json.dumps(info,ensure_ascii=False,allow_nan=False,indent=2)+"\n")
    return verify_dataset(root,segments)

def verify_dataset(root,expected):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    import pyarrow.parquet as pq
    import pyarrow as pa
    ds=LeRobotDataset("local/gento_skye",root=Path(root),video_backend="pyav")
    n=0
    for ep,seg in enumerate(expected):
        for i,ref in enumerate(seg):
            row=ds[n]
            if int(row["episode_index"])!=ep or int(row["frame_index"])!=i or int(row["index"])!=n:raise ValueError("index mismatch")
            if not np.isclose(float(row["timestamp"]),i/20,rtol=0,atol=1e-4):raise ValueError("20Hz mismatch")
            static=row["validity"].numpy()
            for side,key in (("state","observation.state"),("action","action")):
                a=row[key].numpy();mask=row["quality."+side+"_valid"].numpy()
                if a.shape!=(78,) or a.dtype!=np.float32 or mask.dtype!=bool or np.any(mask & ~static):raise ValueError("shape/dtype/mask mismatch")
                if np.any(a[~mask]!=0) or not np.isfinite(a).all():raise ValueError("invalid masked values")
                for start in (0,9,18):
                    if mask[start+3:start+9].all():
                        x,y=a[start+3:start+6],a[start+6:start+9]
                        if not np.allclose([x@x,y@y,x@y],[1,1,0],atol=1e-5):raise ValueError("invalid rows6d")
                for k in (54,77):
                    if mask[k] and not 0<=a[k]<=1:raise ValueError("gripper range")
            for key,val in ref.items():
                if key=="observation.images.head":
                    if not np.allclose(row[key].numpy().transpose(1,2,0)*255,image_array(val),atol=.01):raise ValueError("image mismatch")
                elif not np.array_equal(np.atleast_1d(row[key].numpy()),np.atleast_1d(val)):raise ValueError("roundtrip mismatch: "+key)
            n+=1
    if len(ds)!=n:raise ValueError("frame count mismatch")
    for p in (Path(root)/"data").rglob("*.parquet"):
        table=pq.read_table(p)
        for key,dtype in (("action",pa.float32()),("observation.state",pa.float32()),("quality.state_valid",pa.bool_()),("quality.action_valid",pa.bool_())):
            if table.schema.field(key).type.value_type!=dtype:raise ValueError("Parquet dtype mismatch")
    info=json.loads((Path(root)/"meta/info.json").read_text())
    rows=[f for seg in expected for f in seg]
    for key,side in (("action","action"),("observation.state","state")):
        st,count=physical_stats([f[key] for f in rows],[f["quality."+side+"_valid"] for f in rows])
        for k,v in st.items():
            if not np.allclose(ds.meta.stats[key][k],v):raise ValueError("masked statistics mismatch")
        if info["gento_skye"]["valid_count_per_dimension"][key]!=count.tolist():raise ValueError("valid count mismatch")
    return {"status":"verified","standard_reader":"lerobot==0.4.3 LeRobotDataset","frames":n,"episodes":len(expected),"fps":ds.fps,"schema":VERSION,"cameras":["head"]}
