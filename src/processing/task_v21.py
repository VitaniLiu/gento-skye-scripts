"""Task-level LeRobot v2.1 export, nullable absent cameras, explicit source provenance."""
from pathlib import Path
from io import BytesIO
import hashlib,json,shutil
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from PIL import Image

COLUMNS=["observation.state","action","validity","observation.images.head_color",
"observation.images.left_wrist_camera","observation.images.right_wrist_camera",
"timestamp","frame_index","episode_index","index","task_index"]
IMAGE=pa.struct([("bytes",pa.binary()),("path",pa.string())])
VERSION="gento_skye_78d_columns_v21"
def dump(path,obj):
    Path(path).write_text(json.dumps(obj,ensure_ascii=False,allow_nan=False,indent=2)+"\n",encoding="utf-8")
def jsonl(path,rows):
    Path(path).write_text("".join(json.dumps(r,ensure_ascii=False,allow_nan=False)+"\n" for r in rows),encoding="utf-8")
def digest(path):
    h=hashlib.sha256()
    with Path(path).open("rb") as f:
        for b in iter(lambda:f.read(1024*1024),b""):h.update(b)
    return h.hexdigest()
def policy_for(task):
    config=Path(__file__).resolve().parents[2]/"configs/processing/task_export_v21.json"
    cfg=json.loads(config.read_text())
    if task not in cfg["tasks"]:raise ValueError("Configure waist_height_enabled for task before export: "+task)
    p={**cfg["defaults"],**cfg["tasks"][task]}
    if type(p["waist_height_enabled"]) is not bool:raise ValueError("waist_height_enabled must be bool")
    return p
def names78(source):
    names=list(source["names"])
    for start,prefix in [(0,"left_ee"),(9,"right_ee"),(18,"head_ee")]:
        names[start:start+9]=[prefix+"_"+n for n in ("x","y","z","r1x","r1y","r1z","r2x","r2y","r2z")]
    names[27:32]=["waist_yaw","waist_pitch","waist_height","base_linear_velocity","base_angular_velocity"]
    return names
def structural(policy):
    a=np.zeros(78,np.uint8);a[:27]=1;a[30]=1;a[[54,77]]=1
    a[29]=int(policy["waist_height_enabled"])
    rotation_policy=policy.get("base_rotation_policy","locked_zero")
    if rotation_policy not in ("locked_zero","enabled"):raise ValueError("Unknown base_rotation_policy")
    a[31]=int(rotation_policy=="enabled")
    return a
def transform(values,mask,policy,valid=None):
    a=np.array(values,dtype=np.float32,copy=True)
    if a.shape!=(78,):raise ValueError("Expected 78-dimensional values")
    valid=np.asarray(mask,dtype=bool).copy() if valid is None else valid
    valid &= np.asarray(mask,bool) & np.isfinite(a)
    for start in (0,9,18):
        r1=a[start+3:start+6].astype(float);r2=a[start+6:start+9].astype(float)
        if not valid[start+3:start+9].all() or not np.allclose([r1@r1,r2@r2,r1@r2],[1,1,0],atol=1e-4):
            valid[start+3:start+9]=False
            continue
        R=np.stack((r1,r2,np.cross(r1,r2)))
        a[start+3:start+9]=R[:,:2].T.reshape(6)
    a[29]*=policy["waist_height_scale"]
    a[30]*=policy["base_forward_scale"]
    a[31]*=policy["base_clockwise_scale"]
    valid &= np.isfinite(a)
    a[~valid]=0
    return a
def image_value(value,i):
    shape=value.get("shape") if isinstance(value,dict) else None
    if isinstance(value,dict):
        b=value.get("bytes")
        if b is None:b=Path(value["path"]).read_bytes()
    elif isinstance(value,(str,Path)):b=Path(value).read_bytes()
    else:
        stream=BytesIO();Image.fromarray(np.asarray(value,dtype=np.uint8)).save(stream,format="PNG");b=stream.getvalue()
    if shape is None:
        with Image.open(BytesIO(b)) as im:
            im.load()
            if im.mode!="RGB":raise ValueError("head image must be RGB")
            shape=[3,im.height,im.width]
    return {"bytes":b,"path":f"frame-{i:06d}.png"},shape
def prepare(segments,provenance,policy):
    """Preserve every input frame. Merge input chunks belonging to one source episode."""
    mask=structural(policy).astype(bool);prepared=[];provs=[];by_source={}
    for seg,prov in zip(segments,provenance,strict=True):
        if not seg:continue
        identity=prov["source_episode_path"]
        if identity not in by_source:
            by_source[identity]=len(prepared);prepared.append([]);provs.append(dict(prov))
        k=by_source[identity];rows=prepared[k]
        for row in seg:
            out={};q={}
            for side,key in (("state","observation.state"),("action","action")):
                valid=np.asarray(row["quality."+side+"_valid"],bool).copy() & mask
                original=valid.copy()
                out[key]=transform(row[key],mask,policy,valid)
                reason=np.asarray(row.get("quality."+side+"_reason",np.where(original,0,2)),np.uint8).copy()
                reason[~mask]=3;reason[mask & ~valid]=2;reason[valid]=0
                q[side+"_valid"]=valid.tolist();q[side+"_reason"]=reason.tolist()
                for suffix,source,default in [
                    ("age_ns","quality."+side+"_age_ns",-1),
                    ("time_ns","source."+side+"_time_ns",-1),
                    ("header_time_ns","source."+side+"_header_time_ns",-1)]:
                    q[side+"_"+suffix]=np.asarray(row.get(source,np.full(78,default)),np.int64).tolist()
            out["observation.images.head_color"]=row["observation.images.head"]
            out["_quality"]=q
            out["_source"]={"source_task_path":prov.get("source_task_path",""),
                "source_episode_path":identity,"source_frame_index":len(rows),
                "source_grid_time_ns":int(np.asarray(row["source.grid_time_ns"]).ravel()[0]),
                "source_camera_time_ns":int(np.asarray(row.get("source.camera_head_time_ns",[-1])).ravel()[0])}
            rows.append(out)
    for rows,p in zip(prepared,provs,strict=True):
        p.update(source_input_frame_range=[0,len(rows)-1],source_input_frames=len(rows),
                 source_start_ns=rows[0]["_source"]["source_grid_time_ns"],
                 source_end_ns=rows[-1]["_source"]["source_grid_time_ns"],excluded_input_frames=0)
    if not prepared:raise ValueError("No decodable input frames")
    return prepared,provs
def stats_for(rows):
    result={}
    for key in ("observation.state","action","validity"):
        a=np.asarray([r[key] for r in rows],dtype=float)
        result[key]={n:fn(a,axis=0).tolist() for n,fn in (("min",np.min),("max",np.max),("mean",np.mean),("std",np.std))}
        for n,q in [("q01",.01),("q10",.1),("q50",.5),("q90",.9),("q99",.99)]:result[key][n]=np.quantile(a,q,axis=0).tolist()
        result[key]["count"]=[len(rows)]
    # Deterministic bounded sample, decoded at native resolution; values in RGB/255.
    samples=[]
    for i in np.unique(np.linspace(0,len(rows)-1,min(32,len(rows)),dtype=int)):
        with Image.open(BytesIO(rows[int(i)]["observation.images.head_color"]["bytes"])) as im:
            samples.append(np.asarray(im.resize((160,120)),dtype=float)/255)
    a=np.stack(samples)
    result["observation.images.head_color"]={n:fn(a,axis=(0,1,2)).reshape(3,1,1).tolist() for n,fn in (("min",np.min),("max",np.max),("mean",np.mean),("std",np.std))}
    result["observation.images.head_color"]["count"]=[len(samples)]
    for key in ("timestamp","frame_index","episode_index","index","task_index"):
        a=np.asarray([r[key] for r in rows],float)
        result[key]={n:[float(fn(a))] for n,fn in (("min",np.min),("max",np.max),("mean",np.mean),("std",np.std))}
        result[key]["count"]=[len(rows)]
    # Absent camera fields intentionally have no fabricated image statistics.
    return result
def load_existing(root,policy,progress=None):
    root=Path(root)
    if not root.exists():return [],[]
    info=json.loads((root/"meta/info.json").read_text())
    export=json.loads((root/"meta/export.json").read_text())
    if info["codebase_version"]!="v2.1" or export["policy"]!=policy:raise ValueError("Existing task schema/policy mismatch; explicitly rebuild task")
    provs=export["episodes"];segments=[]
    for ep in range(info["total_episodes"]):
        p=root/info["data_path"].format(episode_chunk=ep//1000,episode_index=ep)
        if progress:progress(f"读取 episode_{ep:06d} ({ep+1}/{info["total_episodes"]})")
        rows=pq.read_table(p).to_pylist()
        quality=root/"meta/frame_quality.parquet";mapping=root/"meta/source_frame_mapping.parquet"
        if quality.exists() and mapping.exists():
            qs=pq.read_table(quality,filters=[("episode_index","=",ep)]).to_pylist()
            ms=pq.read_table(mapping,filters=[("episode_index","=",ep)]).to_pylist()
            if len(qs)!=len(rows) or len(ms)!=len(rows):raise ValueError("Sidecar length mismatch")
            for row,q,m in zip(rows,qs,ms,strict=True):
                row["_quality"]={k:v for k,v in q.items() if k not in ("episode_index","frame_index","index")}
                row["_source"]={k:v for k,v in m.items() if k not in ("episode_index","frame_index","index")}
        segments.append(rows)
    return segments,provs
def write_dataset(root,segments,schema,task,provenance,existing=None,verify=True):
    policy=policy_for(task);new,newprov=prepare(segments,provenance,policy)
    old,oldprov=load_existing(existing,policy) if existing else ([],[])
    replaced={p["source_episode_path"] for p in newprov}
    keep=[i for i,p in enumerate(oldprov) if p["source_episode_path"] not in replaced]
    if any("_quality" not in row for i in keep for row in old[i]):
        raise ValueError("Retained legacy episodes lack quality metadata; reconvert the complete task first")
    combined=[old[i] for i in keep]+new;provs=[oldprov[i] for i in keep]+newprov
    return write_prepared(root,combined,schema,task,provs,policy,verify=verify)
def write_prepared(root,segments,schema,task,provenance,policy,verify=True,episode_index_offset=0,index_offset=0):
    root=Path(root)
    if root.exists():raise FileExistsError(root)
    (root/"meta").mkdir(parents=True)
    mask=structural(policy);names=names78(schema);allrows=[];eps=[];epstats=[];exports=[];shape=None;quality_rows=[];mapping_rows=[]
    fields=pa.schema([
      pa.field("observation.state",pa.list_(pa.float32(),78)),pa.field("action",pa.list_(pa.float32(),78)),
      pa.field("validity",pa.list_(pa.uint8(),78)),
      *[pa.field(k,IMAGE,nullable=True) for k in COLUMNS[3:6]],
      pa.field("timestamp",pa.float32()),*[pa.field(k,pa.int64()) for k in COLUMNS[7:]]])
    global_index=index_offset
    for ep,(seg,prov) in enumerate(zip(segments,provenance,strict=True)):
        rows=[]
        for i,original in enumerate(seg):
            image,current_shape=image_value(original["observation.images.head_color"],i)
            if shape is not None and shape!=current_shape:raise ValueError("Mixed head resolutions")
            shape=current_shape
            row={"observation.state":np.asarray(original["observation.state"],np.float32).tolist(),
                 "action":np.asarray(original["action"],np.float32).tolist(),"validity":mask.tolist(),
                 "observation.images.head_color":image,
                 COLUMNS[4]:None,COLUMNS[5]:None,"timestamp":i/20,"frame_index":i,
                 "episode_index":ep+episode_index_offset,"index":global_index,"task_index":0}
            if "_quality" not in original or "_source" not in original:raise ValueError("Missing frame quality/source metadata")
            quality_rows.append({"episode_index":ep+episode_index_offset,"frame_index":i,"index":global_index,**original["_quality"]})
            mapping_rows.append({"episode_index":ep+episode_index_offset,"frame_index":i,"index":global_index,**original["_source"]})
            rows.append(row);global_index+=1
        if not rows:raise ValueError("Empty episode")
        path=root/f"data/chunk-{ep//1000:03d}/episode_{ep:06d}.parquet";path.parent.mkdir(parents=True,exist_ok=True)
        pq.write_table(pa.Table.from_pylist(rows,schema=fields),path,compression="snappy")
        eps.append({"episode_index":ep,"tasks":[task],"length":len(rows)})
        epstats.append({"episode_index":ep,"stats":stats_for(rows)})
        exports.append({**prov,"output_episode_index":ep,"frames":len(rows),"parquet":str(path.relative_to(root)),
                        "parquet_bytes":path.stat().st_size,"sha256":digest(path)})
        allrows.extend(rows)
    quality_fields=[pa.field("episode_index",pa.int64()),pa.field("frame_index",pa.int64()),pa.field("index",pa.int64())]
    for side in ("state","action"):
        quality_fields.extend([pa.field(side+"_valid",pa.list_(pa.bool_(),78)),pa.field(side+"_reason",pa.list_(pa.uint8(),78))])
        quality_fields.extend(pa.field(side+"_"+suffix,pa.list_(pa.int64(),78)) for suffix in ("age_ns","time_ns","header_time_ns"))
    pq.write_table(pa.Table.from_pylist(quality_rows,schema=pa.schema(quality_fields)),root/"meta/frame_quality.parquet")
    pq.write_table(pa.Table.from_pylist(mapping_rows),root/"meta/source_frame_mapping.parquet")
    features={k:{"dtype":dtype,"shape":[78],"names":names} for k,dtype in [("observation.state","float32"),("action","float32"),("validity","uint8")]}
    for k in COLUMNS[3:6]:
        features[k]={"dtype":"image","shape":shape if k==COLUMNS[3] else [3,0,0],"names":["channels","height","width"]}
        if k!=COLUMNS[3]:features[k].update(available=False,nullable=True,shape_semantics="zero height/width means absent camera; never decode")
    for k in COLUMNS[6:]:features[k]={"dtype":"float32" if k=="timestamp" else "int64","shape":[1],"names":None}
    info={"codebase_version":"v2.1","robot_type":"gento_skye","fps":20,"total_episodes":len(segments),
          "total_frames":len(allrows),"total_tasks":1,"total_videos":0,"total_chunks":(len(segments)+999)//1000,
          "chunks_size":1000,"splits":{"train":f"0:{len(segments)}"},
          "data_path":"data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet","video_path":None,
          "features":features,"encoded_action_dim":78,"physical_action_dim":int(mask.sum()),"valid_action_dim":int(mask.sum()),
          "action_mask":mask.astype(bool).tolist(),"state_mask":mask.astype(bool).tolist(),
          "validity_feature":"validity","validity_applies_to":["observation.state","action"],
          "validity_semantics":"structural availability only; dynamic validity is in meta/frame_quality.parquet",
          "eef_coordinate_frame":"base_link","eef_position_unit":"m","eef_rotation_representation":"rotation_matrix_first_two_columns",
          "eef_rotation_6d_order":["R00","R10","R20","R01","R11","R21"],"chunk_relative":False,"invalid_fill_value":0,
          "camera_role_mapping":{"head":COLUMNS[3],"left_wrist":COLUMNS[4],"right_wrist":COLUMNS[5]},
          "training_columns":COLUMNS,"absent_cameras":COLUMNS[4:6],
          "reader_contract":"LeRobot v2.1 layout; nullable absent-camera extension requires load_training_dataset adapter",
          "image_statistics_method":"up to 32 evenly spaced frames per episode at 160x120, RGB/255; task image stats weighted by sampled-frame counts"}
    dump(root/"meta/info.json",info);jsonl(root/"meta/episodes.jsonl",eps);jsonl(root/"meta/tasks.jsonl",[{"task_index":0,"task":task}])
    jsonl(root/"meta/episodes_stats.jsonl",epstats)
    taskstats=stats_for(allrows)
    # Merge per-episode image moments so aggregate and episode metadata agree.
    key=COLUMNS[3];weights=np.array([e["stats"][key]["count"][0] for e in epstats],float)
    means=np.array([e["stats"][key]["mean"] for e in epstats]);stds=np.array([e["stats"][key]["std"] for e in epstats])
    mean=np.average(means,axis=0,weights=weights);var=np.average(stds**2+means**2,axis=0,weights=weights)-mean**2
    taskstats[key]={"min":np.min([e["stats"][key]["min"] for e in epstats],axis=0).tolist(),
                   "max":np.max([e["stats"][key]["max"] for e in epstats],axis=0).tolist(),
                   "mean":mean.tolist(),"std":np.sqrt(np.maximum(var,0)).tolist(),"count":[int(weights.sum())]}
    dump(root/"meta/stats.json",taskstats)
    dump(root/"meta/schema78.json",{"version":VERSION,"names":names,"validity":mask.tolist(),"valid_dimensions":int(mask.sum()),
         "invalid_dimensions":np.flatnonzero(mask==0).tolist(),"rotation6d":info["eef_rotation_6d_order"],"coordinate_frame":"base_link",
         "body_indices":{"27":"waist_yaw locked","28":"waist_pitch locked","29":"waist_height up positive","30":"base forward positive","31":"base rotation enabled" if policy.get("base_rotation_policy")=="enabled" else "base rotation locked"},
         "policy":policy,"source_mapping":schema,"training_requirement":"apply structural validity AND dynamic per-frame validity in normalization and loss"})
    dump(root/"meta/training_schema.json",{"version":VERSION,"columns":COLUMNS,"absent_cameras":COLUMNS[4:6],
         "image_resolution_policy":"native unchanged; metadata CHW","validity":"structural mask only; use frame_quality.parquet for missing per-frame values",
         "loader":"processing.task_v21.load_training_dataset; excludes absent cameras from transforms",
         "action_label_rule":"same-row controller target, no additional shift",
         "dynamic_quality_provenance":"all decodable aligned input frames retained; missing values zero-filled with authoritative frame_quality.parquet masks"})
    dump(root/"meta/export.json",{"format":"v2.1","task":task,"fps":20,"frames":len(allrows),"policy":policy,"episodes":exports})
    if verify:return verify_dataset(root)
    return {"status":"not_run","reason":"full episode verification is available only in check-only mode",
            "format":"v2.1","frames":len(allrows),"episodes":len(segments),"columns":11}
def verify_dataset(root):
    root=Path(root);info=json.loads((root/"meta/info.json").read_text());exp=json.loads((root/"meta/export.json").read_text())
    assert info["codebase_version"]=="v2.1"
    eps=[json.loads(x) for x in (root/"meta/episodes.jsonl").read_text().splitlines()]
    assert len(eps)==info["total_episodes"]==len(exp["episodes"])
    n=0;mask=np.array(info["action_mask"],bool);numeric=[]
    qp=root/"meta/frame_quality.parquet";mp=root/"meta/source_frame_mapping.parquet"
    if not qp.is_file() or not mp.is_file():raise ValueError("Missing quality/source metadata; rebuild legacy export")
    quality=pq.read_table(qp).to_pylist();mapping=pq.read_table(mp).to_pylist()
    assert len(quality)==len(mapping)==info["total_frames"]
    for ep,e in enumerate(eps):
        p=root/info["data_path"].format(episode_chunk=ep//1000,episode_index=ep)
        assert digest(p)==exp["episodes"][ep]["sha256"]
        t=pq.read_table(p);assert t.column_names==COLUMNS and len(t)==e["length"]
        assert t.schema.field("validity").type.value_type==pa.uint8()
        for key in COLUMNS[:2]:assert t.schema.field(key).type.value_type==pa.float32()
        for i,row in enumerate(t.to_pylist()):
            assert [row["episode_index"],row["frame_index"],row["index"],row["task_index"]]==[ep,i,n,0]
            assert abs(row["timestamp"]-i/20)<1e-4
            assert row[COLUMNS[4]] is None and row[COLUMNS[5]] is None
            assert row["validity"]==mask.astype(np.uint8).tolist()
            assert [quality[n][k] for k in ("episode_index","frame_index","index")]==[ep,i,n]
            assert [mapping[n][k] for k in ("episode_index","frame_index","index")]==[ep,i,n]
            for side,key in (("state","observation.state"),("action","action")):
                valid=np.array(quality[n][side+"_valid"],bool)
                assert valid.shape==(78,) and not np.any(valid & ~mask)
                a=np.array(row[key]);assert np.isfinite(a).all() and np.all(a[~valid]==0)
                for j in (0,9,18):
                    x,y=a[j+3:j+6],a[j+6:j+9]
                    if valid[j+3:j+9].all():assert np.allclose([x@x,y@y,x@y],[1,1,0],atol=1e-4)
                    else:assert not valid[j+3:j+9].any()
                assert np.all((a[[54,77]]>=0)&(a[[54,77]]<=1))
            with Image.open(BytesIO(row[COLUMNS[3]]["bytes"])) as im:
                im.load();assert [3,im.height,im.width]==info["features"][COLUMNS[3]]["shape"]
            numeric.append({k:row[k] for k in COLUMNS[:3]});n+=1
    assert n==info["total_frames"]==exp["frames"]
    stats=json.loads((root/"meta/stats.json").read_text())
    for k in COLUMNS[:3]:
        a=np.array([r[k] for r in numeric])
        for name,fn in [("min",np.min),("max",np.max),("mean",np.mean),("std",np.std)]:
            assert np.allclose(stats[k][name],fn(a,axis=0),atol=1e-6)
    # Independent HF Arrow/image decoding with absent columns left nullable.
    from datasets import Dataset,Image as HFImage
    for ep in range(info["total_episodes"]):
        p=root/info["data_path"].format(episode_chunk=ep//1000,episode_index=ep)
        ds=Dataset(pq.read_table(p))
        ds=ds.cast_column(COLUMNS[3],HFImage(decode=True))
        for i in sorted(set([0,len(ds)-1])):
            assert ds[i][COLUMNS[3]].mode=="RGB" and ds[i][COLUMNS[4]] is None
    return {"status":"verified","format":"v2.1","frames":n,"episodes":len(eps),"columns":11,
            "reader":"PyArrow + Hugging Face Image decode; nullable-camera adapter",
            "stock_lerobot_reader":"not claimed: installed lerobot 0.4.3 is a v3 reader"}
def load_training_dataset(root,with_quality=False):
    """Return HF dataset with head decoded; absent camera columns retained as None."""
    from datasets import Dataset,Image as HFImage
    root=Path(root);info=json.loads((root/"meta/info.json").read_text())
    tables=[pq.read_table(root/info["data_path"].format(episode_chunk=i//1000,episode_index=i)) for i in range(info["total_episodes"])]
    ds=Dataset(pa.concat_tables(tables)).cast_column(COLUMNS[3],HFImage(decode=True))
    if with_quality:return ds,pq.read_table(root/"meta/frame_quality.parquet"),pq.read_table(root/"meta/source_frame_mapping.parquet")
    return ds
