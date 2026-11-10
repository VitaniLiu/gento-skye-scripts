"""Write and independently read LeRobot v3 artifacts; no robot SDK dependency."""
import json,hashlib,os
from pathlib import Path
import numpy as np
from PIL import Image
from processing.schema78 import VALIDITY,RESERVED

def image_array(value):
    if isinstance(value,(str,Path)):
        with Image.open(value) as image:return np.asarray(image.convert('RGB'))
    return np.asarray(value,dtype=np.uint8)

def write_dataset(root,segments,schema,task,provenance):
    if schema.get('schema') != 'gento_skye_78d_v1':
        raise ValueError(f'{Path(__file__).resolve()}:write_dataset: legacy frame exporter cannot encode window optical Action78; use a verified window exporter')
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    import importlib.metadata
    if importlib.metadata.version('lerobot')!='0.4.3':raise ValueError('LeRobot version must be 0.4.3')
    root=Path(root)
    if root.exists():raise FileExistsError(str(root))
    if not segments or not all(segments):raise ValueError('no exportable continuous segments')
    if len(provenance)!=len(segments) or any(item.get('output_episode_index')!=i for i,item in enumerate(provenance)):raise ValueError('each output episode requires explicit source provenance')
    features={
      'observation.state':{'dtype':'float32','shape':(78,),'names':schema['names']},
      'action':{'dtype':'float32','shape':(78,),'names':schema['names']},
      'validity':{'dtype':'uint8','shape':(78,),'names':schema['names']},
      'source.state_time_ns':{'dtype':'int64','shape':(78,),'names':schema['names']},
      'source.action_time_ns':{'dtype':'int64','shape':(78,),'names':schema['names']},
      'quality.state_age_ns':{'dtype':'int64','shape':(78,),'names':schema['names']},
      'quality.action_age_ns':{'dtype':'int64','shape':(78,),'names':schema['names']},
      'source.grid_time_ns':{'dtype':'int64','shape':(1,),'names':['reference_clock_ns']},
    }
    for role in ('head','wrist_left','wrist_right'):
        key='observation.images.'+role;arr=image_array(segments[0][0][key])
        features[key]={'dtype':'image','shape':arr.shape,'names':['height','width','channels']}
        features['quality.camera_'+role+'_age_ns']={'dtype':'int64','shape':(1,),'names':['held_age_ns']}
        features['source.camera_'+role+'_time_ns']={'dtype':'int64','shape':(1,),'names':['source_clock_ns']}
    dataset=LeRobotDataset.create(repo_id='local/gento_skye',fps=20,features=features,root=root,robot_type='gento_skye',use_videos=False,video_backend='pyav')
    try:
        for segment in segments:
            for original in segment:
                frame=dict(original);frame['task']=task
                for role in ('head','wrist_left','wrist_right'):
                    key='observation.images.'+role;frame[key]=image_array(frame[key])
                dataset.add_frame(frame)
            import lerobot.datasets.lerobot_dataset as implementation
            from processing.dataset_stats import stable_episode_stats
            original_stats=implementation.compute_episode_stats
            try:
                implementation.compute_episode_stats=stable_episode_stats
                dataset.save_episode()
            finally:implementation.compute_episode_stats=original_stats
        dataset.finalize()
        info_path=root/'meta/info.json';info=json.loads(info_path.read_text())
        info['gento_skye']={'schema':'gento_skye_78d_v1','frame':'base_link','units':schema['units'],'structural_validity':VALIDITY.tolist(),'sampling':'20Hz bounded causal hold; long/missing gaps split episodes','source_mapping':schema,'provenance':provenance,'missing_policy':'no missing active values or cameras exported; reserved slots are zero'}
        info_path.write_text(json.dumps(info,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    except Exception:
        dataset.finalize();raise
    return verify_dataset(root,segments,task)

def verify_dataset(root,expected=None,task=None):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    import pyarrow.parquet as pq
    dataset=LeRobotDataset('local/gento_skye',root=Path(root),video_backend='pyav')
    frames=[];episode_counts={}
    for i in range(len(dataset)):
        row=dataset[i];ep=int(row['episode_index']);index=int(row['frame_index'])
        if int(row['index'])!=i or index!=episode_counts.get(ep,0):raise ValueError('frame/global index mismatch')
        episode_counts[ep]=index+1
        if not np.isclose(float(row['timestamp']),index/20,atol=1e-4,rtol=0):raise ValueError('20Hz timestamp mismatch')
        if task is not None and row['task']!=task:raise ValueError('task index/semantic mismatch')
        if not np.array_equal(row['validity'].numpy(),VALIDITY):raise ValueError('structural validity mismatch')
        for key in ('observation.state','action'):
            arr=row[key].numpy()
            if arr.shape!=(78,) or arr.dtype!=np.float32 or not np.isfinite(arr).all() or np.any(arr[RESERVED]!=0):raise ValueError('invalid 78D values')
            for start in (0,9,18):
                a,b=arr[start+3:start+6],arr[start+6:start+9]
                if not np.isclose(np.linalg.norm(a),1,atol=1e-5) or not np.isclose(np.linalg.norm(b),1,atol=1e-5) or not np.isclose(a@b,0,atol=1e-5):raise ValueError('invalid rotation6D columns')
        for role in ('head','wrist_left','wrist_right'):
            key='observation.images.'+role
            if row[key].shape[0]!=3:raise ValueError('invalid decoded image')
        if expected is not None:
            ref=expected[ep][index]
            for key in ('observation.state','action','source.state_time_ns','source.action_time_ns','source.grid_time_ns','quality.state_age_ns','quality.action_age_ns'):
                if not np.array_equal(np.atleast_1d(row[key].numpy()),np.atleast_1d(ref[key])):raise ValueError('round-trip value/source-time mismatch '+key)
            for role in ('head','wrist_left','wrist_right'):
                key='observation.images.'+role
                if not np.allclose(row[key].numpy().transpose(1,2,0)*255,image_array(ref[key]),atol=0.01):raise ValueError('image round-trip mismatch')
        frames.append({key:row[key] for key in ('observation.state','action')})
    # Check actual parquet physical element types rather than only metadata.
    tables=[pq.read_table(f) for f in sorted((Path(root)/'data').rglob('*.parquet'))]
    import pyarrow as pa
    for table in tables:
        for key in ('observation.state','action'):
            dtype=table.schema.field(key).type
            if not (pa.types.is_list(dtype) or pa.types.is_fixed_size_list(dtype)) or dtype.value_type!=pa.float32():raise ValueError('Parquet vector dtype mismatch')
    for key in ('observation.state','action'):
        values=np.stack([row[key].numpy() for row in frames]);stats=dataset.meta.stats[key]
        for name,actual in [('min',values.min(0)),('max',values.max(0)),('mean',values.mean(0)),('std',values.std(0))]:
            if not np.allclose(np.asarray(stats[name]).reshape(-1),actual,atol=1e-5,rtol=1e-4):raise ValueError('statistics mismatch '+key+'/'+name)
        if int(np.asarray(stats['count']).reshape(-1)[0])!=len(frames):raise ValueError('statistics count mismatch')
    return {'standard_reader':'lerobot==0.4.3 LeRobotDataset','frames':len(frames),'episodes':len(episode_counts),'fps':dataset.fps,'status':'verified','media':'RGB images embedded in standard Parquet image features; no auxiliary chassis files'}
