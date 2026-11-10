"""Image-free streaming task consistency checks after per-episode image verification."""
from pathlib import Path
import json
import numpy as np
import pyarrow.parquet as pq
from processing.task_v21 import COLUMNS

def verify_metadata(root):
    root=Path(root);info=json.loads((root/'meta/info.json').read_text());exp=json.loads((root/'meta/export.json').read_text())
    episodes=[json.loads(x) for x in (root/'meta/episodes.jsonl').read_text().splitlines()]
    if len(episodes)!=info['total_episodes'] or len(exp['episodes'])!=len(episodes):raise ValueError('Episode metadata mismatch')
    offset=0
    for ep,e in enumerate(episodes):
        p=root/info['data_path'].format(episode_chunk=ep//1000,episode_index=ep);pf=pq.ParquetFile(p)
        if pf.schema_arrow.names!=COLUMNS or pf.metadata.num_rows!=e['length']:raise ValueError('Data schema/count mismatch')
        cursor=0
        for batch in pf.iter_batches(batch_size=256,columns=COLUMNS[6:]):
            t=batch.to_pydict();n=len(batch);frames=np.arange(cursor,cursor+n)
            if not np.array_equal(t['frame_index'],frames) or not np.array_equal(t['index'],frames+offset):raise ValueError('Invalid frame/global indices')
            if not np.allclose(t['timestamp'],frames/20,atol=1e-4,rtol=0) or any(i!=ep for i in t['episode_index']):raise ValueError('Invalid episode timestamps')
            cursor+=n
        offset+=cursor
    if offset!=info['total_frames'] or offset!=exp['frames']:raise ValueError('Total frame count mismatch')
    for name in ['frame_quality.parquet','source_frame_mapping.parquet']:
        pf=pq.ParquetFile(root/'meta'/name)
        if pf.metadata.num_rows!=offset:raise ValueError('Sidecar count mismatch')
        cursor=0;ep=0;local=0
        for batch in pf.iter_batches(batch_size=256,columns=['index','episode_index','frame_index']):
            for row in batch.to_pylist():
                if local==episodes[ep]['length']:ep+=1;local=0
                if [row['index'],row['episode_index'],row['frame_index']]!=[cursor,ep,local]:raise ValueError('Sidecar pairing mismatch')
                cursor+=1;local+=1
    return {'status':'verified','frames':offset,'episodes':len(episodes)}
