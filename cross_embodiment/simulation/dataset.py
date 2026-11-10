"""Strict standalone reader for the documented egoav 78D episode package."""
from pathlib import Path
import hashlib
import json
import numpy as np
import pyarrow.parquet as pq

CAMERAS = ['observation.images.head_color', 'observation.images.left_wrist_camera', 'observation.images.right_wrist_camera']

class Episode:
    def __init__(self, path, config):
        self.path = Path(path).resolve()
        self.root = self.path.parents[2]
        schema_path = self.root/'provenance/source_schema78.json'
        self.schema = json.loads(schema_path.read_text())
        if self.schema.get('version') != config['profile']:
            raise ValueError('Unsupported source profile: supply an independently verified adapter')
        if self.schema.get('coordinate_frame') != 'base_link' or self.schema.get('rotation6d') != 'first two columns [R00,R10,R20,R01,R11,R21]':
            raise ValueError('Unsupported coordinate/rotation convention')
        # These fields are reserved in this source, NOT Skye waist/base commands.
        if self.schema.get('invalid_dimensions') != [27,28,29,30,31,54,77]:
            raise ValueError('Source mask semantics differ from the verified G2 profile')
        table = pq.read_table(self.path, columns=['action','observation.state','validity','timestamp','frame_index','episode_index'])
        self.values = {k:np.asarray(table[c].to_pylist(),float) for k,c in [('action','action'),('state','observation.state')]}
        mask = np.asarray(table['validity'].to_pylist())
        self.time = np.asarray(table['timestamp'].to_pylist(),float)
        self.frames = np.asarray(table['frame_index'].to_pylist(),int)
        episodes=np.asarray(table['episode_index'].to_pylist(),int)
        self.n = len(self.time)
        if not self.n or any(v.shape!=(self.n,78) for v in self.values.values()) or mask.shape!=(self.n,78):
            raise ValueError('Expected nonempty N x 78 arrays')
        if not np.isin(mask,[0,1]).all(): raise ValueError('Nonbinary structural mask')
        self.mask=mask.astype(bool)
        if not np.array_equal(self.mask,np.broadcast_to(np.asarray(self.schema['validity'],bool),self.mask.shape)):
            raise ValueError('Row validity does not match declared structural profile')
        if not np.isfinite(self.time).all() or np.any(np.diff(self.time)<=0): raise ValueError('Invalid timestamps')
        if len(set(episodes.tolist()))!=1 or np.any(np.diff(self.frames)<=0): raise ValueError('Mixed episode or nonmonotonic frames')
        self.index=int(episodes[0]);self.dt=float(np.median(np.diff(self.time))) if self.n>1 else .05
        with self.path.open('rb') as stream:
            self.sha256=hashlib.file_digest(stream,'sha256').hexdigest()
        self.quality='structural-only; no dynamic measurement-quality mask provided'

    def images(self):
        """Stream embedded image rows; never load the whole 457 MB episode."""
        for batch in pq.ParquetFile(self.path).iter_batches(batch_size=4,columns=CAMERAS):
            columns=[batch.column(i).to_pylist() for i in range(len(CAMERAS))]
            for row in zip(*columns):
                yield [v.get('bytes') if isinstance(v,dict) else None for v in row]


def rotation6d(v):
    """Check the declared two-column encoding before tiny orthogonalization."""
    x,y=np.asarray(v[:3],float),np.asarray(v[3:6],float)
    if not np.isfinite([x,y]).all(): raise ValueError('nonfinite pose')
    err=max(abs(np.linalg.norm(x)-1),abs(np.linalg.norm(y)-1),abs(np.dot(x,y)))
    if err>.05: raise ValueError('malformed rotation6d')
    x=x/np.linalg.norm(x);y=y-x*np.dot(x,y);y=y/np.linalg.norm(y)
    return np.column_stack([x,y,np.cross(x,y)]),float(err)
