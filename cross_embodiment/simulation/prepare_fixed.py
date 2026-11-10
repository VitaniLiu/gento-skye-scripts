"""Offline, explicit stationary-base/zero-gripper command adapter."""
from pathlib import Path
import argparse,hashlib,json
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

def digest(p):
 with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('output',type=Path);args=p.parse_args();src=args.source.resolve();out=args.output.resolve()
 allowed=Path(__file__).resolve().parents[2]/'tests/cross_embodiment'
 if not out.is_relative_to(allowed) or out.exists():raise ValueError('Choose a new output under tests/cross_embodiment')
 original=digest(src);(out/'data/chunk-000').mkdir(parents=True);(out/'meta').mkdir();dst=out/'data/chunk-000'/src.name
 pf=pq.ParquetFile(src);writer=None;qualities=[];statequalities=[];frames=[];episodes=[]
 for b in pf.iter_batches(batch_size=8):
  t=pa.Table.from_batches([b]);m=np.array(t['validity'].to_pylist());m[:,32:54]=0;m[:,55:77]=0;m[:,[30,31,54,77]]=1
  for name in ['action','observation.state']:
   a=np.array(t[name].to_pylist());a[:,[30,31,54,77]]=0
   quality=m.astype(bool)&np.isfinite(a)
   for start in [0,9,18]:
    x=a[:,start+3:start+6];y=a[:,start+6:start+9]
    good=(abs(np.linalg.norm(x,axis=1)-1)<.05)&(abs(np.linalg.norm(y,axis=1)-1)<.05)&(abs((x*y).sum(1))<.05)
    quality[:,start:start+9]&=good[:,None]
   (qualities if name=='action' else statequalities).extend(quality.tolist())
   i=t.schema.get_field_index(name);t=t.set_column(i,t.schema.field(i),pa.array(a.tolist(),type=t.schema.field(i).type))
  i=t.schema.get_field_index('validity');t=t.set_column(i,t.schema.field(i),pa.array(m.tolist(),type=t.schema.field(i).type))
  frames.extend(t['frame_index'].to_pylist());episodes.extend(t['episode_index'].to_pylist())
  if writer is None:writer=pq.ParquetWriter(dst,t.schema,compression='zstd')
  writer.write_table(t)
 writer.close()
 pq.write_table(pa.table({'episode_index':episodes,'frame_index':frames,'action_valid':qualities,'state_valid':statequalities}),out/'meta/frame_quality.parquet')
 policy={'waist_height_enabled':False,'base_rotation_policy':'locked_zero','base_clockwise_scale':1.0,'waist_height_scale':1.0}
 (out/'meta/export.json').write_text(json.dumps({'policy':policy,'source':'Explicit user-requested stationary base and zero grippers; not measured G2 controls','quality_provenance':'Derived finite numeric and rotation6d orthonormality checks plus explicit synthetic channel definition. NOT acquisition-quality flags, IK/collision checks or robot readiness.'},indent=2))
 schema=json.loads((src.parents[2]/'meta/schema78.json').read_text());schema.update(version='g2_to_skye_fixed_controls_20261110_v1',output=str(dst),validity=m[0].tolist(),valid_dimensions=int(m[0].sum()),invalid_dimensions=np.where(m[0]==0)[0].tolist(),valid_sample_count_per_dimension=(m[0]*len(frames)).tolist(),body_reserved='27,28,29 remain invalid. 30=explicit zero forward velocity; 31=explicit zero clockwise velocity.',source_mapping={'bindings':[]})
 schema['names'][30]='base_forward_velocity';schema['names'][31]='base_clockwise_velocity'
 (out/'meta/schema78.json').write_text(json.dumps(schema,indent=2))
 for b,c in zip(pf.iter_batches(batch_size=8),pq.ParquetFile(dst).iter_batches(batch_size=8),strict=True):
  for name in b.schema.names:
   x=b.column(b.schema.get_field_index(name));y=c.column(c.schema.get_field_index(name))
   if name in ['action','observation.state']:
    xx=np.array(x.to_pylist());yy=np.array(y.to_pylist());xx[:,[30,31,54,77]]=0;assert np.array_equal(xx,yy)
   elif name!='validity':assert x.equals(y),name
 assert digest(src)==original
 report={'frames':len(frames),'source_sha256':original,'output_sha256':digest(dst),'fixed_zero_indices':[30,31,54,77],'hand_masked_indices':list(range(32,54))+list(range(55,77)),'source_unchanged':True,'other_numeric_and_image_columns_verified':True,'quality_scope':'numeric/rotation format and explicit fixed-command availability only; IK reported separately','replay_ik_passed':False,'no_motion_sent':True}
 (out/'preparation-report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':main()
