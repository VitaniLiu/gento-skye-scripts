"""Create a mask-only derivative; never invent gripper values or quality flags."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

HAND=list(range(32,54))+list(range(55,77));GRIP=[54,77]
def sha(p):
 with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
 source=a.source.resolve();out=a.output.resolve();allowed=Path(__file__).resolve().parents[2]/'tests/cross_embodiment'
 if not out.is_relative_to(allowed) or out.exists():raise ValueError('Use a new directory inside root tests/cross_embodiment')
 schema=json.loads((source.parents[2]/'provenance/source_schema78.json').read_text())
 if schema['names'][54]!='left_gripper' or schema['names'][77]!='right_gripper' or len(schema['names'])!=78:raise ValueError('Unknown 78D layout')
 out.mkdir(parents=True);dst=out/'data/chunk-000'/source.name;dst.parent.mkdir(parents=True)
 (out/'meta').mkdir();(out/'provenance').mkdir()
 before=sha(source);pf=pq.ParquetFile(source);writer=None;n=0;maskrows=[];grip={k:[] for k in ['action','observation.state']}
 try:
  for batch in pf.iter_batches(batch_size=8):
   table=pa.Table.from_batches([batch]);idx=table.schema.get_field_index('validity');mask=np.asarray(table['validity'].to_pylist()).copy()
   if mask.shape[1]!=78 or not np.isin(mask,[0,1]).all():raise ValueError('Invalid mask')
   mask[:,HAND]=0;mask[:,GRIP]=1;maskrows.extend(mask.tolist())
   table=table.set_column(idx,table.schema.field(idx),pa.array(mask.tolist(),type=table.schema.field(idx).type))
   if writer is None:writer=pq.ParquetWriter(dst,table.schema,compression='zstd')
   writer.write_table(table);n+=len(table)
   for k in grip:grip[k].extend(np.asarray(table[k].to_pylist())[:,GRIP].tolist())
 finally:
  if writer:writer.close()
 # Verify every source column, including exact embedded JPEG bytes, row by row.
 verified=0
 for b1,b2 in zip(pf.iter_batches(batch_size=8),pq.ParquetFile(dst).iter_batches(batch_size=8),strict=True):
  for name in b1.schema.names:
   x=b1.column(b1.schema.get_field_index(name));y=b2.column(b2.schema.get_field_index(name))
   if name!='validity':assert x.equals(y),name
   else:
    x=np.asarray(x.to_pylist());y=np.asarray(y.to_pylist());x[:,HAND]=0;x[:,GRIP]=1;assert np.array_equal(x,y)
  verified+=len(b1)
 assert verified==n and sha(source)==before
 current=dict(schema);current.update(version='egoav_78d_hand_masked_gripper_enabled_20261110_v1',source=str(source),output=str(dst),validity=maskrows[0],valid_dimensions=int(sum(maskrows[0])),invalid_dimensions=np.where(np.array(maskrows[0])==0)[0].tolist(),valid_sample_count_per_dimension=np.sum(maskrows,axis=0).tolist(),mask_change='Only validity changed; hand values retained but masked; gripper zeros enabled by user request, not recovered measurements.')
 (out/'meta/schema78.json').write_text(json.dumps(current,indent=2))
 (out/'provenance/original_schema78.json').write_text(json.dumps(schema,indent=2))
 report={'source':str(source),'output':str(dst),'frames':n,'source_sha256':before,'output_sha256':sha(dst),'indices_zero_based':True,'masked_hand_dimensions':HAND,'enabled_gripper_dimensions':GRIP,'valid_dimensions_per_frame':29,'only_changed_column':'validity','all_other_columns_equal_including_embedded_images':True,'source_unchanged':True,'gripper_unique_values':{k:np.unique(v,axis=0).tolist() for k,v in grip.items()},'quality':'No measurement-quality mask synthesized. Enabled grippers remain original zero placeholders.','replay_ready':False}
 (out/'change-report.json').write_text(json.dumps(report,indent=2))
 (out/'README.txt').write_text('Mask-only derivative of G2 yifu episode 80. All 403 rows, all action/state values and embedded image bytes preserved. Zero-based hand indices 32..53 and 55..76 disabled; grippers 54,77 enabled. Body 27..31 remain invalid reserved fields. Gripper values remain original zero placeholders, not measured or synthesized opening trajectories. meta/schema78.json describes this derivative; provenance/original_schema78.json describes the original only. No fake frame_quality or Skye export policy supplied. This episode package is not certified as a stock LeRobot dataset or robot-replay-ready. See change-report.json and compatibility report.\n')
 print(json.dumps(report,indent=2))
if __name__=='__main__':main()
