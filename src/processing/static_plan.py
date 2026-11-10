"""Conservative 20Hz static deletion candidates; all ranges are half-open internally."""
from pathlib import Path
import json
import numpy as np
import pyarrow.parquet as pq
from processing.task_v21 import digest

def intervals(mask):
    edges=np.diff(np.r_[False,np.asarray(mask,bool),False].astype(int))
    return list(zip(np.flatnonzero(edges==1).tolist(),np.flatnonzero(edges==-1).tolist()))

def rotation(v):
    a,b=v[:3],v[3:]
    if not np.allclose([a@a,b@b,a@b],[1,1,0],atol=1e-4):raise ValueError('invalid 6D rotation')
    a=a/np.linalg.norm(a);b=b-a*(a@b);b=b/np.linalg.norm(b)
    return np.column_stack((a,b,np.cross(a,b)))

def angle(a,b):
    return float(np.arccos(np.clip((np.trace(a.T@b)-1)/2,-1,1)))

def channels(mask):
    out=[]
    for name,k in [('left',0),('right',9),('head',18)]:
        for suffix,idx,kind,speed,seam in [('position',list(range(k,k+3)),'position',.005,.001),('rotation',list(range(k+3,k+9)),'rotation',.05,.01)]:
            active=mask[idx]
            if active.any() and not active.all():raise ValueError('partial structural pose mask')
            if active.all():out.append((name+'_'+suffix,idx,kind,speed,seam))
    for name,idx,kind,speed,seam in [('left_hand',range(32,54),'joint',.04,.02),('right_hand',range(55,77),'joint',.04,.02),('waist_rotation',range(27,29),'joint',.05,.02)]:
        idx=[i for i in idx if mask[i]]
        if idx:out.append((name,idx,kind,speed,seam))
    # Hardware adaptation: no unsupported threshold is invented. These channels must
    # remain exactly unchanged; base velocity must additionally be zero.
    for name,i,kind in [('waist_height_guard',29,'guard'),('base_forward_guard',30,'velocity_guard'),('base_rotation_guard',31,'velocity_guard'),('left_gripper_guard',54,'guard'),('right_gripper_guard',77,'guard')]:
        if mask[i]:out.append((name,[i],kind,0.,0.))
    if not out:raise ValueError('no active channels')
    return out

def measure(a,b,idx,kind):
    x,y=a[idx],b[idx]
    if kind=='rotation':return angle(rotation(x),rotation(y))
    if kind=='position':return float(np.linalg.norm(y-x))
    if kind=='velocity_guard':return float(max(np.max(np.abs(x)),np.max(np.abs(y))))
    return float(np.max(np.abs(y-x)))

def edge_trim(state,valid,mask,ch,discontinuity,protected):
    """State-only prefix/suffix crop; action absence is not evidence of motion.

    Keep three frames toward the task. Known protected ranges, invalid state,
    source boundaries, speed and accumulated drift all stop a scan.
    """
    n=len(state);result=[]
    def quiet(a,b,anchor):
        for name,idx,kind,speed,seam in ch:
            # Edge-only tolerances: small gripper sensor noise is not motion.
            if kind=='guard':
                speed,seam=(.005,.001) if name=='waist_height_guard' else (.04,.02)
            try:
                delta=measure(state[a],state[b],idx,kind)
                velocity=delta if kind=='velocity_guard' else delta*20
                if (velocity>=speed if speed else velocity!=0):return False
                if measure(state[anchor],state[b],idx,kind)>seam:return False
            except ValueError:return False
        return True
    good=(valid[:,mask]&np.isfinite(state[:,mask])).all(1)&~protected
    for direction in (1,-1):
        anchor=0 if direction==1 else n-1
        if not good[anchor]:continue
        count=1;previous=anchor
        for current in range(anchor+direction,n if direction==1 else -1,direction):
            if not good[current] or discontinuity[max(previous,current)] or not quiet(previous,current,anchor):break
            count+=1;previous=current
        if count/20>.25 and count>3:
            b,e=(0,count-3) if direction==1 else (n-count+3,n)
            result.append(dict(start=b,end=e,edge='prefix' if direction==1 else 'suffix'))
    # A wholly quiet episode still retains a three-frame sample.
    if len(result)==2 and result[0]['end']>=result[1]['start']:
        result=[dict(start=0,end=max(0,n-3),edge='prefix')]
    return [r for r in result if r['start']<r['end']]

def analyze(state,action,state_valid,action_valid,mask,times,protection=None,boundary_frames=()):
    arrays=[np.asarray(x,float) for x in (state,action)]
    n=len(state);mask=np.asarray(mask,bool)
    if mask.shape!=(78,) or any(a.shape!=(n,78) for a in arrays):raise ValueError('invalid numeric shape')
    valid=[np.asarray(x,bool) for x in (state_valid,action_valid)]
    if any(v.shape!=(n,78) for v in valid):raise ValueError('invalid dynamic mask shape')
    ch=channels(mask);safe=np.ones(n,bool);safe[0]=False
    metrics=[]
    for side,a,v in zip(('state','action'),arrays,valid):
        safe &= (v[:,mask]&np.isfinite(a[:,mask])).all(1)
        for name,idx,kind,threshold,seam in ch:
            values=[None]*n
            for i in range(1,n):
                if not (v[i,idx].all() and v[i-1,idx].all() and np.isfinite(a[i,idx]).all() and np.isfinite(a[i-1,idx]).all()):
                    safe[i]=False;continue
                try:delta=measure(a[i-1],a[i],idx,kind)
                except ValueError:safe[i]=False;continue
                speed=delta if kind=='velocity_guard' else delta*20
                values[i]=speed
                safe[i] &= speed<threshold if threshold else speed==0
            metrics.append(dict(side=side,name=name,threshold=threshold,values=values))
    times=np.asarray(times,dtype=np.int64)
    if times.shape!=(n,):raise ValueError('invalid time shape')
    discontinuity=np.r_[True,np.diff(times)!=50_000_000]
    discontinuity[list(boundary_frames)]=True
    safe[discontinuity]=False
    # Invalid preceding rows cannot form a valid difference even on other channels.
    rowvalid=np.ones(n,bool)
    for a,v in zip(arrays,valid):rowvalid &= (v[:,mask]&np.isfinite(a[:,mask])).all(1)
    safe[1:] &= rowvalid[:-1]
    protection=protection or {'complete':False,'source':'missing target hold/interpolation metadata','ranges':[]}
    protected=np.zeros(n,bool)
    for r in protection.get('ranges',[]):
        b,e=r['start'],r['end']
        if not isinstance(b,int) or not isinstance(e,int) or not 0<=b<e<=n:raise ValueError('invalid protection interval')
        protected[max(0,b-1):min(n,e+1)]=True
    unknown=protection.get('complete') is not True or not protection.get('source')
    # Show low-speed runs even when incomplete provenance prevents deletion.
    eligible=safe & ~protected
    runs=[];drop=np.zeros(n,bool)
    for b,e in intervals(eligible):
        start,end=b+3,e-3;reason='short_or_no_interior';remove=False
        if (e-b)/20>.25 and start<end:
            reason='protection_metadata_incomplete' if unknown else 'seam_rejected'
            if not unknown:
                okay=True
                for a,v in zip(arrays,valid):
                    for _,idx,kind,_,limit in ch:
                        if not (v[start-1,idx].all() and v[end,idx].all()):okay=False;continue
                        try:delta=measure(a[start-1],a[end],idx,kind)
                        except ValueError:okay=False;continue
                        if delta>limit:okay=False
                if okay:remove=True;reason='candidate';drop[start:end]=True
        runs.append(dict(static_start=b,static_end=e,start_frame=start if start<end else b,
                         end_frame=end-1 if start<end else e-1,frames=max(0,end-start),
                         duration_s=(e-b)/20,remove=remove,reason=reason))
    drop[0]=False;drop[-1]=False
    edge_ranges=edge_trim(arrays[0],valid[0],mask,ch,discontinuity,protected)
    for r in edge_ranges:
        b,e=r['start'],r['end'];drop[b:e]=True
        runs.append(dict(static_start=b,static_end=e,start_frame=b,end_frame=e-1,
                         frames=e-b,duration_s=(e-b)/20,remove=True,reason='edge_state_static',edge=r['edge']))
    return dict(edge_trim_ranges=edge_ranges,runs=runs,keep_indices=np.flatnonzero(~drop).tolist(),
                candidate_ranges=[{'start':b,'end':e} for b,e in intervals(drop)],
                protected_ranges=[{'start':b,'end':e} for b,e in intervals(protected)],
                low_speed_ranges=[{'start':b,'end':e} for b,e in intervals(safe)],
                protection=protection,protection_uncertain=unknown,metrics=metrics,
                input_frames=n,removed_frames=int(drop.sum()),output_frames=n-int(drop.sum()))

def fingerprints(source):
    source=Path(source)
    # Includes numeric/image source and all sidecars; stale previews cannot publish.
    return {str(p.relative_to(source)):digest(p) for p in sorted(source.rglob('*')) if p.is_file()}

def manifest_fingerprints(source):
    source=Path(source);export=json.loads((source/'meta/export.json').read_text())
    return {e['parquet']:e['sha256'] for e in export['episodes']}|{'meta/export.json':digest(source/'meta/export.json')}

def signatures(source):
    source=Path(source)
    return {str(p.relative_to(source)):(p.stat().st_size,p.stat().st_mtime_ns)
            for p in sorted(source.rglob('*')) if p.is_file()}

def check_plan_source(source,plan):
    if fingerprints(source)!=plan['metadata_sha256']:raise ValueError('Source changed; regenerate preview')

def make_plan(source,selected=None,trust_manifest=False):
    source=Path(source);before_signature=signatures(source);before=manifest_fingerprints(source) if trust_manifest else fingerprints(source);info=json.loads((source/'meta/info.json').read_text())
    if info['fps']!=20:raise ValueError('Static filter requires 20Hz')
    if info.get('state_mask')!=info['action_mask']:raise ValueError('Different structural state/action layouts require explicit adaptation')
    count=info['total_episodes'];selected=set(range(count)) if selected is None else set(selected)
    if not selected or not selected<=set(range(count)):raise ValueError('Invalid selection')
    if any(pq.ParquetFile(source/'meta'/name).metadata.num_rows!=info['total_frames'] for name in ('frame_quality.parquet','source_frame_mapping.parquet')):raise ValueError('Total sidecar count mismatch')
    pp=source/'meta/target_protection.json'
    protection=json.loads(pp.read_text()) if pp.exists() else {}
    episodes=[]
    for ep in range(count):
        path=source/info['data_path'].format(episode_chunk=ep//1000,episode_index=ep)
        rows=pq.read_table(path,columns=['observation.state','action','frame_index','episode_index']).to_pylist()
        print(f'[扫描 {ep+1}/{count}] episode_{ep:06d}',flush=True)
        qs=pq.read_table(source/'meta/frame_quality.parquet',filters=[('episode_index','=',ep)]).to_pylist()
        ms=pq.read_table(source/'meta/source_frame_mapping.parquet',filters=[('episode_index','=',ep)]).to_pylist()
        if not len(rows)==len(qs)==len(ms):raise ValueError('Sidecar length mismatch')
        for seq in (rows,qs,ms):
            if [r['frame_index'] for r in seq]!=list(range(len(rows))):raise ValueError('Noncontiguous frame indices')
        p=protection.get('episodes',{}).get(str(ep))
        e=analyze([r['observation.state'] for r in rows],[r['action'] for r in rows],
                  [r['state_valid'] for r in qs],[r['action_valid'] for r in qs],info['action_mask'],
                  [r['source_grid_time_ns'] for r in ms],p,
                  [i for i in range(1,len(ms)) if ms[i]['source_episode_path']!=ms[i-1]['source_episode_path']])
        if ep not in selected:
            e.update(edge_trim_ranges=[],keep_indices=list(range(len(rows))),candidate_ranges=[],removed_frames=0,output_frames=len(rows))
            for r in e['runs']:r.update(remove=False,reason='not_selected')
        e.update(episode_index=ep,selected=ep in selected,output_to_source=e['keep_indices'].copy())
        episodes.append(e)
    if signatures(source)!=before_signature:raise ValueError('Source changed while planning')
    return dict(task=source.name,source=str(source),fps=20,method='static_motion_v2_edge_state',review_status='unapproved_candidate',
                selected_episodes=sorted(selected),episodes=episodes,metadata_sha256=before,
                input_frames=sum(e['input_frames'] for e in episodes),removed_frames=sum(e['removed_frames'] for e in episodes),
                output_frames=sum(e['output_frames'] for e in episodes))

def print_terminal_preview(plan,stream=None):
    print('静止删帧候选（尚未批准；原数据保留）',file=stream)
    for e in plan['episodes']:
        if e['selected']:
            print(f"episode_{e['episode_index']:06d}: {e['input_frames']} → {e['output_frames']}, 候选 {e['removed_frames']} 帧",file=stream)
            if e['protection_uncertain']:print('  目标保持/补齐保护信息不完整：中间静止段不删除；首尾仍按 state 独立裁剪。',file=stream)

def write_visualization(path,plan,phase='preview'):
    from processing.static_viewer import write_page
    write_page(Path(path),plan)
