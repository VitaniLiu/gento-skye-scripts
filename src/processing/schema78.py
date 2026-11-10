"""gento_skye_78d_v1: physical values are never inferred from slot position."""
import math,bisect
from decimal import Decimal
import numpy as np
ACTIVE=list(range(32))+[54,77]
RESERVED=list(range(32,54))+list(range(55,77))
VALIDITY=np.array([int(i in ACTIVE) for i in range(78)],dtype=np.uint8)
STEP_NS=50_000_000

def field(message,path):
    if path is None:raise ValueError('source field has no verified mapping')
    value=message
    for key in path.replace('[','.').replace(']','').split('.'):
        value=value[int(key)] if key.isdigit() else value[key]
    return value

def mapped_time(raw,clock):
    if not clock.get('verified') or not clock.get('evidence'):raise ValueError('unverified clock mapping')
    if raw is None or not clock['valid_from_ns']<=raw<=clock['valid_until_ns']:raise ValueError('source time outside calibrated interval')
    # Origin-relative arithmetic avoids loss of epoch nanoseconds in float conversion.
    origin=clock.get('origin_ns',0)
    return origin+int((Decimal(raw-origin)*Decimal(str(clock['scale']))).to_integral_value())+clock['offset_ns']

def rotation(q):
    x,y,z,w=[float(q[k]) for k in ('x','y','z','w')]
    norm=math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or abs(norm-1)>0.01:raise ValueError('invalid/non-unit source quaternion')
    x,y,z,w=(v/norm for v in (x,y,z,w))
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def pose_values(pose,binding,message,t):
    actual=message.get('header',{}).get('frame_id')
    if actual!=binding.get('source_frame'):raise ValueError('source frame differs from mapping: '+str(actual))
    xyz=np.array([pose['position'][k] for k in ('x','y','z')],dtype=float)*binding.get('scale',1.)
    r=rotation(pose['orientation'])
    if actual!='base_link':
        transform=binding.get('transform') or {}
        if not transform.get('verified') or not transform.get('evidence'):raise ValueError('missing evidenced source-to-base transform')
        if transform.get('from_frame')!=actual or transform.get('to_frame')!='base_link':raise ValueError('transform frame direction not explicitly specified')
        if not transform['valid_from_ns']<=t<=transform['valid_until_ns']:raise ValueError('transform outside validity interval')
        m=np.asarray(transform['matrix'],dtype=float)
        if m.shape!=(4,4) or not np.allclose(m[3],[0,0,0,1]) or not np.allclose(m[:3,:3].T@m[:3,:3],np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(m[:3,:3]),1):raise ValueError('invalid rigid transform')
        xyz=m[:3,:3]@xyz+m[:3,3];r=m[:3,:3]@r
    return np.concatenate((xyz,r[:,0],r[:,1]))

def preflight(schema):
    if schema.get('schema') == 'gento_skye_action78_base_rows_v2':
        from processing.base78 import preflight_base
        return preflight_base(schema)
    if schema.get('schema') == 'action78_pre_aligned_body5_20261102_v1':
        from processing.action78_mapping import export_blockers
        return export_blockers(schema)
    errors=[]
    if schema.get('schema')!='gento_skye_78d_v1' or schema.get('fps')!=20:errors.append('schema/fps mismatch')
    if schema.get('validity')!=VALIDITY.tolist():errors.append('structural validity mask mismatch')
    expected=[]
    for side in ('left','right','head'):expected.extend(side+'_ee_'+suffix for suffix in ('x','y','z','r1x','r1y','r1z','r2x','r2y','r2z'))
    expected+=['waist_yaw','waist_pitch','waist_height','base_linear_velocity','base_angular_velocity']+[f'left_hand_{i}' for i in range(22)]+['left_gripper']+[f'right_hand_{i}' for i in range(22)]+['right_gripper']
    if schema.get('names')!=expected:errors.append('field names/order differ from gento_skye_78d_v1')
    if schema.get('base_frame',{}).get('name')!='base_link':errors.append('output frame must be base_link')
    if len(schema.get('names',[]))!=78 or len(set(schema.get('names',[])))!=78:errors.append('78 unique field names required')
    if not schema.get('base_frame',{}).get('evidence') or not schema.get('base_frame',{}).get('verified'):errors.append('physical base_link definition/capture-time model unverified')
    for side in ('state','action'):
        covered=[]
        for b in schema['bindings']:
            if b['target']!=side:continue
            covered+=b['indices']
            if not b.get('verified') or not b.get('evidence') or not b.get('topic') or b.get('path') is None:errors.append(f"{side}{b['indices']}: source semantics/calibration unverified")
            if not isinstance(b.get('max_age_ns'),int) or not 0<b['max_age_ns']<=1_000_000_000:errors.append('invalid bounded hold age')
            if b.get('method')!='hold_previous':errors.append('unsupported sampling method')
            clock=b.get('clock',{})
            if not clock.get('verified') or not clock.get('evidence'):errors.append(f"{side}{b['indices']}: clock mapping unverified")
            if b.get('kind')=='pose' and b.get('source_frame')!='base_link' and not (b.get('transform') or {}).get('verified'):errors.append(f"{side}{b['indices']}: base transform unverified")
        if sorted(covered)!=ACTIVE:errors.append(side+': active dimensions must each be mapped exactly once')
    if {x['role'] for x in schema['cameras']}!={'head','wrist_left','wrist_right'} or len(schema['cameras'])!=3:errors.append('three unique required camera identities missing')
    for camera in schema['cameras']:
        if not camera.get('verified') or not camera.get('identity_evidence') or not camera.get('clock',{}).get('verified') or not camera.get('clock',{}).get('evidence') or camera.get('packet_frame_rule')!='one_access_unit_per_message' or not camera.get('packet_frame_evidence'):errors.append(camera['role']+': camera identity/clock/access-unit evidence missing')
    for component in schema['bindings']+schema['cameras']:
        clock=component.get('clock',{})
        if clock.get('verified'):
            if clock.get('source') not in ('header','bag') or not clock.get('source_domain'):errors.append('source clock identity/type missing')
            if type(clock.get('scale')) not in (int,float) or not math.isfinite(clock['scale']) or clock['scale']<=0:errors.append('clock scale must be positive and finite')
            for key in ('offset_ns','valid_from_ns','valid_until_ns','uncertainty_ns'):
                if type(clock.get(key)) is not int:errors.append('clock integer field missing: '+key)
            if isinstance(clock.get('uncertainty_ns'),int) and clock['uncertainty_ns']>component.get('max_age_ns',0):errors.append('clock uncertainty exceeds sampling tolerance')
        if 'role' in component and clock.get('verified') and clock.get('source')!='header':errors.append('H264 PTS mapping requires source header clock')
    domains={b.get('clock',{}).get('target_domain') for b in schema['bindings']}|{c.get('clock',{}).get('target_domain') for c in schema['cameras']}
    if len(domains)!=1 or None in domains:errors.append('all clock mappings must target one evidenced reference clock')
    return errors

def previous(series,t,max_age):
    index=bisect.bisect_right(series['times'],t)-1
    if index<0 or t-series['times'][index]>max_age:raise ValueError('no sample within bounded hold interval')
    return {**series['items'][index],'mapped_ns':series['times'][index]}

def aligned_segments(schema,streams,images):
    if schema.get('schema') == 'gento_skye_action78_base_rows_v2':
        from processing.base78 import align_base
        return align_base(schema,streams,images)
    bounds=[v['times'] for v in list(streams.values())+list(images.values()) if v['times']]
    if len(bounds)!=len(schema['bindings'])+3:return [],{'skipped':'required stream missing'}
    start=max(x[0] for x in bounds);end=min(x[-1] for x in bounds)
    start=((start+STEP_NS-1)//STEP_NS)*STEP_NS
    segments=[];segment=[];skipped=0;missing_ranges=[];grid_count=max(0,(end-start)//STEP_NS+1)
    if grid_count>2_000_000:raise ValueError('episode timeline exceeds processing limit; review clock mapping')
    for t in range(start,end+1,STEP_NS):
        values={side:np.zeros(78,dtype=np.float32) for side in ('state','action')};raw={side:np.full(78,-1,dtype=np.int64) for side in ('state','action')}
        ages={side:np.full(78,-1,dtype=np.int64) for side in ('state','action')}
        frame={'source.grid_time_ns':np.array([t],dtype=np.int64)}
        try:
            for n,b in enumerate(schema['bindings']):
                sample=previous(streams[n],t,b['max_age_ns']);values[b['target']][b['indices']]=sample['values'];raw[b['target']][b['indices']]=sample['source_ns'];ages[b['target']][b['indices']]=t-sample['mapped_ns']
            for role,stream in images.items():
                cam=next(c for c in schema['cameras'] if c['role']==role);sample=previous(stream,t,cam['max_age_ns'])
                frame['observation.images.'+role]=sample['image'];frame['source.camera_'+role+'_time_ns']=np.array([sample['source_ns']],dtype=np.int64);frame['quality.camera_'+role+'_age_ns']=np.array([t-sample['mapped_ns']],dtype=np.int64)
            if not all(np.isfinite(v).all() for v in values.values()):raise ValueError('nonfinite mapped value')
            frame.update({'observation.state':values['state'],'action':values['action'],'validity':VALIDITY.copy(),'source.state_time_ns':raw['state'],'source.action_time_ns':raw['action'],'quality.state_age_ns':ages['state'],'quality.action_age_ns':ages['action']})
            segment.append(frame)
            if len(segment)>=schema.get('max_segment_frames',20000):segments.append(segment);segment=[]
        except (KeyError,ValueError,IndexError) as exc:
            skipped+=1
            if missing_ranges and missing_ranges[-1]['end_ns']+STEP_NS==t and missing_ranges[-1]['reason']==str(exc):missing_ranges[-1]['end_ns']=t;missing_ranges[-1]['count']+=1
            else:missing_ranges.append({'start_ns':t,'end_ns':t,'count':1,'reason':str(exc)})
            if segment:segments.append(segment);segment=[]
    if segment:segments.append(segment)
    return segments,{'grid_frames':grid_count,'skipped_frames':skipped,'missing_ranges':missing_ranges,'exportable_frames':sum(map(len,segments)),'sampling':'causal bounded previous sample hold; no interpolation or future sample; gaps split output episodes'}
