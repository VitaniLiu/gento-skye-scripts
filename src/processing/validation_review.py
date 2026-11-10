"""Evidence-aware review of raw observations; keeps every original finding."""
import copy,hashlib,json,math
from collections import Counter
from pathlib import Path
from processing.schema78 import preflight

HERE=Path(__file__).resolve()
def review_report(report,config,mapping_path=None):
    if not isinstance(report.get('topics'),dict) or 'files' not in report:
        report['review']={'status':'not_reviewed','reason':'raw report lacks topic/file inventory'}
        return report
    path=Path(mapping_path or Path(config['project_root'])/'configs/processing/action78_posttrain.json').resolve()
    raw=copy.deepcopy(report.get('issues',[]));report['raw_issues']=raw
    try:
        blob=path.read_bytes();s=json.loads(blob);errors=preflight(s);digest=hashlib.sha256(blob).hexdigest()
    except (ValueError,OSError,KeyError,TypeError) as exc:
        s={};errors=[str(exc)];digest=None
    topics=report['topics'];bindings=s.get('bindings',[]);cameras=s.get('cameras',[])
    enabled=[b for b in bindings if b.get('verified')]
    receive=not errors and s.get('time_alignment',{}).get('kind')=='replay_receive_time_v1'
    selected={c['topic']:c for c in cameras}
    expected={}
    required_fields={}
    for b in enabled:
        expected[b['topic']]=b.get('message_type')
        paths=b.get('path',[])
        if isinstance(paths,str):paths=[paths]
        required_fields.setdefault(b['topic'],set()).update(paths)
        if b.get('names_path'):required_fields[b['topic']].add(b['names_path'])
    for g in s.get('command_joint_bundle',{}).get('groups',[]):
        expected[g['topic']]=g.get('message_type')
        required_fields.setdefault(g['topic'],set()).add('positions')
    for c in cameras:required_fields.setdefault(c['topic'],set()).update(['data','timestamp','format','frame_id'])
    def present(topic):
        return topics.get(topic,{}).get('decoded',0)>0 and (not expected.get(topic) or topics[topic].get('type')==expected[topic])
    def sources(b):
        return {g['topic'] for g in s.get('command_joint_bundle',{}).get('groups',[])} if b.get('kind')=='fk_command' else {b['topic']}
    logical={}
    for group in ('pico_action','gamesir_action','body_state','chassis_state'):
        target='action' if group.endswith('action') else 'state'
        base=group in ('gamesir_action','chassis_state')
        bs=[b for b in bindings if b.get('target')==target and (b.get('indices')==[30,31])==base]
        needed=set().union(*(sources(b) for b in bs)) if bs else set()
        logical[group]={'required_sources':sorted(needed),'missing_or_wrong_type':[t for t in sorted(needed) if not present(t)],'unverified_dimensions':[i for b in bs if not b.get('verified') for i in b['indices']],'scope':'configured binding and decoded source presence; not per-frame coverage or execution acknowledgement'}
    report['logical_streams_reviewed']=logical
    def decoded(topic):
        vs=[v for v in report.get('video_decode',[]) if v.get('topic')==topic]
        good=bool(vs) and all(v.get('errors',0)==0 and v.get('decoded_frames',0)>0 for v in vs)
        frames=sum(v.get('decoded_frames',0) for v in vs);packets=sum(v.get('packets',0) for v in vs)
        one_to_one=packets>0 and packets==frames
        return good,one_to_one,frames,packets
    manifest=report.get('recording_manifest') or {}
    closed=manifest.get('file_status')=='closed' and manifest.get('capture_status')=='completed' and manifest.get('lifecycle')=='retained'
    stable=bool(report.get('files')) and all(f.get('stable_during_read') and f.get('footer_valid') for f in report['files'])
    failed=any(x.get('code') in ('MCAP_READ_FAILED','RECORD_DECODE_FAILED','SOURCE_CHANGED_DURING_READ','MCAP_MAGIC_OR_FOOTER_INVALID') for x in raw)
    completed=closed and stable and not failed and not manifest.get('failures')
    issues=[]
    for original in raw:
        x=copy.deepcopy(original);code=x.get('code');topic=x.get('topic');level='WARNING';newcode=code;reason=x.get('reason',code)
        if code=='METADATA_FILE_COUNT_MISMATCH':
            reason=f"元数据单文件计数为{x.get('expected')}，实际读取为{x.get('actual')}；以实际读取为准，不能据此推断丢失数量。"
        elif code=='CRC_VERIFICATION_UNAVAILABLE':
            level='INFO';newcode='CRC_NOT_VERIFIED';reason='本轮未执行显式完整CRC审计；可读性和首尾标记检查不等同于CRC验证通过。'
        elif code=='RECORDING_COMPLETENESS_UNPROVEN':
            if completed:
                level='INFO';newcode='CAPTURE_COMPLETION_REPORTED';reason='录制清单声明已保留、完成并关闭，文件稳定可读；这是结束流程证据，不代表逐帧数据、CRC或物理同步全部完整。'
            else:reason='缺少一致的完成/关闭清单证据，或文件检查/录制失败记录存在问题；未认证完整录制。'
        elif code=='CLOCK_MAPPING_UNKNOWN' and receive:
            level='INFO';newcode='RECEIVE_TIME_ALIGNMENT_ONLY';reason='当前配置明确使用同一domain_24录制器接收时间对齐；设备采样及曝光时钟映射仍未验证。'
        elif code=='SEMANTIC_MAPPING_UNVERIFIED':
            if not errors:
                level='INFO';newcode='MAPPING_CONFIGURATION_CONFIRMED';reason='当前78维映射已通过配置预检；逐帧数据质量及导出验证另行统计，不再使用早期“全部语义未知”结论。'
            else:newcode='MAPPING_PREFLIGHT_FAILED';reason='当前映射预检未通过：'+'; '.join(errors)
        elif code in ('LOGICAL_STREAM_SEMANTICS_UNVERIFIED','LOGICAL_STREAM_NOT_OBSERVED'):
            entry=logical.get(x.get('stream'),{})
            if not errors and entry and not entry['missing_or_wrong_type'] and not entry['unverified_dimensions']:
                level='INFO';newcode='LOGICAL_STREAM_MAPPING_CONFIGURED';reason='当前映射所需topic和类型均已读到；该结论不表示每个时间网格都完整。'
            elif entry:
                newcode='LOGICAL_STREAM_MAPPING_INCOMPLETE';reason='缺少/类型不符的源='+repr(entry['missing_or_wrong_type'])+'；未确认维度='+repr(entry['unverified_dimensions'])
            x['mapping_sources']=entry
        elif code=='EMPTY_FIELDS' and not errors:
            fields=x.get('fields',{})
            def needed(field):
                f=field.replace('[','.').replace(']','')
                return any(p==f or p.startswith(f+'.') or f.startswith(p+'.') for p in required_fields.get(topic,()))
            used=[f for f in fields if needed(f)]
            if used:newcode='REQUIRED_FIELDS_EMPTY';reason='当前映射必需字段为空：'+', '.join(used)
            else:
                level='INFO';newcode='UNUSED_OR_LAYOUT_FIELDS_EMPTY';reason='这些空字段不参与当前转换：'+', '.join(fields)+'；保留原始空值计数，不据此判定关键数据缺失。'
        elif code=='IMAGE_UNVERIFIED' and not errors:
            good,one,frames,packets=decoded(topic)
            if topic not in selected:
                level='INFO';newcode='IMAGE_NOT_SELECTED';reason='此图像流不在当前导出相机集合内；原始解码观察仍保留。'
            elif good and one:
                level='INFO';newcode='IMAGE_BITSTREAM_DECODE_VERIFIED';reason=f'主机已成功解码{frames}帧/{packets}包；这只验证可解码性和数量，不证明曝光同步。'
            elif good:
                newcode='IMAGE_PACKET_FRAME_COUNT_MISMATCH';reason=f'已解码，但包数{packets}与帧数{frames}不符合当前一包一帧协议。'
        elif code=='SAMPLING_GAP':
            policy=config.get('sampling_gap_tolerances',{}).get(topic)
            if policy and x.get('max_gap_ns',0)<=policy['max_gap_s']*1e9:
                level='INFO';newcode='STATUS_INTERVAL_WITHIN_TOLERANCE';reason=f"观察到最大间隔{x['max_gap_ns']/1e9:.6f}s，处于该状态topic配置容差{policy['max_gap_s']}s内；不是运动流中断。"
                x['tolerance_policy']=policy
            else:reason=f"相邻记录最大间隔{x.get('max_gap_ns',0)/1e9:.6f}s，超过该topic配置阈值。"
        elif code=='CAMERA_IDENTITY_AND_CROP_UNVERIFIED':
            problems=[]
            for camera in cameras:
                t=camera['topic'];st=topics.get(t,{})
                observed=set(st.get('frames',{}));policy=config.get('camera_frame_ids',{}).get(camera['role'],{})
                allowed=set(policy.get('expected',[]))
                if not camera.get('verified') or not camera.get('identity_evidence') or not st.get('count'):problems.append(t+'：相机绑定或数据未确认')
                if not allowed or not observed or not observed<=allowed:problems.append(t+'：frame_id不匹配或未观察到，actual='+repr(sorted(observed)))
                dims={tuple(size) for v in report.get('video_decode',[]) if v.get('topic')==t for size in v.get('dimensions',[])}
                crop=camera.get('crop')
                if crop is not None:
                    try:
                        cx,cy,w,h=crop
                        if not dims or min(cx,cy)<0 or min(w,h)<=0 or any(cx+w>dw or cy+h>dh for dw,dh in dims):problems.append(t+'：裁剪区域无效或尺寸未确认')
                    except (ValueError,TypeError):problems.append(t+'：裁剪配置格式错误')
                if len(dims)>1:problems.append(t+'：图像尺寸在录制中变化')
            if not errors and cameras and not problems:
                level='INFO';newcode='CAMERA_MAPPING_CONFIRMED';reason='当前选中相机的topic、软件frame_id及裁剪配置符合检查依据；不表示完成光学标定或曝光同步。'
            else:reason='; '.join(problems) or '当前映射未通过预检'
        elif code=='VIDEO_TIME_MAPPING_UNVERIFIED' and not errors:
            if topic not in selected:
                level='INFO';newcode='VIDEO_NOT_SELECTED';reason='该视频不参与当前导出；原始时间观察保留。'
            elif receive and decoded(topic)[:2]==(True,True):
                level='INFO';newcode='VIDEO_RECEIVE_TIME_ALIGNMENT';reason='按已选接收时间方法对齐视频，原始时间戳仍保留；物理曝光时间对应尚未验证。'
        x.update(severity=level,code=newcode,reason=reason,raw_issue_code=code,raw_issue_index=len(issues))
        x.setdefault('file',str(HERE.with_name('remote_worker.py')));x.setdefault('function','inspect_episode')
        x['review_file']=str(HERE);x['review_function']='review_report'
        issues.append(x)
    if receive and cameras and report.get('files'):
        domain=s.get('time_alignment',{}).get('bag_subdir')
        names={t for f in report['files'] if Path(f['path']).parent.name==domain for t in f.get('counts',{})}
        origins=[topics[t]['first_ns'] for t in names if topics.get(t,{}).get('first_ns') is not None]
        head=topics.get(cameras[0]['topic'],{})
        if origins and head.get('first_ns') is not None:
            origin=min(origins);step=50_000_000;first=origin+((head['first_ns']-origin+step-1)//step)*step
            late={t:math.ceil((topics[t]['first_ns']-first)/step) for b in enabled if b['target']=='action' for t in sources(b) if present(t) and topics[t].get('first_ns',first)>first}
            if late:
                affected=sorted({i for b in enabled if b['target']=='action' and sources(b)&set(late) for i in b['indices']})
                issues.append(dict(severity='WARNING',code='CONTROL_TARGET_STARTS_AFTER_IMAGES',file=str(HERE),function='review_report',location=report['source_path'],reason='部分控制目标晚于首个图像网格开始，开头对应action将保留无效掩码，不用反馈回填。',topics=sorted(late),leading_frames_by_source=late,affected_action_indices=affected,handling='preserve frames with explicit masks; continue'))
    report['issues']=issues
    report['review']={'status':'reviewed','mapping_path':str(path),'mapping_sha256':digest,'mapping_preflight_errors':errors,'severity_counts':dict(Counter(x['severity'] for x in issues)),'scope':'configured protocol and actual source observations; no CRC/physical clock certification'}
    report['recording_completion_review']={'manifest_reported_completed':closed,'stable_readable_files':stable,'read_failures_observed':failed,'capture_end_evidence_consistent':completed,'original_quality_status':manifest.get('quality_status'),'certified_complete':False}
    if completed:report['completion']='reported_completed_not_certified'
    if report.get('status') not in ('partial_or_unreadable','no_readable_mcap','read_failed'):
        report['status']='inspected_with_warnings' if any(x['severity']=='WARNING' for x in issues) else 'inspected'
    return report

def describe_issue(x):
    context=[]
    for name in ('topic','stream'):
        if x.get(name):context.append(name+'='+str(x[name]))
    if x.get('location'):context.append('文件='+str(x['location']))
    context.append('函数='+str(x.get('function','review_report')))
    return str(x.get('code','CHECK_ISSUE'))+'：'+str(x.get('reason','检查发现问题'))+' ['+'; '.join(context)+']'
