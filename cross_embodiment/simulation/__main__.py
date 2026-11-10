import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,hashlib,json,platform
from pathlib import Path
import numpy as np
from .dataset import Episode
from .solver import Solver

PROJECT=Path(__file__).resolve().parents[1]

def fingerprint(config,episode):
    paths=sorted((PROJECT/'simulation').glob('*.py'))+[PROJECT/'models/skye/scene.xml',PROJECT/'models/skye/reference.urdf',PROJECT/'models/skye/provenance.json']
    return {'input_sha256':episode.sha256,'config':config,'sources':{str(p.relative_to(PROJECT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}}

def main():
    p=argparse.ArgumentParser(description='Offline cross-embodiment MuJoCo review; no robot connectivity')
    p.add_argument('episode',type=Path);p.add_argument('--config',type=Path,default=PROJECT/'configs/g2_to_skye.json')
    p.add_argument('--output',type=Path,required=True);p.add_argument('--fields',nargs='+',choices=['action','state'],default=['action','state'])
    mode=p.add_mutually_exclusive_group();mode.add_argument('--solve-only',action='store_true');mode.add_argument('--render-only',action='store_true')
    args=p.parse_args();out=args.output.resolve()
    allowed=(PROJECT.parent/'tests/cross_embodiment').resolve()
    if not out.is_relative_to(allowed) or out==allowed:p.error('Example outputs must be inside root tests/cross_embodiment/')
    config=json.loads(args.config.read_text());episode=Episode(args.episode,config)
    solver=Solver(PROJECT/'models/skye/scene.xml',config);fp=fingerprint(config,episode)
    if args.render_only:
        report=json.loads((out/'report.json').read_text())
        if report['fingerprint']!=fp:raise ValueError('Source/config/code changed: refuse stale render cache')
        detail=json.loads((out/'trajectory.json').read_text())
        results={k:(np.asarray(v['qpos']),v['metrics'],v['targets']) for k,v in detail.items()}
        if set(results)!=set(args.fields):raise ValueError('Requested fields differ from solved fields')
        if any((out/k).exists() for k in results):raise ValueError('Render output already exists; choose a fresh output')
    else:
        if out.exists():raise ValueError('Output exists; choose a new folder or --render-only after --solve-only')
        out.mkdir(parents=True)
        results={field:solver.solve(episode,field) for field in dict.fromkeys(args.fields)}
        report={'status':'solved_not_rendered','episode_index':episode.index,'frames':episode.n,'first_frame':int(episode.frames[0]),'last_frame':int(episode.frames[-1]),'duration_s':float(episode.time[-1]-episode.time[0]+episode.dt),'nominal_fps':1/episode.dt,'source':'G2','target':'Gento Skye','mode':'kinematic; no physics stepping','mask_quality':episode.quality,'fingerprint':fp,'joint_names':[solver.model.joint(i).name for i in range(solver.model.njnt)],'limitations':['Identity base_link transform is an uncalibrated baseline, not validated retargeting.','All frames preserved. Unmatched IK is approximate visualization, never deployment-ready.','Waist fixed at configured height; base stationary: source body slots have no semantics.','44 source hand dimensions are shown numerically but have no target hand model. Gripper slots invalid.','No collision, dynamics, contact, object, timing-offset or task-success validation.','State end-effector IK does not recover unique recorded joint angles.','Target tool definitions come from Skye; source tool calibration is unverified.','Camera key names follow source convention, not independently calibrated.'],'fields':{},'runtime':{'python':platform.python_version(),'mujoco':__import__('mujoco').__version__}}
        for field,(q,metrics,targets) in results.items():
            summary={'unmatched_frames':sum(any(g['status']=='unmatched' for g in r) for r in metrics),'invalid_frames':sum(any(g['status']=='invalid' for g in r) for r in metrics),'groups':{}}
            for j,g in enumerate(config['groups']):
                valid=[r[j] for r in metrics if r[j]['position_error_m'] is not None]
                summary['groups'][g['name']]={'matched_frames':sum(r[j]['status']=='matched' for r in metrics),'max_position_error_m':max((r['position_error_m'] for r in valid),default=None),'max_rotation_error_rad':max((r['rotation_error_rad'] for r in valid),default=None)}
            report['fields'][field]=summary
        (out/'trajectory.json').write_text(json.dumps({k:{'qpos':v[0].tolist(),'metrics':v[1],'targets':v[2]} for k,v in results.items()},allow_nan=False))
        (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    if not args.solve_only:
        from .render import render,website
        render(episode,solver,results,out);report['status']='complete'
        (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False));website(episode,results,report,out)
    print(json.dumps({'output':str(out),'status':report['status'],'fields':report['fields']},indent=2))

if __name__=='__main__':main()
