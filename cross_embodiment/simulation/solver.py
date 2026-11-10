"""MuJoCo kinematic IK only: no ROS, SSH, robot dispatch or physics stepping."""
import json
from pathlib import Path
import numpy as np
import mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from .dataset import rotation6d

class Solver:
    def __init__(self, model_path, config):
        self.model=mujoco.MjModel.from_xml_path(str(model_path));self.data=mujoco.MjData(self.model)
        self.config=config
        key=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_KEY,'home')
        mujoco.mj_resetDataKeyframe(self.model,self.data,key)
        self.data.qpos[self.model.joint('waist_joint_1').qposadr[0]]=config['waist_height_m']
        self.home=self.data.qpos.copy()
        urdf=Path(model_path).with_name('reference.urdf')
        import xml.etree.ElementTree as ET
        self.speeds={j.get('name'):float(j.find('limit').get('velocity')) for j in ET.parse(urdf).getroot().findall('joint') if j.find('limit') is not None}
        self.transform=Rotation.from_quat(config['source_to_target']['rotation_xyzw']).as_matrix()
        self.translation=np.asarray(config['source_to_target']['translation_m'],float)
        mujoco.mj_forward(self.model,self.data)

    def solve(self, episode, field):
        self.data.qpos[:]=self.home
        poses=[];reports=[];targets=[]
        for i,row in enumerate(episode.values[field]):
            groups=[]; frame_targets=[]
            for g in self.config['groups']:
                start=g['start'];v=row[start:start+9]
                info={'group':g['name'],'status':'invalid','position_error_m':None,'rotation_error_rad':None}
                frame_targets.append(None)
                try:
                    if not episode.mask[i,start:start+9].all() or not np.isfinite(v).all():raise ValueError('invalid/nonfinite target; previous joints held')
                    rot,ortho=rotation6d(v[3:]);rot=self.transform@rot;pos=self.transform@v[:3]+self.translation
                    frame_targets[-1]={'position':pos.tolist(),'rotation':rot.tolist()}
                    joints=[self.model.joint(n) for n in g['joints']]
                    ids=np.array([int(j.qposadr[0]) for j in joints]); bounds=np.array([j.range for j in joints]).T
                    seed=self.data.qpos[ids].copy();low,high=bounds
                    if i:
                        step=np.array([self.speeds[n] for n in g['joints']])*(episode.time[i]-episode.time[i-1])
                        low=np.maximum(low,seed-step);high=np.minimum(high,seed+step)
                    bid=int(self.model.body(g['body']).id)
                    def residual(q):
                        self.data.qpos[ids]=q;mujoco.mj_forward(self.model,self.data)
                        dr=Rotation.from_matrix(rot@self.data.xmat[bid].reshape(3,3).T).as_rotvec()
                        return np.r_[self.data.xpos[bid]-pos,self.config['rotation_weight_m']*dr,self.config['continuity_weight']*(q-seed)]
                    fit=least_squares(residual,np.clip(seed,low+1e-10,high-1e-10),bounds=(low,high),max_nfev=self.config['max_nfev'],ftol=1e-7,xtol=1e-7,gtol=1e-7)
                    e=residual(fit.x);p=float(np.linalg.norm(e[:3]));r=float(np.linalg.norm(e[3:6])/self.config['rotation_weight_m'])
                    ok=p<=self.config['position_tolerance_m'] and r<=self.config['rotation_tolerance_rad']
                    info.update(status='matched' if ok else 'unmatched',position_error_m=p,rotation_error_rad=r,solver_converged=bool(fit.success),rotation_input_error=ortho,nfev=fit.nfev)
                except ValueError as exc:
                    info['reason']=str(exc)
                groups.append(info)
            poses.append(self.data.qpos.copy());reports.append(groups);targets.append(frame_targets)
            if i%50==0:print(f'{field}: IK {i+1}/{episode.n}',flush=True)
        return np.asarray(poses),reports,targets
