import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/inference/host'))
sys.path.insert(0,str(ROOT/'src/inference/robot'))
from main import load_configuration,response_rows,task_spec
from trajectory import plan_rows,curve,extrema,joint_vector
from kinematics import Kinematics,JOINTS
from protocol import decode_array
from runtime import Runtime

class TrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.c=load_configuration(ROOT/'configs/inference/default.json','ours_a1')
        self.k=Kinematics(ROOT/'src/inference/models/runtime_expanded.urdf',ROOT/'src/inference/models/robot_Gento_Sky.xml')
        tree=ET.parse(ROOT/'src/inference/models/robot_Gento_Sky.xml').getroot()
        names=[j.get('name') for j in tree.find('worldbody').iter('joint') if j.get('name')]
        seed=dict(zip(names,map(float,tree.find('./keyframe/key[@name="home"]').get('qpos').split())))
        self.q=np.array([seed[n] for n in JOINTS]);self.q[16]=.496314
        self.obs=dict(joint_names=list(JOINTS),joint_positions=self.q.tolist(),left_gripper=0.,right_gripper=0.)
    def row(self,q):
        return dict(groups=[q[:7].tolist(),q[7:14].tolist(),q[14:16].tolist()],body=q[16:].tolist(),vx=.1,wz=.05,left_gripper=.5,right_gripper=None)
    def test_large_jump_is_retimed_and_limits_analytically_hold(self):
        a=self.q.copy();a[4]+=.6;a[16]-=.06
        b=a.copy();b[4]-=.5;b[16]+=.04
        plan=plan_rows([self.row(a),self.row(b)],self.obs,self.k,self.c['inference'],self.c['grippers'])
        np.testing.assert_allclose(plan.q[0],self.q)
        np.testing.assert_allclose(plan.q[-1],b)
        self.assertGreater(plan.report['total_duration_s'],3.)
        self.assertLessEqual(plan.report['max_policy_velocity_ratio'],1.00001)
        self.assertLessEqual(plan.report['max_policy_acceleration_ratio'],1.00001)
        spline=curve(plan.knots,plan.knot_q)
        np.testing.assert_allclose(spline.derivative()([0,plan.times[-1]]),0,atol=1e-9)
        # numerical integration corroborates preserved base displacement
        vx=np.array([r['vx'] for r in plan.rows])
        self.assertAlmostEqual(np.sum(vx[:-1]*np.diff(plan.times)),.1*.05*2,places=4)
        self.assertTrue(all(row['right_gripper'] is None for row in plan.rows))
    def test_joint_overshoot_rejected(self):
        q=self.q.copy();q[0]=self.k.limits[JOINTS[0]][1]+.1
        with self.assertRaisesRegex(ValueError,'position limits'):plan_rows([self.row(q)],self.obs,self.k,self.c['inference'],self.c['grippers'])
    def test_continuity_weight_preserves_arm_pose_accuracy(self):
        q=self.q.copy();q[0]+=.05
        target=self.k.pose(self.k.forward(q)['flange_L'])
        a=np.zeros((1,78));a[0,:9]=target;m=np.zeros_like(a,dtype=bool);m[:,:9]=True
        self.k.action_rows(a,m,dict(zip(JOINTS,self.q)),continuity_weight=.002)
        d=self.k.last_ik_diagnostics[0]
        self.assertLess(d['position_error_m'],.005);self.assertLess(d['rotation_error_rad'],.03)
    def test_head_orientation_ignores_unreachable_xyz(self):
        q=self.q.copy();q[14:16]=[.2,.1]
        a=np.zeros((1,78));a[0,18:27]=self.k.pose(self.k.forward(q)['eyes_link']);a[0,18:21]+=.2
        m=np.zeros_like(a,dtype=bool);m[:,18:27]=True
        row=self.k.action_rows(a,m,dict(zip(JOINTS,self.q)))[0]
        np.testing.assert_allclose(row['groups'][2],q[14:16],atol=1e-4)
    def test_original_arm_failure_threshold_remains(self):
        a=np.zeros((1,78));a[0,:9]=self.k.pose(self.k.forward(self.q)['flange_L']);a[0,0]+=10
        m=np.zeros_like(a,dtype=bool);m[:,:9]=True
        with self.assertRaisesRegex(ValueError,'flange_L'):self.k.action_rows(a,m,dict(zip(JOINTS,self.q)))
    def test_real_failure_chunk_plans_without_robot(self):
        p=ROOT/'logs/inference/ik_failure_802f301a76e944618b18b25b49f35fed.json'
        if not p.exists():self.skipTest('local diagnostic absent')
        d=json.loads(p.read_text(),object_hook=decode_array);s=self.c['inference']
        rows=response_rows(d['response'],d['request']['request_id'],task_spec(s,'toulaji')[1],32,s,d['observation'],self.k)
        plan=plan_rows(rows,d['observation'],self.k,s,self.c['grippers'])
        self.assertEqual(plan.report['source_frames'],32)
        self.assertLessEqual(plan.report['max_policy_velocity_ratio'],1.00001)
        self.assertLessEqual(plan.report['max_policy_acceleration_ratio'],1.00001)
        limits=np.array([self.k.limits[n] for n in JOINTS]);self.assertTrue(np.all(plan.q>=limits[:,0]-1e-6));self.assertTrue(np.all(plan.q<=limits[:,1]+1e-6))
    def test_runtime_does_not_burst_stale_targets(self):
        runtime=object.__new__(Runtime);runtime.config=self.c;sent=[];runtime.publish_base=lambda *a:None
        runtime.publish_upper=lambda *a:sent.append(a)
        row=self.row(self.q);row['time_s']=0.
        with patch('runtime.time.monotonic',side_effect=[0.,.2,.2]),patch('runtime.select.select',return_value=([],[],[])):
            with self.assertRaisesRegex(RuntimeError,'late'):runtime.execute([row],.01)
        self.assertEqual(sent,[])
    def test_runtime_cancel_precedes_any_target(self):
        from io import StringIO
        runtime=object.__new__(Runtime);runtime.config=self.c;runtime.publish_base=lambda *a:None
        row=self.row(self.q);row['time_s']=0.
        with patch('runtime.sys.stdin',StringIO('{"command":"cancel"}\n')),patch('runtime.select.select',return_value=([1],[],[])):
            result=runtime.execute([row],.01)
        self.assertTrue(result['cancelled']);self.assertEqual(result['sent'],0)
