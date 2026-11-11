"""Offline behavioral tests; never import ROS or open a network connection."""
import math
import unittest
from dataclasses import replace
from logic import Sample, ReturnController, Lease, validate_reference, validate_verification

def sample(t=0,x=.2,y=0,yaw=math.pi,vx=0,wz=0,seq=1):
    return Sample(x,y,yaw,vx,0,wz,t,seq)

class LogicTests(unittest.TestCase):
    def test_turns_toward_goal_before_driving(self):
        c=ReturnController((0,0,0),0)
        out=c.step(sample(yaw=0),0)
        self.assertEqual(out.vx,0)
        self.assertGreater(out.wz,0)
        self.assertLessEqual(abs(out.wz),.1)

    def test_heading_wrap_uses_short_turn(self):
        c=ReturnController((0,0,math.radians(-179)),0)
        out=c.step(sample(x=0,yaw=math.radians(178)),0)
        self.assertEqual(out.vx,0)
        self.assertGreater(out.wz,0)
        self.assertLess(out.wz,.1)

    def test_forward_output_is_bounded_and_slow_near_target(self):
        c=ReturnController((0,0,0),0)
        for i in range(3):out=c.step(sample(i*.02,x=.08,seq=i+1),i*.02)
        self.assertGreater(out.vx,0)
        self.assertLessEqual(out.vx,.02)
        self.assertEqual(out.vy,0)

    def test_lateral_offset_requires_turn(self):
        c=ReturnController((0,0,0),0)
        out=c.step(sample(x=0,y=.2,yaw=0),0)
        self.assertEqual(out.vx,0)
        self.assertLess(out.wz,0)

    def test_stale_feedback_aborts_and_never_resumes(self):
        c=ReturnController((0,0,0),0)
        c.step(sample(),0)
        out=c.step(sample(),.21)
        self.assertEqual((out.vx,out.wz),(0,0))
        self.assertEqual(c.finished_reason,'feedback_stale')
        c.step(sample(.22,seq=2),.22)
        self.assertEqual(c.finished_reason,'feedback_stale')

    def test_nonfinite_pose_is_rejected(self):
        c=ReturnController((0,0,0),0)
        out=c.step(sample(x=float('nan')),0)
        self.assertEqual(out.vx,0)
        self.assertEqual(c.finished_reason,'invalid_feedback')

    def test_outside_one_meter_is_rejected(self):
        c=ReturnController((0,0,0),0)
        self.assertEqual(c.step(sample(x=1.01),0).vx,0)
        self.assertEqual(c.finished_reason,'target_out_of_range')

    def test_stationary_pose_alone_does_not_report_success(self):
        c=ReturnController((0,0,0),0)
        for i in range(31):
            t=i*.02;c.step(sample(t,x=0,yaw=0,vx=.03,seq=i+1),t)
        self.assertNotEqual(c.finished_reason,'reset_complete')

    def test_success_requires_half_second_fresh_stable_feedback(self):
        c=ReturnController((0,0,0),0)
        for i in range(20):
            t=i*.02;c.step(sample(t,x=0,yaw=0,seq=i+1),t)
        self.assertIsNone(c.finished_reason)
        for i in range(20,33):
            t=i*.02;c.step(sample(t,x=0,yaw=0,seq=i+1),t)
        self.assertEqual(c.finished_reason,'reset_complete')

    def test_reusing_cached_sample_cannot_complete(self):
        c=ReturnController((0,0,0),0)
        for i in range(10):c.step(sample(x=0,yaw=0),i*.01)
        self.assertIsNone(c.finished_reason)

    def test_position_must_still_match_after_final_rotation(self):
        c=ReturnController((0,0,0),0)
        c.step(sample(x=0,yaw=.1),0)
        c.step(sample(.02,x=.035,yaw=0,seq=2),.02)
        self.assertNotEqual(c.finished_reason,'reset_complete')

    def test_pose_jump_aborts(self):
        c=ReturnController((0,0,0),0)
        c.step(sample(),0)
        c.step(sample(.02,x=.4,seq=2),.02)
        self.assertEqual(c.finished_reason,'pose_jump')

    def test_scheduler_gap_aborts(self):
        c=ReturnController((0,0,0),0)
        c.step(sample(),0)
        c.step(sample(.11,seq=2),.11)
        self.assertEqual(c.finished_reason,'scheduler_late')

    def test_no_progress_is_bounded(self):
        c=ReturnController((0,0,0),0)
        for i in range(450):
            t=i*.02;c.step(sample(t,seq=i+1),t)
            if c.finished_reason:break
        self.assertEqual(c.finished_reason,'no_progress')

    def test_overshoot_correction_is_slow_and_bounded(self):
        c=ReturnController((0,0,0),0)
        c.step(sample(x=.05),0)
        c.step(sample(.02,x=.05,seq=2),.02)
        out=c.step(sample(.04,x=-.025,seq=3),.04)
        self.assertEqual(out.vx,0)  # Brake before changing travel direction.
        for i in range(1,21):
            t=.04+i*.02;out=c.step(sample(t,x=-.025,seq=3+i),t)
        self.assertLess(out.vx,0)
        self.assertGreaterEqual(out.vx,-.02)

    def test_simulated_vehicle_returns_from_offset(self):
        c=ReturnController((0,0,0),0)
        x,y,yaw=.2,.12,.4;vx=wz=0.
        for i in range(6000):
            t=i*.02
            out=c.step(Sample(x,y,yaw,vx,0,wz,t,i+1),t)
            vx+=(out.vx-vx)*.2;wz+=(out.wz-wz)*.2
            x+=math.cos(yaw)*vx*.02;y+=math.sin(yaw)*vx*.02;yaw+=wz*.02
            if c.finished_reason:break
        self.assertEqual(c.finished_reason,'reset_complete')
        self.assertLessEqual(math.hypot(x,y),.02)
        self.assertLessEqual(abs(math.atan2(math.sin(yaw),math.cos(yaw))),math.radians(2))

class LeaseTests(unittest.TestCase):
    def test_nonce_must_be_fresh_once_and_ordered(self):
        l=Lease('a'*32)
        p=l.poll(0)
        msg=dict(type='input',session='a'*32,nonce=p['nonce'],seq=1)
        self.assertTrue(l.accept(msg,.02))
        self.assertFalse(l.accept(msg,.03))
        self.assertTrue(l.fresh(.14))
        self.assertFalse(l.fresh(.16))

    def test_old_nonce_does_not_reenable(self):
        l=Lease('b'*32);p=l.poll(0)
        self.assertFalse(l.accept(dict(type='input',session='b'*32,nonce=p['nonce'],seq=1),.2))



class EntryTests(unittest.TestCase):
    def test_reference_file_is_pinned(self):
        from pathlib import Path
        from logic import REFERENCE_FILE
        raw=(Path(__file__).resolve().parents[1]/'reference'/REFERENCE_FILE).read_bytes()
        self.assertAlmostEqual(validate_reference(raw)['pose']['yaw_rad'],-0.003490960468546934)
        with self.assertRaises(ValueError):validate_reference(raw+b' ')

    def test_target_is_stationary_start_not_position_after_movement(self):
        ref=self.entry().reference()
        target=self.entry().target_of(ref)
        self.assertAlmostEqual(target[0],-0.0013388685765514388)
        self.assertAlmostEqual(target[1],0.00034070432420560143)
        self.assertAlmostEqual(target[2],-0.003490960468546934)
        finish=ref['capture_evidence']['stationary_finish_pose']
        self.assertGreater(math.hypot(target[0]-finish['x_m'],target[1]-finish['y_m']),.8)
        self.assertLess(math.hypot(target[0]-finish['x_m'],target[1]-finish['y_m']),.82)

    def test_previous_session_reference_is_preserved_and_rejected(self):
        from pathlib import Path
        raw=(Path(__file__).resolve().parents[1]/'reference/reset-reference-20261101-163522.json').read_bytes()
        with self.assertRaisesRegex(ValueError,'reference_hash_mismatch'):
            validate_reference(raw)

    def evidence(self):
        from datetime import datetime,timezone
        from logic import REFERENCE_SHA,POSE_GID
        return dict(reference_sha256=REFERENCE_SHA,controller_prefix=POSE_GID[:24],
                    direction_and_scale_verified=True,independent_stop_verified=True,
                    no_odometry_reset_confirmed=True,stop_timeout_s=.15,
                    evidence_note='unit fixture only, no real robot verification',
                    verified_at=datetime.fromtimestamp(1000,timezone.utc).isoformat())

    def test_first_retrace_checks_physical_evidence_without_prior_return(self):
        from unittest.mock import patch
        import json
        import reset
        record = self.evidence()
        with patch.object(reset, 'VERIFICATION') as source, patch.object(reset.time, 'time', return_value=1001):
            source.read_text.return_value = json.dumps(record)
            self.assertEqual(reset.verification('retrace'), record)
            source.read_text.return_value = json.dumps(dict(record, independent_stop_verified=False))
            with self.assertRaisesRegex(ValueError, 'independent_stop_verified'):
                reset.verification('retrace')

    def test_unverified_or_expired_records_cannot_unlock(self):
        good=self.evidence()
        self.assertEqual(validate_verification(good,1001),good)
        for field,value in [('direction_and_scale_verified',False),('independent_stop_verified',False),
                            ('no_odometry_reset_confirmed',1),('stop_timeout_s',.3),
                            ('controller_prefix','other'),('verified_at','bad')]:
            with self.subTest(field=field),self.assertRaises(ValueError):
                validate_verification(dict(good,**{field:value}),1001)
        with self.assertRaises(ValueError):validate_verification(good,90000)

    def entry(self):
        import importlib.util
        from pathlib import Path
        spec=importlib.util.spec_from_file_location('reset_entry',(Path(__file__).resolve().parents[1]/'src'/'reset'/'reset.py'))
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module

    def test_default_preview_and_locked_execute_never_need_ssh(self):
        import subprocess,sys,os,json
        from pathlib import Path
        script=str((Path(__file__).resolve().parents[1]/'src'/'reset'/'reset.py'))
        env=dict(os.environ,PATH='/no-command-available',PYTHONDONTWRITEBYTECODE='1')
        result=subprocess.run([sys.executable,'-B',script],capture_output=True,text=True,env=env)
        self.assertEqual(result.returncode,0,result.stderr)
        row=json.loads(result.stdout)
        self.assertFalse(row['robot_connected']);self.assertFalse(row['execute_unlocked'])
        result=subprocess.run([sys.executable,'-B',script,'--execute'],capture_output=True,text=True,env=env)
        self.assertEqual(result.returncode,2,result.stderr)
        self.assertFalse(json.loads(result.stdout)['publisher_enabled'])

    def test_guard_stops_after_lease_loss(self):
        from pathlib import Path
        import sys
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
        from live_protocol import Feedback
        entry=self.entry();current=[sample()]
        g=entry.ReturnGuard('a'*32,(0,0,0),lambda:current[0])
        poll=g.poll(0);self.assertTrue(g.accept(dict(type='input',session='a'*32,nonce=poll['nonce'],seq=1),.01))
        g.step(.02,Feedback(.02,1,True,0,0,0))
        current[0]=sample(.16,seq=2)
        out=g.step(.16,Feedback(.16,2,True,0,0,0))
        self.assertEqual((out.vx,out.wz),(0,0))
        self.assertEqual(g.finished_reason,'lease_lost')
        poll=g.poll(.17);g.accept(dict(type='input',session='a'*32,nonce=poll['nonce'],seq=2),.18)
        self.assertEqual(g.step(.18,Feedback(.18,3,True,0,0,0)).vx,0)
        self.assertEqual(g.finished_reason,'lease_lost')

    def test_guard_rejects_auto_mode(self):
        from pathlib import Path
        import sys
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
        from live_protocol import Feedback
        entry=self.entry()
        g=entry.ReturnGuard('a'*32,(0,0,0),lambda:sample())
        poll=g.poll(0);g.accept(dict(type='input',session='a'*32,nonce=poll['nonce'],seq=1),.01)
        self.assertEqual(g.step(.02,Feedback(.02,1,False,0,0,0)).vx,0)
        self.assertEqual(g.finished_reason,'manual_or_speed_feedback_lost')



class LateralCorrectionTests(unittest.TestCase):
    def test_near_goal_lateral_residual_replans_instead_of_stalling(self):
        c=ReturnController((0,0,0),0)
        c.step(sample(x=.07),0)
        out=c.step(sample(.02,x=.01,y=.025,seq=2),.02)
        self.assertEqual(out.vx,0)
        self.assertEqual(c.phase,'BRAKE')
        self.assertIsNone(c.finished_reason)



class ReviewRegressionTests(unittest.TestCase):
    def test_asynchronous_feedback_cannot_shorten_half_second_hold(self):
        c=ReturnController((0,0,0),.18);c.transition('VERIFY',.18)
        c.step(replace(sample(x=0,yaw=0),observed_at=.18),.18)
        for i in range(1,22):
            t=.18+i*.02
            c.step(replace(sample(t,x=0,yaw=0,seq=i+1),observed_at=t),t)
        self.assertIsNone(c.finished_reason)
        for i in range(22,28):
            t=.18+i*.02;c.step(replace(sample(t,x=0,yaw=0,seq=i+1),observed_at=t),t)
        self.assertEqual(c.finished_reason,'reset_complete')

    def test_fresh_dds_cannot_refresh_repeated_measurement_header(self):
        from logic import MeasurementFreshness
        c=MeasurementFreshness()
        self.assertEqual(c.sample(1000000000,0),0)
        self.assertIsNone(c.sample(1000000000,.02))
        self.assertEqual(c.last_at,0)
        self.assertIsNone(c.sample(990000000,.04))
        self.assertEqual(c.fault,'measurement_clock_regression')

    def test_lagging_measurement_header_latches_stale_fault(self):
        from logic import MeasurementFreshness
        c=MeasurementFreshness();c.sample(1000000000,0)
        self.assertIsNone(c.sample(1010000000,.25))
        self.assertEqual(c.fault,'measurement_stale')
        self.assertIsNone(c.sample(1300000000,.3))

    def test_detected_source_change_cannot_recover_before_tick(self):
        from logic import SourceLatch
        c=SourceLatch()
        self.assertIsNone(c.observe(True,True,False))
        self.assertEqual(c.observe(False,True,True),'unverified_pose_publisher')
        self.assertEqual(c.observe(True,True,True),'unverified_pose_publisher')

    def test_initial_discovery_gap_is_not_an_execution_fault(self):
        from logic import SourceLatch
        c=SourceLatch()
        self.assertEqual(c.observe(False,False,False),'unverified_pose_publisher')
        self.assertIsNone(c.observe(True,True,False))

    def test_repeated_overshoot_reversals_are_bounded(self):
        c=ReturnController((0,0,0),0);seq=1;t=0
        c.step(sample(t,x=.05,seq=seq),t)
        t+=.02;seq+=1;c.step(sample(t,x=.05,seq=seq),t)
        for i in range(4):
            x=-.025 if i%2==0 else .025
            for j in range(20):
                t+=.02;seq+=1;c.step(sample(t,x=x,seq=seq),t)
                if c.finished_reason:break
            if c.finished_reason:break
        self.assertEqual(c.finished_reason,'correction_limit')

    def test_completion_lost_during_stop_ack_is_not_success(self):
        entry=EntryTests().entry()
        from live_protocol import Feedback
        class Inbox:
            eof=False
            def __init__(self):self.sent=[]
            def receive(self):return []
            def send(self,row):self.sent.append(row)
        channel=Inbox();published=[];fb=[Feedback(.01,1,True,0,0,0)]
        r=entry.CompletionRuntime('execute','a'*32,0,channel,lambda:None,
            published.append,lambda:fb[0],lambda:None,discovery_s=0,
            final_check=lambda now:'pose_outside_tolerance')
        r.ready=True;r.ready_at=0;r.request_stop('reset_complete',0)
        for i,t in enumerate([.02,.08,.14],1):
            fb[0]=Feedback(t,i,True,0,0,0);r.tick(t)
        self.assertTrue(r.finished)
        self.assertTrue(channel.sent[-1]['confirmed'])
        self.assertNotEqual(channel.sent[-1]['reason'],'reset_complete')
        self.assertTrue(all(c.vx==0 and c.wz==0 for c in published))

    def test_raw_yaw_and_distinct_feedback_times_are_preserved(self):
        from types import SimpleNamespace as N
        from live_protocol import Feedback
        entry=EntryTests().entry()
        msg=N(header=N(frame_id='odom'),child_frame_id='base_link',
              pose=N(pose=N(position=N(x=.1,y=.2,z=0),
                            orientation=N(x=0,y=0,z=0,w=.4))))
        s=entry.decode_odometry(msg,1,1,Feedback(1.18,1,True,0,0,0))
        self.assertAlmostEqual(s.yaw,.4)
        self.assertEqual(s.at,1)
        self.assertEqual(s.observed_at,1.18)



class StraightTests(unittest.TestCase):
    def controller(self,target=(0,0,2)):
        from logic import StraightReturnController
        return StraightReturnController(target,0)
    def test_returns_backwards_without_turning(self):
        c=self.controller();out=c.step(sample(x=.81,y=.005,yaw=0),0)
        self.assertLess(out.vx,0)
        self.assertGreaterEqual(out.vx,-.05)
        self.assertEqual((out.vy,out.wz),(0,0))
    def test_refuses_unreachable_lateral_target_instead_of_turning(self):
        c=self.controller();out=c.step(sample(x=.2,y=.03,yaw=0),0)
        self.assertEqual((out.vx,out.vy,out.wz),(0,0,0))
        self.assertEqual(c.finished_reason,'straight_target_off_axis')
    def test_completion_ignores_saved_heading(self):
        c=self.controller()
        for i in range(35):
            out=c.step(sample(i*.02,x=0,yaw=0,seq=i+1),i*.02)
            self.assertEqual((out.vy,out.wz),(0,0))
        self.assertEqual(c.finished_reason,'reset_complete')
    def test_simulated_backward_return_with_inertia(self):
        c=self.controller();x=.809;y=.005;vx=0
        for i in range(6000):
            t=i*.02;out=c.step(Sample(x,y,0,vx,0,0,t,i+1),t)
            self.assertEqual((out.vy,out.wz),(0,0))
            self.assertLessEqual(abs(out.vx),.05)
            vx+=(out.vx-vx)*.15;x+=vx*.02
            if c.finished_reason:break
        self.assertEqual(c.finished_reason,'reset_complete')
        self.assertLessEqual(math.hypot(x,y),.02)
        self.assertLessEqual(abs(vx),.01)
    def test_freshness_fault_latches_without_any_turning(self):
        c=self.controller();c.step(sample(x=.2,yaw=0),0)
        out=c.step(sample(x=.2,yaw=0),.21)
        self.assertEqual((out.vx,out.wz),(0,0))
        self.assertEqual(c.finished_reason,'feedback_stale')
        self.assertEqual(c.step(sample(.22,x=.2,yaw=0,seq=2),.22).vx,0)
    def test_guard_runs_straight_controller(self):
        from live_protocol import Feedback
        entry=EntryTests().entry()
        g=entry.ReturnGuard('a'*32,(0,0,2),lambda:sample(x=.8,yaw=0))
        poll=g.poll(0)
        g.accept(dict(type='input',session='a'*32,nonce=poll['nonce'],seq=1),.01)
        out=g.step(.02,Feedback(.02,1,True,0,0,0))
        self.assertLess(out.vx,0);self.assertEqual(out.wz,0)
    def test_straight_verification_does_not_require_yaw_but_keeps_stop_gate(self):
        good=EntryTests().evidence()
        good.update(direction_and_scale_verified=False,straight_direction_and_scale_verified=True)
        self.assertEqual(validate_verification(good,1001,straight=True),good)
        with self.assertRaisesRegex(ValueError,'independent_stop_verified'):
            validate_verification(dict(good,independent_stop_verified=False),1001,straight=True)
    def test_preview_advertises_zero_angular_output(self):
        import json,subprocess,sys
        from pathlib import Path
        p=(Path(__file__).resolve().parents[1]/'src'/'reset'/'reset.py')
        r=subprocess.run([sys.executable,'-B',str(p)],capture_output=True,text=True,check=True)
        row=json.loads(r.stdout)
        self.assertEqual(row['control_mode'],'straight')
        self.assertFalse(row['restore_heading'])
        self.assertEqual(row['limits']['angular_radps'],0)



class StageTests(unittest.TestCase):
    def test_position_error_uses_start_axes(self):
        from logic import position_error
        error=position_error(sample(x=.01,y=.2,yaw=0),(0,0,math.pi/2))
        self.assertAlmostEqual(error['longitudinal_m'],.2)
        self.assertAlmostEqual(error['lateral_m'],-.01)
        self.assertAlmostEqual(error['distance_m'],math.hypot(.01,.2))
    def test_goal_tolerance_is_selected_by_stage(self):
        from logic import target_problem
        s=sample(x=0,yaw=1)
        self.assertIsNone(target_problem(s,(0,0,0),'straight'))
        self.assertEqual(target_problem(s,(0,0,0),'pose'),'heading_outside_tolerance')
    def test_guard_can_select_future_pose_stage(self):
        from live_protocol import Feedback
        entry=EntryTests().entry()
        g=entry.ReturnGuard('a'*32,(0,0,0),lambda:sample(x=.2,yaw=0),control_mode='pose')
        poll=g.poll(0);g.accept(dict(type='input',session='a'*32,nonce=poll['nonce'],seq=1),.01)
        out=g.step(.02,Feedback(.02,1,True,0,0,0))
        self.assertEqual(out.vx,0);self.assertNotEqual(out.wz,0)
    def test_pose_preview_keeps_separate_verification(self):
        import subprocess,sys,json
        from pathlib import Path
        r=subprocess.run([sys.executable,'-B',str((Path(__file__).resolve().parents[1]/'src'/'reset'/'reset.py')),
                          '--control-mode','pose'],capture_output=True,text=True,check=True)
        row=json.loads(r.stdout)
        self.assertEqual(row['control_mode'],'pose');self.assertTrue(row['restore_heading'])
        self.assertFalse(row['execute_unlocked'])
        self.assertIn('direction_and_scale_verified',row['lock_reason'])
    def test_invalid_stage_is_rejected(self):
        from logic import controller_for
        with self.assertRaises(ValueError):controller_for('unknown')

if __name__=='__main__':unittest.main()
