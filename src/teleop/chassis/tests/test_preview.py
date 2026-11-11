import json
import io
import os
from controller import DpadController, InputFrame, SafetyState
from pathlib import Path
import struct
import subprocess
import sys
import unittest
import tempfile
import preview

ROOT=Path(__file__).resolve().parents[1]

class PreviewTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory(dir=ROOT.parents[2]/'tests')
        self.addCleanup(temp.cleanup)
        self.log_root=Path(temp.name)/'preview-logs'
        self.cli_env=os.environ.copy()
        self.cli_env['CHASSIS_LOGS_DIR']=str(self.log_root)
        self.cli_env['PYTHONDONTWRITEBYTECODE']='1'

    def test_linux_dpad_full_values_and_l1_are_decoded(self):
        axes=[0.0]*8; buttons=[0]*13
        preview.apply_js_event(axes,buttons,struct.pack("IhBB",0,-32767,0x82,7))
        preview.apply_js_event(axes,buttons,struct.pack("IhBB",1,1,1,4))
        self.assertEqual((axes[7],buttons[4]),(-1.0,1))
        preview.apply_js_event(axes,buttons,struct.pack("IhBB",2,32767,2,6))
        self.assertEqual(axes[6],1.0)

    def test_invalid_input_packets_are_rejected(self):
        for raw in [b"short",struct.pack("IhBB",0,1,1,99),struct.pack("IhBB",0,2,1,4),struct.pack("IhBB",0,1,2,99)]:
            with self.subTest(raw=raw),self.assertRaises(ValueError):
                preview.apply_js_event([0.0]*8,[0]*13,raw)

    def run_cli(self,*args):
        r=subprocess.run([sys.executable,"-B",str(ROOT/"src"/"teleop"/"preview.py"),*args],cwd=ROOT,capture_output=True,text=True,timeout=8,env=self.cli_env)
        return r,[json.loads(line) for line in r.stdout.splitlines() if line.startswith("{")]

    def test_demo_outputs_four_exact_targets_and_press_logs_in_project(self):
        r,rows=self.run_cli("--demo","--simulate-ready")
        self.assertEqual(r.returncode,0,r.stderr)
        targets=[(x['vx'],x['wz']) for x in rows if x.get('type')=='target' and (x['vx'] or x['wz'])]
        self.assertEqual(targets,[(.2,0.0),(-.2,0.0),(0.0,.2),(0.0,-.2)])
        startup=[x for x in rows if x.get('type')=='startup']
        self.assertEqual(len(startup),1)
        path=Path(startup[0]['log_path'])
        self.assertEqual(path.parent.parent,self.log_root/'preview')
        self.assertTrue(path.is_file())
        events=[json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual([(e['direction'],e['duration_s']) for e in events],[("up",2.0),("down",1.0),("left",2.0),("right",.5)])
        self.assertTrue(all(e['complete'] for e in events))
        self.assertTrue(all(e['input_source']=='demo' and e['sent_to_robot'] is False for e in events))
        self.assertTrue(all(x.get('sent_to_robot') is False for x in rows))

    def test_default_preview_is_not_assumed_armed(self):
        r,rows=self.run_cli("--demo","--no-log")
        self.assertEqual(r.returncode,0,r.stderr)
        targets=[x for x in rows if x.get('type')=='target']
        self.assertTrue(targets)
        self.assertTrue(all(x['vx']==x['vy']==x['wz']==0 for x in targets))
        self.assertEqual(len([x for x in rows if x.get('type')=='press']),4)

    def test_bad_duration_is_rejected_before_monitoring(self):
        r,_=self.run_cli("--seconds","nan","--no-log")
        self.assertEqual(r.returncode,2)


class ReviewRegressionTests(unittest.TestCase):
    def baseline(self):
        stream=preview.JoystickEventStream(); controller=DpadController()
        for i in range(13):
            samples=stream.feed(struct.pack("IhBB",1000,0,0x81,i),0.0)
        for i in range(8):
            samples=stream.feed(struct.pack("IhBB",1000,0,0x82,i),0.0)
            for now,frame in samples:controller.update(frame,SafetyState(True,True,0),now,"initial")
        return stream,controller

    def feed(self,stream,controller,stamp,value,kind,index):
        return [controller.update(frame,SafetyState(True,True,0),now,str(now))
                for now,frame in stream.feed(struct.pack("IhBB",stamp,value,kind,index),0.5)]

    def test_fast_tap_survives_batching_and_uses_kernel_event_duration(self):
        stream,c=self.baseline()
        self.feed(stream,c,1010,1,1,4)
        pressed=self.feed(stream,c,1020,-32767,2,7)
        released=self.feed(stream,c,1030,0,2,7)
        self.assertEqual(len(pressed),1)
        self.assertEqual(pressed[0].command.vx,.2)
        self.assertEqual(len(released[0].events),1)
        self.assertEqual(released[0].events[0].duration_s,.01)

    def test_queued_l1_release_repress_cannot_bypass_recenter(self):
        stream,c=self.baseline()
        self.feed(stream,c,1010,1,1,4)
        self.feed(stream,c,1020,-32767,2,7)
        self.feed(stream,c,1030,0,1,4)
        results=self.feed(stream,c,1040,1,1,4)
        self.assertEqual(len(results),1)
        self.assertEqual(results[0].command.vx,0.0)
        self.assertEqual(results[0].command.reason,"center_dpad_first")

    def test_closed_terminal_does_not_lose_completed_press_log(self):
        c=DpadController();safety=SafetyState()
        c.update(InputFrame((0,)*8,(0,)*13),safety,0,"start")
        axes=[0]*8;axes[7]=-1
        c.update(InputFrame(tuple(axes),(0,)*13),safety,1,"pressed")
        result=c.update(InputFrame((0,)*8,(0,)*13),safety,2,"released")
        log=io.StringIO();terminal=io.StringIO();terminal.close()
        preview.show_result(result,"joystick",log,out=terminal)
        event=json.loads(log.getvalue())
        self.assertEqual((event['direction'],event['duration_s']),("up",1))

    def test_disconnect_after_queued_tap_keeps_prior_edges(self):
        stream,c=self.baseline()
        reader=preview.Joystick("unused")
        reader.events=stream
        read_fd,write_fd=os.pipe()
        reader.fd=read_fd
        os.set_blocking(read_fd,False)
        try:
            data=b"".join([struct.pack("IhBB",1010,1,1,4),
                           struct.pack("IhBB",1020,-32767,2,7),
                           struct.pack("IhBB",1030,0,2,7)])+b"bad"
            os.write(write_fd,data)
            results=[]
            with self.assertRaises(OSError):
                for now,frame in reader.poll(.5):
                    results.append(c.update(frame,SafetyState(True,True,0),now,str(now)))
            events=[event for result in results for event in result.events]
            self.assertEqual([(event.direction,event.duration_s) for event in events],[("up",.01)])
        finally:
            os.close(read_fd);os.close(write_fd)

    def test_unknown_log_source_is_rejected(self):
        with self.assertRaises(ValueError):
            _,stream=preview.open_project_log("unsupported")
            stream.close()

    def test_malformed_collection_and_huge_number_fail_closed(self):
        malformed=[InputFrame(tuple(i/8 for i in range(8)),(0,)*13),
                   InputFrame(set(i/8 for i in range(8)),(0,)*13),
                   InputFrame((10**400,)+(0,)*7,(0,)*13)]
        for frame in malformed[1:]:
            result=DpadController().update(frame,SafetyState(True,True,0),0,"now")
            self.assertEqual(result.command.reason,"invalid_input")

if __name__=="__main__":unittest.main()
