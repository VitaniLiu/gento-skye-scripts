import sys
from pathlib import Path
import unittest
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "src/inference/robot"), str(ROOT / "src/inference/host")]
from alignment import ObservationHistory
from main import aligned_observation

class AlignmentTests(unittest.TestCase):
    def test_snapshot_uses_same_anchor_and_no_future_numeric_samples(self):
        h = ObservationHistory()
        for stamp in (100,200,300):
            h.append("joints",stamp,{"joint":float(stamp)})
            h.append("left_gripper",stamp+5,float(stamp))
            h.append("right_gripper",stamp+8,float(stamp))
            h.append("base",stamp+10,(float(stamp),-float(stamp)))
        s=h.snapshot(250)
        self.assertEqual(s["timestamp_ns"],250)
        self.assertEqual(s["joint_positions"],[200.])
        self.assertEqual(s["left_gripper"],200.)
        self.assertEqual(s["right_gripper"],200.)
        self.assertEqual(s["base_vx"],200.)
        self.assertTrue(all(t<=250 for t in s["source_timestamps_ns"].values()))
        self.assertEqual(h.snapshot()["timestamp_ns"],300)

    def test_no_cover_is_not_fabricated_and_buffer_is_bounded(self):
        h=ObservationHistory(capacity=2)
        with self.assertRaises(RuntimeError):h.snapshot()
        for stamp in (100,200,300):
            for key in h.streams:h.append(key,stamp,{} if key=="joints" else (0,0) if key=="base" else 0)
        with self.assertRaises(RuntimeError):h.snapshot(150)
        self.assertTrue(all(len(rows)==2 for rows in h.streams.values()))

    def test_host_anchors_to_actual_image_and_requests_numeric_snapshot(self):
        class Robot:
            def __init__(self):self.sent=[]
            def send(self,command,**fields):self.sent.append((command,fields))
            def wait(self,*args,**kw):return {"timestamp_ns":1000 if len(self.sent)==1 else self.sent[-1][1]["timestamp_ns"]}
            def image_anchor(self,cutoff):
                self.cutoff=cutoff;return 970
            def image_history(self,t0):return {"image_timestamps_ns":[970]}
        r=Robot();obs,images=aligned_observation(r)
        self.assertEqual(r.cutoff,1000)
        self.assertEqual(obs["timestamp_ns"],970)
        self.assertEqual(r.sent[-1],("observe",{"timestamp_ns":970}))
        self.assertEqual(images["image_timestamps_ns"][-1],obs["timestamp_ns"])

if __name__ == "__main__":unittest.main()
