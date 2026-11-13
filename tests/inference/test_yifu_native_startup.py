"""Offline tests for the confirmed yifu native startup posture."""
import copy
import json
import sys
import threading
import unittest
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/inference/host"))
from startup import align_startup, StartupCancelled
from kinematics import JOINTS, Kinematics

class NativeStartupTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((ROOT / "configs/inference/default.json").read_text())
        self.record = {"startup": json.loads((ROOT / "tests/inference/yifu_native_startup.json").read_text())}
        self.kin = Kinematics(ROOT / "src/inference/models/runtime_expanded.urdf", ROOT / "src/inference/models/robot_Gento_Sky.xml")
        self.target = [self.record["startup"]["joint_positions"][n] for n in JOINTS[:14]] + [0., 0., .496306, 0., 0.]
        self.start = self.target.copy()
        self.start[4] += 1.57
        self.start[11] -= 1.57
        self.start[5] = self.start[12] = 0.
    def run_case(self, mode="normal", record=None):
        outer = self
        class Robot:
            def __init__(self): self.sent = []; self.rows = None
            def send(self, command, **kw):
                self.sent.append(command)
                if command == "execute": self.rows = kw["rows"]
            def wait(self, kind, timeout):
                if kind == "executed":
                    return dict(cancelled=mode == "cancel", sent=len(self.rows) - (mode == "partial"))
                q = outer.target if mode == "already" or (self.rows is not None and mode != "failure") else outer.start
                return dict(joint_names=list(JOINTS), joint_positions=q, timestamp_ns=200 if self.rows else 100)
        robot = Robot()
        result = align_startup(robot, record or self.record, [True]*78, self.config["inference"], self.kin,
                               self.config["grippers"], threading.Event(), None)
        return robot, result
    def test_exact_targets_preserve_other_channels(self):
        robot, arrival = self.run_case()
        rows = robot.rows
        np.testing.assert_allclose(sum(rows[-1]["groups"], []) + rows[-1]["body"], self.target)
        for row in rows:
            self.assertEqual(row["vx"], 0.)
            self.assertEqual(row["wz"], 0.)
            self.assertIsNone(row["left_gripper"])
            self.assertIsNone(row["right_gripper"])
            np.testing.assert_allclose(row["groups"][2] + row["body"], self.start[14:])
        self.assertGreaterEqual(rows[-1]["time_s"], 8.)
        self.assertEqual(arrival["timestamp_ns"], 200)
    def test_already_arrived_does_not_move(self):
        robot, _ = self.run_case("already")
        self.assertIsNone(robot.rows)
    def test_cancel_stops_startup(self):
        with self.assertRaises(StartupCancelled): self.run_case("cancel")
    def test_partial_execution_rejected(self):
        with self.assertRaises(RuntimeError): self.run_case("partial")
    def test_failed_arrival_rejected(self):
        record = copy.deepcopy(self.record)
        record["startup"]["arrival_timeout_s"] = .001
        with self.assertRaisesRegex(RuntimeError, "arrival failed"): self.run_case("failure", record)
    def test_limit_rejected_before_motion(self):
        record = copy.deepcopy(self.record)
        record["startup"]["joint_positions"]["Joint5_L"] = 10.
        with self.assertRaises(ValueError): self.run_case(record=record)
    def test_missing_startup_is_unchanged(self):
        self.assertIsNone(align_startup(None, {}, [], {}, None, None, None, None))
if __name__ == "__main__": unittest.main()
