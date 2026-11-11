"""Offline startup tests: no ROS, subprocesses, control connections or robot construction."""
import importlib
import hashlib
import json
from pathlib import Path
import sys
import threading
import unittest
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/inference/host"))
from kinematics import JOINTS, Kinematics
from main import response_rows, task_spec


class StartupAlignmentTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue((ROOT / "src/inference/host/startup.py").exists(), "startup alignment is missing")
        self.startup = importlib.import_module("startup")
        self.config = json.loads((ROOT / "configs/inference/default.json").read_text())
        self.tasks = json.loads((ROOT / "configs/inference/tasks_eightpost.json").read_text())["tasks"]
        self.config["inference"]["tasks"] = self.tasks
        self.kin = Kinematics(ROOT / "src/inference/models/runtime_expanded.urdf", ROOT / "src/inference/models/robot_Gento_Sky.xml")
        home = ET.parse(ROOT / "src/inference/models/robot_Gento_Sky.xml").find(".//key[@name=\"home\"]")
        qpos = np.fromstring(home.get("qpos"), sep=" ")
        q = np.r_[qpos[5:12], qpos[12:19], qpos[3:5], qpos[:3]]
        q[16] = self.config["robot"]["reset_height_m"]
        self.home = dict(joint_names=list(JOINTS), joint_positions=q.tolist(), base_vx=0., base_wz=0.,
                         left_gripper=self.config["grippers"]["left"]["closed"],
                         right_gripper=self.config["grippers"]["right"]["closed"], timestamp_ns=100)

    def parameters(self, task):
        _, mask = task_spec(self.config["inference"], task)
        return self.tasks[task], mask

    def test_targets_are_exact_data_state_not_actions_and_base_is_zero(self):
        for task in self.tasks:
            record, mask = self.parameters(task)
            expected = {"gouwu": "e5d5ecdc5807eb885e2e4ace658fb23b09b34a1d6481f2a17c5c0a0b758bdcd1",
                        "toulaji": "704e2ba566d6ccef5476b697c2fad0b8332fdb288b45b325cb29bcf03d323254"}
            state = np.asarray(record["startup"]["state"], dtype="<f4")
            self.assertEqual(hashlib.sha256(state.tobytes()).hexdigest(), expected[task])
            target, active = self.startup.startup_target(record["startup"], mask)
            np.testing.assert_allclose(target[:30], state[:30], rtol=0, atol=0)
            self.assertEqual(target[30:32].tolist(), [0., 0.])
            self.assertFalse(active[30] or active[31])
            np.testing.assert_array_equal(active[:30], mask[:30])

    def run_alignment(self, task, fail_arrival=False, cancel=False):
        record, mask = self.parameters(task)
        outer = self
        class Robot:
            sent = []
            def send(self, command, **fields):
                self.sent.append((command, fields))
            def wait(self, expected, timeout):
                if expected == "executed":
                    return dict(cancelled=cancel, sent=len(self.sent[-1][1]["rows"]))
                if len(self.sent) == 1 or fail_arrival:
                    return outer.home
                row = next(fields["rows"][-1] for command, fields in self.sent if command == "execute")
                return outer.from_row(row)
        robot = Robot()
        # Instance state avoids sharing recorded commands across runs.
        robot.sent = []
        result = self.startup.align_startup(robot, record, mask, self.config["inference"],
            self.kin, self.config["grippers"], threading.Event(), response_rows)
        return robot, result

    def from_row(self, row):
        observation = self.home.copy()
        observation["joint_positions"] = sum(row["groups"], []) + row["body"]
        for side in ("left", "right"):
            bounds = self.config["grippers"][side]
            observation[side + "_gripper"] = bounds["closed"] + row[side + "_gripper"] * (bounds["open"] - bounds["closed"])
        observation["timestamp_ns"] = 200
        return observation

    def test_each_task_plans_from_home_and_confirms_measured_arrival(self):
        for task in self.tasks:
            robot, arrival = self.run_alignment(task)
            execution = [fields for command, fields in robot.sent if command == "execute"]
            self.assertEqual(len(execution), 1)
            self.assertEqual(execution[0]["dt"], .01)
            rows = execution[0]["rows"]
            np.testing.assert_allclose(sum(rows[0]["groups"], []) + rows[0]["body"], self.home["joint_positions"])
            self.assertTrue(all(row["vx"] == row["wz"] == 0 for row in rows))
            self.assertEqual(arrival["timestamp_ns"], 200)
            target, active = self.startup.startup_target(self.tasks[task]["startup"], self.parameters(task)[1])
            self.assertEqual(self.startup.arrival_errors(arrival, target, active, self.config["inference"],
                self.kin, self.config["grippers"], self.tasks[task]["startup"]), {})

    def test_cancel_does_not_complete_startup(self):
        with self.assertRaises(self.startup.StartupCancelled):
            self.run_alignment("toulaji", cancel=True)

    def test_failed_arrival_aborts(self):
        self.tasks["toulaji"]["startup"]["arrival_timeout_s"] = 0
        with self.assertRaisesRegex(RuntimeError, "arrival"):
            self.run_alignment("toulaji", fail_arrival=True)

    def test_legacy_task_does_not_observe_or_execute(self):
        class Robot:
            def send(self, *args, **kwargs):
                raise AssertionError("legacy task should not call endpoint")
        self.assertIsNone(self.startup.align_startup(Robot(), {}, [True]*78, self.config["inference"],
            self.kin, self.config["grippers"], threading.Event(), response_rows))

    def test_already_aligned_does_not_execute(self):
        robot, arrived = self.run_alignment("toulaji")
        outer = self
        class Robot:
            def send(self, command, **fields):
                outer.assertEqual(command, "observe")
            def wait(self, expected, timeout):
                return arrived
        record, mask = self.parameters("toulaji")
        actual = self.startup.align_startup(Robot(), record, mask, self.config["inference"],
            self.kin, self.config["grippers"], threading.Event(), response_rows)
        self.assertEqual(actual, arrived)


class StartupLaunchTests(unittest.TestCase):
    def test_stale_camera_packets_cannot_repopulate_reset_history(self):
        import main
        from images import ImageHistory
        robot = main.RobotClient.__new__(main.RobotClient)
        self.assertTrue(hasattr(robot, "reset_image_history"), "missing startup image reset")
        robot.image_lock = threading.Lock()
        robot.image_after_ns = 0
        robot.history = ImageHistory(dict(input_size=[2,2], crop_xyxy=[0,0,2,2], output_size=[2,2], encoding="PNG"), [-.8, 0])
        rgb = np.zeros((2,2,3), dtype=np.uint8)
        robot.history.append(10, rgb)
        robot.reset_image_history(100)
        robot.decode_camera = True
        robot.messages = __import__("queue").Queue()
        robot.decoder = type("Decoder", (), {"decode": lambda self, stamp, data: (stamp, rgb)})()
        robot.process = type("Process", (), {"stdout": [json.dumps(dict(type="camera", timestamp_ns=t, data="")) for t in [90,100,110]]})()
        robot._read()
        self.assertEqual([frame[0] for frame in robot.history.frames], [110])
        self.assertEqual(robot.history.sample(110)["image_padding_mask"], [True,False])

    def test_launch_cancellation_never_resets_or_infers_policy(self):
        import main
        from unittest.mock import patch
        events = []
        root = ROOT
        config = json.loads((root / "configs/inference/default.json").read_text())
        config["inference"]["tasks"] = json.loads((root / "configs/inference/tasks_eightpost.json").read_text())["tasks"]
        config["inference"].update(model="test", instruction_mode="request")
        config["server"] = dict(timeout_s=1)
        class Policy:
            def __init__(self, *args, **kwargs): pass
            def __enter__(self): return self
            def __exit__(self, *args): pass
            def request(self, command, **fields):
                events.append(command)
                return {}
        class Robot:
            process = type("Process", (), {"poll": lambda self: None})()
            def __init__(self, *args, **kwargs): pass
            def send(self, command, **fields): events.append(command)
            def wait(self, expected, **fields):
                events.append(expected)
                return {}
            def close(self): pass
        self.assertTrue(hasattr(main, "align_startup"), "startup hook is missing")
        with patch.object(main, "load_configuration", return_value=config), patch.object(main, "server_address", return_value=("offline",0,None)), patch.object(main, "PolicyClient", Policy), patch.object(main, "RobotClient", Robot), patch.object(main, "Kinematics"), patch.object(main, "q_listener"), patch.object(main, "align_startup", side_effect=main.StartupCancelled("cancelled")):
            self.assertEqual(main.main(["--task","toulaji","--model","offline","--execute"]), 130)
        self.assertIn("ready", events)
        self.assertIn("finished", events)
        self.assertNotIn("reset", events)
        self.assertNotIn("infer", events)

    def test_launch_acquires_fresh_images_before_policy_reset(self):
        import main
        from unittest.mock import patch
        events = []
        config = json.loads((ROOT / "configs/inference/default.json").read_text())
        config["inference"]["tasks"] = json.loads((ROOT / "configs/inference/tasks_eightpost.json").read_text())["tasks"]
        config["inference"].update(model="test", instruction_mode="request")
        config["server"] = dict(timeout_s=1)
        class ResetReached(Exception): pass
        class Policy:
            def __init__(self, *args, **kwargs): pass
            def __enter__(self): return self
            def __exit__(self, *args): pass
            def request(self, command, **fields):
                events.append(command)
                if command == "reset": raise ResetReached()
                return {}
        class Robot:
            process = type("Process", (), {"poll": lambda self: None})()
            def __init__(self, *args, **kwargs): pass
            def send(self, command, **fields): self.fields = fields; events.append(command)
            def wait(self, expected, **fields):
                events.append(expected)
                return dict(timestamp_ns=self.fields.get("timestamp_ns",200)) if expected == "observation" else {}
            def reset_image_history(self, after_ns): events.append("clear_history")
            def image_anchor(self, cutoff): return 110
            def image_history(self, timestamp): events.append("fresh_images"); return {}
            def close(self): pass
        with patch.object(main, "load_configuration", return_value=config), patch.object(main, "server_address", return_value=("offline",0,None)), patch.object(main, "PolicyClient", Policy), patch.object(main, "RobotClient", Robot), patch.object(main, "Kinematics"), patch.object(main, "q_listener"), patch.object(main, "align_startup", return_value=dict(timestamp_ns=100)):
            with self.assertRaises(ResetReached): main.main(["--task","toulaji","--model","offline","--execute"])
        self.assertLess(events.index("ready"), events.index("clear_history"))
        self.assertLess(events.index("clear_history"), events.index("fresh_images"))
        self.assertLess(events.index("fresh_images"), events.index("reset"))

if __name__ == "__main__":
    unittest.main()
