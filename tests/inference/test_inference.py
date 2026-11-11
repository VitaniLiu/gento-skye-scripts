import json
from pathlib import Path
import socket
import struct
import threading
import unittest
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
import sys
sys.path.insert(0, str(ROOT / "src/inference/host"))
sys.path.insert(0, str(ROOT / "src/inference/robot"))

from kinematics import JOINTS, Kinematics
from main import (build_infer_request, load_configuration, response_masks,
                  response_rows, server_address, task_spec)
from protocol import PolicyClient
from retrace import RecordedPath, Sample
from runtime import home_duration


class InferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_configuration(ROOT / "configs/inference/default.json", "default")
        models = ROOT / "src/inference/models"
        cls.kinematics = Kinematics(models / "runtime_expanded.urdf", models / "robot_Gento_Sky.xml")
        tree = ET.parse(models / "robot_Gento_Sky.xml").getroot()
        names = [j.get("name") for j in tree.find("worldbody").iter("joint") if j.get("name")]
        values = map(float, tree.find("./keyframe/key[@name='home']").get("qpos").split())
        cls.home = dict(zip(names, values))

    def test_every_non_hand_action_field_reaches_the_robot_row(self):
        q = np.asarray([self.home[name] for name in JOINTS])
        action = np.zeros(78)
        action[27:32] = [.03, -.02, .49631398916244507, .07, .04]
        q[16:19] = [action[29], action[28], action[27]]
        frames = self.kinematics.forward(q)
        for start, name in zip((0, 9, 18), ("flange_L", "flange_R", "eyes_link")):
            action[start:start + 9] = self.kinematics.pose(frames[name])
        action[54], action[77] = .25, .75
        row = self.kinematics.action_rows(action[None], np.ones((1, 78), dtype=bool), self.home)[0]
        self.assertTrue(np.allclose(row["body"], [action[29], action[28], action[27]]))
        self.assertEqual(row["vx"], action[30])
        self.assertEqual(row["wz"], -action[31])
        self.assertEqual(row["left_gripper"], action[54])
        self.assertEqual(row["right_gripper"], action[77])
        self.assertEqual([len(group) for group in row["groups"]], [7, 7, 2])

    def test_flange_state_and_ik_round_trip(self):
        config = self.config
        k = self.kinematics
        for delta in (0.0, 0.03, -0.03):
            with self.subTest(delta=delta):
                target = dict(self.home)
                target["Joint2_L"] += delta
                target["Joint2_R"] -= delta
                observation = {"joint_names": list(JOINTS),
                               "joint_positions": [target[n] for n in JOINTS],
                               "base_vx": 0.0, "base_wz": 0.0,
                               "left_gripper": 0.0, "right_gripper": 0.0}
                state = k.state78(observation, config["grippers"])
                frames = k.forward(observation["joint_positions"])
                self.assertNotIn("left_tool", frames)
                self.assertNotIn("right_tool", frames)
                for start, name in ((0, "flange_L"), (9, "flange_R")):
                    np.testing.assert_allclose(state[start:start+9], k.pose(frames[name]), atol=1e-7)
                mask = np.zeros((1, 78), dtype=bool)
                mask[:, :18] = True
                row = k.action_rows(state[None], mask, self.home)[0]
                solved = sum(row["groups"], []) + row["body"]
                actual = k.forward(solved)
                for name in ("flange_L", "flange_R"):
                    self.assertLess(np.linalg.norm(actual[name][:3, 3] - frames[name][:3, 3]), .005)
                    self.assertLess(np.linalg.norm(actual[name][:3, :3] - frames[name][:3, :3]), .043)

    def test_raw_gripper_feedback_normalized_once(self):
        config = self.config
        for fraction in (0.0, 0.5, 1.0):
            observation = {"joint_names": list(JOINTS),
                           "joint_positions": [self.home[name] for name in JOINTS],
                           "base_vx": 0.0, "base_wz": 0.0}
            for side in ("left", "right"):
                bounds = config["grippers"][side]
                observation[side + "_gripper"] = bounds["closed"] + fraction * (bounds["open"] - bounds["closed"])
            state = self.kinematics.state78(observation, config["grippers"])
            np.testing.assert_allclose(state[[54,77]], [fraction,fraction], atol=1e-7)
            mask = np.zeros((1,78), dtype=bool); mask[:,[54,77]] = True
            row = self.kinematics.action_rows(state[None], mask, self.home)[0]
            self.assertAlmostEqual(row["left_gripper"], fraction)
            self.assertAlmostEqual(row["right_gripper"], fraction)

    def test_home_duration_accepts_service_units_and_punctuation(self):
        self.assertEqual(home_duration("ok duration=1.000000s, motion started"), 1.0)
        self.assertEqual(home_duration("no duration field"), 20.0)

    def test_protocol_uses_one_length_prefixed_request_and_response(self):
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        port = listener.getsockname()[1]
        seen = []
        def server():
            connection, _ = listener.accept()
            size = struct.unpack("!I", connection.recv(4))[0]
            seen.append(json.loads(connection.recv(size)))
            response = json.dumps({"ok": True, "action_dim": 78}).encode()
            connection.sendall(struct.pack("!I", len(response)) + response)
            connection.close()
        thread = threading.Thread(target=server)
        thread.start()
        with PolicyClient("127.0.0.1", port, 2) as client:
            result = client.request("info")
        thread.join()
        listener.close()
        self.assertEqual(seen, [{"command": "info"}])
        self.assertEqual(result["action_dim"], 78)

    def test_model_deployment_selects_server_and_task_file(self):
        self.assertEqual(self.config["server"]["model"], "default")
        self.assertEqual(set(self.config["inference"]["tasks"]), {"gouwu", "toulaji"})
        direct = {"server": {"transport": "tcp", "server_host": "127.0.0.1",
                             "server_port": 8000}}
        self.assertEqual(server_address(direct), ("127.0.0.1", 8000, None))

    def test_task_masks_are_sent_as_78_boolean_dimensions(self):
        settings = self.config["inference"]
        _, gouwu = task_spec(settings, "gouwu")
        _, toulaji = task_spec(settings, "toulaji")
        self.assertEqual(len(gouwu), 78)
        self.assertTrue(all(type(value) is bool for value in gouwu))
        self.assertEqual([i for i, enabled in enumerate(gouwu) if not enabled],
                         [27, 28, 29, 31, *range(32, 54), *range(55, 77)])
        self.assertEqual([i for i, enabled in enumerate(toulaji) if not enabled],
                         [27, 28, *range(32, 54), *range(55, 77)])

    def test_response_mask_blocks_robot_fields(self):
        settings = self.config["inference"]
        _, task_mask = task_spec(settings, "gouwu")
        server_mask = [True] * 78
        server_mask[30] = False
        masks = response_masks({"mask": server_mask}, task_mask, 1, 78)
        q = np.asarray([self.home[name] for name in JOINTS])
        frames = self.kinematics.forward(q)
        action = np.zeros(78)
        for start, name in zip((0, 9, 18), ("flange_L", "flange_R", "eyes_link")):
            action[start:start + 9] = self.kinematics.pose(frames[name])
        action[27:32] = [1, 1, 1, 1, 1]
        action[32:54] = np.nan
        action[55:77] = np.nan
        action[54], action[77] = .2, .8
        row = self.kinematics.action_rows(action[None], masks, self.home)[0]
        self.assertTrue(np.allclose(row["body"], [self.home["waist_joint_1"],
                                                   self.home["waist_joint_2"],
                                                   self.home["waist_joint_3"]]))
        self.assertEqual(row["vx"], 0.0)
        self.assertEqual(row["wz"], 0.0)
        self.assertEqual(row["left_gripper"], .2)
        self.assertEqual(row["right_gripper"], .8)

    def test_response_without_mask_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "does not contain a mask"):
            response_masks({}, [True] * 78, 1, 78)

    def test_protocol_waist_height_is_converted_in_both_directions(self):
        config = self.config
        settings = config["inference"]
        _, task_mask = task_spec(settings, "toulaji")
        class FakeKinematics:
            def state78(self, _observation, _calibration):
                state = np.zeros(78, dtype=np.float32)
                state[29] = .49593398
                return state
            def action_rows(self, actions, _masks, _seed):
                self.actions = actions
                return []
        fake = FakeKinematics()
        observation = {"timestamp_ns": 1, "joint_names": list(JOINTS),
                       "joint_positions": [self.home[name] for name in JOINTS]}
        request = build_infer_request(settings, "toulaji", "toulaji", task_mask,
                                      "request", "session", observation, (2, "jpeg"),
                                      fake, config["grippers"])
        self.assertAlmostEqual(request["observation"]["state"][29], .020292997, places=6)
        action = np.zeros((1, 78))
        action[0, 29] = request["observation"]["state"][29]
        server_mask = np.zeros((1, 78), dtype=bool)
        server_mask[0, 29] = True
        response_rows({"request_id": "request", "actions": action.tolist(),
                       "mask": server_mask.tolist()}, "request", task_mask, 1,
                      settings, observation, fake)
        self.assertAlmostEqual(fake.actions[0, 29], .49593398, places=6)

    def test_chassis_return_finishes_at_recorded_origin(self):
        path = RecordedPath()
        for x in (0, .03, .06):
            path.observe(Sample(x, 0, 0, 0, 0, 0))
        controller = path.controller()
        now = 0.0
        sample = Sample(.06, 0, 0, 0, 0, 0)
        for _ in range(500):
            vx, wz = controller.step(sample, now)
            now += .02
            sample = Sample(sample.x + vx * .02, 0, sample.yaw + wz * .02, vx, 0, wz)
            if controller.finished:
                break
        self.assertTrue(controller.finished)
        self.assertLessEqual(abs(sample.x), .007)


if __name__ == "__main__":
    unittest.main()
