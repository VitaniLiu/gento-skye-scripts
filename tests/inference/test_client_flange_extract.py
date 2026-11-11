import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("client_flange_test", ROOT / "test/client_test/client_test.py")
c = importlib.util.module_from_spec(spec)
spec.loader.exec_module(c)

class FlangeExtractionTests(unittest.TestCase):
    def test_recorded_obs_and_command_fk(self):
        schema = c.flange_schema()
        self.assertEqual(c.preflight(schema), [])
        arms = [b for b in schema["bindings"] if b.get("eef") in ("flange_L", "flange_R")]
        self.assertTrue(all(b["kind"] == "fk" for b in arms if b["target"] == "state"))
        self.assertTrue(all(b["kind"] == "fk_command" for b in arms if b["target"] == "action"))
        fk = c.RecordedObservationFK()
        q = np.zeros(len(c.JOINTS)); q[16] = .4963
        poses = fk.poses78(list(c.JOINTS), q)
        frames = c.make_kinematics().forward(q)
        policy = c.policy_for("toulaji")
        for start, name in ((0, "flange_L"), (9, "flange_R")):
            raw = np.zeros(78); raw[start:start+9] = poses[name]
            converted = c.transform(raw, c.structural(policy).astype(bool), policy)
            np.testing.assert_allclose(converted[start:start+9], c.Kinematics.pose(frames[name]), atol=1e-7)
        shared = json.loads((ROOT / "configs/processing/action78_posttrain.json").read_text())
        self.assertEqual(shared["pose_reference"]["eef_definitions"]["left"], "left_tool")

    def test_legacy_and_wrong_frames_are_rejected(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "test/client_test/data") as scratch:
            p = Path(scratch) / "protocol.jsonl"
            for header in ({"kind": "header", "schema": "gento_skye_inference_protocol_episode_v1"},
                           {"kind": "header", "schema": c.PROTOCOL_SCHEMA, "pose_contract": {}}):
                p.write_text(json.dumps(header) + "\n")
                with self.assertRaises(ValueError):
                    next(c.read_protocol(p))
            p.write_text(json.dumps({"kind": "header", "schema": c.PROTOCOL_SCHEMA,
                                    "pose_contract": c.POSE_CONTRACT}) + "\n")
            self.assertEqual(next(c.read_protocol(p))["pose_contract"], c.POSE_CONTRACT)

if __name__ == "__main__":
    unittest.main()
