import base64
from io import BytesIO
import json
from pathlib import Path
import sys
import unittest
import zipfile
import numpy as np
from scipy.spatial.transform import Rotation
from PIL import Image
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/inference/host"))
from images import ImageHistory, prepare_rgb
from main import build_infer_request_from_state, check_info, response_rows, task_spec
from kinematics import JOINTS, Kinematics

class SkyeHandoffTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = json.loads((ROOT / "configs/inference/default.json").read_text())
        cls.settings = cls.config["inference"]
        cls.z = zipfile.ZipFile(ROOT / "tests/inference/skye_handoff_20261111/sample.zip")
        cls.sample = json.loads(cls.z.read("single_sample/sample.json"))

    @classmethod
    def tearDownClass(cls):
        cls.z.close()

    def test_sample_crop_matches_exact_pixels(self):
        original = Image.open(BytesIO(self.z.read("single_sample/current_original_1280x720.png"))).convert("RGB")
        expected = Image.open(BytesIO(self.z.read("single_sample/observed_rgb320/04_frame_000538.png"))).convert("RGB")
        actual = prepare_rgb(np.asarray(original), self.settings["tasks"]["toulaji"]["image"])
        np.testing.assert_array_equal(actual, expected)
        other = prepare_rgb(np.asarray(original), self.settings["tasks"]["gouwu"]["image"])
        reference = original.crop((164,255,1179,704)).resize((320,320), Image.Resampling.LANCZOS)
        np.testing.assert_array_equal(other, reference)

    def test_history_is_causal_and_padding_explicit(self):
        spec = self.settings["tasks"]["toulaji"]["image"]
        h = ImageHistory(spec, self.settings["image_history_offsets_s"])
        rgb = np.zeros((720,1280,3), np.uint8)
        for stamp in (1000000000,1200000000,1400000000,1600000000,1800000000,2000000000):
            h.append(stamp,rgb)
        batch = h.sample(1800000000)
        self.assertEqual(batch["image_timestamps_ns"], [1000000000,1200000000,1400000000,1600000000,1800000000])
        self.assertEqual(batch["image_padding_mask"], [False]*5)
        early = h.sample(1000000000)
        self.assertEqual(early["image_padding_mask"], [True]*4+[False])
        self.assertEqual(Image.open(BytesIO(base64.b64decode(batch["head_color"][0]))).size,(320,320))
        instruction, mask = task_spec(self.settings,"toulaji")
        valid = mask.copy(); valid[54] = False
        request = build_infer_request_from_state(self.settings,"toulaji",instruction,mask,"r","s",1800000000,np.ones(78),batch,state_valid=valid)
        self.assertEqual(len(request["observation"]["images"]["head_color"]),5)
        self.assertEqual(request["observation"]["state"][54],0)
        self.assertFalse(request["observation"]["state_mask"][54])
        self.assertEqual(request["data_contract"],self.settings["data_contract"])

    def test_server_contract_and_direct_wrist_dispatch(self):
        check_info({},self.config,"toulaji",32)
        check_info({"data_contract":self.settings["data_contract"]},self.config,"toulaji",32)
        values = np.asarray(self.sample["arrays"]["source_action_base_wrist_column6d"]["values"]).copy()
        # Sample base-wrist arrays already contain Q: emulate server undoing it.
        Q = self.sample["Q_and_tool_inverse_already_applied_in_source"]["Q_from_restored_wrist"]
        for offset, side in ((0, "left"), (9, "right")):
            for action in values:
                cols = action[offset+3:offset+9].reshape(2,3)
                R = np.column_stack((cols[0],cols[1],np.cross(*cols))) @ np.asarray(Q[side]).T
                action[offset+3:offset+9] = R[:,:2].T.reshape(6)
        class Capture:
            def action_rows(self,actions,masks,seed):
                self.actions = actions; return []
        k=Capture()
        response_rows({"request_id":"r","actions":values.tolist(),"mask":self.sample["arrays"]["action_mask"]["values"]},"r",task_spec(self.settings,"toulaji")[1],32,self.settings,{"joint_names":list(JOINTS),"joint_positions":[0.]*len(JOINTS)},k)
        np.testing.assert_array_equal(k.actions[:,:27],np.asarray(values)[:,:27])

    def test_sample_waist_origin_matches_recorded_feedback(self):
        audit = json.loads((ROOT / "tests/inference/skye_handoff_20261111/waist_audit.json").read_text())
        origin = self.settings["protocol_to_robot"]["waist_height_offset_m"]
        raw = audit["selected_waist_feedback"]["waist"]
        sample = self.sample["arrays"]["state"]["values"][29]
        self.assertAlmostEqual(raw - sample, origin, places=12)
        self.assertAlmostEqual(sample + origin, raw, places=12)

    def test_model_identifier_is_in_infer_request(self):
        settings = dict(self.settings, model="selected-checkpoint")
        instruction, mask = task_spec(settings, "toulaji")
        request = build_infer_request_from_state(settings, "toulaji", instruction,
                    mask, "r", "s", 1, np.zeros(78), (1, "image"))
        self.assertEqual(request["model"], "selected-checkpoint")

    def test_sample_camera_and_q_roundtrip(self):
        a=self.sample["arrays"]
        base=np.asarray(a["source_action_base_wrist_column6d"]["values"])
        camera=np.asarray(a["action"]["values"])
        T=np.asarray(a["T_base_working_camera_c0"]["values"])
        Q=self.sample["Q_and_tool_inverse_already_applied_in_source"]["Q_from_restored_wrist"]
        for offset,side in ((0,"left"),(9,"right")):
            np.testing.assert_allclose(camera[:,offset:offset+3]@T[:3,:3].T+T[:3,3],base[:,offset:offset+3],atol=1e-6)
            for b,c in zip(base,camera):
                rows=c[offset+3:offset+9].reshape(2,3)
                R=np.vstack((rows,np.cross(*rows)))
                aligned=T[:3,:3]@R
                np.testing.assert_allclose(aligned[:,:2].T.reshape(6),b[offset+3:offset+9],atol=2e-6)
                native=aligned@np.asarray(Q[side]).T
                # Native is recovered by undoing Q once, not by relabeling aligned axes.
                np.testing.assert_allclose(native@np.asarray(Q[side]),aligned,atol=2e-6)

if __name__ == "__main__": unittest.main()
