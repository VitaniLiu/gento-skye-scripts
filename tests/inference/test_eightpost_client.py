import base64
from io import BytesIO
import json
from pathlib import Path
import sys
import unittest
import numpy as np
from PIL import Image
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/inference/host"))
from main import load_configuration, task_spec, build_infer_request_from_state, check_info, request_task
from protocol import native_request, encode_array, decode_array, raise_server_error

class EightpostTests(unittest.TestCase):
    def test_all_profiles_and_wire_observations(self):
        stream = BytesIO()
        Image.new("RGB", (320,320), (3,4,5)).save(stream, format="PNG")
        image = base64.b64encode(stream.getvalue()).decode()
        names = ["pi05","alpha","fastwam","groot","ours_direct","ours_pre","ours_a0","ours_a1"]
        for i, name in enumerate(names):
            config = load_configuration(ROOT / "configs/inference/default.json", name)
            settings = config["inference"]
            self.assertEqual(config["server"]["server_port"], 19300+i)
            count = 5 if i >= 4 else 1
            self.assertEqual(len(settings["image_history_offsets_s"]), count)
            for task in ["gouwu","toulaji"]:
                self.assertEqual(request_task(settings, task), task)
                instruction, mask = task_spec(settings, task)
                check_info({"tasks":{task:{"prompt":instruction}}},config,task,instruction,32)
                batch = dict(head_color=[image]*count, image_padding_mask=[False]*count)
                state = np.arange(78,dtype=np.float32)
                request = build_infer_request_from_state(settings,task,instruction,mask,"r","s",100,state,batch)
                wire = native_request(request)
                self.assertEqual(wire["task"],task)
                self.assertEqual(wire["state_frame"],"source")
                self.assertEqual(wire["sample_ns"],100)
                self.assertNotIn("instruction",wire)
                obs = wire["observation"]
                self.assertEqual(obs["head_rgb"].shape,(320,320,3))
                np.testing.assert_array_equal(obs["state"],request["observation"]["state"])
                if count == 5:
                    self.assertEqual(obs["head_rgb_history"].shape,(5,320,320,3))
                    self.assertEqual(obs["history_offsets"],[-16,-12,-8,-4,0])
                else:
                    self.assertNotIn("head_rgb_history",obs)
                decoded = json.loads(json.dumps(wire,default=encode_array),object_hook=decode_array)
                np.testing.assert_array_equal(decoded["observation"]["head_rgb"],obs["head_rgb"])
                actions = np.zeros((32,78),np.float32)
                reply = json.loads(json.dumps(dict(actions=actions),default=encode_array),object_hook=decode_array)
                np.testing.assert_array_equal(reply["actions"],actions)
    def test_errors_and_handshake_stop(self):
        config = load_configuration(ROOT / "configs/inference/default.json","ours_a1")
        for error in ["failed", {"code":"bad_task","message":"unknown task"}]:
            with self.assertRaises(RuntimeError): raise_server_error({"error":error})
            with self.assertRaises(RuntimeError): check_info({"error":error},config,"toulaji","",32)
        self.assertEqual(native_request({"command":"info"}),{"cmd":"info"})
        reset = native_request({"command":"reset"})
        self.assertEqual(reset["cmd"],"reset")
        self.assertTrue(reset["request_id"])
