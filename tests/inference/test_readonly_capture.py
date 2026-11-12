import ast
from pathlib import Path
import sys
import unittest
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/inference/host'))
from capture_simulation import ReadOnlyRobotClient

class ReadOnlyCaptureTests(unittest.TestCase):
    def test_endpoint_has_no_publishers_or_service_clients(self):
        tree=ast.parse((ROOT/'src/inference/robot/observe_only.py').read_text())
        attrs=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
        self.assertNotIn('create_publisher',attrs)
        self.assertNotIn('create_client',attrs)
        self.assertIn('create_subscription',attrs)
    def test_motion_commands_are_rejected_before_io(self):
        client=object.__new__(ReadOnlyRobotClient)
        for command in ['execute','reset','cancel','set_mode','parking_release']:
            with self.assertRaises(ValueError):client.send(command)
    def test_production_endpoint_default_is_unchanged(self):
        self.assertEqual(ReadOnlyRobotClient.endpoint,'observe_only.py')
        from main import RobotClient
        self.assertFalse(hasattr(RobotClient,'endpoint'))
