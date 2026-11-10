"""Prepared replacement tool launch; no processes start on import."""
from pathlib import Path
import importlib.util
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

ROOT = Path(__file__).resolve().parents[2]
_launch_lock = None

def _start_components(context):
    global _launch_lock
    spec = importlib.util.spec_from_file_location('_gento_gripper_control', ROOT/'src/teleop/control.py')
    layer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(layer)
    _launch_lock = layer.claim_gripper_launch(ROOT/'logs/teleop/gripper-tool.lock')
    try:
        config = str(Path(get_package_share_directory('apex_tool'))/'config/dm_gripper.yaml')
        driver = Node(package='dm_gripper_py', executable='dm_motor_test',
            name='dm_gripper_motor_node', namespace='tj', output='screen',
            parameters=[config],
            remappings=[
                ('/tj/control/gripperValueL','/tj/control/gripper_slow_targetL'),
                ('/tj/control/gripperValueR','/tj/control/gripper_slow_targetR')])
        code = ('import sys;sys.path.insert(0,'+repr(str(ROOT/'src/teleop'))+');'
                'from control import gripper_slow_main;'
                'gripper_slow_main('+repr(str(ROOT/'configs/teleop/gripper-slow.json'))+')')
        adapter = ExecuteProcess(cmd=['/usr/bin/python3','-u','-B','-c',code],output='screen')
        handlers = [
            RegisterEventHandler(OnProcessExit(target_action=child,
                on_exit=[EmitEvent(event=Shutdown(reason='gripper component exited'))]))
            for child in (driver,adapter)]
        return handlers+[adapter,driver]
    except BaseException:
        _launch_lock.close()
        _launch_lock = None
        raise

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_start_components)])
