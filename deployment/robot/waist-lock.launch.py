"""Scoped remap: mux body output -> filter -> original robot body input."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction,IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

def generate_launch_description():
    root=Path(__file__).resolve().parents[2]
    native=IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('apex_backend'),'launch','teleop_replay.launch.py')),
        launch_arguments={'robot_platform':'gento','robot_model':'skye',
                          'glove_mode':os.environ.get('APEX_GLOVE_MODE','false'),
                          'use_rviz':'false'}.items())
    lock=ExecuteProcess(cmd=['/usr/bin/python3','-B',str(root/'src/teleop/waist_lock.py'),'--execute'],output='screen')
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(target_action=lock,on_exit=[EmitEvent(event=Shutdown(reason='waist filter exited'))])),
        GroupAction(actions=[SetRemap(src='/tj/control/joint_cmd_body',dst='/tj/control/waist_unlocked/joint_cmd_body'),native]),lock])
