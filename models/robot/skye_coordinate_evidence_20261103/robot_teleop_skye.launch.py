from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from marvin_teleop.launch_config import yaml_bool_launch_default
import os


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('marvin_teleop'),
        'config',
        'skye',
        'robot_param.yaml'
    )

    # ----------------------
    # Launch arguments
    # ----------------------
    use_rviz = LaunchConfiguration('use_rviz')
    glove_mode = LaunchConfiguration('glove_mode')
    teleop_mode = LaunchConfiguration('teleop_mode')
    use_incremental_hand_targets = LaunchConfiguration('use_incremental_hand_targets')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    declare_glove_mode = DeclareLaunchArgument(
        'glove_mode',
        default_value='false',
        description='Dexhand/glove teleop: controller_udp takes its enable '
                    'signal from control/footkey (published by apex_backend).'
    )

    declare_teleop_mode = DeclareLaunchArgument(
        'teleop_mode',
        default_value='0',
        choices=['0', '1', '2', '3'],
        description='Motion scope: 0=full_body, 1=arms_only (body+head frozen), 2=arms_head (body frozen), 3=arms_body (head frozen). Runtime switching uses control/set_teleop_motion_mode.'
    )

    declare_use_incremental_hand_targets = DeclareLaunchArgument(
        'use_incremental_hand_targets',
        default_value=yaml_bool_launch_default(
            config, 'use_incremental_hand_targets'),
        choices=['true', 'false'],
        description='Use devel-style hand clutch increments; false uses legacy absolute targets.'
    )

    ros_namespace = LaunchConfiguration('apex_ros_namespace')
    declare_ros_namespace = DeclareLaunchArgument(
        'apex_ros_namespace',
        default_value=os.environ.get('APEX_ROS_NAMESPACE', 'tj').strip().strip('/'),
        description='ROS namespace for the Apex robot graph'
    )

    # ----------------------
    # Config file
    # ----------------------
    qp_config = os.path.join(
        get_package_share_directory('marvin_qp_controller'),
        'config',
        'skye',
        'robot_param.yaml'
    )
    control_config = os.path.join(
        get_package_share_directory('marvin_ros_control'),
        'config',
        'robot_param_skye.yaml'
    )

    # ----------------------
    # Nodes
    # ----------------------
    motion_node = Node(
        package='marvin_teleop',
        executable='teleop_manager_v2',
        name='motion_node',
        parameters=[
            config,
            {"config_file": 'ccs_m6.MvKDCfg'},
            {"urdf_file": 'marvin_CCS_m6.urdf'},
            {"srdf_file": 'marvin_robot.srdf'},
            {
                'ik_request_topic': 'control/ik_request/vr',
                'ik_result_topic': 'control/ik_result',
                'respect_ik_request_mux': False,
                'joint_cmd_mux_service': 'control/ik_request_mux/select',
                'teleop_joint_cmd_source': 0,
                'idle_joint_cmd_source': 0,
                'require_vr_connected_for_teleop': True,
                'vr_connected_topic': 'info/vr_connected',
                'teleop_mode': ParameterValue(teleop_mode, value_type=int),
                'use_incremental_hand_targets': ParameterValue(
                    use_incremental_hand_targets, value_type=bool),
            },
        ],
        output='screen',
    )

    ik_request_mux = Node(
        package='marvin_qp_controller',
        executable='ik_request_mux',
        name='ik_request_mux',
        parameters=[{
            'source_names': ['idle', 'teleop', 'vla', 'replay'],
            'source_topics': ['', 'control/ik_request/vr', 'control/ik_request/vla',
                              'control/ik_request/replay'],
            'output_topic': 'control/ik_request',
            'active_source': 1,
            'publish_disable_on_switch': True,
            'publish_cached_on_switch': False,
            'clear_target_cache_on_switch': True,
        }],
        output='screen',
    )

    qp_controller_overrides = {
        'enable_viewer': False,
        'ik_request_topic': 'control/ik_request',
        'teleop_ik_source_index': 1,
        # QP feeds joint_cmd_mux source 0 instead of the robot directly; the
        # wrapper's sync defaults (True) re-anchor QP from the mux seed when
        # switching back to teleop and from latest_joint_cmd on QP restart.
        'joint_cmd_output_prefix': 'control/qp_controller',
        'publish_ik_result': False,
        'joint_cmd_output_ramp_duration': 2.0,
        'teleop_full_body_transition_duration': 0.35,
        'require_robot_movable_for_ik_request': True,
        'require_vr_connected_for_teleop': True,
        'vr_connected_topic': 'info/vr_connected',
        'use_incremental_hand_targets': ParameterValue(
            use_incremental_hand_targets, value_type=bool),
    }

    qp_controller_cpp = Node(
        package='marvin_qp_controller',
        executable='qp_controller',
        name='qp_controller',
        parameters=[qp_config, qp_controller_overrides, {'qp_solver': 'daqp'}],
        output='screen',
    )

    # Joint-command mux between the QP controller and the robot. devel-value
    # mapping (control/set_input / control/input_mode): 0=idle 1=teleop
    # 2=planner 3=user 4=replay = gento index + 1; native interface stays at
    # control/joint_cmd_mux/select (-1..3). Boots on teleop so the pipeline is
    # transparent. planner/user are reserved slots (gento-style topics).
    joint_cmd_mux = Node(
        package='marvin_qp_controller',
        executable='joint_cmd_mux',
        name='joint_cmd_mux',
        parameters=[{
            'source_names': ['teleop', 'planner', 'user', 'replay'],
            'source_prefixes': ['control/qp_controller', 'control/plan',
                                'control/user', 'control/replay'],
            'output_prefix': 'control',
            'active_source': 0,
            'transition_duration': 2.0,
            'latest_joint_cmd_rate_hz': 50.0,
            'compat_set_input_service': 'control/set_input',
            'compat_input_mode_topic': 'control/input_mode',
        }],
        output='screen',
    )

    vr_reader = Node(
        package='marvin_teleop',
        executable='controller_udp',
        name='VR_reader',
        parameters=[config, control_config, {
            "glove_mode": ParameterValue(glove_mode, value_type=bool),
            "enable_tcp": True,
            "teleop_mux_source": 1,
        }],
        output='screen',
    )

    # base_link -> tracking_base_link; this must exactly match Gento_Sky.xml.
    tracking_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tracking_base_tf',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'tracking_base_link'],
        output='screen',
    )

    # planner_node = Node(
    #     package='marvin_teleop',
    #     executable='planner_node',
    #     name='planner_node',
    #     parameters=[config],
    #     arguments=['--ros-args', '--log-level', 'INFO']
    # )

    # ----------------------
    # Include ros_control bringup
    # ----------------------
    bringup_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('marvin_ros_control'),
                'launch',
                'bringup_control_gento_skye.launch.py'
            )
        ),
        launch_arguments={
            'use_rviz': use_rviz
        }.items()
    )

    return LaunchDescription([
        declare_use_rviz,
        declare_glove_mode,
        declare_teleop_mode,
        declare_use_incremental_hand_targets,
        declare_ros_namespace,
        GroupAction(actions=[
            PushRosNamespace(ros_namespace),
            motion_node,
            ik_request_mux,
            qp_controller_cpp,
            joint_cmd_mux,
            vr_reader,
            tracking_base_tf,
            # planner_node,
            # bringup_control,
        ]),
    ])
