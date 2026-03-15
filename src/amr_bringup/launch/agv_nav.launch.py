import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _expand_user_path(context, path_subst: LaunchConfiguration) -> str:
    raw = path_subst.perform(context)
    return os.path.abspath(os.path.expanduser(raw))


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_scan_filter = LaunchConfiguration('use_scan_filter')
    scan_filter_params = LaunchConfiguration('scan_filter_params')
    use_topic_health_monitor = LaunchConfiguration('use_topic_health_monitor')
    topic_health_monitor_params = LaunchConfiguration('topic_health_monitor_params')
    use_topic_health_fail_hard = LaunchConfiguration('use_topic_health_fail_hard')
    topic_health_fail_grace_sec = LaunchConfiguration('topic_health_fail_grace_sec')
    base_invert_right_motor = LaunchConfiguration('base_invert_right_motor')
    base_angular_sign = LaunchConfiguration('base_angular_sign')
    use_yield_requester = LaunchConfiguration('use_yield_requester')
    yield_distance_m = LaunchConfiguration('yield_distance_m')
    yield_sound_mode = LaunchConfiguration('yield_sound_mode')
    yield_message = LaunchConfiguration('yield_message')

    lidar_topic_name = PythonExpression(
        ["'scan_raw' if '", use_scan_filter, "' == 'true' else 'scan'"]
    )

    default_params = PathJoinSubstitution([
        FindPackageShare('amr_navigation'),
        'config',
        'nav2_agv_default.yaml',
    ])

    hardware_launch = PathJoinSubstitution([
        FindPackageShare('amr_hardware_bringup'),
        'launch',
        'hardware.launch.py',
    ])

    default_scan_filter_params = PathJoinSubstitution([
        FindPackageShare('amr_bringup'),
        'config',
        'scan_filter.yaml',
    ])

    default_topic_health_monitor_params = PathJoinSubstitution([
        FindPackageShare('amr_tools'),
        'config',
        'topic_health_monitor.yaml',
    ])

    rsp_launch = PathJoinSubstitution([
        FindPackageShare('amr_description'),
        'launch',
        'robot_state_publisher.launch.py',
    ])

    nav2_bringup_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )

    topic_health_monitor_node = Node(
        condition=IfCondition(use_topic_health_monitor),
        package='amr_tools',
        executable='topic_health_monitor',
        name='topic_health_monitor',
        output='screen',
        parameters=[
            topic_health_monitor_params,
            {
                'fail_hard': ParameterValue(use_topic_health_fail_hard, value_type=bool),
                'fail_grace_sec': ParameterValue(topic_health_fail_grace_sec, value_type=float),
            },
        ],
    )

    def _include_nav2(context, *args, **kwargs):
        expanded_map = _expand_user_path(context, map_yaml)
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_bringup_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': expanded_map,
                    'params_file': params_file,
                    'autostart': 'true',
                    'use_composition': 'True',
                    'use_respawn': 'False',
                }.items(),
            )
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to the map YAML file'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the Nav2 parameter file'),
        DeclareLaunchArgument(
            'use_scan_filter',
            default_value='true',
            description='Enable laser_filters scan filter chain (scan_raw -> scan).'),
        DeclareLaunchArgument(
            'scan_filter_params',
            default_value=default_scan_filter_params,
            description='scan_filter YAML file (laser_filters).'),
        DeclareLaunchArgument(
            'use_topic_health_monitor',
            default_value='true',
            description='Enable topic health monitor for scan/imu/odom.'),
        DeclareLaunchArgument(
            'topic_health_monitor_params',
            default_value=default_topic_health_monitor_params,
            description='YAML parameters for amr_tools topic_health_monitor node.'),
        DeclareLaunchArgument(
            'use_topic_health_fail_hard',
            default_value='false',
            description='If true, monitor exits on prolonged unhealthy state and launch shuts down.'),
        DeclareLaunchArgument(
            'topic_health_fail_grace_sec',
            default_value='8.0',
            description='Grace period before fail-hard shutdown (seconds).'),
        DeclareLaunchArgument(
            'base_invert_right_motor',
            default_value='true',
            description='Invert right motor direction (keep true unless wiring differs).'),
        DeclareLaunchArgument(
            'base_angular_sign',
            default_value='1.0',
            description='Multiply cmd_vel angular.z and odom yaw rate by this sign (+1.0 normal, -1.0 if turning is inverted).'),
        DeclareLaunchArgument(
            'use_yield_requester',
            default_value='true',
            description='Play a sound to request passage when an obstacle blocks within a threshold.',
        ),
        DeclareLaunchArgument(
            'yield_distance_m',
            default_value='0.60',
            description='Yield requester trigger distance in meters.',
        ),
        DeclareLaunchArgument(
            'yield_sound_mode',
            default_value='beep',
            description="Yield requester sound mode: 'beep' (aplay), 'espeak', or 'none'.",
        ),
        DeclareLaunchArgument(
            'yield_message',
            default_value='Excuse me, please clear the way',
            description='Message for yield requester when sound_mode=espeak.',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_launch),
            launch_arguments={
                'use_lidar': 'true',
                'use_imu': 'true',
                'use_base': 'true',
                'lidar_topic': lidar_topic_name,
                'base_invert_right_motor': base_invert_right_motor,
                'base_angular_sign': base_angular_sign,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rsp_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        Node(
            condition=IfCondition(use_scan_filter),
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_filter_nav',
            output='screen',
            parameters=[scan_filter_params],
            remappings=[
                ('scan', 'scan_raw'),
                ('scan_filtered', 'scan'),
            ],
        ),
        topic_health_monitor_node,
        RegisterEventHandler(
            condition=IfCondition(use_topic_health_fail_hard),
            event_handler=OnProcessExit(
                target_action=topic_health_monitor_node,
                on_exit=[EmitEvent(event=Shutdown(reason='topic_health_monitor exited (fail-hard)'))],
            ),
        ),
        Node(
            condition=IfCondition(use_yield_requester),
            package='amr_tools',
            executable='yield_requester',
            name='yield_requester',
            output='screen',
            arguments=[
                '--scan-topic', '/scan',
                '--min-distance-m', yield_distance_m,
                '--sound-mode', yield_sound_mode,
                '--message', yield_message,
            ],
        ),
        OpaqueFunction(function=_include_nav2),
    ])
