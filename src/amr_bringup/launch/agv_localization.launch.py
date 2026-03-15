import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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

    lidar_topic_name = PythonExpression(
        ["'scan_raw' if '", use_scan_filter, "' == 'true' else 'scan'"]
    )

    default_params = PathJoinSubstitution([
        FindPackageShare('amr_navigation'),
        'config',
        'nav2_localization.yaml',
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

    hardware_launch = PathJoinSubstitution([
        FindPackageShare('amr_hardware_bringup'),
        'launch',
        'hardware.launch.py',
    ])

    rsp_launch = PathJoinSubstitution([
        FindPackageShare('amr_description'),
        'launch',
        'robot_state_publisher.launch.py',
    ])

    localization_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'localization_launch.py'
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

    def _include_localization(context, *args, **kwargs):
        expanded_map = _expand_user_path(context, map_yaml)
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localization_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': expanded_map,
                    'params_file': params_file,
                    'autostart': 'true',
                    'use_composition': 'False',
                    'use_respawn': 'False',
                }.items(),
            )
        ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value='', description='Full path to map YAML file'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_scan_filter', default_value='true'),
        DeclareLaunchArgument('scan_filter_params', default_value=default_scan_filter_params),
        DeclareLaunchArgument('use_topic_health_monitor', default_value='true'),
        DeclareLaunchArgument('topic_health_monitor_params', default_value=default_topic_health_monitor_params),
        DeclareLaunchArgument('use_topic_health_fail_hard', default_value='false'),
        DeclareLaunchArgument('topic_health_fail_grace_sec', default_value='8.0'),
        DeclareLaunchArgument('base_invert_right_motor', default_value='true'),
        DeclareLaunchArgument('base_angular_sign', default_value='1.0'),

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
            name='scan_filter_localization',
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
        OpaqueFunction(function=_include_localization),
    ])
