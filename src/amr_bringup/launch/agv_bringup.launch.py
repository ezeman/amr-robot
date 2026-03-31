"""Bring Up launch: start all hardware sensors and camera WITHOUT SLAM.

Identical to agv_mapping.launch.py but omits slam_toolbox so the robot
is available for manual driving, diagnostics, or calibration without
building a map.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_product = LaunchConfiguration('lidar_product')
    lidar_baud = LaunchConfiguration('lidar_baud')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_frame = LaunchConfiguration('lidar_frame')
    use_scan_filter = LaunchConfiguration('use_scan_filter')
    scan_filter_params = LaunchConfiguration('scan_filter_params')
    use_topic_health_monitor = LaunchConfiguration('use_topic_health_monitor')
    topic_health_monitor_params = LaunchConfiguration('topic_health_monitor_params')
    use_topic_health_fail_hard = LaunchConfiguration('use_topic_health_fail_hard')
    topic_health_fail_grace_sec = LaunchConfiguration('topic_health_fail_grace_sec')
    lidar_enable_angle_crop = LaunchConfiguration('lidar_enable_angle_crop')
    lidar_angle_crop_min = LaunchConfiguration('lidar_angle_crop_min')
    lidar_angle_crop_max = LaunchConfiguration('lidar_angle_crop_max')
    base_invert_right_motor = LaunchConfiguration('base_invert_right_motor')
    base_angular_sign = LaunchConfiguration('base_angular_sign')
    use_camera = LaunchConfiguration('use_camera')
    camera_model = LaunchConfiguration('camera_model')

    lidar_topic_name = PythonExpression(
        ["'scan_raw' if '", use_scan_filter, "' == 'true' else 'scan'"]
    )

    default_scan_filter_params = PathJoinSubstitution([
        FindPackageShare('amr_bringup'),
        'config',
        'scan_filter_mapping.yaml',
    ])

    default_topic_health_monitor_params = PathJoinSubstitution([
        FindPackageShare('amr_tools'),
        'config',
        'topic_health_monitor.yaml',
    ])

    depthai_prefix = get_package_share_directory('depthai_ros_driver')
    camera_launch = os.path.join(depthai_prefix, 'launch', 'camera.launch.py')
    camera_params_file = os.path.join(depthai_prefix, 'config', 'camera.yaml')

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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_product', default_value='LDLiDAR_STL27L'),
        DeclareLaunchArgument('lidar_baud', default_value='921600'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/lidar'),
        DeclareLaunchArgument('lidar_frame', default_value='lidar_link'),
        DeclareLaunchArgument('use_scan_filter', default_value='true'),
        DeclareLaunchArgument('scan_filter_params', default_value=default_scan_filter_params),
        DeclareLaunchArgument('use_topic_health_monitor', default_value='true'),
        DeclareLaunchArgument('topic_health_monitor_params', default_value=default_topic_health_monitor_params),
        DeclareLaunchArgument('use_topic_health_fail_hard', default_value='false'),
        DeclareLaunchArgument('topic_health_fail_grace_sec', default_value='8.0'),
        DeclareLaunchArgument('lidar_enable_angle_crop', default_value='false'),
        DeclareLaunchArgument('lidar_angle_crop_min', default_value='90.0'),
        DeclareLaunchArgument('lidar_angle_crop_max', default_value='270.0'),
        DeclareLaunchArgument('base_invert_right_motor', default_value='true'),
        DeclareLaunchArgument('base_angular_sign', default_value='1.0'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('camera_model', default_value='OAK-D-LITE'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_launch),
            launch_arguments={
                'use_lidar': 'true',
                'use_imu': 'true',
                'use_base': 'true',
                'lidar_product': lidar_product,
                'lidar_baud': lidar_baud,
                'lidar_port': lidar_port,
                'lidar_frame': lidar_frame,
                'lidar_topic': lidar_topic_name,
                'lidar_enable_angle_crop': lidar_enable_angle_crop,
                'lidar_angle_crop_min': lidar_angle_crop_min,
                'lidar_angle_crop_max': lidar_angle_crop_max,
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
            name='scan_filter_mapping',
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

        # NOTE: No slam_toolbox here — this is Bring Up mode (sensors only).

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            condition=IfCondition(use_camera),
            launch_arguments={
                'name': 'oak',
                'camera_model': camera_model,
                'parent_frame': 'base_link',
                'cam_pos_x': '0.15',
                'cam_pos_y': '0.0',
                'cam_pos_z': '0.25',
                'cam_pitch': '0.0',
                'use_rviz': 'false',
                'rectify_rgb': 'true',
                'params_file': camera_params_file,
            }.items(),
        ),
    ])
