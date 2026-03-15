from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_velocity_smoother = LaunchConfiguration('use_velocity_smoother')

    default_params = PathJoinSubstitution([
        FindPackageShare('amr_wfh_test'),
        'config',
        'wfh_test.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Base parameter file; this launch overrides WFH params for 1m fwd/back x10 rounds.'
        ),
        DeclareLaunchArgument(
            'use_velocity_smoother',
            default_value='false',
            description='Start nav2_velocity_smoother (requires lifecycle activation).'
        ),
        LogInfo(msg='WFH Mode A: forward 1m then backward 1m, 10 rounds (20 legs total). Ensure a clear path.'),
        Node(
            package='amr_wfh_test',
            executable='wfh_odom_runner',
            name='wfh_odom_runner',
            output='screen',
            parameters=[
                params_file,
                {
                    'include_backward': True,
                    'rounds': 10,
                    'forward_distance_m': 1.0,
                    'backward_distance_m': 1.0,
                    'linear_x': 0.20,
                    'backward_linear_x': -0.20,
                    'max_duration_sec': 15.0,
                    'max_total_duration_sec': 240.0,
                    'settle_time_sec': 0.5,
                },
            ],
        ),
        Node(
            condition=IfCondition(use_velocity_smoother),
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[params_file],
        ),
    ])
