from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'),
        DeclareLaunchArgument(
            'model',
            default_value=PathJoinSubstitution([
                FindPackageShare('amr_description'),
                'urdf',
                'agv_base.urdf.xacro',
            ]),
            description='Path to URDF/Xacro file'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command(['xacro', ' ', model]),
            }],
        ),
    ])
