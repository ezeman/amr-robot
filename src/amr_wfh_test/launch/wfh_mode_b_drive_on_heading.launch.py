from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    default_params = PathJoinSubstitution([
        FindPackageShare('amr_wfh_test'),
        'config',
        'wfh_test.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Parameter file for the DriveOnHeading WFH client.'
        ),
        Node(
            package='amr_wfh_test',
            executable='wfh_drive_on_heading_client',
            name='wfh_drive_on_heading_client',
            output='screen',
            parameters=[params_file],
        ),
    ])
