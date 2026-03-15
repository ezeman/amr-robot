from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _has_package(name: str) -> bool:
    try:
        get_package_share_directory(name)
        return True
    except PackageNotFoundError:
        return False


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_velocity_smoother = LaunchConfiguration('use_velocity_smoother')

    default_params = PathJoinSubstitution([
        FindPackageShare('amr_wfh_test'),
        'config',
        'wfh_test.yaml',
    ])

    actions = [
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Parameter file for WFH odom runner (and velocity smoother if present).'
        ),
        DeclareLaunchArgument(
            'use_velocity_smoother',
            default_value='false',
            description='Start nav2_velocity_smoother if available.'
        ),
    ]

    # Mode A runner (odom-based)
    actions.append(
        Node(
            package='amr_wfh_test',
            executable='wfh_odom_runner',
            name='wfh_odom_runner',
            output='screen',
            parameters=[params_file],
        )
    )

    # Optional velocity smoother
    if _has_package('nav2_velocity_smoother'):
        actions.append(
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[params_file],
                condition=IfCondition(use_velocity_smoother),
            )
        )
    else:
        actions.append(LogInfo(msg='nav2_velocity_smoother not installed; skipping velocity smoothing.'))

    ld = LaunchDescription(actions)
    return ld
