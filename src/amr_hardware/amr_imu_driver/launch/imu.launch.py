from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('amr_imu_driver')
    default_params = PathJoinSubstitution([package_share, 'config', 'imu_params.yaml'])

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/imu',
        description='Serial device path for the IMU.')

    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='9600',
        description='Serial baud rate for the IMU connection.')

    frame_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='Frame ID to use for published IMU messages.')

    topic_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='imu/data',
        description='Topic name for the published IMU data.')

    configure_arg = DeclareLaunchArgument(
        'configure_device',
        default_value='false',
        description='Send FF AA configuration commands to the CMP10A.')

    rate_arg = DeclareLaunchArgument(
        'output_rate_hz',
        default_value='100.0',
        description='Expected output frequency of the IMU (affects timer).')

    quat_arg = DeclareLaunchArgument(
        'use_quaternion_if_available',
        default_value='true',
        description='Publish raw quaternion when provided by the CMP10A.')

    log_first_message_arg = DeclareLaunchArgument(
        'log_first_message',
        default_value='true',
        description='Log the first IMU sample received for debugging purposes.')

    imu_node = Node(
        package='amr_imu_driver',
        executable='cmp10a_driver',
        name='cmp10a_driver',
        output='screen',
        parameters=[
            default_params,
            {
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
                'frame_id': LaunchConfiguration('frame_id'),
                'topic_name': LaunchConfiguration('topic_name'),
                'configure_device': LaunchConfiguration('configure_device'),
                'output_rate_hz': LaunchConfiguration('output_rate_hz'),
                'use_quaternion_if_available': LaunchConfiguration('use_quaternion_if_available'),
                'log_first_message': LaunchConfiguration('log_first_message'),
            },
        ],
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        frame_arg,
        topic_arg,
        configure_arg,
        rate_arg,
        quat_arg,
        log_first_message_arg,
        imu_node,
    ])
