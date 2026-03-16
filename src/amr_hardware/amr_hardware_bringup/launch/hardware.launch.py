from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_lidar = LaunchConfiguration('use_lidar')
    use_imu = LaunchConfiguration('use_imu')
    use_base = LaunchConfiguration('use_base')
    use_battery = LaunchConfiguration('use_battery')

    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baud = LaunchConfiguration('lidar_baud')
    lidar_frame = LaunchConfiguration('lidar_frame')
    lidar_product = LaunchConfiguration('lidar_product')
    lidar_topic = LaunchConfiguration('lidar_topic')
    lidar_enable_angle_crop = LaunchConfiguration('lidar_enable_angle_crop')
    lidar_angle_crop_min = LaunchConfiguration('lidar_angle_crop_min')
    lidar_angle_crop_max = LaunchConfiguration('lidar_angle_crop_max')

    imu_port = LaunchConfiguration('imu_port')
    imu_baud = LaunchConfiguration('imu_baud')
    imu_frame = LaunchConfiguration('imu_frame')
    imu_topic = LaunchConfiguration('imu_topic')

    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    publish_base_tf = LaunchConfiguration('publish_base_tf')
    base_invert_right_motor = LaunchConfiguration('base_invert_right_motor')
    base_angular_sign = LaunchConfiguration('base_angular_sign')

    battery_i2c_bus = LaunchConfiguration('battery_i2c_bus')
    battery_i2c_address = LaunchConfiguration('battery_i2c_address')
    battery_publish_rate_hz = LaunchConfiguration('battery_publish_rate_hz')
    battery_voltage_min = LaunchConfiguration('battery_voltage_min')
    battery_voltage_max = LaunchConfiguration('battery_voltage_max')
    battery_voltage_scale = LaunchConfiguration('battery_voltage_scale')
    battery_percentage_mode = LaunchConfiguration('battery_percentage_mode')
    battery_i2c_retry_sec = LaunchConfiguration('battery_i2c_retry_sec')
    battery_topic = LaunchConfiguration('battery_topic')

    return LaunchDescription([
        DeclareLaunchArgument('use_lidar', default_value='true', description='Start lidar driver'),
        DeclareLaunchArgument('use_imu', default_value='true', description='Start IMU driver'),
        DeclareLaunchArgument('use_base', default_value='true', description='Start base/motor driver'),
        DeclareLaunchArgument('use_battery', default_value='true', description='Start battery I2C monitor'),

        DeclareLaunchArgument('lidar_port', default_value='/dev/lidar', description='Serial port for lidar'),
        DeclareLaunchArgument('lidar_baud', default_value='921600', description='Baudrate for lidar'),
        DeclareLaunchArgument('lidar_frame', default_value='lidar_link', description='Frame id for lidar scans'),
        DeclareLaunchArgument('lidar_product', default_value='LDLiDAR_STL27L', description='LD LiDAR product name parameter'),
        DeclareLaunchArgument('lidar_topic', default_value='scan', description='LaserScan topic name published by lidar driver'),
        DeclareLaunchArgument('lidar_enable_angle_crop', default_value='false', description='Enable angle crop in lidar driver'),
        DeclareLaunchArgument('lidar_angle_crop_min', default_value='90.0', description='Angle crop min (deg)'),
        DeclareLaunchArgument('lidar_angle_crop_max', default_value='270.0', description='Angle crop max (deg)'),

        DeclareLaunchArgument('imu_port', default_value='/dev/imu', description='Serial port for IMU'),
        DeclareLaunchArgument('imu_baud', default_value='115200', description='Baudrate for IMU'),
        DeclareLaunchArgument('imu_frame', default_value='imu_link', description='Frame id for IMU'),
        DeclareLaunchArgument('imu_topic', default_value='imu/data', description='IMU topic name'),

        DeclareLaunchArgument('odom_frame', default_value='odom', description='Odom frame id'),
        # Use base_footprint as the odom child to avoid TF having 2 parents for base_link
        # (base_footprint->base_link comes from robot_state_publisher).
        DeclareLaunchArgument('base_frame', default_value='base_footprint', description='Base frame id (odom child)'),
        DeclareLaunchArgument('publish_base_tf', default_value='true', description='Publish odom->base_link TF from base driver'),
        DeclareLaunchArgument(
            'base_invert_right_motor',
            default_value='true',
            description='Invert right motor direction (keep true unless wiring differs).',
        ),
        DeclareLaunchArgument(
            'base_angular_sign',
            default_value='1.0',
            description='Multiply cmd_vel angular.z and odom yaw rate by this sign (+1.0 normal, -1.0 if turning is inverted).',
        ),

        DeclareLaunchArgument('battery_i2c_bus', default_value='7', description='I2C bus number for battery module'),
        DeclareLaunchArgument('battery_i2c_address', default_value='64', description='I2C address for battery module (decimal, 64 = 0x40)'),
        DeclareLaunchArgument('battery_publish_rate_hz', default_value='2.0', description='Battery publish rate in Hz'),
        DeclareLaunchArgument('battery_voltage_min', default_value='22.4', description='Voltage mapped to 0% battery (LiFePO4 8S)'),
        DeclareLaunchArgument('battery_voltage_max', default_value='29.2', description='Voltage mapped to 100% battery (LiFePO4 8S)'),
        DeclareLaunchArgument('battery_voltage_scale', default_value='2.547', description='Scale factor for voltage reading (calibrated)'),
        DeclareLaunchArgument('battery_percentage_mode', default_value='lifepo4_8s', description='SOC mode: lifepo4_8s or linear'),
        DeclareLaunchArgument('battery_i2c_retry_sec', default_value='5.0', description='Retry interval when I2C device is busy/unavailable'),
        DeclareLaunchArgument('battery_topic', default_value='/battery_state', description='BatteryState topic name'),

        Node(
            condition=IfCondition(use_lidar),
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='ldlidar',
            output='screen',
            parameters=[{
                'product_name': lidar_product,
                'topic_name': lidar_topic,
                'frame_id': lidar_frame,
                'port_name': lidar_port,
                'port_baudrate': ParameterValue(lidar_baud, value_type=int),
                'laser_scan_dir': True,
                'enable_angle_crop_func': ParameterValue(lidar_enable_angle_crop, value_type=bool),
                'angle_crop_min': ParameterValue(lidar_angle_crop_min, value_type=float),
                'angle_crop_max': ParameterValue(lidar_angle_crop_max, value_type=float),
            }],
        ),

        Node(
            condition=IfCondition(use_imu),
            package='amr_imu_driver',
            executable='cmp10a_driver',
            name='imu_driver',
            output='screen',
            parameters=[{
                'port': imu_port,
                'baud': ParameterValue(imu_baud, value_type=int),
                'frame_id': imu_frame,
                'topic_name': imu_topic,
                'log_first_message': True,
            }],
        ),

        Node(
            condition=IfCondition(use_base),
            package='zlac8015d_ros2_driver',
            executable='zlac8015d_node',
            name='base_controller',
            output='screen',
            parameters=[{
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'publish_tf': ParameterValue(publish_base_tf, value_type=bool),
                'odom_topic': 'odom',
                'wheel_radius': 0.1,
                'base_width': 0.4,
                'invert_right_motor': ParameterValue(base_invert_right_motor, value_type=bool),
                'angular_sign': ParameterValue(base_angular_sign, value_type=float),
            }],
        ),

        Node(
            condition=IfCondition(use_battery),
            package='amr_battery_driver',
            executable='battery_node',
            name='battery_node',
            output='screen',
            parameters=[{
                'i2c_bus': ParameterValue(battery_i2c_bus, value_type=int),
                'i2c_address': ParameterValue(battery_i2c_address, value_type=int),
                'publish_rate_hz': ParameterValue(battery_publish_rate_hz, value_type=float),
                'battery_voltage_min': ParameterValue(battery_voltage_min, value_type=float),
                'battery_voltage_max': ParameterValue(battery_voltage_max, value_type=float),
                'voltage_scale': ParameterValue(battery_voltage_scale, value_type=float),
                'percentage_mode': battery_percentage_mode,
                'i2c_retry_sec': ParameterValue(battery_i2c_retry_sec, value_type=float),
                'topic_name': battery_topic,
            }],
        ),
    ])
