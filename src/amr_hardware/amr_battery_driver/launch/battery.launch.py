from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('i2c_bus', default_value='7', description='I2C bus number'),
        DeclareLaunchArgument('i2c_address', default_value='64', description='INA219 I2C address (decimal, 64 = 0x40)'),
        DeclareLaunchArgument('publish_rate_hz', default_value='2.0', description='Battery publish rate'),
        DeclareLaunchArgument('battery_voltage_min', default_value='22.4', description='0% battery voltage (LiFePO4 8S)'),
        DeclareLaunchArgument('battery_voltage_max', default_value='29.2', description='100% battery voltage (LiFePO4 8S)'),
        DeclareLaunchArgument('voltage_scale', default_value='2.547', description='Scale factor for measured voltage (calibrated)'),
        DeclareLaunchArgument('percentage_mode', default_value='lifepo4_8s', description='SOC mode: lifepo4_8s or linear'),
        DeclareLaunchArgument('i2c_retry_sec', default_value='5.0', description='Retry interval when I2C device is busy/unavailable'),
        DeclareLaunchArgument('topic_name', default_value='/battery_state', description='BatteryState topic'),

        Node(
            package='amr_battery_driver',
            executable='battery_node',
            name='battery_node',
            output='screen',
            parameters=[{
                'i2c_bus': ParameterValue(LaunchConfiguration('i2c_bus'), value_type=int),
                'i2c_address': ParameterValue(LaunchConfiguration('i2c_address'), value_type=int),
                'publish_rate_hz': ParameterValue(LaunchConfiguration('publish_rate_hz'), value_type=float),
                'battery_voltage_min': ParameterValue(LaunchConfiguration('battery_voltage_min'), value_type=float),
                'battery_voltage_max': ParameterValue(LaunchConfiguration('battery_voltage_max'), value_type=float),
                'voltage_scale': ParameterValue(LaunchConfiguration('voltage_scale'), value_type=float),
                'percentage_mode': LaunchConfiguration('percentage_mode'),
                'i2c_retry_sec': ParameterValue(LaunchConfiguration('i2c_retry_sec'), value_type=float),
                'topic_name': LaunchConfiguration('topic_name'),
            }],
        ),
    ])
