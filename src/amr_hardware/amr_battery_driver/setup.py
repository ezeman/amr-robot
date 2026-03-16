from setuptools import find_packages, setup

package_name = 'amr_battery_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/battery.launch.py']),
    ],
    install_requires=['setuptools', 'smbus2'],
    zip_safe=True,
    maintainer='narong',
    maintainer_email='narong@agv.local',
    description='I2C battery monitor node (INA219) publishing sensor_msgs/BatteryState.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_node = amr_battery_driver.battery_node:main',
        ],
    },
)
