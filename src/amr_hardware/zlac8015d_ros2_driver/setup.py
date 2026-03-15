from setuptools import find_packages, setup

package_name = 'zlac8015d_ros2_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pymodbus', 'tf-transformations'],
    zip_safe=True,
    maintainer='narong',
    maintainer_email='narong@agv.local',
    description='ROS 2 driver for ZLAC8015D dual motor controller on AGV platforms.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         'zlac8015d_node = zlac8015d_ros2_driver.zlac8015d_node:main',
        ],
    },
)
