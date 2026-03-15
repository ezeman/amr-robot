from setuptools import find_packages, setup

package_name = 'amr_imu_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/amr_imu_driver']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/imu.launch.py']),
        ('share/' + package_name + '/config', ['config/imu_params.yaml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='narong',
    maintainer_email='narong@agv.local',
    description='Serial IMU driver for the AMR platform.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_imu_node = amr_imu_driver.serial_imu_node:main',
            'cmp10a_driver = amr_imu_driver.cmp10a_driver:main',
        ],
    },
)
