from setuptools import find_packages, setup

package_name = 'amr_wfh_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/wfh_test.yaml']),
        ('share/' + package_name + '/launch', [
            'launch/wfh_mode_a_odom.launch.py',
            'launch/wfh_mode_a_fwd_back_1m_10rounds.launch.py',
            'launch/wfh_mode_b_drive_on_heading.launch.py',
        ]),
        ('share/' + package_name + '/scripts', ['scripts/check_interfaces.sh']),
        ('share/' + package_name, ['README_WFH_1M.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='narong',
    maintainer_email='narong@agv.local',
    description='Walk-Forward-Heading 1m test utilities.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wfh_odom_runner = amr_wfh_test.wfh_odom_runner:main',
            'wfh_drive_on_heading_client = amr_wfh_test.wfh_drive_on_heading_client:main',
        ],
    },
)
