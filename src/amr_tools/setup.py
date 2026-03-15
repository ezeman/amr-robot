from setuptools import find_packages, setup

package_name = 'amr_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/room_route.yaml', 'config/record_waypoints.yaml', 'config/topic_health_monitor.yaml']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='narong',
    maintainer_email='narong@agv.local',
    description='Utility nodes for the AGV workspace.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'route_loop_runner = amr_tools.route_loop_runner:main',
            'record_waypoints = amr_tools.record_waypoints:main',
            'set_initial_pose = amr_tools.set_initial_pose:main',
            'yield_requester = amr_tools.yield_requester:main',
            'validate_configs = amr_tools.validate_configs:main',
            'topic_health_monitor = amr_tools.topic_health_monitor:main',
            'preflight_topics = amr_tools.preflight_topics:main',
            'web_api_server = amr_tools.web_api_server:main',
        ],
    },
)
