from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[
                {'frame_id': 'base_link'},
                {'subscribe_depth': True},
                {'subscribe_rgbd': True},
                {'subscribe_scan': True},
                {'use_sim_time': False},
                'config/slam_params.yaml'
            ],
            remappings=[
                ('scan', '/scan'),
                ('rgb/image', '/oakd/rgb/image_raw'),
                ('depth/image', '/oakd/depth/image_raw'),
                ('rgb/camera_info', '/oakd/rgb/camera_info')
            ]
        )
    ])


