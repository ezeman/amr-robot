# AMR Robot with Jetson Orin + rtabmap_ros (ROS2 Humble)

## Requirements
- Jetson Orin Nano (JetPack 6)
- Ubuntu 22.04
- ROS2 Humble
- LiDAR: LDrobot STL-27L
- Camera: Oak-D Lite

## Installation
```bash
mkdir -p ~/amr_ws/src
cd ~/amr_ws/src
git clone https://github.com/LDROBOTSensorTeam/ldlidar_stl_ros2.git
git clone https://github.com/luxonis/depthai-ros.git
# (Add amr_description, amr_slam, amr_bringup manually or clone from your repo)
cd ~/amr_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Launch the Robot
```bash
ros2 launch amr_bringup amr_bringup.launch.py
```
