# AGV Workspace Guide / คู่มือการใช้งานเวิร์กสเปซ AGV

Clean ROS 2 Humble workspace for a diff-drive AGV on Jetson Orin Nano. Hardware drivers are reused from `amr_ws` but navigation, BTs, and bringup are rebuilt for AGV-style navigation (no spin, Regulated Pure Pursuit by default).

## Setup / การเตรียมระบบ
- English: install dependencies then build once.
  ```bash
  cd ~/agv_ws
  sudo apt update
  sudo apt install ros-$ROS_DISTRO-nav2-regulated-pure-pursuit-controller python3-pymodbus python3-tf-transformations python3-serial
  rosdep install --from-paths src -y --ignore-src
  colcon build --symlink-install
  source install/setup.bash
  ```

## Clean Shell (avoid mixing `amr_ws` and `agv_ws`) / เชลล์แบบสะอาด (กันการปน workspace)
- English/ไทย:
  ```bash
  ~/agv_ws/scripts/use_agv_ws.sh
  ```
  This opens a new clean shell and prints which `amr_bringup` prefix is in use.
- Tip: run a single command in a clean env:
  ```bash
  ~/agv_ws/scripts/agv.sh ros2 pkg prefix amr_bringup
  ```
- ไทย: ติดตั้งแพ็กเกจที่จำเป็นและคอมไพล์
  ```bash
  cd ~/agv_ws
  sudo apt update
  sudo apt install ros-$ROS_DISTRO-nav2-regulated-pure-pursuit-controller python3-pymodbus python3-tf-transformations python3-serial
  rosdep install --from-paths src -y --ignore-src
  colcon build --symlink-install
  source install/setup.bash
  ```

## Hardware Only / ทดสอบฮาร์ดแวร์
- English: start lidar, IMU, and base drivers only.
  ```bash
  ros2 launch amr_hardware_bringup hardware.launch.py \
    lidar_port:=/dev/lidar lidar_product:=LDLiDAR_STL27L lidar_baud:=921600 imu_port:=/dev/imu \
    odom_frame:=odom base_frame:=base_footprint
  ```
- ไทย: รันทดสอบไดรเวอร์อย่างเดียว
  ```bash
  ros2 launch amr_hardware_bringup hardware.launch.py \
    lidar_port:=/dev/lidar lidar_product:=LDLiDAR_STL27L lidar_baud:=921600 imu_port:=/dev/imu \
    odom_frame:=odom base_frame:=base_footprint
  ```
- Check topics / ตรวจสอบท็อปปิก: `./scripts/check_topics.sh`
- Lidar udev: use scripts in `src/amr_hardware/ldlidar_stl_ros2/scripts` (create/delete udev rules).
- Motor direction note:
  - ROS convention: `angular.z > 0` should rotate CCW (left).
  - If your robot rotates right when `angular.z > 0`, launch with `base_angular_sign:=-1.0` (this also fixes odom yaw sign). If it already rotates left, keep `base_angular_sign:=1.0`.
    ```bash
    ros2 launch amr_hardware_bringup hardware.launch.py base_angular_sign:=-1.0
    ```

## Full Navigation / นำทางเต็มระบบ
- English:
  ```bash
  ros2 launch amr_bringup agv_nav.launch.py \
    map:=/path/to/map.yaml \
    params_file:=`ros2 pkg prefix amr_navigation`/share/amr_navigation/config/nav2_agv_default.yaml \
    use_sim_time:=false
  ```
- ไทย:
  ```bash
  ros2 launch amr_bringup agv_nav.launch.py \
    map:=/path/to/map.yaml \
    params_file:=`ros2 pkg prefix amr_navigation`/share/amr_navigation/config/nav2_agv_default.yaml \
    use_sim_time:=false
  ```
- After launch, you must set AMCL initial pose (otherwise `map->odom` is missing and Nav2 will not become active):
  ```bash
  ros2 run amr_tools set_initial_pose --x 0.0 --y 0.0 --yaw 0.0
  ```
  If you run this before AMCL is fully up, it still works (publisher uses TRANSIENT_LOCAL durability).
- Quick checks:
  - `ros2 run tf2_ros tf2_echo map odom`
  - `ros2 action list | grep -E \"navigate_(to_pose|through_poses)\"`
  - `ros2 lifecycle get /velocity_smoother` (must be `active [3]` for motion because controller outputs to `cmd_vel_nav`)
- Frames: `map -> odom` (AMCL) -> `base_link` (base driver) with `lidar_link` and `imu_link` from robot_state_publisher.
- Default speed: Nav2 is configured for ~`0.18 m/s` linear speed (Regulated Pure Pursuit `desired_linear_vel`) and velocity_smoother clamps `/cmd_vel` to `±0.18 m/s`.

## Route Loop Runner / เครื่องมือวิ่งเส้นทาง
- English:
  ```bash
  ros2 run amr_tools route_loop_runner --route \
    `ros2 pkg prefix amr_tools`/share/amr_tools/config/room_route.yaml --loop --yaw-mode heading --start-mode nearest
  ```
- ไทย:
  ```bash
  ros2 run amr_tools route_loop_runner --route \
    `ros2 pkg prefix amr_tools`/share/amr_tools/config/room_route.yaml --loop --yaw-mode heading --start-mode nearest
  ```
- Edit waypoints in `amr_tools/config/room_route.yaml` (map frame). Use `--loop` to repeat and `--pause` to set delay.
  - Tip: `--yaw-mode heading` sets yaw from the segment direction (often smoother than recorded yaw and reduces stop-and-rotate).
  - Tip: `--start-mode nearest` starts from the closest waypoint to the robot (prevents trying to drive back to the recording start point).
  - Go+Back test: `--roundtrips 5` runs 5 roundtrips (forward then reverse) and stops.
  - If you see `Failed to make progress` or `ABORTED`: check that `/velocity_smoother` is active and that `/cmd_vel` has a publisher.
  - If you see `RegulatedPurePursuitController detected collision ahead!`: your lidar is still seeing the robot chassis/posts; tune `amr_bringup/config/scan_filter.yaml` `remove_robot_self_hits` box (expand `max_x/max_y`).

## No-Spin Behavior / ยืนยันว่าไม่มีการหมุนอยู่กับที่
- English: controller is `nav2_regulated_pure_pursuit_controller` with `use_rotate_to_heading: false`; BT files (`nav_to_pose_agv_like_no_spin.xml` and `follow_waypoints_agv_like_no_spin.xml`) only use backup + wait recoveries. Expected motion is AGV-like without in-place spins.
- ไทย: ใช้คอนโทรลเลอร์ Regulated Pure Pursuit ปิด `rotate_to_heading` และ BT มีเพียง backup + wait ดังนั้นหุ่นจะไม่หมุนอยู่กับที่โดยค่าเริ่มต้น

## Dynamic Obstacles / สิ่งกีดขวางชั่วคราว
- English: when the path is blocked, the BT clears local/global costmaps, waits ~3s, backs up slightly, then replans (no spin). If the corridor is fully blocked, it will keep retrying until the progress timeout is reached.
- ไทย: ถ้าเส้นทางถูกขวาง BT จะเคลียร์ costmap (local/global) รอ ~3 วินาที ถอยหลังเล็กน้อย แล้วคำนวณเส้นทางใหม่ (ไม่มี spin) ถ้าทางตันจริงจะรอ/ลองใหม่จนกว่าจะหมดเวลา progress timeout
- Yield sound: `agv_nav.launch.py` runs `amr_tools/yield_requester` by default. It beeps when an obstacle is within `0.60 m` in front for ~1.5s.
  - Disable: `use_yield_requester:=false`
  - Change distance: `yield_distance_m:=0.80`
  - Use TTS (optional): install `espeak` then launch with `yield_sound_mode:=espeak yield_message:="ขอทางหน่อยครับ"`
  - If beep doesn't play: install `alsa-utils` (provides `aplay`) or set `yield_sound_mode:=none` (logs only)

## Package Layout / โครงสร้างแพ็กเกจ
- Hardware: `src/amr_hardware/{ldlidar_stl_ros2, amr_imu_driver, zlac8015d_ros2_driver, amr_hardware_bringup}`
- Description: `src/amr_description` (URDF + robot_state_publisher launch)
- Navigation: `src/amr_navigation` (Nav2 params + simple BTs)
- Bringup: `src/amr_bringup` (hardware + description + Nav2)
- Tools: `src/amr_tools` (route loop runner + sample route)

## Stop Everything / หยุดโปรเซสทั้งหมด
```bash
~/agv_ws/scripts/kill_ros.sh
```
