# Phase 1–2 Workflow: Mapping + Fixed Waypoints (AGV-like) / เวิร์กโฟลว์ 2 เฟส: ทำแผนที่ + เวย์พอยต์ถาวร

This guide produces a “fixed map” and a long-term waypoint route file.

## Phase 1: Mapping (slam_toolbox) / เฟส 1: ทำแผนที่ (slam_toolbox)

### 1) Start mapping stack
```bash
source /opt/ros/humble/setup.bash
source ~/agv_ws/install/setup.bash
ros2 launch amr_bringup agv_mapping.launch.py
```
If your lidar is STL27L, ensure parameters match:
```bash
ros2 launch amr_bringup agv_mapping.launch.py lidar_product:=LDLiDAR_STL27L lidar_baud:=921600 lidar_port:=/dev/lidar
```
Motor direction note:
- ROS convention: `angular.z > 0` should rotate CCW (left).
- If your robot rotates right when `angular.z > 0`, add `base_angular_sign:=-1.0` to mapping/localization/nav launches. If it already rotates left, keep `base_angular_sign:=1.0`.
By default, a scan filter chain is enabled (`scan_raw` -> `scan`).
- Mapping uses `amr_bringup/config/scan_filter_mapping.yaml` (wide FOV, only self-hit + range filtering).
- Navigation/localization uses `amr_bringup/config/scan_filter.yaml` (corridor-focused).

### 2) Drive the corridor repeatedly
- English: drive back-and-forth multiple times to reduce drift and improve loop closures.
- ไทย: ขับไป–กลับหลายรอบในทางเชื่อม เพื่อให้ map นิ่งและปิด loop ได้ดี

### 3) Save and declare the map “FIXED”
Open a new terminal (same sourced environment) and run:
```bash
./scripts/save_fixed_map.sh corridor_fixed_v1
```
Outputs:
- `~/agv_ws/maps/corridor_fixed_v1/corridor_fixed_v1.yaml`
- `~/agv_ws/maps/corridor_fixed_v1/corridor_fixed_v1.pgm`
- `~/agv_ws/maps/corridor_fixed_v1/FIXED_MAP.txt` (hash + timestamp)

Rule: never overwrite a fixed map; create a new name for new versions.

## Phase 2: Record waypoints (AMCL + map_server) / เฟส 2: อัดเวย์พอยต์ (AMCL + map_server)

### 1) Stop SLAM and start localization (AMCL)
```bash
source /opt/ros/humble/setup.bash
source ~/agv_ws/install/setup.bash
ros2 launch amr_bringup agv_localization.launch.py \
  map:=~/agv_ws/maps/corridor_fixed_v1/corridor_fixed_v1.yaml
```
Tip: `agv_localization.launch.py` expands `~` automatically, but using an absolute path is always safe.

### 2) Set initial pose
Use RViz2 (2D Pose Estimate) or publish `/initialpose`.

Recommended (robust, correct timestamp):
```bash
ros2 run amr_tools set_initial_pose --x 0.0 --y 0.0 --yaw 0.0
```

Alternative (CLI publish):
```bash
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}
}"
```
Check that `map -> odom` exists:
```bash
ros2 run tf2_ros tf2_echo map odom
```

Important: the base driver should publish `odom -> base_footprint` and URDF publishes `base_footprint -> base_link` (single-parent TF tree). If your base driver publishes `odom -> base_link` while URDF also publishes `base_footprint -> base_link`, SLAM will drift/freeze on turns.

### 3) Record waypoints from TF (map->base_link)
- Spacing rule (default):
  - straight segments: every 1.0 m
  - turning segments: every 0.3 m (detected by curvature threshold)
```bash
ros2 run amr_tools record_waypoints --ros-args \
  --params-file $(ros2 pkg prefix amr_tools)/share/amr_tools/config/record_waypoints.yaml \
  -p output_path:=~/agv_ws/routes/corridor_fixed_v1_waypoints.yaml
```
Stop with Ctrl+C to save.

### 4) Run the waypoint route (NavigateThroughPoses)
```bash
ros2 run amr_tools route_loop_runner --route ~/agv_ws/routes/corridor_fixed_v1_waypoints.yaml --yaw-mode heading --start-mode nearest
```
If it prints `Waiting for TF map -> base_link`, localization is not ready yet. Set AMCL initial pose first and confirm TF exists:
```bash
ros2 run amr_tools set_initial_pose --x 0.0 --y 0.0 --yaw 0.0
ros2 run tf2_ros tf2_echo map base_link
```

## Notes / หมายเหตุ
- Localization requires consistent TF: `map->odom` (AMCL), `odom->base_link` (base driver), and URDF frames.
- If waypoint density is too high/low in turns, adjust:
  - `turn_interval_m` (default 0.3)
  - `turning_curvature_threshold_rad_per_m` (default 0.35)
