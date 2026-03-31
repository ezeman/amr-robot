# AGV Robot — Feature Summary

> สรุป Feature ทั้งหมดของระบบ AGV ครอบคลุม Web UI, REST API และ ROS 2 Nodes  
> Workspace: `~/agv_ws` | Platform: Jetson (ezeJetson) + ROS 2 Humble

---

## 1. Web UI — AGV Ops Console (port 8090)

| Section | Feature | รายละเอียด |
|---------|---------|------------|
| **Connection** | API Base URL | กำหนด URL ของ API Server (auto-detect จาก `window.location.hostname`) |
| | API Key | รองรับ authentication ด้วย X-API-Key header |
| | Health Check | ตรวจสอบสถานะ API Server |
| | SSE Connect/Disconnect | เชื่อมต่อ Server-Sent Events สำหรับ realtime updates |
| **Mission State** | Live Status Indicators | แสดงสถานะ 5 component: SLAM, Localization, Navigation, Waypoint Record, Route Follow |
| **Hardware Status** | Hardware Grid | แสดงสถานะ hardware 5 ตัว: LiDAR, IMU, Encoder, 3D Camera, Battery |
| | Battery Display | แสดง voltage (V), percentage (%), สถานะ (NORMAL / LOW / CRITICAL / OFFLINE) |
| **Battery Tuning** | Live Parameter Tuning | ปรับ voltage_scale, V min/max, percentage_mode ได้แบบ realtime ผ่าน Web UI |
| | Percentage Mode | เลือก `lifepo4_8s` (OCV curve) หรือ `linear` |
| **Mission Presets** | Quick Workflows | ปุ่มลัด: Start Mapping, Save Fixed Map, Start Delivery Mission, Stop All |
| | Preset Fields | กำหนด map name และ route file สำหรับ preset |
| **SLAM & Map** | Start/Stop SLAM | เริ่ม/หยุด SLAM mapping พร้อมกำหนด LiDAR product, baudrate, port |
| | Save Map | บันทึก map เป็น fixed map (YAML + PGM) |
| | Map List | แสดงรายการ map ทั้งหมด พร้อม chip selector |
| **Waypoints** | Record Waypoints | บันทึก waypoint จาก TF (map → base_link) เป็นไฟล์ YAML |
| | Route Following | เริ่ม route ด้วย yaw mode (heading/from_file/zero), start mode (nearest/fixed), pause duration |
| | Route List | แสดงรายการ route ทั้งหมด พร้อม chip selector |
| **Localization & Nav** | Start/Stop Localization | เริ่ม/หยุด AMCL localization พร้อมเลือก map |
| | Start/Stop Navigation | เริ่ม/หยุด Nav2 stack พร้อม yield requester toggle |
| **System Tools** | Get Status | ดูสถานะ process ที่ทำงานอยู่ |
| | Validate Config | ตรวจสอบ YAML config ทั้งหมด |
| | Kill ROS Processes | หยุด ROS process ทั้งหมด |
| **Logs** | Realtime Events | แสดง SSE events แบบ realtime พร้อมปุ่ม fold/unfold (ซ่อนเป็น default) |
| | API Response Log | แสดง JSON response จาก API พร้อมปุ่ม fold/unfold |

---

## 2. Web Camera & Detection UI (port 8091)

| Feature | รายละเอียด |
|---------|------------|
| **ROS Bridge Connection** | เชื่อมต่อ rosbridge WebSocket (port 9090) ด้วย roslib.js |
| **Camera Feed** | แสดง RGB feed จาก OAK-D-LITE แบบ realtime บน canvas |
| **Object Detection** | แสดง bounding boxes, labels, confidence % จาก NN detections |
| **Depth Overlay** | ซ้อน depth heatmap บน RGB feed (toggle on/off) |
| **Camera Settings** | ปรับ refresh rate (0.5–15 fps), JPEG quality (10–100%) |
| **Topic Config** | กำหนด topic สำหรับ RGB, detection, depth |
| **Display Options** | Toggle: bounding boxes, labels, confidence, depth overlay, min confidence, box opacity |
| **Snapshot / Record** | จับภาพ snapshot, fullscreen, record, clear stats |
| **Detection Stats** | Frame count, detection count, FPS, latency, per-class object counts |
| **Detection Log** | รายการ detection แต่ละ frame (max 200 entries) |

---

## 3. REST API (port 8088)

### Authentication

- Environment variable: `AGV_API_KEY=<secret>`
- Request header: `X-API-Key: <secret>`
- SSE query param: `/api/v1/events?api_key=<secret>`

### GET Endpoints

| Endpoint | รายละเอียด |
|----------|------------|
| `/api/v1/health` | ตรวจสอบ API readiness (ok, time_utc, workspace, status) |
| `/api/v1/status` | รายการ process ที่ทำงานอยู่ (PID, uptime, log path, command) |
| `/api/v1/node_health` | สถานะ hardware nodes (lidar, imu, encoder, camera, battery) พร้อม node/topic alive flags |
| `/api/v1/battery` | สถานะ battery: voltage (V), percentage (0–1), state (normal/low/critical), thresholds |
| `/api/v1/battery/params` | อ่านค่า calibration: voltage_scale, V min/max, percentage_mode |
| `/api/v1/maps` | รายการ map ทั้งหมด (name, YAML path, FIXED_MAP flag) |
| `/api/v1/routes` | รายการ route ทั้งหมด (name, YAML path) |
| `/api/v1/events` | SSE stream สำหรับ realtime updates |

### POST Endpoints

| Endpoint | Parameters | รายละเอียด |
|----------|-----------|------------|
| `/api/v1/slam/start` | lidar_product, lidar_baud, lidar_port, base_angular_sign | เริ่ม SLAM mapping |
| `/api/v1/slam/stop` | — | หยุด SLAM |
| `/api/v1/slam/save_map` | map_name, timeout_sec? | บันทึก map (YAML + PGM + FIXED_MAP.txt) |
| `/api/v1/localization/start` | map, base_angular_sign | เริ่ม AMCL localization |
| `/api/v1/localization/stop` | — | หยุด localization |
| `/api/v1/navigation/start` | map, base_angular_sign, use_yield_requester | เริ่ม Nav2 navigation |
| `/api/v1/navigation/stop` | — | หยุด navigation |
| `/api/v1/waypoints/record/start` | output_path?, map_frame?, base_frame? | เริ่มบันทึก waypoints จาก TF |
| `/api/v1/waypoints/record/stop` | — | หยุดบันทึก waypoints (save YAML) |
| `/api/v1/route/start` | route, yaw_mode, start_mode, pause_sec, loop?, reverse?, roundtrips? | เริ่ม route following |
| `/api/v1/route/stop` | — | หยุด route following |
| `/api/v1/config/validate` | strict?, timeout_sec? | ตรวจสอบ YAML config ทั้งหมด |
| `/api/v1/system/kill_ros` | — | Kill ROS process ทั้งหมด |
| `/api/v1/battery/params` | voltage_scale?, battery_voltage_min?, battery_voltage_max?, percentage_mode? | ปรับค่า battery calibration แบบ live |

### SSE Event Types

| Event | Payload | เมื่อไร |
|-------|---------|--------|
| `snapshot` | status, mission_state, node_health, battery | เมื่อ client เชื่อมต่อครั้งแรก |
| `heartbeat` | status, mission_state, node_health, battery | ทุก 2 วินาที (ถ้าไม่มี event อื่น) |
| `mission_state_changed` | component, active, reason, active_components | เมื่อ SLAM/localization/nav/route/waypoint เปลี่ยนสถานะ |
| `map_saved` | map_name, map_yaml, map_pgm | หลัง save map สำเร็จ |
| `map_save_failed` | map_name, exit_code, stderr | เมื่อ save map ล้มเหลว |
| `route_progress` | remaining_poses, distance_remaining_m, recoveries | ระหว่าง route following (parse จาก log) |
| `process_started` | name, pid, log_path, command | เมื่อ launch process เริ่ม |
| `process_stopped` | name, exit_code | เมื่อ process หยุดปกติ |
| `process_exited` | name, exit_code | เมื่อ process exit (ปกติหรือ crash) |

---

## 4. ROS 2 Nodes

### Hardware Nodes

| Node | Package | หน้าที่ | Topic Published | Parameters หลัก |
|------|---------|--------|-----------------|-----------------|
| `/ldlidar` | ldlidar_stl_ros2 | LiDAR driver (LD STL27L) | `/scan` (LaserScan) | product_name, port_name, port_baudrate, frame_id |
| `/imu_driver` | amr_imu_driver | IMU/compass (CMP10A) | `/imu/data` (Imu) | port, baud, frame_id |
| `/base_controller` | zlac8015d_ros2_driver | Motor control (ZLAC8015D) | `/odom` (Odometry) ← `/cmd_vel` | wheel_radius, base_width, angular_sign |
| `/battery_node` | amr_battery_driver | Battery monitor (INA219 I2C) | `/battery_state` (BatteryState) | i2c_bus=7, i2c_address=0x40, voltage_scale=2.547, percentage_mode=lifepo4_8s |

### Navigation & Localization Nodes

| Node | Package | หน้าที่ |
|------|---------|--------|
| `/slam_toolbox` | slam_toolbox | SLAM mapping (async) |
| `/amcl` | nav2_amcl | Monte Carlo localization |
| `/map_server` | nav2_map_server | Static map loader |
| `/bt_navigator` | nav2_bt_navigator | Behavior tree navigator |
| `/controller_server` | nav2_controller | Regulated Pure Pursuit controller |
| `/planner_server` | nav2_planner | NavfnPlanner (GridBased) |
| `/velocity_smoother` | nav2_velocity_smoother | Smooth acceleration ramps |
| `/recoveries_server` | nav2_recoveries | Backup, wait, clear costmaps |
| `/robot_state_publisher` | robot_state_publisher | URDF → TF broadcast |

### Utility Nodes (amr_tools)

| Node | หน้าที่ | วิธีใช้ |
|------|--------|--------|
| `/record_waypoints` | บันทึก pose จาก TF เป็น YAML route | auto-detect straight/turn intervals |
| `/route_loop_runner` | วิ่งตาม waypoint route (NavigateThroughPoses) | yaw mode, start mode, loop, reverse, roundtrips |
| `/set_initial_pose` | ตั้งค่า AMCL initial pose | x, y, yaw, repeat count |
| `/topic_health_monitor` | ตรวจสอบ critical topics (scan/imu/odom) | timeout, fail_hard, grace period |
| `/yield_requester` | ขอทางด้วยเสียงเมื่อมีสิ่งกีดขวาง | scan-based, cooldown, espeak/beep |
| `/preflight_topics` | ตรวจสอบ topics ก่อนเริ่ม nav | timeout-based verification |

### Scan Filter

| Config | หน้าที่ | ค่า |
|--------|--------|-----|
| `remove_robot_self_hits` | ตัด laser ที่ยิงโดนตัวเอง | box: -0.65→0.85 m (x), -0.55→0.55 m (y) |
| `keep_front_corridor` | โฟกัสเฉพาะทางข้างหน้า | mapping: 8m × 4.4m / nav: 2m × 4.4m |
| `range_filter` | จำกัดระยะ 0.1–8.0 m | lower=0.10, upper=8.0 |

---

## 5. Launch Files

| Launch File | หน้าที่ | Nodes ที่เปิด |
|-------------|--------|---------------|
| `hardware.launch.py` | เปิด hardware ทั้งหมด | LiDAR, IMU, Base controller, Battery (แต่ละตัว toggle on/off ได้) |
| `agv_mapping.launch.py` | SLAM mapping | Hardware + SLAM toolbox + Scan filter + Topic health + Optional camera |
| `agv_localization.launch.py` | Localization only | Hardware + AMCL + Map server + Scan filter |
| `agv_nav.launch.py` | Full navigation | Hardware + Localization + Nav2 stack + Yield requester + Topic health |
| `battery.launch.py` | Battery only | Battery node (standalone) |

---

## 6. Battery System

| รายการ | ค่า |
|--------|-----|
| **Hardware** | INA219 I2C sensor, bus 7, address 0x40 |
| **Battery Type** | LiFePO4 8S (25.6V nominal) |
| **Voltage Range** | 22.4V (0%) — 29.2V (100%) |
| **Calibration** | voltage_scale = 2.547 (27.2V meter ÷ 10.68V raw) |
| **SOC Mode** | `lifepo4_8s` — 13-point OCV interpolation curve |
| **OCV Curve** | 22.4V→0%, 24.0V→5%, 25.6V→12%, 26.0V→22%, 26.2V→32%, 26.3V→42%, 26.4V→52%, 26.5V→62%, 26.6V→72%, 26.8V→82%, 27.0V→90%, 27.6V→97%, 28.4V→100% |
| **Alternative** | `linear` — simple min/max interpolation |
| **Publish Rate** | 2 Hz → `/battery_state` (BatteryState) |
| **Web UI Polling** | 5 seconds interval |

---

## 7. Waypoint & Route System

### Recording

- TF-based: ดึง pose จาก `map → base_link`
- Auto-detect straight vs turn: `turning_curvature_threshold = 0.35 rad/m`
- Straight interval: 1.0 m / Turn interval: 0.3 m
- Output: YAML (frame_id, waypoints: [{x, y, yaw}, ...])

### Route Following

| Option | ค่าที่เลือกได้ |
|--------|---------------|
| Yaw Mode | `heading` (auto), `from_file` (ตามที่บันทึก), `zero` (ไม่หมุน) |
| Start Mode | `nearest` (ใกล้สุด), `fixed` (ตามลำดับ) |
| Pause | กำหนดวินาทีระหว่าง waypoint |
| Loop | วนไม่จบ |
| Reverse | วิ่งย้อนกลับ |
| Roundtrips | วิ่งไป-กลับ N รอบ |
| Action | `NavigateThroughPoses` (Nav2) |

---

## 8. Shell Scripts

| Script | หน้าที่ |
|--------|--------|
| `agv.sh` | Wrapper สำหรับรัน command ใน ROS environment ของ agv_ws |
| `use_agv_ws.sh` | เปิด shell ใหม่ที่ source agv_ws |
| `save_fixed_map.sh <name>` | บันทึก SLAM map เป็น fixed map (YAML + PGM + FIXED_MAP.txt) |
| `kill_ros.sh` | Kill ROS process ทั้งหมด |
| `check_topics.sh` | ตรวจสอบว่า critical topics publish อยู่ |
| `test_encoder_distance.py` | ทดสอบ calibrate motor distance |
| `install_web_services.sh` | ติดตั้ง systemd user services ทั้งหมด |

---

## 9. Systemd User Services

| Service | Port | หน้าที่ |
|---------|------|--------|
| `agv-web-api.service` | 8088 | REST API Server (Python) |
| `agv-web-ui.service` | 8090 | Static file server สำหรับ Web UI |
| `agv-rosbridge.service` | 9090 | rosbridge WebSocket server |
| `agv-web-rviz.service` | — | Web-based RViz (optional) |

---

## 10. Navigation Configuration

| Parameter | ค่า | หมายเหตุ |
|-----------|-----|---------|
| Controller | Regulated Pure Pursuit | FollowPath plugin |
| Controller Frequency | 12 Hz | |
| Desired Linear Vel | ~0.18 m/s | AGV speed |
| Rotate to Heading | Disabled | AGV-like (no in-place spin) |
| BT (navigate_to_pose) | `nav_to_pose_agv_like_no_spin.xml` | Custom ไม่มี spin |
| BT (navigate_through_poses) | `follow_waypoints_agv_like_no_spin.xml` | Custom ไม่มี spin |
| AMCL Particles | 500–1500 | |
| Laser Model | likelihood_field | |
| Recovery Behaviors | backup, wait, clear costmaps | ไม่มี spin recovery |

---

## 11. Architecture Overview

```
┌─────────────────────────────────────────────────────┐
│                    Web Browser                       │
│  ┌──────────────┐  ┌──────────────┐                 │
│  │  AGV Ops UI  │  │  Camera UI   │                 │
│  │  (port 8090) │  │  (port 8091) │                 │
│  └──────┬───────┘  └──────┬───────┘                 │
└─────────┼─────────────────┼─────────────────────────┘
          │ HTTP/SSE        │ WebSocket
          ▼                 ▼
┌─────────────────┐  ┌──────────────┐
│  REST API       │  │  rosbridge   │
│  (port 8088)    │  │  (port 9090) │
└────────┬────────┘  └──────┬───────┘
         │                  │
         ▼                  ▼
┌─────────────────────────────────────────────────────┐
│                   ROS 2 Humble                       │
│                                                      │
│  ┌──────────┐ ┌──────┐ ┌──────┐ ┌─────────┐        │
│  │  LiDAR   │ │ IMU  │ │ Base │ │ Battery │        │
│  │  Driver   │ │Driver│ │Ctrl  │ │  Node   │        │
│  └────┬─────┘ └──┬───┘ └──┬───┘ └────┬────┘        │
│       │          │        │           │              │
│       ▼          ▼        ▼           ▼              │
│    /scan     /imu/data  /odom   /battery_state       │
│       │          │        │                          │
│       ▼          ▼        ▼                          │
│  ┌──────────────────────────────┐                    │
│  │  Scan Filter → SLAM/AMCL    │                    │
│  │  Nav2 Stack (Controller,    │                    │
│  │  Planner, BT Navigator)     │                    │
│  └──────────────┬───────────────┘                    │
│                 │                                    │
│                 ▼                                    │
│             /cmd_vel → Base Controller → Motors      │
└─────────────────────────────────────────────────────┘
```

---

*Generated: 2026-03-17 | Branch: `agv-ros-webx` | Repo: `ezeman/amr-robot`*
