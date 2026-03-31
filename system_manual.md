# AGV Robot — System Manual

> คู่มือระบบ AGV ครบถ้วน: การติดตั้ง, การตั้งค่า, การ Calibrate, การใช้งาน และการแก้ไขปัญหา  
> Workspace: `~/agv_ws` | Platform: Jetson Orin Nano | ROS 2 Humble  
> Robot IP: `192.168.1.41`

---

## สารบัญ

1. [ภาพรวมระบบ](#1-ภาพรวมระบบ)
2. [สถาปัตยกรรม](#2-สถาปัตยกรรม)
3. [Hardware Specifications](#3-hardware-specifications)
4. [Software Dependencies](#4-software-dependencies)
5. [การติดตั้ง (Installation)](#5-การติดตั้ง)
6. [การ Calibrate](#6-การ-calibrate)
7. [การใช้งาน (Operation)](#7-การใช้งาน)
8. [Web UI — คู่มือผู้ใช้](#8-web-ui)
9. [REST API Reference](#9-rest-api-reference)
10. [Joystick Teleoperation](#10-joystick-teleoperation)
11. [Configuration Reference](#11-configuration-reference)
12. [Troubleshooting](#12-troubleshooting)
13. [ROS Topic & TF Reference](#13-ros-topic--tf-reference)
14. [Command Reference](#14-command-reference)
15. [File Structure](#15-file-structure)

---

## 1. ภาพรวมระบบ

AGV Robot เป็นหุ่นยนต์ขับเคลื่อนแบบ Differential Drive (2 ล้อขับ + ล้อ caster) ทำงานบน ROS 2 Humble สำหรับงาน:

- **SLAM Mapping** — สร้างแผนที่จาก LiDAR
- **AMCL Localization** — ระบุตำแหน่งบนแผนที่
- **Nav2 Navigation** — วางแผนเส้นทางและหลบสิ่งกีดขวาง
- **Waypoint Route Following** — วิ่งตามจุด waypoint ที่บันทึกไว้
- **Web-based Remote Control** — ควบคุมผ่าน Browser บน Tablet/PC

### ขั้นตอนการทำงาน 3 เฟส

```
เฟส 1: MAPPING              เฟส 2: LOCALIZATION          เฟส 3: NAVIGATION
─────────────────           ──────────────────           ──────────────────
เปิด SLAM                   โหลด fixed map               โหลด fixed map
บังคับหุ่นยนต์เดินสำรวจ      ตั้ง initial pose             เริ่ม Nav2
สร้างแผนที่                  AMCL ติดตามตำแหน่ง           วิ่งตาม route
บันทึก fixed map             บันทึก waypoints             หลบสิ่งกีดขวางอัตโนมัติ
```

---

## 2. สถาปัตยกรรม

```
┌──────────────────────────────────────────────────────────┐
│                     WEB BROWSER (Tablet/PC)               │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────┐  │
│  │  AGV Ops UI  │  │  Camera UI   │  │  Web RViz      │  │
│  │  port 8090   │  │  port 8091   │  │  port 8091     │  │
│  └──────┬───────┘  └──────┬───────┘  └──────┬─────────┘  │
└─────────┼─────────────────┼─────────────────┼────────────┘
          │ HTTP/SSE        │ WebSocket       │ WebSocket
          ▼                 ▼                 ▼
   ┌──────────────┐  ┌──────────────┐
   │  REST API    │  │  rosbridge   │
   │  port 8088   │  │  port 9090   │
   └──────┬───────┘  └──────┬───────┘
          │ subprocess       │ ROS topics
          ▼                  ▼
┌──────────────────────────────────────────────────────────┐
│                     ROS 2 HUMBLE                          │
│                                                           │
│  HARDWARE LAYER           NAVIGATION LAYER                │
│  ├─ LiDAR (STL27L)       ├─ SLAM Toolbox                 │
│  ├─ IMU (CMP10A)         ├─ AMCL + Map Server             │
│  ├─ Motor (ZLAC8015D)    ├─ Nav2 (Controller, Planner)    │
│  └─ Battery (INA219)     ├─ Velocity Smoother             │
│                           └─ Behavior Trees (no spin)     │
│  UTILITY LAYER                                            │
│  ├─ Scan Filter           ├─ Waypoint Recorder            │
│  ├─ Topic Health Monitor  ├─ Route Loop Runner             │
│  ├─ Yield Requester       └─ Set Initial Pose             │
└──────────────────────────────────────────────────────────┘
```

### TF Frame Tree

```
map
 └─ odom                    (published by AMCL)
     └─ base_footprint      (published by base_controller)
         └─ base_link       (published by URDF)
             ├─ lidar_link
             ├─ imu_link
             └─ [อื่นๆ]
```

---

## 3. Hardware Specifications

### 3.1 ตัวหุ่นยนต์

| พารามิเตอร์ | ค่า | หมายเหตุ |
|------------|-----|---------|
| Wheel Radius | 0.10 m | calibrate ด้วย `test_encoder_distance.py` |
| Base Width (Axle) | 0.40 m | ระยะระหว่างล้อซ้าย-ขวา |
| Chassis | 0.65 × 0.50 × 0.30 m | ยาว × กว้าง × สูง |
| น้ำหนัก | ~15 kg | |
| Robot Footprint (Nav2) | `[[0.42,0.26],[0.42,-0.26],[-0.32,-0.26],[-0.32,0.26]]` | สี่เหลี่ยม 0.74 × 0.52 m |
| ความเร็วสูงสุด | 0.18 m/s (linear), 1.2 rad/s (angular) | จำกัดโดย velocity smoother |

### 3.2 LiDAR

| รายการ | ค่า |
|--------|-----|
| รุ่น | LD LiDAR STL27L |
| Port | `/dev/lidar` (udev symlink) |
| Baud Rate | 921,600 bps |
| Topic | `/scan` (filtered), `/scan_raw` (raw) |
| Max Range | 25 m (mapping), 8 m (nav filter) |
| Frame ID | `lidar_link` |
| ตำแหน่งติดตั้ง | x=0.18, y=0.0, z=0.23 m (จาก base_link) |
| Update Rate | 10 Hz |

### 3.3 IMU

| รายการ | ค่า |
|--------|-----|
| รุ่น | Yahboom CMP10A 10-axis |
| Port | `/dev/imu` (udev symlink) |
| Baud Rate | 115,200 bps |
| Topic | `/imu/data` |
| Frame ID | `imu_link` |
| ตำแหน่งติดตั้ง | x=0.15, y=0.0, z=0.15 m |
| Output Rate | 100 Hz |
| ข้อมูล | Accelerometer ±16g, Gyro ±2000°/s, Quaternion |

### 3.4 Motor Controller

| รายการ | ค่า |
|--------|-----|
| รุ่น | ZLAC8015D |
| Command Topic | `/cmd_vel` (Twist) |
| Feedback Topic | `/odom` (Odometry) |
| ความเร็วสูงสุด | ±0.30 m/s (hardware), จำกัดที่ ±0.18 m/s (nav) |
| Invert Right Motor | `true` (default) |
| Angular Sign | `1.0` (ปกติ), `-1.0` (ถ้าหมุนผิดทาง) |

### 3.5 Battery

| รายการ | ค่า |
|--------|-----|
| ชนิด | LiFePO4 8S (25.6V nominal) |
| ช่วงแรงดัน | 22.4V (0%) — 29.2V (100%) |
| Sensor | INA219 I2C (bus 7, address 0x40) |
| Voltage Scale | 2.547 (calibrated) |
| Topic | `/battery_state` (BatteryState) |
| Publish Rate | 2 Hz |
| SOC Mode | `lifepo4_8s` — 13-point OCV curve |
| Low Warning | ≤20% |
| Critical Warning | ≤10% |

### 3.6 Camera (Optional)

| รายการ | ค่า |
|--------|-----|
| รุ่น | OAK-D-LITE |
| RGB Topic | `/oak/rgb/image_raw/compressed` |
| Detection Topic | `/oak/nn/detections` |
| Depth Topic | `/oak/stereo/image_raw` |
| Driver | depthai_ros_driver |

---

## 4. Software Dependencies

### 4.1 ระบบปฏิบัติการ

- Ubuntu 22.04 (Jammy)
- ROS 2 Humble Hawksbill
- Python 3.10+

### 4.2 ROS 2 Packages

```bash
# Navigation Stack
sudo apt install \
  ros-humble-nav2-bringup \
  ros-humble-nav2-regulated-pure-pursuit-controller \
  ros-humble-nav2-controller \
  ros-humble-nav2-planner \
  ros-humble-nav2-navfn-planner \
  ros-humble-nav2-costmap-2d \
  ros-humble-nav2-amcl \
  ros-humble-nav2-velocity-smoother \
  ros-humble-nav2-waypoint-follower \
  ros-humble-slam-toolbox

# Supporting
sudo apt install \
  ros-humble-laser-filters \
  ros-humble-robot-state-publisher \
  ros-humble-rosbridge-server \
  ros-humble-tf2-ros \
  ros-humble-tf-transformations

# Optional (Camera)
sudo apt install ros-humble-depthai-ros-driver
```

### 4.3 Python Dependencies

```bash
pip3 install pyserial smbus2 PyYAML
```

### 4.4 Build

```bash
cd ~/agv_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 5. การติดตั้ง

### 5.1 Clone Repository

```bash
git clone https://github.com/ezeman/amr-robot.git ~/agv_ws
cd ~/agv_ws
git checkout agv-ros-webx
```

### 5.2 Build Workspace

```bash
cd ~/agv_ws
colcon build --symlink-install
```

### 5.3 udev Rules สำหรับ Devices

**LiDAR** → `/dev/lidar`:
```bash
sudo tee /etc/udev/rules.d/99-agv-lidar.rules > /dev/null << 'EOF'
SUBSYSTEMS=="usb", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000a", SYMLINK+="lidar"
EOF
```

**IMU** → `/dev/imu`:
```bash
sudo tee /etc/udev/rules.d/99-agv-imu.rules > /dev/null << 'EOF'
SUBSYSTEMS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="imu"
EOF
```

Apply rules:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 5.4 I2C Permission (Battery)

```bash
# ตรวจสอบ I2C bus 7
ls -la /dev/i2c-7

# ตรวจสอบ INA219 address 0x40
i2cdetect -y 7   # ควรเห็น 40

# เพิ่ม user เข้ากลุ่ม i2c
sudo usermod -aG i2c $USER
# logout แล้ว login ใหม่
```

### 5.5 ติดตั้ง Systemd Services

```bash
cd ~/agv_ws
./scripts/install_web_services.sh

# เปิดใช้งาน auto-start
systemctl --user enable agv-web-api.service
systemctl --user enable agv-web-ui.service
systemctl --user enable agv-rosbridge.service

# ตรวจสอบ
systemctl --user status agv-web-api.service
```

### Systemd Services

| Service | Port | หน้าที่ | Restart |
|---------|------|--------|---------|
| `agv-web-api.service` | 8088 | REST API Server | always, 2s |
| `agv-web-ui.service` | 8090 | Web UI (static files) | always, 2s |
| `agv-web-rviz.service` | 8091 | Web RViz | always, 2s |
| `agv-rosbridge.service` | 9090 | rosbridge WebSocket | always, 2s |

### 5.6 Clean Shell Environment

```bash
# ใช้ wrapper สำหรับ command เดียว (หลีกเลี่ยง workspace ชนกัน)
~/agv_ws/scripts/agv.sh ros2 topic list

# เปิด interactive shell ที่ source agv_ws
~/agv_ws/scripts/use_agv_ws.sh
```

---

## 6. การ Calibrate

### 6.1 Encoder / Wheel Radius

```bash
# ทดสอบระยะทาง
python3 ~/agv_ws/scripts/test_encoder_distance.py --distance 1.0 --speed 0.15

# วัดจริงด้วยเทปวัด
# correction_factor = ระยะวัดจริง / ระยะ encoder
# อัปเดต wheel_radius ใน zlac8015d driver
```

### 6.2 Motor Direction

```bash
# เริ่ม hardware
ros2 launch amr_hardware_bringup hardware.launch.py

# ทดสอบหมุน (ควรหมุนซ้าย/ทวนเข็ม)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" --rate 10

# ถ้าหมุนขวาแทน → ใช้ base_angular_sign:=-1.0
```

### 6.3 Battery Voltage

```bash
# 1. วัดแรงดันจริงด้วย multimeter → เช่น 27.2V
# 2. อ่านค่า raw จาก sensor
ros2 topic echo /battery_state --once | grep voltage
# output: voltage: 10.68

# 3. คำนวณ scale
# new_scale = 27.2 / 10.68 = 2.547

# 4. อัปเดต parameter
ros2 param set /battery_node voltage_scale 2.547

# 5. ตรวจสอบ
ros2 topic echo /battery_state --once | grep voltage
# ควรแสดง ~27.2
```

### 6.4 Scan Filter (ตัด laser ยิงโดนตัวเอง)

ถ้า Nav2 เห็น collision ทั้งที่ทางว่าง:

1. ดู `/scan_raw` ใน RViz — มี laser ยิงโดนตัวหุ่นยนต์ไหม
2. ขยาย box ใน `src/amr_bringup/config/scan_filter.yaml`:
   ```yaml
   remove_robot_self_hits:
     min_x: -0.70  # ขยายขึ้น
     max_x:  0.90
     min_y: -0.60
     max_y:  0.60
   ```
3. ทดสอบจนกว่า `/scan` จะสะอาด

### 6.5 AMCL Initial Pose

```bash
# วิธี A: ใช้ RViz → 2D Pose Estimate button (แนะนำ)

# วิธี B: CLI
ros2 run amr_tools set_initial_pose --x 0.0 --y 0.0 --yaw 0.0 --repeat 5

# ตรวจสอบ TF
ros2 run tf2_ros tf2_echo map odom
```

---

## 7. การใช้งาน

### 7.1 ตรวจสอบ Hardware (ก่อนเริ่มทุกครั้ง)

```bash
# เริ่ม hardware stack
~/agv_ws/scripts/agv.sh ros2 launch amr_hardware_bringup hardware.launch.py \
  lidar_port:=/dev/lidar lidar_product:=LDLiDAR_STL27L \
  imu_port:=/dev/imu base_angular_sign:=1.0

# ตรวจสอบ
~/agv_ws/scripts/check_topics.sh

# ผลที่ควรได้:
# [OK] /scan present
# [OK] /odom present
# [OK] /imu/data present
```

### 7.2 เฟส 1: สร้างแผนที่ (SLAM Mapping)

**เริ่ม SLAM:**
```bash
ros2 launch amr_bringup agv_mapping.launch.py \
  lidar_product:=LDLiDAR_STL27L \
  lidar_baud:=921600 \
  lidar_port:=/dev/lidar \
  base_angular_sign:=1.0
```

**บังคับหุ่นยนต์สำรวจ** (Teleoperate ด้วย PS4 หรือ cmd_vel):
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" --rate 10
```

**เคล็ดลับ Mapping:**
- ขับไป-กลับหลายรอบเพื่อปิด loop
- หมุน 360° เป็นระยะเพื่อลด drift
- รอ 2–5 วินาทีระหว่างการเคลื่อนที่หลักๆ

**บันทึกแผนที่:**
```bash
~/agv_ws/scripts/save_fixed_map.sh corridor_v1

# Output:
# ~/agv_ws/maps/corridor_v1/corridor_v1.yaml
# ~/agv_ws/maps/corridor_v1/corridor_v1.pgm
# ~/agv_ws/maps/corridor_v1/FIXED_MAP.txt
```

> **สำคัญ:** ห้าม overwrite fixed map เด็ดขาด สร้างเวอร์ชันใหม่แทน (v2, v3, ...)

### 7.3 เฟส 2: Localization + บันทึก Waypoints

**เริ่ม Localization:**
```bash
ros2 launch amr_bringup agv_localization.launch.py \
  map:=~/agv_ws/maps/corridor_v1/corridor_v1.yaml \
  base_angular_sign:=1.0
```

**ตั้ง Initial Pose:**
```bash
ros2 run amr_tools set_initial_pose --x 0.0 --y 0.0 --yaw 0.0 --repeat 5
```

**บันทึก Waypoints:**
```bash
ros2 run amr_tools record_waypoints \
  --ros-args \
  -p output_path:=~/agv_ws/routes/corridor_v1_route.yaml \
  -p straight_interval_m:=1.0 \
  -p turn_interval_m:=0.3

# ขับหุ่นยนต์ตามเส้นทาง...
# กด Ctrl+C เมื่อเสร็จ → ไฟล์ YAML ถูกบันทึก
```

**พารามิเตอร์ Recording:**

| พารามิเตอร์ | ค่า Default | คำอธิบาย |
|------------|-------------|---------|
| `straight_interval_m` | 1.0 | ระยะห่างจุดบนทางตรง |
| `turn_interval_m` | 0.3 | ระยะห่างจุดบนทางโค้ง (ถี่กว่า) |
| `turning_curvature_threshold_rad_per_m` | 0.35 | threshold ตรวจจับโค้ง |
| `min_step_m` | 0.02 | ระยะขั้นต่ำก่อนรับจุดใหม่ |
| `sample_rate_hz` | 20.0 | ความถี่ดึง TF |

### 7.4 เฟส 3: Navigation + Route Following

**เริ่ม Full Navigation:**
```bash
ros2 launch amr_bringup agv_nav.launch.py \
  map:=~/agv_ws/maps/corridor_v1/corridor_v1.yaml \
  base_angular_sign:=1.0 \
  use_yield_requester:=true
```

**ตรวจสอบว่า Navigation พร้อม:**
```bash
# Nav2 lifecycle ต้องเป็น active [3]
ros2 lifecycle get /velocity_smoother

# ตรวจ action server
ros2 action list | grep navigate
```

**วิ่งตาม Route:**

```bash
# วิ่งครั้งเดียว
ros2 run amr_tools route_loop_runner \
  --route ~/agv_ws/routes/corridor_v1_route.yaml \
  --yaw-mode heading \
  --start-mode nearest

# วิ่งวนไม่จบ
ros2 run amr_tools route_loop_runner \
  --route ~/agv_ws/routes/corridor_v1_route.yaml \
  --loop \
  --yaw-mode heading \
  --start-mode nearest \
  --pause 3.0

# วิ่งไป-กลับ 5 รอบ
ros2 run amr_tools route_loop_runner \
  --route ~/agv_ws/routes/corridor_v1_route.yaml \
  --roundtrips 5 \
  --yaw-mode heading \
  --start-mode nearest
```

**Route Runner Options:**

| Option | ค่า | คำอธิบาย |
|--------|-----|---------|
| `--yaw-mode` | `heading` | หันหน้าไปทาง waypoint ถัดไป (smooth) |
| | `from_file` | ใช้ yaw ที่บันทึกไว้ |
| | `zero` | ไม่สนใจ yaw |
| `--start-mode` | `nearest` | เริ่มจาก waypoint ที่ใกล้ที่สุด |
| | `fixed` | เริ่มจากจุดแรก |
| `--pause` | วินาที | หยุดพักระหว่าง waypoint |
| `--loop` | — | วนไม่จบ |
| `--reverse` | — | วิ่งย้อนกลับ |
| `--roundtrips` | N | วิ่งไป-กลับ N รอบ |
| `--stop-on-failure` | — | หยุดเมื่อ navigation fail (default) |

### 7.5 หยุดทั้งหมด

```bash
# หยุด ROS process ทั้งหมด
~/agv_ws/scripts/kill_ros.sh
```

---

## 8. Web UI

### 8.1 การเข้าใช้งาน

- **URL:** `http://192.168.1.41:8090`
- **API Server:** `http://192.168.1.41:8088`
- เบราว์เซอร์จะ auto-detect IP จาก `window.location.hostname`

### 8.2 ส่วนประกอบหน้าจอ

#### Connection Panel
- กำหนด API Base URL และ API Key (ถ้ามี)
- ปุ่ม **Check Health** — ตรวจสอบ API Server
- ปุ่ม **Connect Events** — เชื่อมต่อ SSE สำหรับ realtime update
- แสดงสถานะ: API Health, SSE Connection

#### Mission State Dashboard
แสดงสถานะ 5 component แบบ realtime:
- **SLAM** — กำลัง mapping หรือไม่
- **Localization** — AMCL ทำงานหรือไม่
- **Navigation** — Nav2 ทำงานหรือไม่
- **Waypoint Record** — กำลังบันทึก waypoint หรือไม่
- **Route Follow** — กำลังวิ่งตาม route หรือไม่

#### Hardware Status Grid
แสดงสถานะ hardware 5 ตัว:
- **LiDAR** — ONLINE / NO DATA / OFFLINE
- **IMU** — ONLINE / OFFLINE
- **Encoder** — ONLINE / OFFLINE
- **3D Camera** — ONLINE / OFFLINE
- **Battery** — NORMAL / LOW / CRITICAL / OFFLINE + แสดง % และ V

#### Battery Tuning Panel
ปรับค่า calibration แบบ realtime:
- **Voltage Scale** — ตัวคูณ calibration (default: 2.547)
- **V Min @ 0%** — แรงดันต่ำสุด (default: 22.4V)
- **V Max @ 100%** — แรงดันสูงสุด (default: 29.2V)
- **Percentage Mode** — `lifepo4_8s` (OCV curve) หรือ `linear`
- ปุ่ม **Read** / **Apply**

#### Mission Presets
ปุ่มลัดสำหรับ workflow ที่ใช้บ่อย:
- **Start Mapping** — เริ่ม SLAM ด้วย preset map name
- **Save Fixed Map** — บันทึกแผนที่
- **Start Delivery Mission** — เริ่ม nav + route following
- **Stop All Mission** — หยุดทั้งหมด

#### SLAM & Map Management
- กำหนด LiDAR product, baudrate, port
- ปุ่ม Start/Stop SLAM, Save Map
- แสดงรายการ map ทั้งหมด (chip selector)

#### Waypoints Management
- กำหนด output path, route file, yaw mode, start mode, pause
- ปุ่ม Start/Stop Record, Start/Stop Route
- แสดงรายการ route ทั้งหมด (chip selector)

#### Localization & Navigation
- เลือก map, toggle yield requester
- ปุ่ม Start/Stop Localization, Start/Stop Navigation

#### System Tools
- **Get Status** — ดูสถานะ process
- **Validate Config** — ตรวจสอบ YAML
- **Kill ROS Processes** — หยุด ROS ทั้งหมด

#### Log Panels (ซ่อนเป็น default, กด view เพื่อดู)
- **Realtime Events** — SSE events แบบ realtime
- **API Response Log** — JSON response จาก API

### 8.3 Web Camera & Detection UI

- **URL:** `http://192.168.1.41:8091` (web_camera) หรือผ่าน web_rviz
- เชื่อมต่อผ่าน rosbridge WebSocket (port 9090)
- แสดง camera feed แบบ realtime + bounding boxes + confidence
- ปรับ display options: refresh rate, quality, min confidence
- แสดงสถิติ detection: frame count, FPS, latency, per-class counts

---

## 9. REST API Reference

### Base URL: `http://192.168.1.41:8088/api/v1`

### Authentication (Optional)
- ตั้ง environment: `AGV_API_KEY=<secret>`
- ส่ง header: `X-API-Key: <secret>`
- SSE: `/api/v1/events?api_key=<secret>`

### GET Endpoints

| Endpoint | คำอธิบาย | Response |
|----------|---------|----------|
| `/health` | ตรวจสอบ API | `{ok, time_utc, workspace, status}` |
| `/status` | สถานะ processes | `[{pid, uptime, log_path, command}]` |
| `/node_health` | สถานะ hardware nodes | `{lidar, imu, encoder, camera, battery}` |
| `/battery` | สถานะ battery | `{voltage, percentage, state, thresholds}` |
| `/battery/params` | ค่า calibration | `{voltage_scale, battery_voltage_min/max, percentage_mode}` |
| `/maps` | รายการ map | `[{name, yaml_path, fixed}]` |
| `/routes` | รายการ route | `[{name, yaml_path}]` |
| `/events` | SSE stream | realtime events (ดูด้านล่าง) |

### POST Endpoints

| Endpoint | Body | คำอธิบาย |
|----------|------|---------|
| `/slam/start` | `{lidar_product, lidar_baud, lidar_port, base_angular_sign}` | เริ่ม SLAM |
| `/slam/stop` | `{}` | หยุด SLAM |
| `/slam/save_map` | `{map_name, timeout_sec?}` | บันทึก map |
| `/localization/start` | `{map, base_angular_sign}` | เริ่ม AMCL |
| `/localization/stop` | `{}` | หยุด AMCL |
| `/navigation/start` | `{map, base_angular_sign, use_yield_requester}` | เริ่ม Nav2 |
| `/navigation/stop` | `{}` | หยุด Nav2 |
| `/waypoints/record/start` | `{output_path?, map_frame?, base_frame?}` | เริ่มบันทึก waypoints |
| `/waypoints/record/stop` | `{}` | หยุดบันทึก |
| `/route/start` | `{route, yaw_mode, start_mode, pause_sec, loop?, reverse?, roundtrips?}` | เริ่ม route |
| `/route/stop` | `{}` | หยุด route |
| `/config/validate` | `{strict?, timeout_sec?}` | ตรวจสอบ config |
| `/system/kill_ros` | `{}` | Kill ROS ทั้งหมด |
| `/battery/params` | `{voltage_scale?, battery_voltage_min/max?, percentage_mode?}` | ปรับค่า battery |

### SSE Event Types

| Event | Payload | เมื่อไร |
|-------|---------|--------|
| `snapshot` | status, mission_state, node_health, battery | เชื่อมต่อครั้งแรก |
| `heartbeat` | status, mission_state, node_health, battery | ทุก 2 วินาที |
| `mission_state_changed` | component, active, reason | สถานะเปลี่ยน |
| `map_saved` | map_name, map_yaml, map_pgm | save map สำเร็จ |
| `map_save_failed` | map_name, exit_code, stderr | save map ล้มเหลว |
| `route_progress` | remaining_poses, distance_remaining_m | ระหว่าง route |
| `process_started` | name, pid, log_path | process เริ่ม |
| `process_stopped` | name, exit_code | process หยุดปกติ |
| `process_exited` | name, exit_code | process exit |

### ตัวอย่างการใช้ API

```bash
# ตรวจสอบสถานะ
curl http://192.168.1.41:8088/api/v1/health

# รายการ map
curl http://192.168.1.41:8088/api/v1/maps

# เริ่ม mapping
curl -X POST http://192.168.1.41:8088/api/v1/slam/start \
  -H 'Content-Type: application/json' \
  -d '{"lidar_product": "LDLiDAR_STL27L", "lidar_baud": 921600}'

# บันทึก map
curl -X POST http://192.168.1.41:8088/api/v1/slam/save_map \
  -H 'Content-Type: application/json' \
  -d '{"map_name": "corridor_v1"}'

# เริ่ม navigation
curl -X POST http://192.168.1.41:8088/api/v1/navigation/start \
  -H 'Content-Type: application/json' \
  -d '{"map": "corridor_v1"}'

# เริ่ม route
curl -X POST http://192.168.1.41:8088/api/v1/route/start \
  -H 'Content-Type: application/json' \
  -d '{"route": "corridor_v1_route.yaml", "loop": true, "yaw_mode": "heading"}'

# SSE realtime events
curl http://192.168.1.41:8088/api/v1/events
```

---

## 10. Joystick Teleoperation

### PS4 Controller Setup

**Launch:**
```bash
ros2 launch ps4_teleop_bringup ps4_teleop.launch.py
```

**การควบคุม:**

| ปุ่ม/แกน | หน้าที่ |
|----------|--------|
| **R1** (กดค้าง) | เปิดใช้งาน (Enable) — ต้องกดค้างตลอดเวลาบังคับ |
| **L1** (กดค้าง) | Turbo mode (ความเร็วสูง) |
| **Left Stick ↕** | เดินหน้า/ถอยหลัง (linear) |
| **Right Stick ↔** | หมุนซ้าย/ขวา (angular) |

**ความเร็ว:**

| โหมด | Linear | Angular |
|------|--------|---------|
| Normal (R1) | 0.20 m/s | 0.70 rad/s |
| Turbo (R1+L1) | 0.35 m/s | 1.20 rad/s |

**Mux Priority** (ps4_mux_bringup):
- Joystick cmd_vel มี priority สูงกว่า navigation
- กดปุ่มบน PS4 จะ override Nav2 ทันที (safety)

---

## 11. Configuration Reference

### 11.1 Nav2 Navigation (`nav2_agv_default.yaml`)

#### Controller (Regulated Pure Pursuit)

| พารามิเตอร์ | ค่า | คำอธิบาย |
|------------|-----|---------|
| `desired_linear_vel` | 0.18 m/s | ความเร็วเป้าหมาย |
| `lookahead_dist` | 0.5 m | ระยะมองไปข้างหน้า |
| `min_lookahead_dist` | 0.35 m | ลดลงเมื่ออยู่ใกล้โค้ง |
| `max_lookahead_dist` | 0.9 m | เพิ่มบนทางตรง |
| `use_rotate_to_heading` | false | ไม่หมุนในที่ (AGV-style) |
| `controller_frequency` | 12 Hz | |

#### Goal Checker

| พารามิเตอร์ | ค่า | คำอธิบาย |
|------------|-----|---------|
| `xy_goal_tolerance` | 0.15 m | ระยะยอมรับถึง goal |
| `yaw_goal_tolerance` | 3.14 rad | ไม่สนใจ yaw (≈180°) |

#### Velocity Smoother

| พารามิเตอร์ | ค่า |
|------------|-----|
| `max_velocity` | [0.18, 0.0, 1.2] m/s |
| `min_velocity` | [-0.18, 0.0, -1.2] m/s |
| `max_accel` | [0.8, 0.0, 3.0] m/s² |
| `max_decel` | [-0.8, 0.0, -3.0] m/s² |

#### Costmap

| พารามิเตอร์ | ค่า | คำอธิบาย |
|------------|-----|---------|
| `robot_radius` | 0.22 m | ใช้ใน costmap inflation |
| `footprint` | `[[0.42,0.26],...,[-0.32,0.26]]` | รูปร่างจริงของหุ่นยนต์ |
| `resolution` (global) | 0.05 m | ละเอียดแผนที่ |
| `resolution` (local) | 0.05 m | |

#### Behavior Trees (AGV-style no spin)

| BT | ไฟล์ |
|----|------|
| Navigate to Pose | `nav_to_pose_agv_like_no_spin.xml` |
| Navigate through Poses | `follow_waypoints_agv_like_no_spin.xml` |

Recovery behaviors: **backup, wait, clear costmaps** (ไม่มี spin)

### 11.2 AMCL Localization

| พารามิเตอร์ | ค่า |
|------------|-----|
| `max_particles` | 1500 |
| `min_particles` | 500 |
| `laser_model_type` | likelihood_field |
| `robot_model_type` | DifferentialMotionModel |
| `alpha1–5` | 0.2 (odometry error models) |
| `update_min_d` | 0.15 m |
| `update_min_a` | 0.2 rad |

### 11.3 SLAM Toolbox

| พารามิเตอร์ | ค่า |
|------------|-----|
| Mode | `async_slam_toolbox_node` (mapping) |
| `resolution` | 0.05 m |
| `max_laser_range` | 12.0 m |
| `minimum_travel_distance` | 0.2 m |
| `minimum_travel_heading` | 0.2 rad |

### 11.4 Scan Filter Profiles

**Navigation (`scan_filter.yaml`):**

| Filter | ค่า | หน้าที่ |
|--------|-----|--------|
| `remove_robot_self_hits` | box: x[-0.65,0.85], y[-0.55,0.55] | ตัด laser โดนตัวเอง |
| `keep_front_corridor` | box: x[-1.0,8.0], y[-2.2,2.2] | โฟกัสทางข้างหน้า |
| `range_filter` | 0.10–8.0 m | จำกัดระยะ |

**Mapping (`scan_filter_mapping.yaml`):**

| Filter | ค่า | หน้าที่ |
|--------|-----|--------|
| `remove_robot_self_hits` | เหมือน nav | ตัด laser โดนตัวเอง |
| `range_filter` | 0.10–12.0 m | ระยะกว้างกว่าสำหรับ loop closure |

### 11.5 Topic Health Monitor

| พารามิเตอร์ | ค่า |
|------------|-----|
| Topics ที่ตรวจ | `/scan`, `/imu/data`, `/odom` |
| `timeout_sec` | 1.5 |
| `check_period_sec` | 0.5 |
| `fail_hard` | false (soft warning) หรือ true (exit ถ้า stale) |
| `fail_grace_sec` | 8.0 |

### 11.6 Yield Requester

| พารามิเตอร์ | ค่า Default |
|------------|-------------|
| `--min-distance-m` | 0.60 |
| `--front-angle-deg` | 25.0 |
| `--hold-sec` | 1.5 |
| `--cooldown-sec` | 6.0 |
| `--sound-mode` | beep |

### 11.7 Battery LiFePO4 8S OCV Curve

SOC (State of Charge) จาก 13-point interpolation:

| Voltage | SOC |
|---------|-----|
| 22.4V | 0% |
| 24.0V | 5% |
| 25.6V | 12% |
| 26.0V | 22% |
| 26.2V | 32% |
| 26.3V | 42% |
| 26.4V | 52% |
| 26.5V | 62% |
| 26.6V | 72% |
| 26.8V | 82% |
| 27.0V | 90% |
| 27.6V | 97% |
| 28.4V | 100% |

---

## 12. Troubleshooting

### 12.1 Decision Tree

```
หุ่นยนต์ไม่เคลื่อน?
├─ /cmd_vel มี publish ไหม?
│  └─ ไม่มี → ส่ง nav goal หรือ teleoperate
├─ /odom อัปเดตไหม?
│  └─ ไม่ → เริ่ม hardware.launch.py
└─ Motor ไม่ตอบสนอง?
   └─ reboot หุ่นยนต์

Navigation drift / หลง?
├─ AMCL หลุด?
│  └─ set_initial_pose ใหม่
├─ Map เก่า?
│  └─ สร้าง map ใหม่
└─ Sensor noisy?
   └─ ตรวจสาย I2C/Serial

เห็น collision ทั้งที่ทางว่าง?
├─ ดู /scan_raw ใน RViz
│  ├─ เห็น robot posts? → ขยาย scan filter box
│  └─ เห็น phantom obstacles? → ตรวจ reflections
└─ Lidar miscalibrated? → ขยาย filter bounds

Web API ไม่ตอบ?
├─ systemctl --user status agv-web-api.service
├─ journalctl --user -u agv-web-api.service
└─ lsof -i :8088 (port ถูกใช้ไหม)

Battery แสดง --% / -- V?
├─ ตรวจ I2C: i2cdetect -y 7
├─ ตรวจ topic: ros2 topic echo /battery_state --once
└─ ตรวจ voltage_scale parameter
```

### 12.2 Hardware Diagnostics

**LiDAR:**
```bash
# ตรวจสอบ port
ls -la /dev/lidar

# ตรวจสอบ topic rate
ros2 topic hz /scan    # ควรได้ ~10 Hz

# ถ้าไม่มี topic → ลอง unplug/replug USB
```

**IMU:**
```bash
ls -la /dev/imu
ros2 topic hz /imu/data    # ควรได้ ~100 Hz
```

**Motor:**
```bash
# ทดสอบ command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --rate 10

# ตรวจ odometry
ros2 topic echo /odom
```

**Battery:**
```bash
sudo i2cdetect -y 7       # ควรเห็น 40
ros2 topic echo /battery_state --once
```

### 12.3 Navigation Issues

**"Regulated Pure Pursuit detected collision ahead":**
- scan filter ไม่ตัด self-hits → ขยาย box ใน `scan_filter.yaml`

**"Failed to make progress" / ABORTED:**
- ตรวจ velocity_smoother lifecycle: `ros2 lifecycle get /velocity_smoother`
- clear costmaps: Nav2 recovery ทำอัตโนมัติ
- เพิ่ม controller_patience

**Robot overshoots goal:**
- ลด xy_goal_tolerance: 0.10 m
- เพิ่ม waypoint density (re-record ด้วย interval ถี่ขึ้น)

### 12.4 AMCL Issues

**AMCL particles กระจาย / map→odom ไม่มี:**
```bash
# ตรวจ lifecycle
ros2 lifecycle get /amcl

# set initial pose ใหม่
ros2 run amr_tools set_initial_pose --x <x> --y <y> --yaw <yaw>

# ตรวจ TF tree
ros2 run tf2_ros tf2_echo map odom
```

### 12.5 ROS Cleanup

```bash
# หยุด ROS ทั้งหมด
~/agv_ws/scripts/kill_ros.sh

# Reset ROS daemon
ros2 daemon stop && ros2 daemon start

# ตรวจ orphan
ros2 topic list
ros2 node list
```

---

## 13. ROS Topic & TF Reference

### Sensor Topics

| Topic | Type | Publisher | Rate |
|-------|------|----------|------|
| `/scan_raw` | LaserScan | ldlidar | 10 Hz |
| `/scan` | LaserScan | scan_filter | 10 Hz |
| `/imu/data` | Imu | cmp10a_driver | 100 Hz |
| `/odom` | Odometry | zlac8015d_node | varies |
| `/battery_state` | BatteryState | battery_node | 2 Hz |

### Navigation Topics

| Topic | Type | หน้าที่ |
|-------|------|--------|
| `/cmd_vel` | Twist | คำสั่งความเร็ว (หลัง smoother) |
| `/cmd_vel_nav` | Twist | คำสั่งจาก controller (ก่อน smoother) |
| `/plan` | Path | global path plan |
| `/local_plan` | Path | local plan |
| `/amcl_pose` | PoseWithCovarianceStamped | ตำแหน่งจาก AMCL |
| `/map` | OccupancyGrid | แผนที่ |
| `/initialpose` | PoseWithCovarianceStamped | ตั้ง AMCL pose |

### Health Topics

| Topic | Type | หน้าที่ |
|-------|------|--------|
| `/amr/health_ok` | Bool | สถานะรวม hardware |
| `/amr/health_detail` | String | รายละเอียดปัญหา |
| `/diagnostics` | DiagnosticArray | diagnostic ละเอียด |

### Nav2 Actions

| Action | Server | หน้าที่ |
|--------|--------|--------|
| `NavigateToPose` | `/bt_navigator` | นำทางไปจุดเดียว |
| `NavigateThroughPoses` | `/bt_navigator` | นำทางผ่านหลายจุด |

### Nav2 Lifecycle Nodes

```bash
ros2 lifecycle get /map_server          # [3] = active
ros2 lifecycle get /amcl                # [3] = active
ros2 lifecycle get /controller_server   # [3] = active
ros2 lifecycle get /planner_server      # [3] = active
ros2 lifecycle get /behavior_server     # [3] = active
ros2 lifecycle get /velocity_smoother   # [3] = active
```

State: `[1]` unconfigured → `[2]` inactive → `[3]` active → `[4]` finalized

---

## 14. Command Reference

### Launch Commands

```bash
# Source environment
source /opt/ros/humble/setup.bash
source ~/agv_ws/install/setup.bash

# Hardware only
ros2 launch amr_hardware_bringup hardware.launch.py

# SLAM Mapping
ros2 launch amr_bringup agv_mapping.launch.py \
  lidar_product:=LDLiDAR_STL27L lidar_baud:=921600 lidar_port:=/dev/lidar

# Localization only
ros2 launch amr_bringup agv_localization.launch.py \
  map:=~/agv_ws/maps/<name>/<name>.yaml

# Full Navigation
ros2 launch amr_bringup agv_nav.launch.py \
  map:=~/agv_ws/maps/<name>/<name>.yaml
```

### Utility Commands

```bash
# บันทึก waypoints
ros2 run amr_tools record_waypoints \
  --ros-args -p output_path:=~/agv_ws/routes/route.yaml

# วิ่งตาม route
ros2 run amr_tools route_loop_runner \
  --route ~/agv_ws/routes/route.yaml --yaw-mode heading --loop

# ตั้ง initial pose
ros2 run amr_tools set_initial_pose --x 0.0 --y 0.0 --yaw 0.0

# ตรวจ hardware
~/agv_ws/scripts/check_topics.sh

# ตรวจ config
ros2 run amr_tools validate_configs --workspace ~/agv_ws

# Kill ROS
~/agv_ws/scripts/kill_ros.sh
```

### Debug Commands

```bash
# Topics
ros2 topic list
ros2 topic echo /scan
ros2 topic hz /scan

# Parameters
ros2 param list
ros2 param get /battery_node voltage_scale
ros2 param set /battery_node voltage_scale 2.547

# Nodes
ros2 node list
ros2 node info /ldlidar

# TF
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames.py

# RViz
rviz2 -d ~/agv_ws/src/amr_description/rviz/default.rviz
```

### Shell Scripts

| Script | หน้าที่ | ตัวอย่าง |
|--------|--------|---------|
| `agv.sh` | รัน command ใน agv_ws environment | `agv.sh ros2 topic list` |
| `use_agv_ws.sh` | เปิด shell ที่ source agv_ws | `./use_agv_ws.sh` |
| `save_fixed_map.sh` | บันทึก SLAM map | `./save_fixed_map.sh corridor_v1` |
| `kill_ros.sh` | Kill ROS process ทั้งหมด | `./kill_ros.sh` |
| `check_topics.sh` | ตรวจ critical topics | `./check_topics.sh` |
| `test_encoder_distance.py` | calibrate encoder | `python3 test_encoder_distance.py --distance 1.0` |
| `install_web_services.sh` | ติดตั้ง systemd services | `./install_web_services.sh` |

### Service Management

```bash
# ดูสถานะ
systemctl --user status agv-web-api.service

# เริ่ม/หยุด/restart
systemctl --user start agv-web-api.service
systemctl --user stop agv-web-api.service
systemctl --user restart agv-web-api.service

# ดู log
journalctl --user -u agv-web-api.service -f

# เปิด/ปิด auto-start
systemctl --user enable agv-web-api.service
systemctl --user disable agv-web-api.service
```

---

## 15. File Structure

```
~/agv_ws/
├── scripts/                              # Utility scripts
│   ├── agv.sh                           # Clean environment runner
│   ├── use_agv_ws.sh                    # Interactive agv_ws shell
│   ├── check_topics.sh                  # Hardware preflight check
│   ├── kill_ros.sh                      # ROS process killer
│   ├── save_fixed_map.sh               # Map persistence (YAML+PGM+marker)
│   ├── install_web_services.sh          # Systemd service installer
│   └── test_encoder_distance.py         # Wheel calibration tool
│
├── systemd/user/                         # Systemd user services
│   ├── agv-web-api.service              # REST API (port 8088)
│   ├── agv-web-ui.service               # Web UI (port 8090)
│   ├── agv-web-rviz.service             # Web RViz (port 8091)
│   └── agv-rosbridge.service            # rosbridge (port 9090)
│
├── maps/                                 # Saved SLAM maps
│   ├── corridor_fixed_v1/               # ตัวอย่าง:
│   │   ├── corridor_fixed_v1.yaml       #   metadata
│   │   ├── corridor_fixed_v1.pgm        #   occupancy grid image
│   │   └── FIXED_MAP.txt                #   lock marker
│   └── ...
│
├── routes/                               # Waypoint route files (YAML)
│   ├── corridor_fixed_v1_waypoints.yaml
│   ├── A-Fl1.yaml
│   └── ...
│
├── web_ui/                               # AGV Ops Console (port 8090)
│   ├── index.html
│   ├── app.js
│   └── styles.css
│
├── web_camera/                           # Camera & Detection UI
│   ├── index.html
│   ├── app.js
│   └── styles.css
│
├── web_rviz/                             # Web RViz (port 8091)
│   ├── index.html
│   ├── app.js
│   └── styles.css
│
├── src/
│   ├── amr_bringup/                     # Launch entry points
│   │   ├── launch/
│   │   │   ├── agv_mapping.launch.py    #   SLAM mapping workflow
│   │   │   ├── agv_localization.launch.py #  AMCL localization
│   │   │   └── agv_nav.launch.py        #   Full navigation
│   │   └── config/
│   │       ├── scan_filter.yaml         #   Navigation scan filter
│   │       ├── scan_filter_mapping.yaml #   Mapping scan filter
│   │       └── slam_toolbox_mapping.yaml #  SLAM params
│   │
│   ├── amr_navigation/                  # Nav2 configuration
│   │   └── config/
│   │       ├── nav2_agv_default.yaml    #   Full nav2 params
│   │       ├── nav2_localization.yaml   #   AMCL-only params
│   │       ├── nav_to_pose_agv_like_no_spin.xml     # BT
│   │       └── follow_waypoints_agv_like_no_spin.xml # BT
│   │
│   ├── amr_tools/                       # Utility nodes + Web API
│   │   └── amr_tools/
│   │       ├── web_api_server.py        #   REST API server
│   │       ├── route_loop_runner.py     #   Route follower
│   │       ├── record_waypoints.py      #   Waypoint recorder
│   │       ├── set_initial_pose.py      #   AMCL pose setter
│   │       ├── yield_requester.py       #   Obstacle beeper
│   │       ├── topic_health_monitor.py  #   Sensor watchdog
│   │       ├── preflight_topics.py      #   Startup check
│   │       └── validate_configs.py      #   Config validator
│   │
│   ├── amr_description/                 # Robot model (URDF)
│   │   └── urdf/
│   │       └── agv_base.urdf.xacro
│   │
│   └── amr_hardware/                    # Hardware drivers
│       ├── amr_hardware_bringup/
│       │   └── launch/
│       │       └── hardware.launch.py   #   All hardware launch
│       ├── amr_battery_driver/
│       │   └── amr_battery_driver/
│       │       └── battery_node.py      #   INA219 I2C + LiFePO4 curve
│       └── amr_imu_driver/
│           └── amr_imu_driver/
│               └── cmp10a_driver.py     #   CMP10A serial IMU
│
├── install/                             # colcon build output
├── build/                               # build intermediates
├── log/                                 # ROS logs
│
├── README.md
├── README_AGV_WS.md
├── README_MAPPING_WAYPOINTS.md
├── README_WEB_API.md
├── README_WEB_UI.md
├── README_WEB_RVIZ.md
├── feature.md                           # Feature summary
└── system_manual.md                     # คู่มือนี้
```

---

## Port Summary

| Port | Service | Protocol | เข้าถึง |
|------|---------|----------|--------|
| 8088 | REST API Server | HTTP | `http://192.168.1.41:8088` |
| 8090 | Web UI Dashboard | HTTP | `http://192.168.1.41:8090` |
| 8091 | Web RViz / Camera | HTTP | `http://192.168.1.41:8091` |
| 9090 | rosbridge | WebSocket | `ws://192.168.1.41:9090` |

---

## Critical Parameters Quick Reference

| Component | Parameter | Default | ปรับอย่างไร |
|-----------|----------|---------|------------|
| Wheel | radius | 0.10 m | `test_encoder_distance.py` |
| Motor | angular_sign | 1.0 | -1.0 ถ้าหมุนผิดทาง |
| Nav2 | desired_linear_vel | 0.18 m/s | เพิ่มเป็น 0.25 สำหรับเร็วขึ้น |
| Nav2 | lookahead_dist | 0.5 m | 0.7 smooth / 0.35 tight |
| Nav2 | xy_goal_tolerance | 0.15 m | ลดเป็น 0.10 สำหรับแม่นยำ |
| Smoother | max_velocity | [0.18, 0, 1.2] | ลดเป็น 0.15 สำหรับพื้นที่แคบ |
| Scan Filter | self-hits box | ±0.55, ±0.65 m | ขยายถ้า Nav2 เห็น robot posts |
| AMCL | max_particles | 1500 | เพิ่มสำหรับ map ใหญ่ |
| Battery | voltage_scale | 2.547 | calibrate ด้วย multimeter |
| Battery | percentage_mode | lifepo4_8s | linear สำหรับ battery ประเภทอื่น |
| Yield | min_distance | 0.60 m | เพิ่มเป็น 0.80 สำหรับระยะห่างมากขึ้น |

---

> **Version:** 1.0 | **Branch:** `agv-ros-webx` | **Repo:** `ezeman/amr-robot`  
> **Generated:** 2026-03-17
