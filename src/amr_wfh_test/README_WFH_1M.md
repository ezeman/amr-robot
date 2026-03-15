# Walk Forward Heading (WFH) 1m Test / ทดสอบเดินหน้า 1 เมตร

Simple 1-meter forward test for the diff-drive robot (Jetson Orin Nano, ROS 2 Humble). Two modes:
- Mode A: hardware-only using `/odom` + `/cmd_vel`
- Mode B: Nav2 DriveOnHeading action (requires behavior_server running)

## Build / คอมไพล์
```bash
cd ~/agv_ws
colcon build --symlink-install
source install/setup.bash
```

## Mode A (hardware-only) / โหมด A (ฮาร์ดแวร์อย่างเดียว)
- Requires `/odom` publishing and `/cmd_vel` subscriber (base driver running).
- Command:
```bash
ros2 launch amr_wfh_test wfh_mode_a_odom.launch.py
```
- Uses params in `amr_wfh_test/config/wfh_test.yaml` (target=1.0 m, 0.20 m/s).
- Optionally disable velocity smoother: `use_velocity_smoother:=false`
- Example result: reached 1.007 m in ~5.1 s at 0.20 m/s and stopped cleanly (publishes zero cmd_vel for 0.5 s).

### Mode A (Forward+Backward 1m x10 rounds) / โหมด A (เดินหน้า+ถอยหลัง 1m จำนวน 10 รอบ)
- English: forward 1.0 m then backward 1.0 m, repeat 10 rounds (20 legs total).
```bash
ros2 launch amr_wfh_test wfh_mode_a_fwd_back_1m_10rounds.launch.py
```
- ไทย: เดินหน้า 1 เมตร แล้วถอยหลัง 1 เมตร ทำซ้ำ 10 รอบ (รวม 20 ช่วง)
```bash
ros2 launch amr_wfh_test wfh_mode_a_fwd_back_1m_10rounds.launch.py
```
- Example result (tested): completed 20/20 legs successfully; total ~113 s; last leg reached 1.009 m in 5.70 s; finishes with zero cmd_vel for 0.50 s (settle).

## Mode B (Nav2 DriveOnHeading) / โหมด B (Nav2)
- Requires Nav2 `behavior_server` with DriveOnHeading plugin active (global_frame=odom recommended).
- Command:
```bash
ros2 launch amr_wfh_test wfh_mode_b_drive_on_heading.launch.py
```

## Verify distance / ตรวจสอบระยะ
- Watch logs: node reports progress and success/timeouts.
- Inspect odom delta:
```bash
ros2 topic echo -n 5 /odom | grep -E 'pose|frame_id'
```
- Target: traveled distance ≥ 1.0 m in odom frame, minimal yaw change (angular.z commanded = 0).

## Interface check / ตรวจสอบท็อปปิก
```bash
bash $(ros2 pkg prefix amr_wfh_test)/share/amr_wfh_test/scripts/check_interfaces.sh
```

## Common issues / ปัญหาที่พบบ่อย
- `/odom` missing or frozen: start base driver, confirm TF `odom -> base_link`.
- Frame IDs not `odom`/`base_link`: node will warn but still run; fix TF if drift is large.
- Robot does not move: check motor controller enable, power, and `/cmd_vel` subscriber.
- Timeout before 1 m: increase `max_duration_sec` or `linear_x` in the param file, verify wheel radius/odometry tuning.
- Nav2 mode fails: ensure `behavior_server` is up with DriveOnHeading plugin and `global_frame=odom`.

## Parameter file / ไฟล์พารามิเตอร์
- `config/wfh_test.yaml` holds WFH node defaults, velocity_smoother safety limits, and behavior_server settings for DriveOnHeading.
