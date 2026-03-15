# AGV Web RViz (Port 8091)

Web-based RViz dashboard for ROS2 over rosbridge.

## Features

- Live occupancy map (`/map`)
- Robot pose overlay:
  - AMCL pose (`/amcl_pose`)
  - Odom pose (`/wheel/odom`)
- Laser scan overlay (`/scan`)
- Global/local plan overlays (`/plan`, `/local_plan`)
- Pan/zoom map canvas
- View controls: Fit Map, Center Robot, Reset View
- Follow Robot mode for auto-centering the robot in view
- Click-to-set Initial Pose (`/initialpose`)
- Click-to-set Nav Goal (`/goal_pose`)
- Layer toggles
- Auto reconnect to rosbridge with configurable delay
- Health indicator based on topic freshness

## Run Web RViz

```bash
cd ~/agv_ws/web_rviz
python3 -m http.server 8091 --bind 0.0.0.0
```

Open:

- `http://<robot-ip>:8091/`

## Service (Auto-start)

This workspace includes `agv-web-rviz.service` for user-level systemd.

```bash
cd ~/agv_ws
./scripts/install_web_services.sh
systemctl --user status agv-web-rviz.service
```

## ROS Bridge Requirement

Web RViz uses WebSocket to rosbridge (default `ws://192.168.1.41:9090`).

The UI now auto-fills rosbridge URL from current page host as `ws://<current-host>:9090`.

Start rosbridge if needed:

```bash
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

If rosbridge runs on another host/port, change URL in the Web RViz connection panel.
