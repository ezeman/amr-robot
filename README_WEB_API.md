# AGV Web API (ROS2)

REST API for controlling AGV workflows from external Web UI devices (tablet, drug cabinet system, etc.).

## Build

```bash
cd ~/agv_ws
colcon build --packages-select amr_tools
source install/setup.bash
```

## Run API Server

```bash
ros2 run amr_tools web_api_server --host 0.0.0.0 --port 8088 --workspace ~/agv_ws
```

Optional API key:

```bash
AGV_API_KEY=your_secret ros2 run amr_tools web_api_server --host 0.0.0.0 --port 8088
```

Then pass header:

```http
X-API-Key: your_secret
```

## Core Endpoints

- `GET /api/v1/health`
- `GET /api/v1/status`
- `GET /api/v1/maps`
- `GET /api/v1/routes`
- `GET /api/v1/events` (realtime stream for dashboard)

## Realtime Stream (Dashboard)

Use Server-Sent Events (SSE) from Web UI to receive live updates.

Browser example:

```javascript
const es = new EventSource('http://<robot-ip>:8088/api/v1/events');
es.addEventListener('snapshot', (e) => console.log('snapshot', JSON.parse(e.data)));
es.addEventListener('heartbeat', (e) => console.log('heartbeat', JSON.parse(e.data)));
es.addEventListener('mission_state_changed', (e) => console.log('mission', JSON.parse(e.data)));
es.addEventListener('map_saved', (e) => console.log('map_saved', JSON.parse(e.data)));
es.addEventListener('map_save_failed', (e) => console.log('map_save_failed', JSON.parse(e.data)));
es.addEventListener('route_progress', (e) => console.log('route_progress', JSON.parse(e.data)));
es.addEventListener('process_started', (e) => console.log('started', JSON.parse(e.data)));
es.addEventListener('process_stopped', (e) => console.log('stopped', JSON.parse(e.data)));
es.addEventListener('process_exited', (e) => console.log('exited', JSON.parse(e.data)));
```

Events include:
- `snapshot`: initial current process status
- `heartbeat`: periodic status updates
- `mission_state_changed`: mission lifecycle component changes (slam/localization/navigation/waypoint_record/route_follow)
- `map_saved`: emitted when `slam/save_map` succeeds
- `map_save_failed`: emitted when `slam/save_map` fails
- `route_progress`: route feedback from `route_loop_runner` logs (remaining poses, distance, recoveries)
- `process_started`
- `process_stopped`
- `process_exited`

### SLAM + Save Map

- `POST /api/v1/slam/start`
- `POST /api/v1/slam/stop`
- `POST /api/v1/slam/save_map`

Example:

```bash
curl -X POST http://<robot-ip>:8088/api/v1/slam/start \
  -H 'Content-Type: application/json' \
  -d '{"lidar_product":"LDLiDAR_STL27L","lidar_baud":921600,"lidar_port":"/dev/lidar"}'

curl -X POST http://<robot-ip>:8088/api/v1/slam/save_map \
  -H 'Content-Type: application/json' \
  -d '{"map_name":"cabinet_floor1_v1"}'
```

### Localization / Navigation

- `POST /api/v1/localization/start`
- `POST /api/v1/localization/stop`
- `POST /api/v1/navigation/start`
- `POST /api/v1/navigation/stop`

Example:

```bash
curl -X POST http://<robot-ip>:8088/api/v1/navigation/start \
  -H 'Content-Type: application/json' \
  -d '{"map":"cabinet_floor1_v1"}'
```

`map` can be:
- absolute YAML path
- map name (server resolves to `~/agv_ws/maps/<name>/<name>.yaml` or `~/agv_ws/maps/<name>.yaml`)

### Record Waypoints

- `POST /api/v1/waypoints/record/start`
- `POST /api/v1/waypoints/record/stop`

Example:

```bash
curl -X POST http://<robot-ip>:8088/api/v1/waypoints/record/start \
  -H 'Content-Type: application/json' \
  -d '{"output_path":"/home/narong/agv_ws/routes/cabinet_round_v1.yaml"}'
```

### Follow Existing Waypoints

- `POST /api/v1/route/start`
- `POST /api/v1/route/stop`

Example:

```bash
curl -X POST http://<robot-ip>:8088/api/v1/route/start \
  -H 'Content-Type: application/json' \
  -d '{"route":"cabinet_round_v1.yaml","loop":true,"yaw_mode":"heading","start_mode":"nearest"}'
```

### Other Useful APIs

- `POST /api/v1/config/validate`
- `POST /api/v1/system/kill_ros`

## Notes

- The server uses `~/agv_ws/scripts/agv.sh` to keep ROS environment clean.
- Long-running jobs are managed as background processes. Check `/api/v1/status` for process PID and log file path.
- Log files are stored in `~/agv_ws/log/api/`.
- Enable network firewall rules so only trusted devices can call this API.

## Web UI

Use the included dashboard UI in [web_ui/index.html](web_ui/index.html).
Quick start is documented in [README_WEB_UI.md](README_WEB_UI.md).
