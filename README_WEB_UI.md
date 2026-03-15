# AGV Web UI

Web dashboard for operating AGV REST API from another device (tablet, drug cabinet UI, nurse station PC).

## Files

- `web_ui/index.html`
- `web_ui/styles.css`
- `web_ui/app.js`

## Run (Static)

```bash
cd ~/agv_ws/web_ui
python3 -m http.server 8090
```

Open browser:

- `http://<robot-ip>:8090`

Set API URL in UI:

- `http://<robot-ip>:8088`

## Authentication

If API server is started with `AGV_API_KEY`, enter same key in UI.

- REST calls send `X-API-Key` header
- SSE events use query string `?api_key=...`

## Features Included

- One-click Mission Presets:
	- Start Mapping
	- Save Fixed Map
	- Start Delivery Mission (navigation + route follow)
	- Stop All Mission
- Connection health checks
- Realtime events panel (`/api/v1/events`)
- Mission state dashboard
- SLAM start/stop + save map
- Localization start/stop
- Navigation start/stop
- Waypoint recording start/stop
- Route follow start/stop
- Map and route list quick selectors
- Config validation + kill ROS tools

## Recommended Network Setup

- Keep robot and operator tablet on trusted LAN/VLAN
- Restrict inbound ports to trusted subnets only
- Use API key in production

## Auto Start On Boot (systemd user services)

Install and start both services:

```bash
cd ~/agv_ws
./scripts/install_web_services.sh
```

Services:

- `agv-web-api.service` -> API on port `8088`
- `agv-web-ui.service` -> Web UI on port `8090`
- `agv-web-rviz.service` -> Web RViz on port `8091`
- `agv-rosbridge.service` -> rosbridge websocket on port `9090`

Useful commands:

```bash
systemctl --user status agv-web-api.service
systemctl --user status agv-web-ui.service
systemctl --user status agv-web-rviz.service
systemctl --user status agv-rosbridge.service
systemctl --user restart agv-web-api.service
systemctl --user restart agv-web-ui.service
systemctl --user restart agv-web-rviz.service
systemctl --user restart agv-rosbridge.service
```
