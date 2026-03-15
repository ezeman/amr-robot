# BOOKMARK: AGV Web/API Work Continuation

Updated: 2026-03-14
Workspace: /home/narong/agv_ws

## Current Status

- AGV Web API implemented in ROS2 package `amr_tools`.
- Web UI implemented under `web_ui/` and reachable on LAN.
- Realtime event stream (`/api/v1/events`) supported.
- Mission preset buttons added in Web UI:
  - Start Mapping
  - Save Fixed Map
  - Start Delivery Mission
  - Stop All Mission
- User-level systemd services created and enabled:
  - `agv-web-api.service` (port 8088)
  - `agv-web-ui.service` (port 8090)

## Access URLs

- Web UI: http://192.168.1.41:8090/
- API Health: http://192.168.1.41:8088/api/v1/health
- API Events: http://192.168.1.41:8088/api/v1/events

## Service Control Commands

```bash
systemctl --user status agv-web-api.service
systemctl --user status agv-web-ui.service
systemctl --user restart agv-web-api.service
systemctl --user restart agv-web-ui.service
```

## Key Files To Continue

- src/amr_tools/amr_tools/web_api_server.py
- web_ui/index.html
- web_ui/app.js
- README_WEB_API.md
- README_WEB_UI.md
- scripts/install_web_services.sh
- systemd/user/agv-web-api.service
- systemd/user/agv-web-ui.service

## Suggested Next Improvements

1. Add safety confirmation (PIN/confirm modal) for high-impact actions (`Stop All Mission`, `Kill ROS`).
2. Add tablet-focused operator mode (large controls, reduced panels).
3. Add preset profile persistence (save/load multiple mission presets).
4. Add stricter API auth (JWT or mTLS) if deployed in production network.
5. Add event filtering in UI (by type, severity, component).

## Quick Resume Checklist

1. Verify services are running.
2. Open Web UI URL from operator device.
3. Click `Check Health` and `Connect Events`.
4. Validate mission presets with a dry run.
5. Continue development from `web_ui/app.js` and `web_api_server.py`.
