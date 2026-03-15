#!/usr/bin/env bash
set -euo pipefail

AGV_WS="${AGV_WS:-$HOME/agv_ws}"
SRC_DIR="$AGV_WS/systemd/user"
DST_DIR="$HOME/.config/systemd/user"

mkdir -p "$DST_DIR"
cp -f "$SRC_DIR/agv-web-api.service" "$DST_DIR/agv-web-api.service"
cp -f "$SRC_DIR/agv-web-ui.service" "$DST_DIR/agv-web-ui.service"
cp -f "$SRC_DIR/agv-web-rviz.service" "$DST_DIR/agv-web-rviz.service"
cp -f "$SRC_DIR/agv-rosbridge.service" "$DST_DIR/agv-rosbridge.service"

systemctl --user daemon-reload
systemctl --user enable --now agv-web-api.service
systemctl --user enable --now agv-web-ui.service
systemctl --user enable --now agv-web-rviz.service
systemctl --user enable --now agv-rosbridge.service

systemctl --user --no-pager --full status agv-web-api.service | sed -n '1,20p'
systemctl --user --no-pager --full status agv-web-ui.service | sed -n '1,20p'
systemctl --user --no-pager --full status agv-web-rviz.service | sed -n '1,20p'
systemctl --user --no-pager --full status agv-rosbridge.service | sed -n '1,20p'

echo "Installed and started user services:"
echo "  agv-web-api.service"
echo "  agv-web-ui.service"
echo "  agv-web-rviz.service"
echo "  agv-rosbridge.service"
