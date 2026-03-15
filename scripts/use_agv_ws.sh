#!/usr/bin/env bash
set -euo pipefail

AGV_WS="${AGV_WS:-$HOME/agv_ws}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "Missing ROS setup: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  exit 1
fi

if [[ ! -f "${AGV_WS}/install/setup.bash" ]]; then
  echo "Missing workspace setup: ${AGV_WS}/install/setup.bash" >&2
  echo "Build first:" >&2
  echo "  cd ${AGV_WS} && colcon build --symlink-install" >&2
  exit 2
fi

# Start a clean interactive shell with ONLY ROS + agv_ws sourced.
# Keep minimal UI-related env vars so RViz2 can still work on desktop sessions.
exec env -i \
  HOME="$HOME" \
  USER="${USER:-narong}" \
  LOGNAME="${LOGNAME:-${USER:-narong}}" \
  TERM="${TERM:-xterm-256color}" \
  PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
  LANG="${LANG:-C.UTF-8}" \
  LC_ALL="${LC_ALL:-}" \
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-}" \
  DISPLAY="${DISPLAY:-}" \
  XAUTHORITY="${XAUTHORITY:-}" \
  XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" \
  WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" \
  bash --noprofile --norc -c "
    source /opt/ros/${ROS_DISTRO}/setup.bash
    source \"${AGV_WS}/install/setup.bash\"
    ros2 daemon stop >/dev/null 2>&1 || true
    ros2 daemon start >/dev/null 2>&1 || true
    echo \"[use_agv_ws] amr_bringup prefix: \$(ros2 pkg prefix amr_bringup)\"
    echo \"[use_agv_ws] Ready: run your ros2 commands IN THIS SHELL.\"
    echo \"[use_agv_ws] Note: if you 'exit', you return to your old shell (may use amr_ws again).\"
    exec bash --noprofile --norc
  "
