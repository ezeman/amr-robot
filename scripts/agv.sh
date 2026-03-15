#!/usr/bin/env bash
set -euo pipefail

# Run any command in a clean shell with ONLY ROS + agv_ws sourced.
# Example:
#   ~/agv_ws/scripts/agv.sh ros2 pkg prefix amr_bringup
#   ~/agv_ws/scripts/agv.sh ros2 launch amr_bringup agv_mapping.launch.py

AGV_WS="${AGV_WS:-$HOME/agv_ws}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <command...>" >&2
  exit 1
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "Missing ROS setup: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  exit 1
fi

if [[ ! -f "${AGV_WS}/install/setup.bash" ]]; then
  echo "Missing workspace setup: ${AGV_WS}/install/setup.bash" >&2
  exit 2
fi

# Optional preflight before critical bringup launches.
# Enabled by default; set AGV_PREFLIGHT_TOPICS=0 to bypass.
preflight_needed=0
if [[ "${AGV_PREFLIGHT_TOPICS:-1}" == "1" ]] && [[ $# -ge 4 ]]; then
  if [[ "$1" == "ros2" && "$2" == "launch" && "$3" == "amr_bringup" ]]; then
    case "$4" in
      agv_localization.launch.py|agv_nav.launch.py)
        preflight_needed=1
        ;;
    esac
  fi
fi

preflight_cmd=""
if [[ "$preflight_needed" == "1" ]]; then
  preflight_timeout="${AGV_PREFLIGHT_TIMEOUT_SEC:-8.0}"
  preflight_period="${AGV_PREFLIGHT_CHECK_PERIOD_SEC:-0.3}"
  preflight_cmd="\
    echo '[agv.sh] Running topic preflight check...'; \
    ros2 run amr_tools preflight_topics \
      --timeout-sec ${preflight_timeout} \
      --check-period-sec ${preflight_period}; \
  "
fi

quoted_cmd=""
for arg in "$@"; do
  quoted_cmd+="$(printf "%q " "$arg")"
done

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
    set -eo pipefail
    source /opt/ros/${ROS_DISTRO}/setup.bash
    source \"${AGV_WS}/install/setup.bash\"
    if [[ \"\${AGV_RESET_DAEMON:-0}\" == \"1\" ]]; then
      ros2 daemon stop >/dev/null 2>&1 || true
      ros2 daemon start >/dev/null 2>&1 || true
    fi
    ${preflight_cmd}
    exec ${quoted_cmd}
  "
