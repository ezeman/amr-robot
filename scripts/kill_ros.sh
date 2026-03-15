#!/usr/bin/env bash
set -euo pipefail

# Kill common ROS2 processes used in this workspace (bringup / nav2 / slam / hardware).
# Usage:
#   ~/agv_ws/scripts/kill_ros.sh          # graceful (SIGINT -> SIGTERM)
#   ~/agv_ws/scripts/kill_ros.sh --force  # SIGKILL at the end

force=0
if [[ "${1:-}" == "--force" ]]; then
  force=1
fi

self_pid="$$"
parent_pid="${PPID:-0}"

match_pids() {
  local regex="$1"
  ps -eo pid=,cmd= | awk -v re="$regex" -v self="$self_pid" -v parent="$parent_pid" '
    $0 ~ re {
      if ($1 != self && $1 != parent) print $1
    }'
}

patterns=(
  "ros2 launch "
  "component_container_isolated"
  "nav2_container"
  "slam_toolbox"
  "rviz2"
  "ldlidar_stl_ros2_node"
  "scan_to_scan_filter_chain"
  "cmp10a_driver"
  "zlac8015d_node"
)

pids=()
for re in "${patterns[@]}"; do
  while read -r pid; do
    [[ -z "$pid" ]] && continue
    pids+=("$pid")
  done < <(match_pids "$re" || true)
done

if [[ ${#pids[@]} -eq 0 ]]; then
  echo "[kill_ros] No matching ROS2 processes found."
  exit 0
fi

# unique
readarray -t pids < <(printf "%s\n" "${pids[@]}" | sort -n | uniq)

echo "[kill_ros] Killing PIDs: ${pids[*]}"
kill -INT "${pids[@]}" 2>/dev/null || true
sleep 2
kill -TERM "${pids[@]}" 2>/dev/null || true
sleep 1
if [[ "$force" == "1" ]]; then
  kill -KILL "${pids[@]}" 2>/dev/null || true
fi

exit 0

