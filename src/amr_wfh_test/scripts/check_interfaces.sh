#!/usr/bin/env bash
set -euo pipefail

echo "Checking /odom and /cmd_vel interfaces..."
ros2 topic list | grep -E '/odom|/cmd_vel' || {
  echo "No /odom or /cmd_vel topics found. Ensure hardware drivers are running."
}

echo "Sample odom message:"
if ros2 topic echo -n 1 /odom 2>/dev/null; then
  echo "Odom received successfully."
else
  echo "Failed to read /odom. Is the base driver publishing?"
fi

echo "If /cmd_vel is missing, ensure the base driver subscribes to /cmd_vel."
