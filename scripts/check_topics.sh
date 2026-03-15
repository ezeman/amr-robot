#!/usr/bin/env bash
set -euo pipefail

topics=("/scan" "/odom" "/cmd_vel" "/tf" "/tf_static")
echo "Checking required topics..."

list="$(ros2 topic list || true)"
if [[ -z "${list}" ]]; then
  echo "ros2 topic list returned no topics (is the system running?)"
fi

for t in "${topics[@]}"; do
  if echo "${list}" | grep -qx "${t}"; then
    echo "[OK] ${t} present"
  else
    echo "[MISSING] ${t}"
    continue
  fi

  # Attempt to sample the publish rate briefly.
  if command -v timeout >/dev/null 2>&1; then
    if timeout 3 ros2 topic hz -w 5 "${t}" > /tmp/check_topics_rate.txt 2>/dev/null; then
      rate_line=$(tail -n 1 /tmp/check_topics_rate.txt)
      echo "  rate: ${rate_line}"
    else
      echo "  rate: unavailable (no messages in sampling window)"
    fi
  fi
done
