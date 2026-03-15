#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <map_name>"
  echo "Example: $0 corridor_fixed_v1"
  exit 1
fi

MAP_NAME="$1"
ROOT_DIR="${HOME}/agv_ws/maps/${MAP_NAME}"
mkdir -p "${ROOT_DIR}"

echo "Saving map to: ${ROOT_DIR}"
echo "This uses nav2_map_server/map_saver_cli subscribing to /map."

ros2 run nav2_map_server map_saver_cli -f "${ROOT_DIR}/${MAP_NAME}"

YAML_FILE="${ROOT_DIR}/${MAP_NAME}.yaml"
PGM_FILE="${ROOT_DIR}/${MAP_NAME}.pgm"

if [[ ! -f "${YAML_FILE}" || ! -f "${PGM_FILE}" ]]; then
  echo "Map files not found after saving. Expected:"
  echo "  ${YAML_FILE}"
  echo "  ${PGM_FILE}"
  exit 2
fi

sha_yaml="$(sha256sum "${YAML_FILE}" | awk '{print $1}')"
sha_pgm="$(sha256sum "${PGM_FILE}" | awk '{print $1}')"

cat > "${ROOT_DIR}/FIXED_MAP.txt" <<EOF
MAP_NAME=${MAP_NAME}
CREATED_AT=$(date -Iseconds)
YAML_SHA256=${sha_yaml}
PGM_SHA256=${sha_pgm}
NOTE=This map is declared FIXED. Do not overwrite; create a new map_name for new versions.
EOF

echo "Map saved and marked FIXED:"
echo "  ${YAML_FILE}"
echo "  ${PGM_FILE}"
echo "  ${ROOT_DIR}/FIXED_MAP.txt"
