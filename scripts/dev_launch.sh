#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

docker compose up -d participant-dev

LAUNCH_PACKAGE="spike_workshop_ui_backend"
LAUNCH_FILE="ui_backend.launch.py"
# Example: ./scripts/dev_launch.sh instrument.launch.py
if [[ "${1:-}" == "instrument.launch.py" ]]; then
  LAUNCH_PACKAGE="spike_workshop_instrument"
  LAUNCH_FILE="instrument.launch.py"
fi
LAUNCH_CMD="source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --symlink-install && source /ros2_ws/install/setup.bash && ros2 launch ${LAUNCH_PACKAGE} ${LAUNCH_FILE}"

exec docker compose exec participant-dev bash -lc "${LAUNCH_CMD}"
