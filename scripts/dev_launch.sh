#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

docker compose up -d participant-dev

LAUNCH_CMD='source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --symlink-install && source /ros2_ws/install/setup.bash && ros2 launch spike_workshop_instrument instrument.launch.py'

exec docker compose exec participant-dev bash -lc "${LAUNCH_CMD}"
