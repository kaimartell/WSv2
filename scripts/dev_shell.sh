#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

docker compose up -d participant-dev

SHELL_CMD='source /opt/ros/humble/setup.bash && cd /ros2_ws && if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi; exec bash'

exec docker compose exec participant-dev bash -lc "${SHELL_CMD}"
