#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="${SPIKE_WORKSHOP_IMAGE:-spike_workshop_participant:latest}"
CONTAINER_NAME="${SPIKE_WORKSHOP_CONTAINER:-spikews}"
SKIP_BUILD="${SPIKE_WORKSHOP_SKIP_BUILD:-0}"

"${ROOT_DIR}/scripts/preflight.sh"

if [[ "${SKIP_BUILD}" != "1" ]]; then
  echo "Building ${IMAGE_NAME} ..."
  docker build -f "${ROOT_DIR}/docker/Dockerfile" -t "${IMAGE_NAME}" "${ROOT_DIR}"
elif ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
  echo "ERROR: ${IMAGE_NAME} not found and SPIKE_WORKSHOP_SKIP_BUILD=1."
  exit 1
fi

if [[ "$#" -gt 0 ]]; then
  CMD=("$@")
else
  CMD=("bash")
fi

exec docker run --rm -it --name "${CONTAINER_NAME}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-25}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
  -p 9090:9090 \
  -v "${ROOT_DIR}/patterns:/patterns" \
  "${IMAGE_NAME}" "${CMD[@]}"
