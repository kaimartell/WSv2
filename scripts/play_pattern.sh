#!/bin/sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)

if [ "${1:-}" = "" ]; then
  echo "Usage: $0 <pattern_path_or_name> [extra ros2 launch args...]" >&2
  echo "Examples:" >&2
  echo "  $0 pulse_4" >&2
  echo "  $0 /patterns/presets/bounce_encoder.yaml" >&2
  echo "  $0 patterns/user/my_pattern.yaml host_agent_url:=http://host.docker.internal:8000" >&2
  exit 1
fi

PATTERN_INPUT=$1
shift

QUEUE_POLICY=${QUEUE_POLICY:-fifo}
CONTAINER_NAME=${SPIKE_WORKSHOP_CONTAINER:-spike_workshop_participant}

resolve_pattern_file() {
  input=$1

  case "$input" in
    /patterns/*)
      printf '%s\n' "$input"
      return 0
      ;;
    patterns/*)
      printf '/%s\n' "$input"
      return 0
      ;;
    /*)
      printf '%s\n' "$input"
      return 0
      ;;
  esac

  if [ -f "$ROOT_DIR/patterns/presets/$input" ]; then
    printf '/patterns/presets/%s\n' "$input"
    return 0
  fi
  if [ -f "$ROOT_DIR/patterns/user/$input" ]; then
    printf '/patterns/user/%s\n' "$input"
    return 0
  fi

  if [ "${input##*.}" != "yaml" ]; then
    if [ -f "$ROOT_DIR/patterns/presets/$input.yaml" ]; then
      printf '/patterns/presets/%s.yaml\n' "$input"
      return 0
    fi
    if [ -f "$ROOT_DIR/patterns/user/$input.yaml" ]; then
      printf '/patterns/user/%s.yaml\n' "$input"
      return 0
    fi
  fi

  printf '/patterns/%s\n' "$input"
}

PATTERN_FILE=$(resolve_pattern_file "$PATTERN_INPUT")

if command -v ros2 >/dev/null 2>&1; then
  if [ -f /opt/ros/humble/setup.sh ]; then
    # shellcheck disable=SC1091
    . /opt/ros/humble/setup.sh
  fi
  if [ -f /ros2_ws/install/setup.sh ]; then
    # shellcheck disable=SC1091
    . /ros2_ws/install/setup.sh
  fi

  echo "Playing pattern: $PATTERN_FILE"
  exec ros2 launch spike_workshop_instrument instrument.launch.py \
    mode:=pattern \
    pattern_file:="$PATTERN_FILE" \
    queue_policy:="$QUEUE_POLICY" \
    "$@"
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "ERROR: ros2 not found and docker is not installed." >&2
  exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
  echo "ERROR: ros2 not found locally and container '${CONTAINER_NAME}' is not running." >&2
  echo "Start it with: ./scripts/start_container.sh" >&2
  exit 1
fi

quote_for_bash() {
  printf '%s' "$1" | sed "s/'/'\\''/g"
}

EXTRA_ARGS=''
for arg in "$@"; do
  escaped=$(quote_for_bash "$arg")
  EXTRA_ARGS="$EXTRA_ARGS '$escaped'"
done

PF_ESCAPED=$(quote_for_bash "$PATTERN_FILE")
QP_ESCAPED=$(quote_for_bash "$QUEUE_POLICY")

echo "Playing pattern in container '$CONTAINER_NAME': $PATTERN_FILE"
exec docker exec -it "$CONTAINER_NAME" bash -lc "source /opt/ros/humble/setup.bash && if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi && ros2 launch spike_workshop_instrument instrument.launch.py mode:=pattern pattern_file:='$PF_ESCAPED' queue_policy:='$QP_ESCAPED' $EXTRA_ARGS"
