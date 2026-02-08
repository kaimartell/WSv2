#!/bin/sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
HOST_AGENT_DIR="$ROOT_DIR/host_agent"

HOST_AGENT_HOST=${HOST_AGENT_HOST:-127.0.0.1}
HOST_AGENT_PORT=${HOST_AGENT_PORT:-8000}
SPIKE_BACKEND=${SPIKE_BACKEND:-spike_usb}
SPIKE_SERIAL=${SPIKE_SERIAL:-auto}
SPIKE_PORT=${SPIKE_PORT:-A}
SPIKE_BAUD=${SPIKE_BAUD:-115200}
SPIKE_HUB_NAME=${SPIKE_HUB_NAME:-}
SPIKE_HUB_ADDRESS=${SPIKE_HUB_ADDRESS:-}
SPIKE_SCAN_TIMEOUT=${SPIKE_SCAN_TIMEOUT:-4.0}
SPIKE_CONNECT_TIMEOUT=${SPIKE_CONNECT_TIMEOUT:-6.0}

if [ ! -d "$HOST_AGENT_DIR" ]; then
  echo "ERROR: host_agent directory not found at $HOST_AGENT_DIR" >&2
  exit 1
fi

list_serial_candidates() {
  for pat in \
    /dev/cu.usbmodem* \
    /dev/cu.usbserial* \
    /dev/cu.SLAB_USBtoUART* \
    /dev/cu.wchusbserial* \
    /dev/tty.usbmodem* \
    /dev/tty.usbserial* \
    /dev/tty.SLAB_USBtoUART*
  do
    for dev in $pat; do
      [ -e "$dev" ] || continue
      printf '%s\n' "$dev"
    done
  done | awk 'NF && !seen[$0]++'
}

resolve_serial() {
  if [ "$SPIKE_SERIAL" != "auto" ] && [ -n "$SPIKE_SERIAL" ]; then
    printf '%s\n' "$SPIKE_SERIAL"
    return 0
  fi

  candidates=$(list_serial_candidates || true)
  count=$(printf '%s\n' "$candidates" | awk 'NF {c+=1} END {print c+0}')

  if [ "$count" -eq 1 ]; then
    printf '%s\n' "$candidates" | awk 'NF {print; exit}'
    return 0
  fi

  if [ "$count" -eq 0 ]; then
    echo "ERROR: No SPIKE USB serial device found." >&2
    echo "       Set SPIKE_SERIAL=/dev/cu.usbmodemXXXX or run mock mode:" >&2
    echo "       SPIKE_BACKEND=mock ./scripts/start_host_agent_usb.sh" >&2
    exit 1
  fi

  echo "ERROR: Multiple serial devices detected. Set SPIKE_SERIAL explicitly." >&2
  printf '%s\n' "$candidates" | awk 'NF {print "  - " $0}' >&2
  exit 1
}

cd "$HOST_AGENT_DIR"

if [ "$SPIKE_BACKEND" = "mock" ]; then
  echo "Starting host_agent (mock) on http://$HOST_AGENT_HOST:$HOST_AGENT_PORT"
  exec python3 -m host_agent \
    --host "$HOST_AGENT_HOST" \
    --port "$HOST_AGENT_PORT" \
    --backend mock
fi

if [ "$SPIKE_BACKEND" = "spike_ble" ]; then
  echo "Starting host_agent (spike_ble) on http://$HOST_AGENT_HOST:$HOST_AGENT_PORT"
  exec python3 -m host_agent \
    --host "$HOST_AGENT_HOST" \
    --port "$HOST_AGENT_PORT" \
    --backend spike_ble \
    --hub-name "$SPIKE_HUB_NAME" \
    --hub-address "$SPIKE_HUB_ADDRESS" \
    --motor-port "$SPIKE_PORT" \
    --scan-timeout "$SPIKE_SCAN_TIMEOUT" \
    --connect-timeout "$SPIKE_CONNECT_TIMEOUT"
fi

SERIAL=$(resolve_serial)
echo "Starting host_agent (spike_usb) on http://$HOST_AGENT_HOST:$HOST_AGENT_PORT"
echo "Using SPIKE_SERIAL=$SERIAL SPIKE_PORT=$SPIKE_PORT SPIKE_BAUD=$SPIKE_BAUD"
exec python3 -m host_agent \
  --host "$HOST_AGENT_HOST" \
  --port "$HOST_AGENT_PORT" \
  --backend "$SPIKE_BACKEND" \
  --serial-port "$SERIAL" \
  --motor-port "$SPIKE_PORT" \
  --baud "$SPIKE_BAUD"
