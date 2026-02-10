#!/bin/sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
HOST_AGENT_DIR="$ROOT_DIR/host_agent"

HOST_AGENT_HOST=${HOST_AGENT_HOST:-0.0.0.0}
HOST_AGENT_PORT=${HOST_AGENT_PORT:-8000}
SPIKE_BACKEND=${SPIKE_BACKEND:-spike_usb}
SPIKE_SERIAL=${SPIKE_SERIAL:-auto}
SPIKE_PORT=${SPIKE_PORT:-A}
SPIKE_BAUD=${SPIKE_BAUD:-115200}
SPIKE_HUB_NAME=${SPIKE_HUB_NAME:-}
SPIKE_HUB_ADDRESS=${SPIKE_HUB_ADDRESS:-}
SPIKE_SCAN_TIMEOUT=${SPIKE_SCAN_TIMEOUT:-4.0}
SPIKE_CONNECT_TIMEOUT=${SPIKE_CONNECT_TIMEOUT:-6.0}

LIST_ONLY=0
DRY_RUN=0

usage() {
  cat <<'EOF'
Usage:
  ./scripts/start_host_agent_usb.sh [--list] [--dry-run]

Options:
  --list      List detected serial candidates and exit 0.
  --dry-run   Print the command that would run and exit 0.
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --list)
      LIST_ONLY=1
      ;;
    --dry-run)
      DRY_RUN=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument '$1'" >&2
      usage >&2
      exit 1
      ;;
  esac
  shift
done

if [ ! -d "$HOST_AGENT_DIR" ]; then
  echo "ERROR: host_agent directory not found at $HOST_AGENT_DIR" >&2
  exit 1
fi

platform_name() {
  case "$(uname -s 2>/dev/null || echo unknown)" in
    Darwin) echo "macos" ;;
    Linux) echo "linux" ;;
    *) echo "other" ;;
  esac
}

print_override_hint() {
  echo "Example override:" >&2
  echo "  SPIKE_SERIAL=/dev/cu.usbmodemXXXX ./scripts/start_host_agent_usb.sh" >&2
}

list_serial_candidates() {
  platform=$(platform_name)
  case "$platform" in
    macos)
      patterns="
/dev/cu.usbmodem*
/dev/cu.usbserial*
/dev/tty.usbmodem*
/dev/tty.usbserial*
/dev/cu.SLAB_USBtoUART*
/dev/tty.SLAB_USBtoUART*
"
      ;;
    linux)
      patterns="
/dev/ttyACM*
/dev/ttyUSB*
"
      ;;
    *)
      patterns="
/dev/ttyACM*
/dev/ttyUSB*
/dev/cu.usbmodem*
/dev/cu.usbserial*
/dev/tty.usbmodem*
/dev/tty.usbserial*
"
      ;;
  esac

  printf '%s\n' "$patterns" |
    awk 'NF' |
    while IFS= read -r pat; do
      for dev in $pat; do
        [ -e "$dev" ] || continue
        printf '%s\n' "$dev"
      done
    done |
    awk 'NF && !seen[$0]++' |
    sort
}

print_candidates() {
  candidates=$1
  if [ -z "${candidates:-}" ]; then
    echo "  (none)"
    return 0
  fi
  printf '%s\n' "$candidates" | awk 'NF {print "  - " $0}'
}

resolve_serial_macos() {
  candidates=$1
  count=$(printf '%s\n' "$candidates" | awk 'NF {c+=1} END {print c+0}')

  if [ "$count" -eq 1 ]; then
    selected=$(printf '%s\n' "$candidates" | awk 'NF {print; exit}')
    echo "Auto-selected SPIKE_SERIAL=$selected (single detected serial candidate)." >&2
    printf '%s\n' "$selected"
    return 0
  fi

  selected=$(
    printf '%s\n' "$candidates" | awk '
      /^\/dev\/cu\./ {
        suffix = substr($0, 9)
        cu[suffix] = $0
        seen[suffix] = 1
        next
      }
      /^\/dev\/tty\./ {
        suffix = substr($0, 10)
        tty[suffix] = $0
        seen[suffix] = 1
        next
      }
      {
        other = 1
      }
      END {
        group_count = 0
        key = ""
        for (k in seen) {
          group_count++
          key = k
        }
        if (other || group_count != 1) {
          exit 2
        }
        if (key in cu) {
          print cu[key]
          exit 0
        }
        if (key in tty) {
          print tty[key]
          exit 0
        }
        exit 3
      }
    ' 2>/dev/null
  ) || selected=""

  if [ -n "$selected" ]; then
    if printf '%s\n' "$selected" | grep -q '^/dev/cu\.'; then
      suffix=${selected#/dev/cu.}
      paired_tty="/dev/tty.$suffix"
      if printf '%s\n' "$candidates" | awk -v p="$paired_tty" '$0 == p {found=1} END {exit(found?0:1)}'; then
        echo "Auto-selected SPIKE_SERIAL=$selected (paired with $paired_tty, prefer cu.* on macOS)." >&2
      else
        echo "Auto-selected SPIKE_SERIAL=$selected (preferred cu.* on macOS)." >&2
      fi
    else
      echo "Auto-selected SPIKE_SERIAL=$selected (no cu.* peer found)." >&2
    fi
    printf '%s\n' "$selected"
    return 0
  fi

  echo "ERROR: Multiple distinct serial devices detected. Set SPIKE_SERIAL explicitly. Candidates:" >&2
  print_candidates "$candidates" >&2
  print_override_hint
  return 1
}

resolve_serial() {
  if [ "$SPIKE_SERIAL" != "auto" ] && [ -n "$SPIKE_SERIAL" ]; then
    if [ ! -e "$SPIKE_SERIAL" ]; then
      echo "ERROR: SPIKE_SERIAL is set but does not exist: $SPIKE_SERIAL" >&2
      print_override_hint
      exit 1
    fi
    echo "Using SPIKE_SERIAL=$SPIKE_SERIAL (explicit override)." >&2
    printf '%s\n' "$SPIKE_SERIAL"
    return 0
  fi

  candidates=$(list_serial_candidates || true)
  count=$(printf '%s\n' "$candidates" | awk 'NF {c+=1} END {print c+0}')
  platform=$(platform_name)

  if [ "$count" -eq 0 ]; then
    echo "ERROR: No SPIKE USB serial device found." >&2
    echo "Detected candidates: (none)" >&2
    echo "Set SPIKE_SERIAL=/dev/cu.usbmodemXXXX or run mock mode:" >&2
    echo "  SPIKE_BACKEND=mock ./scripts/start_host_agent_usb.sh" >&2
    exit 1
  fi

  if [ "$platform" = "macos" ]; then
    resolve_serial_macos "$candidates"
    return $?
  fi

  if [ "$count" -eq 1 ]; then
    selected=$(printf '%s\n' "$candidates" | awk 'NF {print; exit}')
    echo "Auto-selected SPIKE_SERIAL=$selected (single detected serial candidate)." >&2
    printf '%s\n' "$selected"
    return 0
  fi

  echo "ERROR: Multiple distinct serial devices detected. Set SPIKE_SERIAL explicitly. Candidates:" >&2
  print_candidates "$candidates" >&2
  print_override_hint
  return 1
}

if [ "$LIST_ONLY" -eq 1 ]; then
  platform=$(platform_name)
  candidates=$(list_serial_candidates || true)
  echo "Detected serial candidates ($platform):"
  print_candidates "$candidates"
  exit 0
fi

cd "$HOST_AGENT_DIR"

if [ "$SPIKE_BACKEND" = "mock" ]; then
  echo "Starting host_agent (mock) on http://$HOST_AGENT_HOST:$HOST_AGENT_PORT"
  if [ "$DRY_RUN" -eq 1 ]; then
    echo "DRY RUN: python3 -m host_agent --host $HOST_AGENT_HOST --port $HOST_AGENT_PORT --backend mock"
    exit 0
  fi
  exec python3 -m host_agent \
    --host "$HOST_AGENT_HOST" \
    --port "$HOST_AGENT_PORT" \
    --backend mock
fi

if [ "$SPIKE_BACKEND" = "spike_ble" ]; then
  echo "Starting host_agent (spike_ble) on http://$HOST_AGENT_HOST:$HOST_AGENT_PORT"
  if [ "$DRY_RUN" -eq 1 ]; then
    echo "DRY RUN: python3 -m host_agent --host $HOST_AGENT_HOST --port $HOST_AGENT_PORT --backend spike_ble --hub-name $SPIKE_HUB_NAME --hub-address $SPIKE_HUB_ADDRESS --motor-port $SPIKE_PORT --scan-timeout $SPIKE_SCAN_TIMEOUT --connect-timeout $SPIKE_CONNECT_TIMEOUT"
    exit 0
  fi
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
echo "Override anytime with: SPIKE_SERIAL=$SERIAL ./scripts/start_host_agent_usb.sh"
if [ "$DRY_RUN" -eq 1 ]; then
  echo "DRY RUN: python3 -m host_agent --host $HOST_AGENT_HOST --port $HOST_AGENT_PORT --backend $SPIKE_BACKEND --serial-port $SERIAL --motor-port $SPIKE_PORT --baud $SPIKE_BAUD"
  exit 0
fi
exec python3 -m host_agent \
  --host "$HOST_AGENT_HOST" \
  --port "$HOST_AGENT_PORT" \
  --backend "$SPIKE_BACKEND" \
  --serial-port "$SERIAL" \
  --motor-port "$SPIKE_PORT" \
  --baud "$SPIKE_BAUD"
