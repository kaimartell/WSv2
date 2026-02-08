# Host Agent

`host_agent` runs on the participant host OS (macOS), outside Docker.
The ROS container talks to it through `http://host.docker.internal:8000`.

## Run

Mock backend (always works):

```bash
python3 -m host_agent --port 8000 --backend mock
```

USB backend (SPIKE Prime over serial RAW REPL):

```bash
python3 -m host_agent --port 8000 --backend spike_usb --serial-port auto --motor-port A
```

BLE backend (optional):

```bash
python3 -m host_agent --port 8000 --backend spike_ble
```

List serial + BLE candidates:

```bash
python3 -m host_agent --list
```

## Install dependencies

USB:

```bash
python3 -m pip install -e ".[spike_usb]"
```

BLE:

```bash
python3 -m pip install -e ".[spike_ble]"
```

## USB backend details

`spike_usb` targets LEGO SPIKE Prime official firmware APIs:
- `from hub import port`
- `import motor`
- `motor.run(...)`, `motor.stop(...)`

Manual REPL sanity check:

```python
import motor
from hub import port
motor.run(port.A, 1000)
```

`/motor/run` is non-blocking in USB backend:
- run starts motion quickly
- ROS timing controls stop via `/motor/stop` or speed=0 commands

## Smoke test

```bash
python3 -m host_agent.tools.spike_usb_smoketest --serial-port auto --motor-port A --speed 0.2 --duration 0.3
```

## HTTP API

Base endpoints:
- `GET /health` -> `{ "ok": true, "backend": "...", "spike_connected": bool }`
- `GET /state` -> `{ "state": "idle|running", "last_speed": float, "timestamp": float }`

Motor endpoints:
- `POST /motor/run` body `{ "speed": float, "duration": float, "port": "A" }`
- `POST /motor/stop` body `{ "port": "A", "stop_action": "coast|brake|hold" }`
- `POST /motor/reset_relative` body `{ "port": "A" }`
- `POST /motor/run_for_degrees` body `{ "port": "A", "speed": float, "degrees": int, "stop_action": "..." }`
- `POST /motor/run_to_absolute` body `{ "port": "A", "speed": float, "position_degrees": int, "stop_action": "..." }`
- `POST /motor/run_to_relative` body `{ "port": "A", "speed": float, "degrees": int, "stop_action": "..." }`
- `POST /motor/set_duty_cycle` body `{ "port": "A", "speed": float }`
- `POST /motor/status` body `{ "port": "A" }`

## Troubleshooting

- No serial device: verify cable is data-capable (LEGO official cable can matter).
- SPIKE app conflict: close LEGO SPIKE app if it holds the port.
- Multiple serial candidates: pass explicit `--serial-port /dev/cu.usbmodemXXXX`.
- RAW REPL sync errors: reset the hub and retry.

When unavailable, backends should still keep server running and return structured errors (`accepted:false` / `stopped:false`) instead of crashing.
