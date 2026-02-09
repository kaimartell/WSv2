# spike-ros-workshop

Build a physical instrument using ROS 2 and LEGO SPIKE Prime.

This repository gives participants ROS flow:
- launch ROS once
- keep the graph running
- generate/list/play/stop patterns through ROS services
- keep shareable YAML patterns under `patterns/`

## ROS

ROS graph:
- nodes: `instrument_node`, `spike_hw_client_node`
- topics: `/actuate`, `/status`, `/done`, `/spike/state`
- services: `/spike/ping`, `/instrument/list_patterns`, `/instrument/generate_score`, `/instrument/play_pattern`, `/instrument/stop`

## Pre-Workshop Setup

- Install Docker Desktop and start it: https://docs.docker.com/desktop/setup/install/mac-install/
  - Pay attention to Silicon vs Intel chip
  - Follow install instructions, can skip account creation
  - Verify CLI install with
  ```bash
  docker version
  ```
- Install Python 3.10+: https://www.python.org/downloads/
- Connect LEGO SPIKE Prime hub + motor + USB data cable
- Clone this repo


```bash
git clone https://github.com/kaimartell/ROS2-Workshop.git
cd spike-ros-workshop

cd host_agent
python3 -m pip install -e ".[spike_usb]"
cd ..
```

## Start Guide

1. Open terminal and start host agent:

```bash
./scripts/start_host_agent_usb.sh
```

USB auto-selection notes:
- On macOS, if a single hub appears as both `/dev/cu.usbmodem...` and `/dev/tty.usbmodem...`, the script treats that as one device and auto-selects `/dev/cu.*`.
- If multiple distinct devices are present, the script asks for explicit `SPIKE_SERIAL`.
- List candidates without starting:
  - `./scripts/start_host_agent_usb.sh --list`
- Get command without shell script:
  - `./scripts/start_host_agent_usb.sh --dry-run`
  - Run outputted python3 command after DRY RUN:


2. In another terminal window, start container:
```bash
./scripts/start_container.sh
```
This step may take a few minutes for the container to install all dependencies


3. In container terminal #1, launch once (idle-ready):

```bash
ros2 launch spike_workshop_instrument instrument.launch.py
```


4. Open a third terminal window and enter the docker container with:

```bash
docker exec -it spike-workshop-participant bash
```


5. In this third terminal, verify connectivity:

```bash
ros2 service call /spike/ping std_srvs/srv/Trigger "{}"
ros2 service call /instrument/list_patterns std_srvs/srv/Trigger "{}"
```


## Play an instrument

Keep `ros2 launch spike_workshop_instrument instrument.launch.py` running in one container terminal, then use ROS services from a second terminal in the same container.

### 1) Play a preset (optional)

```bash
ros2 service call /instrument/play_pattern spike_workshop_interfaces/srv/PlayPattern "{pattern_name: 'pulse_4', pattern_path: ''}"
```

Success criteria:
- `list_patterns` includes `presets/pulse_4`
- `play_pattern` returns `accepted: true`

### 2) Generate a 4-bar score (ROS-first)

Score format:
- 4 bars separated by `|`
- 4 tokens per bar (16 beats total)
- motor lane tokens: `F` (forward), `B` (backward), `S` (stop)
- melody lane tokens: note names (`C4`, `A#4`) or Hz (`440`) or rest (`-`)

Generate a score with using a service call, with parameters:

```bash
ros2 service call /instrument/generate_score spike_workshop_interfaces/srv/GenerateScore \
  "{
    name: 'my_score', \
    bpm: 120.0, \
    repeats: 2, \
    speed: 0.5, \
    motor: 'F S F S | F S F S | B S B S | B S B S', \
    melody: 'C4 D4 E4 F4 | G4 A4 B4 C5 | - - G4 - | C5 - - -', \
    volume: 60
  }"
```

Motor-only example (leave melody empty):

```bash
ros2 service call /instrument/generate_score spike_workshop_interfaces/srv/GenerateScore \
"{name: 'motor_only', bpm: 100.0, repeats: 1, speed: 0.6, motor: 'F F S S | B B S S | F F S S | B B S S', melody: '', volume: 60}"
```

Melody-only example (leave motor empty):

```bash
ros2 service call /instrument/generate_score spike_workshop_interfaces/srv/GenerateScore "{name: 'melody_only', bpm: 110.0, repeats: 1, speed: 0.5, motor: '', melody: 'C4 D4 E4 F4 | G4 A4 B4 C5 | C5 B4 A4 G4 | F4 E4 D4 C4', volume: 70}"
```

Success criteria:
- response includes `ok: true`
- YAML is written under `/patterns/user/<name>.yaml`

### 3) Play your score

```bash
ros2 service call /instrument/play_pattern spike_workshop_interfaces/srv/PlayPattern "{pattern_name: 'my_score', pattern_path: ''}"
```

Success criteria:
- response includes `accepted: true`
- `/status` moves to `running` then back to `idle`
- `/done` publishes once when complete

### 4) Stop playback

```bash
ros2 service call /instrument/stop std_srvs/srv/Trigger "{}"
```

Success criteria:
- response includes `success: true`
- motor stops immediately (best effort)

### 5) Debugging / ROS introspection

```bash
ros2 node list
ros2 topic list
ros2 service list | grep instrument
ros2 topic echo /status
ros2 topic echo /done
ros2 topic echo /spike/state
ros2 service call /spike/ping std_srvs/srv/Trigger "{}"
```

### 6) Troubleshooting

Host agent unreachable:
- `curl http://localhost:8000/health`
- restart with `./scripts/start_host_agent_usb.sh`

`spike_connected: false`:
- close LEGO SPIKE app
- run `./scripts/start_host_agent_usb.sh --list`
- choose explicit port:

```bash
SPIKE_SERIAL=/dev/cu.usbmodemXXXX ./scripts/start_host_agent_usb.sh
```

No motion:
- verify ping:

```bash
ros2 service call /spike/ping std_srvs/srv/Trigger "{}"
```

No beep:
- hub firmware must provide `hub.sound.beep`
- try a melody step in score and check host agent logs for sound errors

Manual emergency stop:

```bash
curl -s -X POST http://localhost:8000/motor/stop \
  -H "Content-Type: application/json" \
  -d '{"port":"A"}'
```

Core presets shipped for workshop:
- `pulse_4`
- `score_motor_basic`
- `score_call_response`

Backward compatibility:
- `/instrument/generate_pattern` still works.
- For `template_name: score_4bar`, legacy fields (`duration_sec`, `gap_sec`, `degrees`) are ignored.

`pattern_wizard` and `pattern_menu` are deprecated and intentionally excluded from the workshop flow.

