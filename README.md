# spike-ros-workshop

Build a physical instrument using ROS 2 and LEGO SPIKE Prime.

This repository gives participants a ROS-first workshop flow:
- launch ROS once
- keep the graph running
- generate/list/play/stop patterns through ROS services
- keep shareable YAML patterns on disk under `patterns/`

## Why This Is ROS

You interact with a live ROS graph:
- nodes: `instrument_node`, `spike_hw_client_node`
- topics: `/actuate`, `/status`, `/done`, `/spike/state`
- services: `/spike/ping`, `/instrument/list_patterns`, `/instrument/generate_pattern`, `/instrument/play_pattern`, `/instrument/stop`

Instead of restarting launch files for each pattern, you control playback via service calls.

## Pre-Workshop Setup

- Install Docker Desktop and start it.
- Install Python 3.9+.
- Connect LEGO SPIKE Prime hub + motor + USB data cable.
- Clone this repo.

```bash
git clone https://github.com/<your-org-or-user>/spike-ros-workshop.git
cd spike-ros-workshop

cd host_agent
python3 -m pip install -e ".[spike_usb]"
cd ..
```

## 10-Minute ROS-First Quick Start

1. Start host agent:

```bash
./scripts/start_host_agent_usb.sh
```

Mock fallback:

```bash
SPIKE_BACKEND=mock ./scripts/start_host_agent_usb.sh
```

2. Start container:

```bash
./scripts/start_container.sh
```

3. In container terminal #1, launch once (idle-ready):

```bash
ros2 launch spike_workshop_instrument instrument.launch.py
```

4. In container terminal #2, verify core services:

```bash
ros2 service call /spike/ping std_srvs/srv/Trigger "{}"
ros2 service call /instrument/list_patterns std_srvs/srv/Trigger "{}"
```

5. Generate a user pattern via ROS service:

```bash
ros2 service call /instrument/generate_pattern spike_workshop_interfaces/srv/GeneratePattern "{template_name: 'pulse', output_name: 'my_pulse', output_dir: '/patterns/user', speed: 0.6, duration_sec: 0.3, gap_sec: 0.2, repeats: 4, bpm: 120.0, degrees: 180}"
```

6. Play it by name (no relaunch):

```bash
ros2 service call /instrument/play_pattern spike_workshop_interfaces/srv/PlayPattern "{pattern_name: 'my_pulse', pattern_path: ''}"
```

Play a melody preset:

```bash
ros2 service call /instrument/play_pattern spike_workshop_interfaces/srv/PlayPattern "{pattern_name: 'melody_scale', pattern_path: ''}"
```

7. Stop reliably:

```bash
ros2 service call /instrument/stop std_srvs/srv/Trigger "{}"
```

## Interactive Helper

Inside container:

```bash
ros2 run spike_workshop_tools pattern_menu
```

The menu can:
- list patterns
- build a 4-bar score (`score_4bar`) from presets or pasted score strings
- play patterns
- stop playback

It also prints equivalent `ros2 service call ...` commands for learning.

## Create A 4-Bar Score

Score format:
- 4 bars separated by `|`
- 4 beat tokens per bar (16 beats total)
- motor tokens: `F` (forward), `B` (backward), `S` (stop)
- melody tokens: note names (`C4`, `A#4`, etc), integer frequency Hz, or `-` for rest

Example score strings:

```text
motor_score:  F S F S | F S F S | B S B S | B S B S
melody_score: C4 D4 E4 F4 | G4 A4 B4 C5 | - - G4 - | C5 - - -
```

Generate a score pattern via ROS service:

```bash
ros2 service call /instrument/generate_pattern spike_workshop_interfaces/srv/GeneratePattern "{template_name: 'score_4bar', output_name: 'my_score', output_dir: '/patterns/user', speed: 0.5, duration_sec: 0.0, gap_sec: 0.0, repeats: 2, bpm: 120.0, degrees: 0, motor_score: 'F S F S | F S F S | B S B S | B S B S', melody_score: 'C4 D4 E4 F4 | G4 A4 B4 C5 | - - G4 - | C5 - - -', beep_volume: 60}"
```

Play the generated score (without relaunch):

```bash
ros2 service call /instrument/play_pattern spike_workshop_interfaces/srv/PlayPattern "{pattern_name: 'my_score', pattern_path: ''}"
```

## ROS Commands You Will Use

Observe graph:

```bash
ros2 node list
ros2 topic list
ros2 service list
```

Observe status and completion:

```bash
ros2 topic echo /status
ros2 topic echo /done
ros2 topic echo /spike/state
```

Topic trigger still works for non-service workflow:

```bash
ros2 topic pub /actuate std_msgs/msg/Empty "{}" -1
```

## Patterns Quick Reference

Preset patterns in `patterns/presets/`:
- `pulse_4.yaml`: 4 timed pulses (`run_for_time` + `sleep`)
- `metronome_90.yaml`: finite metronome-like pulses
- `sweep_3s.yaml`: signed velocity sweep
- `dance_basic.yaml`: mixed timed + degree actions
- `clock_tick.yaml`: absolute encoder positions
- `bounce_encoder.yaml`: relative encoder bounce + reset
- `melody_scale.yaml`: C major speaker scale with `beep` steps
- `melody_call_response.yaml`: interleaved beeps + motor motion
- `score_motor_basic.yaml`: 4-bar motor-only score
- `score_melody_scale.yaml`: 4-bar melody-only score
- `score_call_response.yaml`: 4-bar motor + melody response pattern
- `score_back_and_forth_groove.yaml`: 4-bar groove with both lanes

Schema and step types:
- `docs/pattern_schema.md`

Pattern mode execution is sequential and timing-aware (`duration_sec`, optional `wait_sec`, `gap_sec`).

Beep step example:

```yaml
- type: "beep"
  freq_hz: 440
  duration_ms: 120
  volume: 60
  gap_sec: 0.05
```

Speaker beeps use `from hub import sound` / `sound.beep(...)` and require firmware that provides `hub.sound`.

Generate a melody pattern (no Python editing):

```bash
python3 scripts/pattern_wizard.py --preset melody \
  --name my_melody \
  --notes C4,D4,E4,F4,G4,A4,B4,C5 \
  --duration-ms 140 \
  --volume 60 \
  --gap-sec 0.05 \
  --out patterns/user/my_melody.yaml
```

## Optional Shortcut Script

`play_pattern.sh` is still available as a convenience fallback:

```bash
./scripts/play_pattern.sh pulse_4
```

It is optional now. ROS services are the default workshop workflow.

## Known Issues / FAQ

### Host agent not reachable

```bash
curl http://localhost:8000/health
```

If needed, restart:

```bash
./scripts/start_host_agent_usb.sh
```

### USB serial auto-detect failed

```bash
python3 -m host_agent --list
SPIKE_SERIAL=/dev/cu.usbmodemXXXX ./scripts/start_host_agent_usb.sh
```

### LEGO SPIKE app conflict

Close the SPIKE app; it can hold the serial port.

### `ros2` not found in container shell

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```

### Ctrl-C pressed but motor still moving

Use manual stop:

```bash
curl -s -X POST http://localhost:8000/motor/stop \
  -H "Content-Type: application/json" \
  -d '{"port":"A"}'

curl -s -X POST http://localhost:8000/sound/stop \
  -H "Content-Type: application/json" \
  -d '{}'
```

## Participant Fork/Clone

```bash
git clone https://github.com/<their-user>/spike-ros-workshop.git
cd spike-ros-workshop
```

## Maintainer: Publish To GitHub

```bash
git init
git add .
git commit -m "Initial workshop packaging"
git remote add origin https://github.com/<your-org-or-user>/spike-ros-workshop.git
git branch -M main
git push -u origin main
```
