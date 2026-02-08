# spike-ros-workshop

Build a physical instrument using ROS 2 and LEGO SPIKE Prime.

This repository packages a participant-side workshop environment:
- ROS 2 Humble in Docker
- host-side `host_agent` for SPIKE communication
- no-code instrument behaviors + pattern/score playback
- YAML pattern presets and a pattern wizard

## Pre-Workshop Setup

- [ ] Install Docker Desktop and start it.
- [ ] Install Python 3.9+.
- [ ] Connect LEGO SPIKE Prime hub + motor + USB data cable.
- [ ] Clone this repo and `cd` into it.
- [ ] Install host agent dependencies.

```bash
git clone https://github.com/<your-org-or-user>/spike-ros-workshop.git
cd spike-ros-workshop

cd host_agent
python3 -m pip install -e ".[spike_usb]"
cd ..
```

Quick verification:

```bash
docker info >/dev/null && echo "Docker OK"
python3 --version
```

## 10-Minute Quick Start (Command-First)

1. Start host agent (USB):

```bash
./scripts/start_host_agent_usb.sh
```

If you want mock mode:

```bash
SPIKE_BACKEND=mock ./scripts/start_host_agent_usb.sh
```

2. Start the ROS container:

```bash
./scripts/start_container.sh
```

3. Inside the container shell, launch instrument mode:

```bash
ros2 launch spike_workshop_instrument instrument.launch.py \
  mode:=pulse speed:=0.6 duration:=1.0 repeats:=4 queue_policy:=edges
```

4. In another container shell, trigger behavior:

```bash
ros2 topic pub /actuate std_msgs/msg/Empty "{}" -1
```

5. Observe:

```bash
ros2 topic echo /status
ros2 topic echo /done
ros2 topic echo /spike/state
ros2 service call /spike/ping std_srvs/srv/Trigger "{}"
```

## Common Workshop Tasks

### Play a preset pattern

From host (uses running container):

```bash
./scripts/play_pattern.sh pulse_4
./scripts/play_pattern.sh clock_tick
./scripts/play_pattern.sh bounce_encoder
```

Equivalent manual launch (inside container):

```bash
ros2 launch spike_workshop_instrument instrument.launch.py \
  mode:=pattern \
  pattern_file:=/patterns/presets/bounce_encoder.yaml \
  queue_policy:=fifo
```

### Generate a user pattern (no coding)

Interactive wizard:

```bash
python3 scripts/pattern_wizard.py
```

Non-interactive preset:

```bash
python3 scripts/pattern_wizard.py \
  --preset pulse --speed 0.4 --duration 0.3 --repeats 6 \
  --out patterns/user/pulse6.yaml
```

Play your generated pattern:

```bash
./scripts/play_pattern.sh /patterns/user/pulse6.yaml
```

### Check host and ROS connectivity

```bash
curl http://localhost:8000/health
python3 -m host_agent --list
ros2 service call /spike/ping std_srvs/srv/Trigger "{}"
```

## Patterns Quick Reference

Preset files are in `patterns/presets/`:

- `pulse_4.yaml`: finite pulse timing using `run_for_time` + `sleep`
- `metronome_90.yaml`: finite metronome-style pattern at 90 BPM
- `sweep_3s.yaml`: signed velocity sweep across a fixed duration
- `dance_basic.yaml`: mixed timed + degree motions
- `clock_tick.yaml`: absolute encoder positioning (`run_to_absolute_position`)
- `bounce_encoder.yaml`: relative encoder positioning (`run_to_relative_position` + reset)

Schema reference: `docs/pattern_schema.md`

## Known Issues / FAQ

### Docker Desktop is running but container start fails
- Re-run `./scripts/preflight.sh`.
- Ensure enough disk space in Docker Desktop settings.

### USB serial auto-detect failed
- Run `python3 -m host_agent --list`.
- Set explicit serial path:

```bash
SPIKE_SERIAL=/dev/cu.usbmodemXXXX ./scripts/start_host_agent_usb.sh
```

### LEGO SPIKE app conflict
- Close the SPIKE app before starting host agent; it can lock the serial port.

### `ros2` command not found in container
- Source ROS setup manually:

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```

### Pattern launch fails due to parameter typing
- Keep numeric params numeric, e.g. `bpm:=90.0` not `bpm:=ninety`.

### Pulses collapse or timing looks compressed
- Use `queue_policy:=edges` for pulse/sequence.
- Use `queue_policy:=fifo` for pattern playback.

## Workshop Day Flow (Recommended)

- 0:00-0:15 Setup checks and host-agent connectivity
- 0:15-0:30 ROS graph basics (`node list`, `topic echo`, `/actuate`)
- 0:30-0:50 Preset pattern playback and observation
- 0:50-1:15 Pattern wizard + custom participant patterns
- 1:15-1:30 Encoder/position patterns (`clock_tick`, `bounce_encoder`)
- 1:30-1:45 Debugging lab (rescue paths)
- 1:45-2:00 Share-out and next steps

## Participant Fork/Clone Guidance

Participants should fork (optional) and clone:

```bash
git clone https://github.com/<their-user>/spike-ros-workshop.git
cd spike-ros-workshop
```

If no fork is needed, clone the workshop org repo directly.

## Maintainer: Publish to GitHub

If this repo is not initialized yet:

```bash
git init
git add .
git commit -m "Initial workshop packaging"
```

Set remote and push:

```bash
git remote add origin https://github.com/<your-org-or-user>/spike-ros-workshop.git
git branch -M main
git push -u origin main
```

If remote already exists, update URL:

```bash
git remote set-url origin https://github.com/<your-org-or-user>/spike-ros-workshop.git
git push -u origin main
```

## Rename Folder (if your local folder is still `repo/`)

From parent directory:

```bash
mv repo spike-ros-workshop
cd spike-ros-workshop
```

All scripts/docs in this repository assume the folder name `spike-ros-workshop` in examples.
