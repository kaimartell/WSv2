# SPIKE ROS 2 Workshop Notebook

Use this document as the live workshop handout (Notion-ready). Copy/paste commands exactly.

## 0) Goals

By the end of this workshop, you will:
- [ ] Run a ROS 2 Humble environment in Docker.
- [ ] Start a host-side SPIKE agent (`host_agent`).
- [ ] Trigger behaviors via ROS topics.
- [ ] Play and edit YAML patterns with no coding in ROS nodes.
- [ ] Use encoder-aware actions (degrees and absolute/relative position).

## 1) Pre-Flight Checklist

- [ ] Docker Desktop is installed and running.
- [ ] Python 3.9+ is installed.
- [ ] LEGO SPIKE hub is connected with a USB **data** cable.
- [ ] Motor is connected to hub port A (or note your chosen port).
- [ ] Repository cloned locally.

```bash
git clone https://github.com/<your-org-or-user>/spike-ros-workshop.git
cd spike-ros-workshop
```

Install host dependencies:

```bash
cd host_agent
python3 -m pip install -e ".[spike_usb]"
cd ..
```

What you should see:
- Pip install finishes without errors.
- `host_agent` package installed in editable mode.

## 2) Start Host Agent (Outside Docker)

Recommended command:

```bash
./scripts/start_host_agent_usb.sh
```

What you should see:
- A startup line showing host/port and selected serial device.
- Server listening on `http://127.0.0.1:8000`.

Health check (new terminal):

```bash
curl http://localhost:8000/health
```

What you should see:
- JSON like:
  - `{"ok": true, "backend": "spike_usb", "spike_connected": true|false, ...}`

### Rescue Path A: USB auto-detect failed

```bash
python3 -m host_agent --list
```

Pick your serial device and rerun:

```bash
SPIKE_SERIAL=/dev/cu.usbmodemXXXX ./scripts/start_host_agent_usb.sh
```

### Rescue Path B: No serial devices appear

- [ ] Confirm cable is data-capable (not charge-only).
- [ ] Close LEGO SPIKE app (it can lock serial).
- [ ] Unplug/replug hub.

Fallback for workshop continuity:

```bash
SPIKE_BACKEND=mock ./scripts/start_host_agent_usb.sh
```

## 3) Start ROS Container

```bash
./scripts/start_container.sh
```

What you should see:
- Docker build (first run) then interactive shell prompt in container.
- `ros2` command available.

Quick check inside container:

```bash
ros2 --help >/dev/null && echo "ros2 ready"
```

What you should see:
- `ros2 ready`

### Rescue Path C: `ros2` not found

Inside container:

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```

Then rerun `ros2 --help`.

## 4) Launch Instrument (Baseline)

Inside container terminal #1:

```bash
ros2 launch spike_workshop_instrument instrument.launch.py \
  mode:=pulse speed:=0.6 duration:=1.0 repeats:=4 queue_policy:=edges
```

What you should see:
- `instrument_node` and `spike_hw_client_node` logs.
- Log line: `Ping service ready on /spike/ping`.

Inside container terminal #2:

```bash
ros2 node list
ros2 topic list
```

What you should see:
- Nodes include `instrument_node`, `spike_hw_client_node`.
- Topics include `/actuate`, `/status`, `/done`, `/spike/state`, `/spike/cmd`, `/spike/action`.

## 5) Trigger and Observe

Trigger one actuation:

```bash
ros2 topic pub /actuate std_msgs/msg/Empty "{}" -1
```

Observe status:

```bash
ros2 topic echo /status
```

Observe completion:

```bash
ros2 topic echo /done
```

Ping host agent from ROS:

```bash
ros2 service call /spike/ping std_srvs/srv/Trigger "{}"
```

What you should see:
- `/status` transitions idle -> running -> idle.
- `/done` message published once for finite behavior.
- `/spike/ping` returns success and backend details.

## 6) Play Preset Patterns (No Code)

From host terminal (with container running):

```bash
./scripts/play_pattern.sh pulse_4
./scripts/play_pattern.sh metronome_90
./scripts/play_pattern.sh sweep_3s
./scripts/play_pattern.sh dance_basic
./scripts/play_pattern.sh clock_tick
./scripts/play_pattern.sh bounce_encoder
```

What you should see:
- Launch starts with `mode:=pattern`.
- Pattern-specific movement behavior executes.
- `/status` shows pattern step progress.

Pattern reference:
- `pulse_4`: timed pulses
- `metronome_90`: finite tempo pulses
- `sweep_3s`: signed velocity sweep
- `dance_basic`: mixed time + degree motions
- `clock_tick`: absolute encoder positions
- `bounce_encoder`: relative encoder moves and return

## 7) Generate Your Own Pattern with Wizard

### Interactive mode

```bash
python3 scripts/pattern_wizard.py
```

What you should see:
- Prompts for template, motor port, and parameters.
- Output file written under `patterns/user/`.
- Printed play command.

### Non-interactive mode

```bash
python3 scripts/pattern_wizard.py \
  --preset pulse \
  --speed 0.4 \
  --duration 0.3 \
  --repeats 6 \
  --out patterns/user/pulse6.yaml
```

Play generated pattern:

```bash
./scripts/play_pattern.sh /patterns/user/pulse6.yaml
```

What you should see:
- Pattern file created.
- Launch command starts and plays your file.

## 8) Encoder/Position Action Examples

These examples are pattern steps used in YAML.

Reset relative encoder:

```yaml
- type: "reset_relative_position"
```

Move by relative degrees:

```yaml
- type: "run_to_relative_position"
  velocity: 0.4
  degrees: 180
  stop_action: "hold"
```

Move to absolute position:

```yaml
- type: "run_to_absolute_position"
  velocity: 0.4
  position_degrees: 270
  stop_action: "hold"
```

Run fixed angle:

```yaml
- type: "run_for_degrees"
  velocity: 0.5
  degrees: -120
  stop_action: "brake"
```

## 9) Advanced Launch Parameter Notes

Common launch args:

```bash
ros2 launch spike_workshop_instrument instrument.launch.py \
  mode:=pattern \
  pattern_file:=/patterns/presets/bounce_encoder.yaml \
  host_agent_url:=http://host.docker.internal:8000 \
  queue_policy:=fifo \
  http_timeout_sec:=1.5 \
  action_http_timeout_sec:=8.0
```

Queue policy guidance:
- `edges`: preserve run/stop transitions for pulse-like behavior.
- `fifo`: preserve explicit pattern order (recommended for `mode:=pattern`).
- `latest`: joystick-like continuous control.

## 10) Troubleshooting Matrix

### Problem: `curl /health` fails

Checks:
- [ ] host agent process is running.
- [ ] host/port are correct (`127.0.0.1:8000`).

Fix:

```bash
./scripts/start_host_agent_usb.sh
```

### Problem: `/spike/ping` fails in ROS

Checks:
- [ ] launch `host_agent_url` is `http://host.docker.internal:8000`.
- [ ] host agent health works on host.

Fix:

```bash
ros2 launch spike_workshop_instrument instrument.launch.py \
  mode:=pulse host_agent_url:=http://host.docker.internal:8000
```

### Problem: Pattern file not found

Checks:
- [ ] file exists in `patterns/presets/` or `patterns/user/` on host.
- [ ] mounted path in container is `/patterns/...`.

Fix:

```bash
./scripts/play_pattern.sh /patterns/presets/pulse_4.yaml
```

### Problem: Launch arg parsing/type errors

Checks:
- [ ] numeric values are numeric (`90.0`, `0.4`, `6`).
- [ ] YAML keys match schema exactly.

Schema reference:
- `docs/pattern_schema.md`

### Problem: Timing looks compressed or pulses missing

Fix options:
- [ ] use `queue_policy:=edges` for pulse/sequence.
- [ ] use `queue_policy:=fifo` for patterns.
- [ ] increase durations/off-time.

### Problem: USB backend connected=false but workshop must continue

Use mock backend:

```bash
SPIKE_BACKEND=mock ./scripts/start_host_agent_usb.sh
```

## 11) Workshop Day Flow (Suggested)

- [ ] 00:00-00:15 Setup + health checks
- [ ] 00:15-00:30 ROS topics/services basics
- [ ] 00:30-00:50 Preset pattern playback
- [ ] 00:50-01:15 Pattern wizard and personal pattern creation
- [ ] 01:15-01:30 Encoder/position pattern lab
- [ ] 01:30-01:45 Debugging and rescue paths
- [ ] 01:45-02:00 Share-outs and wrap-up

## 12) Glossary (Tiny)

- **ROS node**: a running process in ROS (e.g., `instrument_node`).
- **ROS topic**: pub/sub channel for streaming messages (e.g., `/status`).
- **ROS service**: request/response RPC (e.g., `/spike/ping`).
- **host agent**: host-side HTTP bridge to hardware backends.
- **hub**: LEGO SPIKE Prime controller.
- **pattern**: YAML-defined sequence of motor steps.
- **action (here)**: higher-level motor command sent on `/spike/action` JSON.

## 13) Maintainer GitHub Publishing Commands

If needed:

```bash
# if repo not initialized
git init
git add .
git commit -m "Workshop packaging and docs"

# push to GitHub
git remote add origin https://github.com/<ORG_OR_USER>/spike-ros-workshop.git
git branch -M main
git push -u origin main
```

If you need to rename local folder from `repo` first:

```bash
cd ..
mv repo spike-ros-workshop
cd spike-ros-workshop
```
