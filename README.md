# spike-ros-workshop

Build a physical instrument using ROS 2 and LEGO SPIKE Prime

This repository gives participants ROS flow:
- launch ROS once
- keep the graph running
- generate/list/play/stop patterns through ROS services
- keep shareable YAML patterns under `patterns/`

## ROS

ROS graph:
- nodes: `instrument_node`, `spike_hw_client_node`
- topics: `/actuate`, `/status`, `/done`, `/spike/state`
- services: `/spike/ping`, `/instrument/list_patterns`, `/instrument/generate_score`, `/instrument/play_pattern`, `/instrument/play_score`, `/instrument/stop`


## Pre-Workshop Setup

- Scroll down to the Appendix at the bottom for learning material

- Install Docker Desktop and start it: https://docs.docker.com/desktop/setup/install/mac-install/
  - Pay attention to Silicon vs Intel chip
  - Follow install instructions, can skip account creation
  - Verify CLI install with (type into terminal)
  ```bash
  docker version
  ```
  - Success looks like:
  ```bash
    Client:
      Version:           29.1.3
      ...
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


3. In container terminal #1, launch once:

```bash
ros2 launch spike_workshop_instrument instrument.launch.py
```


4. Open a third terminal window and enter the docker container with:

```bash
docker exec -it spikews bash
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

### 2) Generate a 4-bar score

Score format:
- 4 bars separated by `|`
- 4 beats per bar (16 beats total)
- motor beats: `F` (forward), `B` (backward), `S` (stop)
- melody notes: note names (`C4`, `A#4`) or Hz (`440`) or rest (`-`)

Generate a score with using a service call, with parameters, ex:

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

Motor-only ex:

```bash
ros2 service call /instrument/generate_score spike_workshop_interfaces/srv/GenerateScore \
  "{
    name: 'motor_only', \
    bpm: 100.0, \
    repeats: 1, \
    speed: 0.6, \
    motor: 'F F S S | B B S S | F F S S | B B S S', \
    melody: '', \
    volume: 60
  }"
```

Melody-only ex:

```bash
ros2 service call /instrument/generate_score spike_workshop_interfaces/srv/GenerateScore \
  "{
    name: 'melody_only', \
    bpm: 110.0, \
    repeats: 1, \
    speed: 0.5, \
    motor: '', \
    melody: 'C4 D4 E4 F4 | G4 A4 B4 C5 | C5 B4 A4 G4 | F4 E4 D4 C4', \
    volume: 70
  }"
```

Success:
- response includes `ok: true`

### 3) Play your score

```bash
ros2 service call /instrument/play_pattern spike_workshop_interfaces/srv/PlayPattern "{pattern_name: 'Your_score_name', pattern_path: ''}"
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

### 5) Debugging / ROS Commands

```bash
ros2 node list
ros2 topic list
ros2 service list | grep instrument
ros2 topic echo /status
ros2 topic echo /done
ros2 topic echo /spike/state
ros2 service call /spike/ping std_srvs/srv/Trigger "{}"
```

Timing mode checks:

```bash
# Launch in host-scheduled mode (recommended)
ros2 launch spike_workshop_instrument instrument.launch.py execution_mode:=host_score

# Launch in streaming mode (for comparison)
ros2 launch spike_workshop_instrument instrument.launch.py execution_mode:=stream
```

### 6) Troubleshooting

Host agent unreachable:
- `curl http://localhost:8000/health`
- from inside the container, also check: `curl http://host.docker.internal:8000/health`
- restart with `./scripts/start_host_agent_usb.sh`
- if starting manually, bind host agent on all interfaces for Docker access:
  - `python3 -m host_agent --host 0.0.0.0 --port 8000 --backend spike_usb --serial-port ...`

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

Host-score batch syntax error (for example `SyntaxError: invalid syntax`):
- restart host agent with debug snippet logging enabled:

```bash
python3 -m host_agent --host 0.0.0.0 --port 8000 --backend spike_usb --serial-port /dev/cu.usbmodemXXXX --motor-port A --debug-repl-snippets
```

- reproduce once, then inspect:
  - `artifacts/last_score_batch_snippet.py`
  - `artifacts/last_score_batch_events.json`
- those files include compile-check details and the exact snippet sent for the last batch window.

Manual stop:

```bash
ros2 service call /instrument/stop std_srvs/srv/Trigger "{}"
```

## ROS 2 Learning Appendix

### What ROS2 Is (Mental Model)

ROS2 is a structured environment where small programs called **nodes** exchange data

Core components:
- **Node**: a process with a name and responsibilities
- **Topic**: a named stream of messages (publish and subscribe structure)
- **Service**: request/response route for commands that need a reply
- **Launch file**: starts multiple nodes with parameters in one command


- Use **topics** for continuous states and events (`/status`, `/spike/state`).
- Use **services** for explicit commands (`/instrument/generate_score`, `/instrument/play_pattern`).

### How Discovery Works

ROS 2 nodes auto-discover each other on the same DDS domain.

In this workshop:
- `ROS_DOMAIN_ID=25` keeps your workshop isolated from nearby ROS traffic
- Container and host (your mac) communicate for hardware via HTTP (`host.docker.internal`), while ROS stays inside the container

### Commands Explained

`ros2 node list`
- Shows running nodes in the graph
- Expected: `instrument_node`, `spike_hw_client_node`.

`ros2 topic list`
- Shows available pub/sub channels
- Expected: `/actuate`, `/status`, `/done`, `/spike/state`, `/spike/cmd`, `/spike/action`.

`ros2 service list`
- Shows callable request/response endpoints
- Expected services include:
  - `/spike/ping`
  - `/instrument/list_patterns`
  - `/instrument/generate_score`
  - `/instrument/play_pattern`
  - `/instrument/stop`

### Example Commands

`ros2 service call /spike/ping std_srvs/srv/Trigger "{}"`
- Asks hardware bridge to verify host agent connectivity
- Useful first check before trying motion

`ros2 service call /instrument/generate_score ...`
- Converts your compact 4-bar score into YAML pattern steps under `/patterns/user`
- Returns whether generation succeeded and where the file was written

`ros2 service call /instrument/play_pattern ...`
- Starts playing your pattern on the existing graph

`ros2 service call /instrument/stop std_srvs/srv/Trigger "{}"`
- Stops active playback and sends stop commands

`ros2 topic echo /status`
- Live textual status stream from `instrument_node`
- Great for understanding step progression and current state

`ros2 topic echo /done`
- Emits completion event when a pattern finishes

`ros2 topic echo /spike/state`
- Bridge status from host agent/hub side

### What the ROS Graph Means

A ROS graph is a map of:
- which nodes exist
- which topics/services connect them

Think of it as a wiring diagram:
- nodes are components
- topic arrows are signal wires
- services are control calls

### This Workshop Graph

Launch command:

```bash
ros2 launch spike_workshop_instrument instrument.launch.py
```

Starts:
- `instrument_node`
- `spike_hw_client_node`

Graph sketch:

```text
                     (service calls from your terminal)
                             |
                             v
                    /instrument/generate_score
                    /instrument/play_pattern
                    /instrument/stop
                    /instrument/list_patterns
                             |
                             v
                      [instrument_node]
                         |         \
           /spike/cmd ---+          \--- /spike/action
                         |                 (actions, beep, etc.)
                         v
                   [spike_hw_client_node] ----HTTP----> [host_agent on macOS]
                         |
                    /spike/state
                         |
                      [instrument_node]

   [instrument_node] -> /status (progress/state text)
   [instrument_node] -> /done   (completion event)
```

### Data Flow

1. You call `/instrument/generate_score` with `motor` and optional `melody` strings
2. `instrument_node` parses score bars/beats and writes normalized YAML to `/patterns/user/<name>.yaml`
3. You call `/instrument/play_pattern`
4. `instrument_node` executes steps sequentially and publishes:
   - simple motor commands on `/spike/cmd`
   - advanced actions on `/spike/action`
5. `spike_hw_client_node` forwards those to host agent HTTP endpoints
6. Host agent executes USB REPL commands on SPIKE hub
7. Bridge publishes `/spike/state`; instrument publishes `/status` and `/done`


### Visualizing the Graph

If using a native Linux environment or a VM running Linux, the ROS Graph can be visualized using

```bash
rqt_graph
```

However, if using Mac, ROS tools are unavailable, unless using services like Xforwarding

### Message and Service Types You Use Most

Topics:
- `/status` -> `std_msgs/msg/String`
- `/done` -> `std_msgs/msg/Empty`
- `/spike/state` -> `std_msgs/msg/String`
- `/spike/cmd` -> `std_msgs/msg/String` (JSON payload)
- `/spike/action` -> `std_msgs/msg/String` (JSON payload)

Services:
- `/spike/ping` -> `std_srvs/srv/Trigger`
- `/instrument/list_patterns` -> `std_srvs/srv/Trigger`
- `/instrument/stop` -> `std_srvs/srv/Trigger`
- `/instrument/play_pattern` -> `spike_workshop_interfaces/srv/PlayPattern`
- `/instrument/play_score` -> `spike_workshop_interfaces/srv/PlayPattern`
- `/instrument/generate_score` -> `spike_workshop_interfaces/srv/GenerateScore`

## Timing and Jitter Bench (Advanced)

Run this on host (outside Docker) with host_agent running:

```bash
cd host_agent
python3 -m host_agent.tools.timing_bench --host 127.0.0.1 --port 8000 --trials 3 --mode both --output-dir ../artifacts
```

Expected report fields:
- `cases[].stream.summary.delta_ms`
- `cases[].host_score.summary.delta_ms`
- `cases[].stream.summary.pair_delta_ms`
- `cases[].host_score.summary.pair_delta_ms`
- `cases[].recommendation`

The benchmark prints a recommendation line such as:
- `host_score improves p95 by X ms`

### Container-Side ROS Integration Timing Check

After launching ROS in the container, run:

```bash
ros2 run spike_workshop_tools timing_integration --host-agent-url http://host.docker.internal:8000 --stop-on-timeout
```

This will:
- call `/spike/ping`
- call `/instrument/generate_score`
- call `/instrument/play_score`
- wait for `/done` (or timeout)
- fetch `GET /debug/timing`
- write a JSON report to `/tmp/ros_timing_report_<timestamp>.json`







Execution modes:
- `host_score` (default, recommended): `instrument_node` sends one score payload to host_agent (`/score/play`) and host_agent schedules motor+beep events locally.
- `stream` (compatibility mode): actions are forwarded one-by-one over HTTP as they are published.
- Why default to `host_score`: fewer round-trips and better motor/beep alignment under USB RAW REPL overhead.
