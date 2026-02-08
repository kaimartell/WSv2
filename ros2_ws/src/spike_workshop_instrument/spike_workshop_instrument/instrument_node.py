import json
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

from spike_workshop_instrument.behaviors import build_steps
from spike_workshop_instrument.pattern_schema import (
    PatternValidationError,
    load_and_validate_pattern,
)


class InstrumentNode(Node):
    def __init__(self) -> None:
        super().__init__("instrument_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("participant_id", 1),
                ("name", "instrument"),
                ("mode", "pulse"),
                ("speed", 0.6),
                ("duration", 1.0),
                ("repeats", 4),
                ("bpm", 120.0),
                ("amplitude", 0.6),
                ("sweep_steps", 8),
                ("seed", 0),
                ("sequence_file", ""),
                ("pattern_file", ""),
                ("cmd_topic", "/spike/cmd"),
                ("action_topic", "/spike/action"),
                ("done_topic", "/done"),
                ("status_topic", "/status"),
            ],
        )

        cmd_topic = str(self.get_parameter("cmd_topic").value)
        action_topic = str(self.get_parameter("action_topic").value)
        done_topic = str(self.get_parameter("done_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)

        self._cmd_pub = self.create_publisher(String, cmd_topic, 10)
        self._action_pub = self.create_publisher(String, action_topic, 10)
        self._done_pub = self.create_publisher(Empty, done_topic, 10)
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self._actuate_sub = self.create_subscription(Empty, "/actuate", self._on_actuate, 10)
        self._spike_state_sub = self.create_subscription(
            String, "/spike/state", self._on_spike_state, 10
        )

        self._status_timer = self.create_timer(0.5, self._publish_status)

        self._lock = threading.Lock()
        self._running = False
        self._last_action = "idle"
        self._worker: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._host_connected = False
        self._metronome_timer = None
        self._metronome_stop_timer = None
        self._metronome_speed = 0.0
        self._metronome_bpm = 120.0
        self._metronome_beat_period = 0.5
        self._metronome_on_duration = 0.1
        self._metronome_beat_count = 0

        self.get_logger().info(
            "instrument_node ready. Trigger behavior with: ros2 topic pub /actuate std_msgs/msg/Empty '{}' -1"
        )

    def _context_ok(self) -> bool:
        try:
            return rclpy.ok(context=self.context)
        except Exception:  # noqa: BLE001
            return False

    def _safe_log(self, level: str, message: str) -> None:
        if not self._context_ok():
            return
        logger = self.get_logger()
        if level == "warning":
            logger.warning(message)
        elif level == "error":
            logger.error(message)
        else:
            logger.info(message)

    def _on_actuate(self, _msg: Empty) -> None:
        mode = str(self.get_parameter("mode").value).strip().lower()
        with self._lock:
            if self._running:
                self._last_action = "busy"
                busy = True
            else:
                self._running = True
                self._last_action = "starting"
                busy = False

        if busy:
            self._safe_log("info", "Received /actuate while running; ignoring trigger.")
            self._publish_status()
            return

        if mode == "metronome":
            self._start_metronome()
        else:
            self._worker = threading.Thread(target=self._run_behavior, daemon=True)
            self._worker.start()
        self._publish_status()

    def _cancel_metronome_timers(self) -> None:
        if self._metronome_timer is not None:
            self._metronome_timer.cancel()
            self._metronome_timer = None
        if self._metronome_stop_timer is not None:
            self._metronome_stop_timer.cancel()
            self._metronome_stop_timer = None

    def _start_metronome(self) -> None:
        params = self._read_behavior_params()

        self._cancel_metronome_timers()

        self._metronome_speed = float(params["speed"])
        self._metronome_bpm = max(1.0, float(params["bpm"]))
        self._metronome_beat_period = 60.0 / self._metronome_bpm

        requested_on = max(0.0, float(params["duration"]))
        if requested_on == 0.0:
            self._metronome_on_duration = min(0.1, self._metronome_beat_period * 0.3)
        else:
            self._metronome_on_duration = min(requested_on, self._metronome_beat_period)

        self._metronome_beat_count = 0

        self._metronome_timer = self.create_timer(
            self._metronome_beat_period, self._on_metronome_beat
        )
        self._safe_log(
            "info", f"Metronome started at {self._metronome_bpm:.1f} BPM (continuous)."
        )

        with self._lock:
            self._last_action = f"metronome running bpm={self._metronome_bpm:.1f}"
        self._publish_status()

        # Start first beat immediately instead of waiting one full period.
        self._on_metronome_beat()

    def _on_metronome_beat(self) -> None:
        if self._stop_event.is_set():
            return
        with self._lock:
            if not self._running:
                return

        self._metronome_beat_count += 1
        if not self._publish_motor_command(
            speed=self._metronome_speed, duration=self._metronome_on_duration
        ):
            return

        with self._lock:
            self._last_action = (
                f"metronome beat {self._metronome_beat_count} "
                f"speed={self._metronome_speed:.2f} bpm={self._metronome_bpm:.1f}"
            )
        self._publish_status()

        if self._metronome_on_duration < self._metronome_beat_period:
            if self._metronome_stop_timer is not None:
                self._metronome_stop_timer.cancel()
                self._metronome_stop_timer = None
            self._metronome_stop_timer = self.create_timer(
                self._metronome_on_duration, self._on_metronome_stop
            )

    def _on_metronome_stop(self) -> None:
        if self._metronome_stop_timer is not None:
            self._metronome_stop_timer.cancel()
            self._metronome_stop_timer = None

        if self._stop_event.is_set():
            return
        with self._lock:
            if not self._running:
                return
        self._publish_motor_command(speed=0.0, duration=0.0, log_command=False)

    def _run_behavior(self) -> None:
        params = self._read_behavior_params()
        mode = str(params["mode"])
        normalized_mode = mode.strip().lower()

        if normalized_mode == "metronome":
            # Metronome is continuous and timer-driven via _start_metronome().
            return

        if normalized_mode == "pattern":
            self._run_pattern(params)
            return

        self._run_standard_behavior(params)

    def _run_standard_behavior(self, params: Dict[str, Any]) -> None:
        mode = str(params["mode"])
        try:
            steps = build_steps(**params)
        except Exception as exc:  # noqa: BLE001
            self._safe_log("error", f"Failed to construct behavior steps: {exc}")
            with self._lock:
                self._running = False
                self._last_action = f"error: {exc}"
            self._publish_status()
            return

        self._safe_log("info", f"Executing mode={mode} with {len(steps)} steps")

        for index, step in enumerate(steps, start=1):
            if self._stop_event.is_set():
                break

            speed = float(step.get("speed", 0.0))
            step_duration = max(0.0, float(step.get("duration", 0.0)))

            if not self._publish_motor_command(speed=speed, duration=step_duration):
                break

            with self._lock:
                self._last_action = (
                    f"{mode} step {index}/{len(steps)} speed={speed:.2f} duration={step_duration:.2f}s"
                )
            self._publish_status()

            if not self._sleep_with_cancel(step_duration):
                break

        if not self._stop_event.is_set() and self._context_ok():
            try:
                self._done_pub.publish(Empty())
                self._safe_log("info", "Behavior complete; published /done")
            except Exception:  # noqa: BLE001
                pass

        with self._lock:
            self._running = False
            self._last_action = "idle"

        self._publish_motor_command(speed=0.0, duration=0.0, log_command=False)
        self._publish_status()

    @staticmethod
    def _coerce_speed(value: Any) -> float:
        return max(-1.0, min(1.0, float(value)))

    def _resolve_pattern_path(self, pattern_file: str) -> Path:
        raw = str(pattern_file or "").strip()
        if not raw:
            raise PatternValidationError("pattern_file must be set when mode=pattern")

        requested = Path(raw).expanduser()
        if requested.is_file():
            return requested

        candidates = [
            Path.cwd() / requested,
            Path("/patterns") / requested,
            Path("/ros2_ws") / requested,
            Path("/ros2_ws/src/spike_workshop_instrument/config/patterns") / requested.name,
        ]
        for candidate in candidates:
            if candidate.is_file():
                return candidate

        raise PatternValidationError(f"pattern file not found: {requested}")

    def _run_pattern(self, params: Dict[str, Any]) -> None:
        pattern_file = str(params.get("pattern_file", ""))
        try:
            resolved = self._resolve_pattern_path(pattern_file)
            pattern = load_and_validate_pattern(str(resolved))
        except Exception as exc:  # noqa: BLE001
            self._safe_log("error", f"Failed to load pattern: {exc}")
            with self._lock:
                self._running = False
                self._last_action = f"pattern error: {exc}"
            self._publish_status()
            return

        pattern_name = str(pattern.get("name", resolved.stem))
        steps = pattern.get("steps", [])
        self._safe_log(
            "info",
            f"Executing pattern={pattern_name} from {resolved} with {len(steps)} steps",
        )

        success = True
        for index, step in enumerate(steps, start=1):
            if self._stop_event.is_set():
                success = False
                break
            if not self._execute_pattern_step(step=step, index=index, total=len(steps)):
                success = False
                break

        if success and (not self._stop_event.is_set()) and self._context_ok():
            try:
                self._done_pub.publish(Empty())
                self._safe_log("info", "Pattern complete; published /done")
            except Exception:  # noqa: BLE001
                pass

        with self._lock:
            self._running = False
            self._last_action = "idle"

        self._publish_action(
            {
                "type": "stop",
                "port": str(pattern.get("defaults", {}).get("motor_port", "A")),
                "stop_action": str(pattern.get("defaults", {}).get("stop_action", "coast")),
            },
            log_command=False,
        )
        self._publish_motor_command(speed=0.0, duration=0.0, log_command=False)
        self._publish_status()

    def _execute_pattern_step(self, *, step: Dict[str, Any], index: int, total: int) -> bool:
        step_type = str(step.get("type", "")).strip().lower()
        if not step_type:
            return False

        if step_type == "sleep":
            duration_sec = max(0.0, float(step.get("duration_sec", 0.0)))
            with self._lock:
                self._last_action = (
                    f"pattern sleep step {index}/{total} duration={duration_sec:.2f}s"
                )
            self._publish_status()
            return self._sleep_with_cancel(duration_sec)

        action: Dict[str, Any] = {"type": step_type}

        if "port" in step:
            action["port"] = str(step["port"])
        if "comment" in step:
            action["comment"] = str(step["comment"])
        if "stop_action" in step:
            action["stop_action"] = str(step["stop_action"])

        if step_type in {
            "run",
            "run_for_time",
            "run_for_degrees",
            "run_to_absolute_position",
            "run_to_relative_position",
            "set_duty_cycle",
        }:
            action["speed"] = self._coerce_speed(step.get("velocity", 0.0))

        if step_type == "run_for_time":
            action["duration_sec"] = max(0.0, float(step.get("duration_sec", 0.0)))
        if step_type in {"run_for_degrees", "run_to_relative_position"}:
            action["degrees"] = int(step.get("degrees", 0))
        if step_type == "run_to_absolute_position":
            action["position_degrees"] = int(step.get("position_degrees", 0))

        if not self._publish_action(action):
            return False

        with self._lock:
            self._last_action = f"pattern step {index}/{total} type={step_type}"
        self._publish_status()
        return True

    def _on_spike_state(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        state = str(payload.get("state", ""))
        with self._lock:
            self._host_connected = state not in {"", "unreachable"}

    def _sleep_with_cancel(self, total_seconds: float) -> bool:
        remaining = max(0.0, float(total_seconds))
        while remaining > 0.0:
            if self._stop_event.is_set():
                return False
            chunk = min(0.05, remaining)
            time.sleep(chunk)
            remaining -= chunk
        return True

    def _read_behavior_params(self) -> Dict[str, Any]:
        return {
            "mode": str(self.get_parameter("mode").value),
            "speed": float(self.get_parameter("speed").value),
            "duration": float(self.get_parameter("duration").value),
            "repeats": int(self.get_parameter("repeats").value),
            "bpm": float(self.get_parameter("bpm").value),
            "amplitude": float(self.get_parameter("amplitude").value),
            "sweep_steps": int(self.get_parameter("sweep_steps").value),
            "seed": int(self.get_parameter("seed").value),
            "sequence_file": str(self.get_parameter("sequence_file").value),
            "pattern_file": str(self.get_parameter("pattern_file").value),
        }

    def _publish_motor_command(
        self, *, speed: float, duration: float, log_command: bool = True
    ) -> bool:
        if not self._context_ok():
            return False
        payload = {"speed": float(speed), "duration": max(0.0, float(duration))}
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        try:
            self._cmd_pub.publish(msg)
        except Exception:  # noqa: BLE001
            return False
        if log_command:
            self._safe_log("info", f"Published /spike/cmd {msg.data}")
        return True

    def _publish_action(self, payload: Dict[str, Any], log_command: bool = True) -> bool:
        if not self._context_ok():
            return False
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        try:
            self._action_pub.publish(msg)
        except Exception:  # noqa: BLE001
            return False
        if log_command:
            self._safe_log("info", f"Published /spike/action {msg.data}")
        return True

    def _build_status_text(self) -> str:
        with self._lock:
            state = "running" if self._running else "idle"
            last_action = self._last_action
            host_connected = self._host_connected

        return (
            f"state={state};last_action={last_action};"
            f"participant_id={int(self.get_parameter('participant_id').value)};"
            f"name={str(self.get_parameter('name').value)};"
            f"mode={str(self.get_parameter('mode').value)};"
            f"host_connected={str(host_connected).lower()}"
        )

    def _publish_status(self) -> None:
        if not self._context_ok():
            return
        msg = String()
        msg.data = self._build_status_text()
        try:
            self._status_pub.publish(msg)
        except Exception:  # noqa: BLE001
            pass

    def request_shutdown_cleanup(self) -> None:
        self._stop_event.set()

        if self._status_timer is not None:
            self._status_timer.cancel()

        self._cancel_metronome_timers()

        if self._worker is not None and self._worker.is_alive():
            self._worker.join(timeout=2.0)

        with self._lock:
            self._running = False
            self._last_action = "idle"

        if self._context_ok():
            self._publish_action({"type": "stop"}, log_command=False)
            self._publish_motor_command(speed=0.0, duration=0.0, log_command=False)
        else:
            # ROS context already shut down; skip publish to avoid invalid-context errors.
            pass


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[InstrumentNode] = None
    try:
        node = InstrumentNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.request_shutdown_cleanup()
            try:
                node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
        if rclpy.ok():
            rclpy.shutdown()
