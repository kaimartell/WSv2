import json
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from spike_workshop_interfaces.srv import GeneratePattern, GenerateScore, PlayPattern
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

from spike_workshop_instrument.behaviors import build_steps
from spike_workshop_instrument.pattern_schema import (
    PatternValidationError,
    load_and_validate_pattern,
)
from spike_workshop_instrument.pattern_templates import (
    TEMPLATES,
    build_score_pattern,
    build_template_pattern,
    ensure_yaml_output_path,
    write_pattern_yaml,
)

STATE_IDLE_POLL_TIMEOUT_SEC = 6.0
DEFAULT_BLOCKING_WAIT_SEC = {
    "run_for_degrees": 0.8,
    "run_to_absolute_position": 1.0,
    "run_to_relative_position": 1.0,
    "reset_relative_position": 0.2,
}


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
                ("execution_mode", "host_score"),
                ("allow_override", False),
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

        self._play_pattern_srv = self.create_service(
            PlayPattern, "/instrument/play_pattern", self._on_play_pattern
        )
        self._play_score_srv = self.create_service(
            PlayPattern, "/instrument/play_score", self._on_play_score
        )
        self._stop_srv = self.create_service(Trigger, "/instrument/stop", self._on_stop)
        self._list_patterns_srv = self.create_service(
            Trigger, "/instrument/list_patterns", self._on_list_patterns
        )
        self._generate_pattern_srv = self.create_service(
            GeneratePattern,
            "/instrument/generate_pattern",
            self._on_generate_pattern,
        )
        self._generate_score_srv = self.create_service(
            GenerateScore,
            "/instrument/generate_score",
            self._on_generate_score,
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
        self._spike_state_value = "unknown"
        self._spike_state_counter = 0
        self._spike_state_last_update_monotonic = 0.0
        self._score_state_counter = 0
        self._score_playing = False
        self._score_status_ok = False
        self._score_current_index = 0
        self._score_total_events = 0
        self._score_start_time_monotonic = 0.0
        self._score_expected_end_monotonic = 0.0
        self._score_last_error = ""
        self._active_pattern_path = ""

        self._safe_log(
            "info",
            (
                "instrument_node ready. Services: /instrument/list_patterns, "
                "/instrument/generate_score, /instrument/generate_pattern, "
                "/instrument/play_pattern, /instrument/play_score, /instrument/stop"
            ),
        )
        self._safe_log(
            "info",
            "Trigger topic still available: ros2 topic pub /actuate std_msgs/msg/Empty '{}' -1",
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

    def _execution_mode(self) -> str:
        raw = str(self.get_parameter("execution_mode").value).strip().lower()
        if raw not in {"stream", "host_score"}:
            return "host_score"
        return raw

    def _pattern_roots(self) -> List[Tuple[str, Path]]:
        return [
            ("user", Path("/patterns/user")),
            ("presets", Path("/patterns/presets")),
        ]

    def _discover_patterns(self) -> List[str]:
        entries: List[str] = []
        for scope, root in self._pattern_roots():
            if not root.is_dir():
                continue
            for path in sorted(root.glob("*.yaml")):
                entries.append(f"{scope}/{path.stem}")
        return entries

    def _resolve_pattern_name(self, pattern_name: str) -> Path:
        raw = str(pattern_name or "").strip()
        if not raw:
            raise PatternValidationError("pattern_name must be provided")

        requested = Path(raw)
        if requested.is_absolute():
            return self._resolve_pattern_path(str(requested))

        names: List[str] = []
        if raw.lower().endswith(".yaml"):
            names.append(raw)
            stem = raw[:-5]
            if stem:
                names.append(stem)
        else:
            names.append(raw)
            names.append(f"{raw}.yaml")

        # Allow explicit scope prefix: user/foo or presets/foo.
        if "/" in raw:
            scope, suffix = raw.split("/", 1)
            suffix = suffix.strip()
            for candidate_scope, root in self._pattern_roots():
                if scope.strip().lower() != candidate_scope:
                    continue
                scoped_names = []
                if suffix.lower().endswith(".yaml"):
                    scoped_names.append(suffix)
                else:
                    scoped_names.append(suffix)
                    scoped_names.append(f"{suffix}.yaml")
                for name in scoped_names:
                    candidate = root / name
                    if candidate.is_file():
                        return candidate
                raise PatternValidationError(
                    f"pattern '{raw}' not found in {root}"
                )

        for _scope, root in self._pattern_roots():
            for name in names:
                candidate = root / name
                if candidate.is_file():
                    return candidate

        raise PatternValidationError(
            f"pattern '{raw}' not found in /patterns/user or /patterns/presets"
        )

    def _resolve_pattern_path(self, pattern_file: str) -> Path:
        raw = str(pattern_file or "").strip()
        if not raw:
            raise PatternValidationError("pattern_file must be set")

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

    def _prepare_start(self, *, run_label: str, allow_override: bool) -> Tuple[bool, str]:
        need_stop = False
        with self._lock:
            if self._running:
                if not allow_override:
                    self._last_action = "busy"
                    return False, "instrument busy; stop current playback first"
                need_stop = True

        if need_stop:
            self._stop_active_playback(reason=f"override:{run_label}", wait_timeout=2.0)

        with self._lock:
            if self._running:
                return False, "instrument still running"
            self._stop_event.clear()
            self._running = True
            self._last_action = f"starting {run_label}"
            self._score_last_error = ""
        self._publish_status()
        return True, "accepted"

    def _publish_best_effort_stop(self) -> None:
        if not self._context_ok():
            return
        self._publish_action({"type": "score_stop"}, log_command=False)
        self._publish_action({"type": "stop"}, log_command=False)
        self._publish_motor_command(speed=0.0, duration=0.0, log_command=False)

    def _stop_active_playback(self, *, reason: str, wait_timeout: float = 1.5) -> str:
        self._stop_event.set()
        self._cancel_metronome_timers()

        worker: Optional[threading.Thread]
        with self._lock:
            worker = self._worker

        if (
            worker is not None
            and worker.is_alive()
            and worker is not threading.current_thread()
        ):
            worker.join(timeout=max(0.1, wait_timeout))

        with self._lock:
            was_running = self._running
            self._running = False
            self._worker = None
            self._active_pattern_path = ""
            self._last_action = f"stopped ({reason})"
            self._score_playing = False
            self._score_status_ok = False
            self._score_current_index = 0
            self._score_total_events = 0

        self._publish_best_effort_stop()
        self._publish_status()

        if was_running:
            return "playback stopped"
        return "instrument already idle; stop command forwarded"

    def _start_worker(self, target, *, name: str, args: Tuple[Any, ...] = ()) -> None:
        worker = threading.Thread(target=target, args=args, name=name, daemon=True)
        with self._lock:
            self._worker = worker
        worker.start()

    def _on_actuate(self, _msg: Empty) -> None:
        mode = str(self.get_parameter("mode").value).strip().lower()
        allow_override = bool(self.get_parameter("allow_override").value)

        ok, message = self._prepare_start(run_label=f"actuate:{mode}", allow_override=allow_override)
        if not ok:
            self._safe_log("info", f"Received /actuate but ignored: {message}")
            return

        if mode == "metronome":
            self._start_metronome()
            return

        self._start_worker(self._run_behavior, name="instrument_behavior_worker")

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

    def _on_play_pattern(
        self,
        request: PlayPattern.Request,
        response: PlayPattern.Response,
    ) -> PlayPattern.Response:
        return self._start_pattern_playback(
            request=request,
            response=response,
            force_host_score=False,
            service_name="play_pattern",
        )

    def _on_play_score(
        self,
        request: PlayPattern.Request,
        response: PlayPattern.Response,
    ) -> PlayPattern.Response:
        return self._start_pattern_playback(
            request=request,
            response=response,
            force_host_score=True,
            service_name="play_score",
        )

    def _start_pattern_playback(
        self,
        *,
        request: PlayPattern.Request,
        response: PlayPattern.Response,
        force_host_score: bool,
        service_name: str,
    ) -> PlayPattern.Response:
        allow_override = bool(self.get_parameter("allow_override").value)

        try:
            if str(request.pattern_path or "").strip():
                resolved = self._resolve_pattern_path(str(request.pattern_path))
            else:
                resolved = self._resolve_pattern_name(str(request.pattern_name))
            _validated = load_and_validate_pattern(str(resolved))
        except Exception as exc:  # noqa: BLE001
            response.accepted = False
            response.message = f"pattern resolve/validate failed: {exc}"
            return response

        ok, message = self._prepare_start(
            run_label=f"pattern:{resolved.name}", allow_override=allow_override
        )
        if not ok:
            response.accepted = False
            response.message = message
            return response

        with self._lock:
            self._active_pattern_path = str(resolved)

        self._start_worker(
            self._run_pattern_from_path,
            name="instrument_pattern_worker",
            args=(resolved, f"service:{service_name}", force_host_score),
        )

        response.accepted = True
        response.message = f"playing pattern {resolved}"
        return response

    def _on_stop(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        message = self._stop_active_playback(reason="service stop", wait_timeout=2.0)
        response.success = True
        response.message = message
        return response

    def _on_list_patterns(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        entries = self._discover_patterns()
        response.success = True
        response.message = "\n".join(entries)
        return response

    def _on_generate_pattern(
        self,
        request: GeneratePattern.Request,
        response: GeneratePattern.Response,
    ) -> GeneratePattern.Response:
        template = str(request.template_name or "").strip().lower()
        if template not in TEMPLATES:
            response.ok = False
            response.message = f"unsupported template '{template}', expected one of {list(TEMPLATES)}"
            response.written_path = ""
            return response

        output_name = str(request.output_name or "").strip() or f"{template}_{int(time.time())}"
        output_dir = str(request.output_dir or "").strip() or "/patterns/user"

        try:
            if template == "score_4bar":
                output_path = self._generate_score_file(
                    output_name=output_name,
                    output_dir=output_dir,
                    bpm=float(request.bpm),
                    repeats=int(request.repeats),
                    speed=float(request.speed),
                    motor_score=str(request.motor_score),
                    melody_score=str(request.melody_score),
                    volume=int(request.beep_volume),
                )
                response.ok = True
                response.message = (
                    f"generated template=score_4bar as {output_path.name}; "
                    "ignored legacy fields duration_sec/gap_sec/degrees; "
                    f"play with /instrument/play_pattern pattern_name: '{output_path.stem}'"
                )
                response.written_path = str(output_path)
                return response

            pattern = build_template_pattern(
                template_name=template,
                output_name=output_name,
                speed=float(request.speed),
                duration_sec=float(request.duration_sec),
                gap_sec=float(request.gap_sec),
                repeats=int(request.repeats),
                bpm=float(request.bpm),
                degrees=int(request.degrees),
                motor_score=str(request.motor_score),
                melody_score=str(request.melody_score),
                beep_volume=int(request.beep_volume),
            )
            output_path = ensure_yaml_output_path(output_dir=output_dir, output_name=output_name)
            write_pattern_yaml(output_path, pattern)
            load_and_validate_pattern(str(output_path))
        except Exception as exc:  # noqa: BLE001
            response.ok = False
            response.message = f"generate failed: {exc}"
            response.written_path = ""
            return response

        response.ok = True
        response.message = (
            f"generated template={template} as {output_path.name}; "
            f"play with /instrument/play_pattern pattern_name: '{output_path.stem}'"
        )
        response.written_path = str(output_path)
        return response

    def _generate_score_file(
        self,
        *,
        output_name: str,
        output_dir: str,
        bpm: float,
        repeats: int,
        speed: float,
        motor_score: str,
        melody_score: str,
        volume: int,
    ) -> Path:
        pattern = build_score_pattern(
            output_name=output_name,
            bpm=bpm,
            repeats=repeats,
            speed=speed,
            motor_score=motor_score,
            melody_score=melody_score,
            beep_volume=volume,
            motor_port="A",
        )
        output_path = ensure_yaml_output_path(output_dir=output_dir, output_name=output_name)
        write_pattern_yaml(output_path, pattern)
        load_and_validate_pattern(str(output_path))
        return output_path

    def _on_generate_score(
        self,
        request: GenerateScore.Request,
        response: GenerateScore.Response,
    ) -> GenerateScore.Response:
        output_name = str(request.name or "").strip() or f"score_{int(time.time())}"
        output_dir = "/patterns/user"
        try:
            output_path = self._generate_score_file(
                output_name=output_name,
                output_dir=output_dir,
                bpm=float(request.bpm),
                repeats=int(request.repeats),
                speed=float(request.speed),
                motor_score=str(request.motor),
                melody_score=str(request.melody),
                volume=int(request.volume),
            )
        except Exception as exc:  # noqa: BLE001
            response.ok = False
            response.message = f"generate score failed: {exc}"
            response.output_path = ""
            response.pattern_name = ""
            return response

        response.ok = True
        response.output_path = str(output_path)
        response.pattern_name = output_path.stem
        response.message = (
            f"generated score as {output_path.name}; "
            f"play with /instrument/play_pattern pattern_name: '{output_path.stem}'"
        )
        return response

    def _run_behavior(self) -> None:
        params = self._read_behavior_params()
        mode = str(params["mode"])
        normalized_mode = mode.strip().lower()

        if normalized_mode == "metronome":
            # Metronome is timer-driven via _start_metronome().
            return

        if normalized_mode == "pattern":
            pattern_file = str(params.get("pattern_file", ""))
            try:
                resolved = self._resolve_pattern_path(pattern_file)
            except Exception as exc:  # noqa: BLE001
                self._safe_log("error", f"Failed to resolve pattern from mode=pattern: {exc}")
                with self._lock:
                    self._running = False
                    self._last_action = f"pattern error: {exc}"
                self._publish_status()
                return

            with self._lock:
                self._active_pattern_path = str(resolved)
            self._run_pattern_from_path(resolved, "actuate")
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
            self._worker = None

        self._publish_best_effort_stop()
        self._publish_status()

    @staticmethod
    def _coerce_speed(value: Any) -> float:
        return max(-1.0, min(1.0, float(value)))

    def _pattern_to_host_score_events(
        self, pattern: Dict[str, Any]
    ) -> Tuple[List[Dict[str, Any]], int]:
        steps = list(pattern.get("steps", []))
        events: List[Dict[str, Any]] = []
        timeline_ms = 0
        event_index = 0

        def add_event(t_rel_ms: int, action: Dict[str, Any]) -> None:
            nonlocal event_index
            event_index += 1
            events.append(
                {
                    "event_id": f"evt_{event_index:04d}",
                    "t_rel_ms": int(max(0, t_rel_ms)),
                    "action": action,
                }
            )

        for step in steps:
            step_type = str(step.get("type", "")).strip().lower()
            gap_sec = max(0.0, float(step.get("gap_sec", 0.0)))
            wait_override = self._optional_nonnegative_float(step.get("wait_sec"))
            port = str(step.get("port", "A")).strip().upper() or "A"
            stop_action = str(step.get("stop_action", "coast")).strip().lower() or "coast"

            if step_type == "sleep":
                timeline_ms += int(round(max(0.0, float(step.get("duration_sec", 0.0))) * 1000.0))
                timeline_ms += int(round(gap_sec * 1000.0))
                continue

            if step_type == "run_for_time":
                duration_sec = max(0.0, float(step.get("duration_sec", 0.0)))
                speed = self._coerce_speed(step.get("velocity", 0.0))
                add_event(
                    timeline_ms,
                    {
                        "type": "run",
                        "port": port,
                        "speed": float(speed),
                    },
                )
                timeline_ms += int(round(duration_sec * 1000.0))
                add_event(
                    timeline_ms,
                    {
                        "type": "stop",
                        "port": port,
                        "stop_action": stop_action,
                    },
                )
                if wait_override is not None:
                    timeline_ms += int(round(wait_override * 1000.0))
                timeline_ms += int(round(gap_sec * 1000.0))
                continue

            if step_type == "beep":
                freq_hz = int(step.get("freq_hz", 440))
                duration_ms = int(step.get("duration_ms", 120))
                volume = int(step.get("volume", 50))
                add_event(
                    timeline_ms,
                    {
                        "type": "beep",
                        "freq_hz": freq_hz,
                        "duration_ms": duration_ms,
                        "volume": volume,
                    },
                )
                wait_after_sec = (
                    wait_override
                    if wait_override is not None
                    else max(0.0, float(duration_ms) / 1000.0)
                )
                timeline_ms += int(round(wait_after_sec * 1000.0))
                timeline_ms += int(round(gap_sec * 1000.0))
                continue

            if step_type in {"run", "set_duty_cycle"}:
                add_event(
                    timeline_ms,
                    {
                        "type": "run" if step_type == "run" else "set_duty_cycle",
                        "port": port,
                        "speed": float(self._coerce_speed(step.get("velocity", 0.0))),
                    },
                )
                if wait_override is not None:
                    timeline_ms += int(round(wait_override * 1000.0))
                timeline_ms += int(round(gap_sec * 1000.0))
                continue

            if step_type == "stop":
                add_event(
                    timeline_ms,
                    {
                        "type": "stop",
                        "port": port,
                        "stop_action": stop_action,
                    },
                )
                if wait_override is not None:
                    timeline_ms += int(round(wait_override * 1000.0))
                timeline_ms += int(round(gap_sec * 1000.0))
                continue

            action_payload: Dict[str, Any] = {
                "type": step_type,
                "port": port,
                "stop_action": stop_action,
            }
            if step_type in {
                "run_for_degrees",
                "run_to_absolute_position",
                "run_to_relative_position",
                "set_duty_cycle",
            }:
                action_payload["speed"] = float(self._coerce_speed(step.get("velocity", 0.0)))
            if step_type in {"run_for_degrees", "run_to_relative_position"}:
                action_payload["degrees"] = int(step.get("degrees", 0))
            if step_type == "run_to_absolute_position":
                action_payload["position_degrees"] = int(step.get("position_degrees", 0))
            add_event(timeline_ms, action_payload)

            if wait_override is not None:
                timeline_ms += int(round(wait_override * 1000.0))
            else:
                timeline_ms += int(
                    round(DEFAULT_BLOCKING_WAIT_SEC.get(step_type, 0.4) * 1000.0)
                )
            timeline_ms += int(round(gap_sec * 1000.0))

        events.sort(key=lambda row: int(row.get("t_rel_ms", 0)))
        return events, timeline_ms

    def _run_pattern_via_host_score(
        self,
        *,
        pattern: Dict[str, Any],
        resolved: Path,
        source: str,
    ) -> None:
        events, expected_length_ms = self._pattern_to_host_score_events(pattern)
        if not events:
            self._safe_log("warning", f"Pattern {resolved} has no executable events for host_score mode")
            with self._lock:
                self._running = False
                self._worker = None
                self._active_pattern_path = ""
                self._last_action = "idle"
            self._publish_status()
            return

        score_counter_before = self._get_score_status_counter()
        start_monotonic = time.monotonic() + 0.05
        expected_end_monotonic = start_monotonic + (expected_length_ms / 1000.0)

        payload = {
            "type": "score_play",
            "score": {
                "port": "A",
                "events": events,
                "start_mode": "at_time",
                "start_time_ms": 50,
            },
        }
        if not self._publish_action(payload):
            self._safe_log("error", f"Failed to publish score_play for {resolved}")
            with self._lock:
                self._running = False
                self._worker = None
                self._active_pattern_path = ""
                self._last_action = "score_play publish failed"
            self._publish_status()
            return

        self._safe_log(
            "info",
            (
                f"Host-scheduled score started for pattern={resolved.name} source={source}; "
                f"events={len(events)} expected_length_ms={expected_length_ms}"
            ),
        )
        with self._lock:
            self._score_playing = True
            self._score_status_ok = False
            self._score_current_index = 0
            self._score_total_events = len(events)
            self._score_start_time_monotonic = start_monotonic
            self._score_expected_end_monotonic = expected_end_monotonic
            self._score_last_error = ""
            self._last_action = f"host_score started events={len(events)}"
        self._publish_status()

        deadline = expected_end_monotonic + 12.0
        seen_score_status = False
        invalid_score_status_count = 0
        invalid_status_warned = False
        remote_started = False
        success = False

        while not self._stop_event.is_set():
            with self._lock:
                counter_now = self._score_state_counter
                remote_playing = self._score_playing
                remote_status_ok = self._score_status_ok
                remote_current_index = self._score_current_index
                remote_total_events = self._score_total_events
                remote_error = self._score_last_error

            if counter_now > score_counter_before:
                seen_score_status = True
                if not remote_status_ok:
                    invalid_score_status_count += 1
                    if remote_started and invalid_score_status_count >= 3:
                        if not invalid_status_warned:
                            self._safe_log(
                                "warning",
                                (
                                    "Host score status became invalid while waiting for completion: "
                                    f"{remote_error or 'unknown status error'}; "
                                    "continuing to wait for recovery until deadline."
                                ),
                            )
                            invalid_status_warned = True
                        time.sleep(0.05)
                        continue
                else:
                    invalid_score_status_count = 0
                    invalid_status_warned = False
                    if (not remote_started) and (remote_playing or remote_current_index > 0):
                        remote_started = True
                        self._safe_log(
                            "info",
                            (
                                "Host score acknowledged start: "
                                f"current_index={remote_current_index}, "
                                f"total_events={remote_total_events}, "
                                f"playing={str(remote_playing).lower()}"
                            ),
                        )
                    if not remote_started:
                        time.sleep(0.05)
                        continue
                    if not remote_playing:
                        success = (
                            (remote_total_events > 0)
                            and (remote_current_index >= remote_total_events)
                            and (not bool(remote_error))
                        )
                        if not success:
                            self._safe_log(
                                "warning",
                                (
                                    "Host score ended without full completion: "
                                    f"current_index={remote_current_index}, "
                                    f"total_events={remote_total_events}, "
                                    f"last_error={remote_error or '<none>'}"
                                ),
                            )
                        break

            if time.monotonic() > deadline:
                self._safe_log(
                    "warning",
                    (
                        f"Timed out waiting for host_score completion for {resolved.name}; "
                        "falling back to local completion handling."
                    ),
                )
                break
            time.sleep(0.05)

        if self._stop_event.is_set():
            self._publish_action({"type": "score_stop"}, log_command=False)
            success = False

        if success and seen_score_status and self._context_ok():
            try:
                self._done_pub.publish(Empty())
                self._safe_log("info", "Host-scheduled pattern complete; published /done")
            except Exception:  # noqa: BLE001
                pass

        with self._lock:
            self._running = False
            self._worker = None
            self._active_pattern_path = ""
            self._score_playing = False
            self._score_status_ok = False
            self._score_current_index = 0
            self._score_total_events = 0
            self._last_action = "idle" if success else "host_score finished with warning"

        self._publish_best_effort_stop()
        self._publish_status()

    def _run_pattern_from_path(
        self,
        resolved: Path,
        source: str,
        force_host_score: bool = False,
    ) -> None:
        try:
            pattern = load_and_validate_pattern(str(resolved))
        except Exception as exc:  # noqa: BLE001
            self._safe_log("error", f"Failed to load pattern from {resolved}: {exc}")
            with self._lock:
                self._running = False
                self._worker = None
                self._active_pattern_path = ""
                self._last_action = f"pattern error: {exc}"
            self._publish_status()
            return

        pattern_name = str(pattern.get("name", resolved.stem))
        steps = pattern.get("steps", [])
        self._safe_log(
            "info",
            f"Executing pattern={pattern_name} from {resolved} source={source} with {len(steps)} steps",
        )

        execution_mode = "host_score" if force_host_score else self._execution_mode()
        if execution_mode == "host_score":
            self._run_pattern_via_host_score(
                pattern=pattern,
                resolved=resolved,
                source=source,
            )
            return

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
            self._worker = None
            self._active_pattern_path = ""
            self._last_action = "idle"

        self._publish_best_effort_stop()
        self._publish_status()

    def _execute_pattern_step(self, *, step: Dict[str, Any], index: int, total: int) -> bool:
        step_type = str(step.get("type", "")).strip().lower()
        if not step_type:
            return False

        gap_sec = max(0.0, float(step.get("gap_sec", 0.0)))
        wait_override = self._optional_nonnegative_float(step.get("wait_sec"))

        self._safe_log("info", f"Pattern step {index}/{total}: type={step_type}")

        if step_type == "sleep":
            duration_sec = max(0.0, float(step.get("duration_sec", 0.0)))
            with self._lock:
                self._last_action = (
                    f"pattern sleep step {index}/{total} duration={duration_sec:.2f}s"
                )
            self._publish_status()
            if not self._sleep_pattern(
                duration_sec,
                reason=f"step {index}/{total} sleep",
            ):
                return False
            return self._apply_step_gap(gap_sec=gap_sec, index=index, total=total)

        if step_type == "run_for_time":
            speed = self._coerce_speed(step.get("velocity", 0.0))
            duration_sec = max(0.0, float(step.get("duration_sec", 0.0)))
            port = str(step.get("port", "A"))
            stop_action = str(step.get("stop_action", "coast"))

            run_action: Dict[str, Any] = {
                "type": "run",
                "speed": speed,
                "port": port,
            }
            if "comment" in step:
                run_action["comment"] = str(step["comment"])

            if not self._publish_action(run_action):
                return False

            with self._lock:
                self._last_action = (
                    f"pattern run_for_time step {index}/{total} "
                    f"speed={speed:.2f} duration={duration_sec:.2f}s"
                )
            self._publish_status()

            if not self._sleep_pattern(
                duration_sec,
                reason=f"step {index}/{total} run_for_time duration",
            ):
                return False

            stop_payload = {
                "type": "stop",
                "port": port,
                "stop_action": stop_action,
            }
            if not self._publish_action(stop_payload):
                return False
            self._publish_motor_command(speed=0.0, duration=0.0, log_command=False)

            if wait_override is not None and wait_override > 0.0:
                if not self._sleep_pattern(
                    wait_override,
                    reason=f"step {index}/{total} wait_sec after run_for_time",
                ):
                    return False

            return self._apply_step_gap(gap_sec=gap_sec, index=index, total=total)

        if step_type == "beep":
            freq_hz = int(step.get("freq_hz", 440))
            duration_ms = int(step.get("duration_ms", 120))
            volume = int(step.get("volume", 50))
            beep_action: Dict[str, Any] = {
                "type": "beep",
                "freq_hz": freq_hz,
                "duration_ms": duration_ms,
                "volume": volume,
            }
            if "comment" in step:
                beep_action["comment"] = str(step["comment"])

            self._safe_log(
                "info",
                f"Beep freq={freq_hz}Hz duration={duration_ms}ms volume={volume}",
            )

            if not self._publish_action(beep_action):
                return False

            with self._lock:
                self._last_action = (
                    f"pattern beep step {index}/{total} "
                    f"freq={freq_hz}Hz duration={duration_ms}ms volume={volume}"
                )
            self._publish_status()

            wait_after_sec = (
                wait_override if wait_override is not None else max(0.0, float(duration_ms) / 1000.0)
            )
            if wait_after_sec > 0.0:
                if not self._sleep_pattern(
                    wait_after_sec,
                    reason=f"step {index}/{total} beep wait",
                ):
                    return False

            return self._apply_step_gap(gap_sec=gap_sec, index=index, total=total)

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

        if step_type in {"run_for_degrees", "run_to_absolute_position", "run_to_relative_position"}:
            if wait_override is not None:
                if not self._sleep_pattern(
                    wait_override,
                    reason=f"step {index}/{total} explicit wait_sec",
                ):
                    return False
            else:
                state_counter_before = self._get_spike_state_counter()
                if not self._wait_for_step_idle_or_default(
                    step_type=step_type,
                    state_counter_before=state_counter_before,
                    index=index,
                    total=total,
                ):
                    return False
        elif wait_override is not None and wait_override > 0.0:
            if not self._sleep_pattern(
                wait_override,
                reason=f"step {index}/{total} explicit wait_sec",
            ):
                return False

        return self._apply_step_gap(gap_sec=gap_sec, index=index, total=total)

    def _on_spike_state(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        state = str(payload.get("state", "")).strip().lower()
        score = payload.get("score")
        with self._lock:
            self._host_connected = state not in {"", "unreachable"}
            self._spike_state_value = state or "unknown"
            self._spike_state_counter += 1
            self._spike_state_last_update_monotonic = time.monotonic()
            if isinstance(score, dict):
                self._score_state_counter += 1
                self._score_status_ok = bool(score.get("ok", False))
                self._score_playing = bool(score.get("playing", False))
                self._score_current_index = int(
                    score.get("current_index", self._score_current_index)
                )
                self._score_total_events = int(
                    score.get("total_events", self._score_total_events)
                )
                self._score_start_time_monotonic = float(
                    score.get("start_time_monotonic", self._score_start_time_monotonic)
                )
                self._score_expected_end_monotonic = float(
                    score.get("expected_end_monotonic", self._score_expected_end_monotonic)
                )
                if not self._score_status_ok:
                    score_error = str(score.get("error", "") or "")
                    self._score_last_error = score_error or "score status unavailable"
                else:
                    self._score_last_error = str(score.get("last_error", "") or "")

    @staticmethod
    def _optional_nonnegative_float(value: Any) -> Optional[float]:
        if value is None:
            return None
        try:
            return max(0.0, float(value))
        except (TypeError, ValueError):
            return None

    def _sleep_pattern(self, duration_sec: float, *, reason: str) -> bool:
        safe_duration = max(0.0, float(duration_sec))
        if safe_duration <= 0.0:
            return True
        self._safe_log("info", f"Pattern sleep {safe_duration:.2f}s ({reason})")
        return self._sleep_with_cancel(safe_duration)

    def _apply_step_gap(self, *, gap_sec: float, index: int, total: int) -> bool:
        if gap_sec <= 0.0:
            return True
        return self._sleep_pattern(
            gap_sec,
            reason=f"step {index}/{total} gap_sec",
        )

    def _get_spike_state_counter(self) -> int:
        with self._lock:
            return self._spike_state_counter

    def _get_score_status_counter(self) -> int:
        with self._lock:
            return self._score_state_counter

    def _wait_for_step_idle_or_default(
        self,
        *,
        step_type: str,
        state_counter_before: int,
        index: int,
        total: int,
    ) -> bool:
        fallback_sec = DEFAULT_BLOCKING_WAIT_SEC.get(step_type, 0.6)
        with self._lock:
            has_state = self._spike_state_counter > 0

        if not has_state:
            self._safe_log(
                "info",
                (
                    f"Pattern step {index}/{total} ({step_type}) has no /spike/state updates yet; "
                    f"using fallback wait {fallback_sec:.2f}s."
                ),
            )
            return self._sleep_pattern(
                fallback_sec,
                reason=f"step {index}/{total} fallback wait (no state)",
            )

        deadline = time.monotonic() + STATE_IDLE_POLL_TIMEOUT_SEC
        while time.monotonic() < deadline:
            if self._stop_event.is_set():
                return False
            with self._lock:
                state_now = self._spike_state_value
                counter_now = self._spike_state_counter
            if counter_now > state_counter_before and state_now == "idle":
                return True
            time.sleep(0.05)

        self._safe_log(
            "warning",
            (
                f"Pattern step {index}/{total} ({step_type}) timed out waiting for /spike/state idle; "
                f"using fallback wait {fallback_sec:.2f}s."
            ),
        )
        return self._sleep_pattern(
            fallback_sec,
            reason=f"step {index}/{total} fallback wait (state timeout)",
        )

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
            "execution_mode": str(self.get_parameter("execution_mode").value),
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
            active_pattern_path = self._active_pattern_path
            score_playing = self._score_playing
            score_status_ok = self._score_status_ok
            score_current_index = self._score_current_index
            score_total_events = self._score_total_events
            score_start = self._score_start_time_monotonic
            score_expected_end = self._score_expected_end_monotonic
            score_last_error = self._score_last_error

        return (
            f"state={state};last_action={last_action};"
            f"participant_id={int(self.get_parameter('participant_id').value)};"
            f"name={str(self.get_parameter('name').value)};"
            f"mode={str(self.get_parameter('mode').value)};"
            f"execution_mode={self._execution_mode()};"
            f"playing={str(score_playing).lower()};"
            f"score_status_ok={str(score_status_ok).lower()};"
            f"score_progress={score_current_index}/{score_total_events};"
            f"start_time_monotonic={score_start:.3f};"
            f"expected_end_monotonic={score_expected_end:.3f};"
            f"last_error={score_last_error};"
            f"active_pattern={active_pattern_path};"
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

        self._stop_active_playback(reason="shutdown", wait_timeout=1.0)

        if self._context_ok():
            self._publish_best_effort_stop()


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
