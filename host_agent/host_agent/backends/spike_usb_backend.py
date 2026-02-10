import ast
import glob
import json
import logging
import os
import re
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from host_agent.score_timeline import normalize_score_payload
from host_agent.serial_repl import RawExecResult, SerialMicroPythonClient
from host_agent.timing_telemetry import TimingTelemetry


USB_OK_MARKER = "__SPIKE_USB_OK__"
USB_ERR_MARKER = "__SPIKE_USB_ERR__"
USB_STATUS_MARKER = "__SPIKE_USB_STATUS__"
STOP_SPEED_EPSILON = 1e-3
SOUND_PROBE_CACHE_TTL_SEC = 10.0
SCORE_BATCH_MAX_EVENTS = 24
SCORE_BATCH_MAX_SPAN_MS = 900
SCORE_SCHEDULER_LEAD_SEC = 0.03
VALID_STOP_ACTIONS = {"coast", "brake", "hold"}

_SERIAL_CANDIDATE_REGEX = re.compile(
    r"^/dev/(cu|tty)\.(usbmodem|usbserial|SLAB_USBtoUART|wchusbserial|ttyUSB|ttyACM)",
    re.IGNORECASE,
)


def _is_serial_candidate(device: str, description: str = "", hwid: str = "") -> bool:
    if _SERIAL_CANDIDATE_REGEX.match(device):
        return True

    lowered = f"{device} {description} {hwid}".lower()
    return any(
        token in lowered
        for token in ("usb", "modem", "serial", "lego", "spike", "cp210", "acm", "uart")
    )


def list_serial_ports() -> List[Dict[str, str]]:
    rows: List[Dict[str, str]] = []
    seen: set[str] = set()

    try:
        from serial.tools import list_ports  # type: ignore

        for port in list_ports.comports():
            device = str(port.device)
            if not device or device in seen:
                continue
            if not _is_serial_candidate(
                device=device,
                description=str(getattr(port, "description", "")),
                hwid=str(getattr(port, "hwid", "")),
            ):
                continue
            seen.add(device)
            rows.append(
                {
                    "device": device,
                    "description": str(getattr(port, "description", "")),
                    "hwid": str(getattr(port, "hwid", "")),
                }
            )
    except Exception:  # noqa: BLE001
        pass

    for pattern in (
        "/dev/cu.usbmodem*",
        "/dev/cu.usbserial*",
        "/dev/cu.SLAB_USBtoUART*",
        "/dev/cu.wchusbserial*",
        "/dev/tty.usbmodem*",
        "/dev/tty.usbserial*",
        "/dev/tty.SLAB_USBtoUART*",
    ):
        for device in glob.glob(pattern):
            if device in seen:
                continue
            seen.add(device)
            rows.append({"device": device, "description": "", "hwid": ""})

    rows.sort(key=lambda row: row.get("device", ""))
    return rows


class SpikeUsbBackend:
    """SPIKE backend via USB serial RAW REPL execution."""

    name = "spike_usb"

    def __init__(
        self,
        serial_port: str = "auto",
        baud: int = 115200,
        motor_port: str = "A",
        prompt_timeout: float = 2.0,
        command_timeout: float = 8.0,
        sync_retries: int = 2,
        debug_repl_snippets: bool = False,
        **_: Any,
    ) -> None:
        self._serial_port_config = (serial_port or "auto").strip()
        self._baud = int(baud)
        self._motor_port = (motor_port or "A").strip().upper()
        self._prompt_timeout = max(0.5, float(prompt_timeout))
        self._command_timeout = max(1.0, float(command_timeout))
        self._sync_retries = max(2, int(sync_retries))
        env_debug = str(os.environ.get("HOST_AGENT_DEBUG_SNIPPETS", "")).strip().lower()
        self._debug_repl_snippets = bool(debug_repl_snippets) or env_debug in {
            "1",
            "true",
            "yes",
            "on",
        }

        self._lock = threading.Lock()
        self._client: Optional[SerialMicroPythonClient] = None
        self._state = "idle"
        self._last_speed = 0.0
        self._timestamp = time.time()
        self._spike_connected = False
        self._last_error = ""
        self._resolved_port = ""
        self._sound_supported_cache: Optional[bool] = None
        self._sound_probe_monotonic = 0.0
        self._score_lock = threading.Lock()
        self._score_thread: Optional[threading.Thread] = None
        self._score_stop_event = threading.Event()
        self._score_playing = False
        self._score_current_index = 0
        self._score_total_events = 0
        self._score_started_monotonic = 0.0
        self._score_expected_end_monotonic = 0.0
        self._score_last_error = ""
        self._score_batch_mode_used = False
        self._score_batch_disabled = False
        self._score_batch_disabled_reason = ""
        self._score_compile_check: Dict[str, Any] = {"ok": True}
        self._score_session_id = 0
        self._timing = TimingTelemetry(max_events=5000)

    @staticmethod
    def _trim_text(text: str, limit: int = 800) -> str:
        normalized = (text or "").strip()
        if len(normalized) <= limit:
            return normalized
        return f"{normalized[:limit]}...(truncated)"

    @staticmethod
    def _clip_speed(speed: float) -> float:
        return max(-1.0, min(1.0, float(speed)))

    @staticmethod
    def _velocity_units(speed: float) -> int:
        return int(max(-1000, min(1000, round(float(speed) * 1000.0))))

    @staticmethod
    def _duty_units(speed: float) -> int:
        return int(max(-100, min(100, round(float(speed) * 100.0))))

    @staticmethod
    def _clamp_freq_hz(value: int) -> int:
        return int(max(50, min(5000, int(value))))

    @staticmethod
    def _clamp_duration_ms(value: int) -> int:
        return int(max(10, min(5000, int(value))))

    @staticmethod
    def _clamp_volume(value: int) -> int:
        return int(max(0, min(100, int(value))))

    @staticmethod
    def _prepare_snippet(lines: List[str]) -> str:
        clean_lines = [str(line).replace("\r", "") for line in lines]
        return "\n".join(clean_lines).rstrip("\n") + "\n"

    @staticmethod
    def _normalize_stop_action(stop_action: Any) -> str:
        candidate = str(stop_action or "coast").strip().lower()
        if candidate in VALID_STOP_ACTIONS:
            return candidate
        return "coast"

    @staticmethod
    def _snippet_excerpt(snippet: str, line_no: int, radius: int = 2) -> str:
        rows = (snippet or "").splitlines()
        if not rows:
            return ""
        if line_no <= 0:
            line_no = 1
        start = max(1, line_no - max(1, radius))
        end = min(len(rows), line_no + max(1, radius))
        excerpt_lines: List[str] = []
        for idx in range(start, end + 1):
            marker = ">" if idx == line_no else " "
            excerpt_lines.append(f"{marker}{idx:04d}: {rows[idx - 1]}")
        return "\n".join(excerpt_lines)

    def _compile_check_snippet(self, snippet: str, *, label: str) -> Dict[str, Any]:
        if "\r" in snippet:
            return {
                "ok": False,
                "label": label,
                "line": 0,
                "error": "snippet contains carriage-return characters",
                "offending_line": "",
                "excerpt": self._snippet_excerpt(snippet, 1),
            }
        if not snippet.endswith("\n"):
            return {
                "ok": False,
                "label": label,
                "line": 0,
                "error": "snippet must end with newline",
                "offending_line": "",
                "excerpt": self._snippet_excerpt(snippet, 1),
            }

        try:
            ast.parse(snippet)
        except SyntaxError as exc:
            line_no = int(exc.lineno or 0)
            rows = snippet.splitlines()
            offending = rows[line_no - 1] if 0 < line_no <= len(rows) else ""
            error_msg = str(exc.msg or "syntax error")
            return {
                "ok": False,
                "label": label,
                "line": line_no,
                "error": error_msg,
                "offending_line": offending,
                "excerpt": self._snippet_excerpt(snippet, line_no or 1),
            }

        return {"ok": True, "label": label}

    def _write_score_batch_artifacts(
        self,
        *,
        snippet: str,
        encoded_actions: List[Dict[str, Any]],
        compile_check: Dict[str, Any],
        reason: str = "",
    ) -> None:
        if not self._debug_repl_snippets:
            return

        artifacts_dir = Path("artifacts")
        try:
            artifacts_dir.mkdir(parents=True, exist_ok=True)
            snippet_path = artifacts_dir / "last_score_batch_snippet.py"
            events_path = artifacts_dir / "last_score_batch_events.json"
            snippet_path.write_text(snippet, encoding="utf-8")
            events_payload: Dict[str, Any] = {
                "timestamp": time.time(),
                "reason": str(reason or ""),
                "event_count": len(encoded_actions),
                "compile_check": compile_check,
                "events": encoded_actions,
            }
            events_path.write_text(
                json.dumps(events_payload, indent=2, sort_keys=True),
                encoding="utf-8",
            )
        except Exception as exc:  # noqa: BLE001
            logging.warning("[SPIKE_USB] Failed writing score batch artifacts: %s", exc)

    @staticmethod
    def _is_syntax_error_text(value: str) -> bool:
        normalized = str(value or "").lower()
        return "syntaxerror" in normalized or "invalid syntax" in normalized

    def _record_error(self, message: str) -> None:
        self._last_error = str(message)
        self._timestamp = time.time()

    def _normalize_port(self, port: str = "") -> str:
        candidate = str(port or "").strip().upper()
        if not candidate:
            candidate = self._motor_port
        if candidate not in {"A", "B", "C", "D", "E", "F"}:
            candidate = self._motor_port if self._motor_port in {"A", "B", "C", "D", "E", "F"} else "A"
        return candidate

    def _resolve_serial_port(self) -> Tuple[Optional[str], str]:
        configured = self._serial_port_config
        if configured and configured.lower() != "auto":
            self._resolved_port = configured
            return configured, ""

        candidates = list_serial_ports()
        devices = [row["device"] for row in candidates if row.get("device")]

        if len(devices) == 1:
            self._resolved_port = devices[0]
            return devices[0], ""

        if not devices:
            return None, "No USB serial candidates found. Use --serial-port /dev/cu.usbmodemXXXX."

        joined = ", ".join(devices)
        return (
            None,
            f"Multiple USB serial candidates found ({joined}). Use --serial-port to select one.",
        )

    def _reset_client(self) -> None:
        if self._client is not None:
            try:
                self._client.close()
            except Exception:  # noqa: BLE001
                pass
        self._client = None

    def _ensure_client(self) -> Tuple[Optional[SerialMicroPythonClient], str]:
        port, error = self._resolve_serial_port()
        if port is None:
            return None, error

        if self._client is not None:
            return self._client, ""

        try:
            self._client = SerialMicroPythonClient(
                port=port,
                baud=self._baud,
                prompt_timeout=self._prompt_timeout,
                exec_timeout=self._command_timeout,
                sync_retries=self._sync_retries,
            )
            self._client.open()
        except Exception as exc:  # noqa: BLE001
            self._client = None
            return None, str(exc)
        return self._client, ""

    def _execute(
        self,
        snippet: str,
        timeout: Optional[float] = None,
        *,
        retry_on_syntax_error: bool = False,
    ) -> RawExecResult:
        client, error = self._ensure_client()
        if client is None:
            return RawExecResult(ok=False, error=error)

        try:
            result = client.execute_raw(
                snippet,
                timeout=timeout,
                retry_on_syntax_error=retry_on_syntax_error,
            )
        except Exception as exc:  # noqa: BLE001
            result = RawExecResult(ok=False, error=str(exc))
            self._reset_client()

        if not result.ok and "raw repl sync failed" in (result.error or "").lower():
            result.error = (
                "raw repl sync failed; reset hub and close other apps that may hold the serial port"
            )
            self._reset_client()

        return result

    def _extract_error_from_result(self, result: RawExecResult, default: str) -> str:
        stdout = result.stdout or ""
        stderr = result.stderr or ""
        result_error = (result.error or "").strip()

        marker_index = stdout.find(USB_ERR_MARKER)
        if marker_index >= 0:
            detail = stdout[marker_index + len(USB_ERR_MARKER) :].strip(" :\n\r\t")
            if detail:
                return detail

        if result_error and "raw repl sync failed" in result_error.lower():
            return result_error
        if stderr.strip():
            return stderr.strip()
        if result_error:
            return result_error
        if stdout.strip() and USB_OK_MARKER not in stdout:
            return self._trim_text(stdout.strip(), limit=300)
        return default

    @staticmethod
    def _successful(result: RawExecResult) -> bool:
        return bool(result.ok and USB_OK_MARKER in (result.stdout or "") and not result.stderr.strip())

    def _build_core_snippet(self, *, port_letter: str, body_lines: List[str]) -> str:
        lines = [
            "try:",
            "    import motor",
            "except Exception as _e:",
            "    raise RuntimeError('Not running LEGO SPIKE firmware / knowledge base API. import motor failed: ' + repr(_e))",
            "",
            "try:",
            "    from hub import port",
            "except Exception as _e:",
            "    raise RuntimeError('Not running LEGO SPIKE firmware / knowledge base API. from hub import port failed: ' + repr(_e))",
            "",
            f"_letter = {port_letter!r}",
            "try:",
            "    _p = getattr(port, _letter)",
            "except Exception as _e:",
            "    _available = [_n for _n in dir(port) if len(_n) == 1 and 'A' <= _n <= 'Z']",
            "    raise RuntimeError('Invalid motor port ' + _letter + ' available=' + repr(_available) + ' err=' + repr(_e))",
            "",
        ]
        lines.extend(body_lines)
        lines.extend([
            "",
            f"print({USB_OK_MARKER!r})",
        ])

        wrapped: List[str] = ["try:"]
        for line in lines:
            if line:
                wrapped.append(f"    {line}")
            else:
                wrapped.append("")
        wrapped.extend(
            [
                "except Exception as _e:",
                f"    print({USB_ERR_MARKER!r} + ' ' + repr(_e))",
            ]
        )
        return self._prepare_snippet(wrapped)

    def _build_sound_snippet(self, body_lines: List[str]) -> str:
        lines = [
            "try:",
            "    from hub import sound",
            "except Exception as _e:",
            "    raise RuntimeError('Not running LEGO SPIKE firmware / knowledge base API. from hub import sound failed: ' + repr(_e))",
            "",
        ]
        lines.extend(body_lines)
        lines.extend(
            [
                "",
                f"print({USB_OK_MARKER!r})",
            ]
        )

        wrapped: List[str] = ["try:"]
        for line in lines:
            if line:
                wrapped.append(f"    {line}")
            else:
                wrapped.append("")
        wrapped.extend(
            [
                "except Exception as _e:",
                f"    print({USB_ERR_MARKER!r} + ' ' + repr(_e))",
            ]
        )
        return self._prepare_snippet(wrapped)

    def _stop_lines(self, stop_action: str) -> List[str]:
        normalized = str(stop_action or "coast").strip().lower()
        return [
            f"_stop_action = {normalized!r}",
            "try:",
            "    motor.stop(_p)",
            "except Exception as _e1:",
            "    try:",
            "        motor.run(_p, 0)",
            "    except Exception as _e2:",
            "        try:",
            "            motor.set_duty_cycle(_p, 0)",
            "        except Exception as _e3:",
            "            raise RuntimeError('stop failed for action=' + repr(_stop_action) + ': ' + repr(_e1) + '; ' + repr(_e2) + '; ' + repr(_e3))",
        ]

    def _set_idle(self) -> None:
        self._state = "idle"
        self._last_speed = 0.0
        self._timestamp = time.time()

    def _set_running(self, speed: float) -> None:
        self._state = "running"
        self._last_speed = float(speed)
        self._timestamp = time.time()

    def _probe_sound_supported(self, *, force: bool = False) -> bool:
        now = time.monotonic()
        if (
            (not force)
            and self._sound_supported_cache is not None
            and (now - self._sound_probe_monotonic) < SOUND_PROBE_CACHE_TTL_SEC
        ):
            return bool(self._sound_supported_cache)

        snippet = self._build_sound_snippet(
            body_lines=[
                "if not hasattr(sound, 'beep'):",
                "    raise RuntimeError('Missing hub.sound.beep API; firmware/API mismatch.')",
            ]
        )
        result = self._execute(snippet=snippet, timeout=max(2.0, self._prompt_timeout + 1.0))
        supported = self._successful(result)

        self._sound_probe_monotonic = now
        self._sound_supported_cache = supported
        return supported

    def _run_checked(
        self,
        *,
        snippet: str,
        timeout: float,
        success_payload: Dict[str, Any],
        failure_default: str,
        fail_key: str,
    ) -> Dict[str, Any]:
        result = self._execute(snippet=snippet, timeout=timeout)
        if self._successful(result):
            self._spike_connected = True
            self._last_error = ""
            return dict(success_payload)

        self._spike_connected = False
        error = self._extract_error_from_result(result, default=failure_default)
        self._record_error(error)
        logging.warning("[SPIKE_USB] %s failed: %s", fail_key, error)
        if self._debug_repl_snippets:
            logging.warning("[SPIKE_USB] failing snippet:\n%s", snippet)

        payload = {fail_key: False, "error": error}
        payload["stdout"] = self._trim_text(result.stdout)
        payload["stderr"] = self._trim_text(result.stderr)
        if self._debug_repl_snippets:
            payload["snippet"] = self._trim_text(snippet, limit=3000)
        return payload

    def run_with_diagnostics(self, speed: float, duration: float, port: str = "") -> Dict[str, Any]:
        with self._lock:
            safe_speed = self._clip_speed(speed)
            advisory_duration = max(0.0, float(duration))
            port_letter = self._normalize_port(port)
            velocity = self._velocity_units(safe_speed)

            snippet = self._build_core_snippet(
                port_letter=port_letter,
                body_lines=[
                    f"_velocity = {velocity}",
                    "motor.run(_p, _velocity)",
                ],
            )

            payload = self._run_checked(
                snippet=snippet,
                timeout=max(2.0, self._command_timeout),
                success_payload={
                    "accepted": True,
                    "note": "nonblocking; duration handled by ROS stop commands",
                    "duration_advisory_sec": advisory_duration,
                    "port": port_letter,
                },
                failure_default="USB run command failed",
                fail_key="accepted",
            )
            if bool(payload.get("accepted", False)):
                if abs(safe_speed) < STOP_SPEED_EPSILON:
                    self._set_idle()
                else:
                    self._set_running(safe_speed)
            return payload

    def run(self, speed: float, duration: float, port: str = "") -> Dict[str, Any]:
        return self.run_with_diagnostics(speed=speed, duration=duration, port=port)

    def stop_with_diagnostics(self, port: str = "", stop_action: str = "coast") -> Dict[str, Any]:
        with self._lock:
            port_letter = self._normalize_port(port)
            snippet = self._build_core_snippet(
                port_letter=port_letter,
                body_lines=self._stop_lines(stop_action),
            )
            payload = self._run_checked(
                snippet=snippet,
                timeout=max(2.0, self._command_timeout),
                success_payload={"stopped": True, "port": port_letter, "stop_action": stop_action},
                failure_default="USB stop command failed",
                fail_key="stopped",
            )
            self._set_idle()
            return payload

    def stop(self, port: str = "", stop_action: str = "coast") -> Dict[str, Any]:
        return self.stop_with_diagnostics(port=port, stop_action=stop_action)

    def run_for_degrees(
        self,
        port: str,
        speed: float,
        degrees: int,
        stop_action: str = "coast",
    ) -> Dict[str, Any]:
        with self._lock:
            port_letter = self._normalize_port(port)
            safe_speed = self._clip_speed(speed)
            velocity = self._velocity_units(safe_speed)
            move_degrees = int(degrees)
            if move_degrees == 0:
                return {"accepted": False, "error": "degrees cannot be 0"}

            body_lines = [
                f"_velocity = {velocity}",
                f"_degrees = {move_degrees}",
                "if hasattr(motor, 'run_for_degrees'):",
                "    motor.run_for_degrees(_p, _degrees, _velocity)",
                "elif hasattr(motor, 'run_angle'):",
                "    motor.run_angle(_p, _velocity, _degrees)",
                "else:",
                "    raise RuntimeError('Missing motor.run_for_degrees API; firmware/API mismatch (also checked run_angle).')",
            ]
            body_lines.extend(self._stop_lines(stop_action))

            payload = self._run_checked(
                snippet=self._build_core_snippet(port_letter=port_letter, body_lines=body_lines),
                timeout=max(3.0, self._command_timeout),
                success_payload={"accepted": True, "port": port_letter},
                failure_default="USB run_for_degrees failed",
                fail_key="accepted",
            )
            self._set_idle()
            return payload

    def run_to_absolute(
        self,
        port: str,
        speed: float,
        position_degrees: int,
        stop_action: str = "coast",
    ) -> Dict[str, Any]:
        with self._lock:
            port_letter = self._normalize_port(port)
            safe_speed = self._clip_speed(speed)
            velocity = self._velocity_units(safe_speed)
            target = int(position_degrees)

            body_lines = [
                f"_velocity = {velocity}",
                f"_target = {target}",
                "if hasattr(motor, 'run_to_absolute_position'):",
                "    motor.run_to_absolute_position(_p, _target, _velocity)",
                "elif hasattr(motor, 'run_to_position'):",
                "    motor.run_to_position(_p, _target, _velocity)",
                "elif hasattr(motor, 'run_target'):",
                "    motor.run_target(_p, _velocity, _target)",
                "else:",
                "    raise RuntimeError('Missing motor.run_to_absolute_position API; firmware/API mismatch (checked run_to_position/run_target).')",
            ]
            body_lines.extend(self._stop_lines(stop_action))

            payload = self._run_checked(
                snippet=self._build_core_snippet(port_letter=port_letter, body_lines=body_lines),
                timeout=max(3.0, self._command_timeout),
                success_payload={"accepted": True, "port": port_letter},
                failure_default="USB run_to_absolute failed",
                fail_key="accepted",
            )
            self._set_idle()
            return payload

    def run_to_relative(
        self,
        port: str,
        speed: float,
        degrees: int,
        stop_action: str = "coast",
    ) -> Dict[str, Any]:
        with self._lock:
            port_letter = self._normalize_port(port)
            safe_speed = self._clip_speed(speed)
            velocity = self._velocity_units(safe_speed)
            delta = int(degrees)
            if delta == 0:
                return {"accepted": False, "error": "degrees cannot be 0"}

            body_lines = [
                f"_velocity = {velocity}",
                f"_delta = {delta}",
                "if hasattr(motor, 'run_to_relative_position'):",
                "    motor.run_to_relative_position(_p, _delta, _velocity)",
                "elif hasattr(motor, 'run_for_degrees'):",
                "    motor.run_for_degrees(_p, _delta, _velocity)",
                "elif hasattr(motor, 'run_angle'):",
                "    motor.run_angle(_p, _velocity, _delta)",
                "else:",
                "    raise RuntimeError('Missing motor.run_to_relative_position API; firmware/API mismatch (checked run_for_degrees/run_angle).')",
            ]
            body_lines.extend(self._stop_lines(stop_action))

            payload = self._run_checked(
                snippet=self._build_core_snippet(port_letter=port_letter, body_lines=body_lines),
                timeout=max(3.0, self._command_timeout),
                success_payload={"accepted": True, "port": port_letter},
                failure_default="USB run_to_relative failed",
                fail_key="accepted",
            )
            self._set_idle()
            return payload

    def reset_relative(self, port: str = "A") -> Dict[str, Any]:
        with self._lock:
            port_letter = self._normalize_port(port)
            body_lines = [
                "if hasattr(motor, 'reset_relative_position'):",
                "    try:",
                "        motor.reset_relative_position(_p, 0)",
                "    except TypeError:",
                "        motor.reset_relative_position(_p)",
                "elif hasattr(motor, 'set_relative_position'):",
                "    motor.set_relative_position(_p, 0)",
                "else:",
                "    raise RuntimeError('Missing motor.reset_relative_position API; firmware/API mismatch.')",
            ]

            payload = self._run_checked(
                snippet=self._build_core_snippet(port_letter=port_letter, body_lines=body_lines),
                timeout=max(2.0, self._command_timeout),
                success_payload={"accepted": True, "port": port_letter},
                failure_default="USB reset_relative failed",
                fail_key="accepted",
            )
            self._set_idle()
            return payload

    def set_duty_cycle(self, port: str, speed: float) -> Dict[str, Any]:
        with self._lock:
            port_letter = self._normalize_port(port)
            duty = self._duty_units(speed)

            body_lines = [
                f"_duty = {duty}",
                "if hasattr(motor, 'set_duty_cycle'):",
                "    motor.set_duty_cycle(_p, _duty)",
                "elif hasattr(motor, 'run'):",
                "    motor.run(_p, int(_duty * 10))",
                "else:",
                "    raise RuntimeError('Missing motor.set_duty_cycle API; firmware/API mismatch.')",
            ]
            payload = self._run_checked(
                snippet=self._build_core_snippet(port_letter=port_letter, body_lines=body_lines),
                timeout=max(2.0, self._command_timeout),
                success_payload={"accepted": True, "port": port_letter},
                failure_default="USB set_duty_cycle failed",
                fail_key="accepted",
            )
            if bool(payload.get("accepted", False)):
                if abs(duty) <= 0:
                    self._set_idle()
                else:
                    self._set_running(float(duty) / 100.0)
            return payload

    def sound_beep(
        self,
        freq_hz: int,
        duration_ms: int,
        volume: int,
    ) -> Dict[str, Any]:
        with self._lock:
            safe_freq = self._clamp_freq_hz(freq_hz)
            safe_duration = self._clamp_duration_ms(duration_ms)
            safe_volume = self._clamp_volume(volume)

            snippet = self._build_sound_snippet(
                body_lines=[
                    "if not hasattr(sound, 'beep'):",
                    "    raise RuntimeError('Missing hub.sound.beep API; firmware/API mismatch.')",
                    f"sound.beep({safe_freq}, {safe_duration}, {safe_volume})",
                ]
            )
            payload = self._run_checked(
                snippet=snippet,
                timeout=max(2.0, self._command_timeout),
                success_payload={
                    "accepted": True,
                    "freq_hz": safe_freq,
                    "duration_ms": safe_duration,
                    "volume": safe_volume,
                },
                failure_default="USB sound_beep failed",
                fail_key="accepted",
            )
            if bool(payload.get("accepted", False)):
                self._sound_supported_cache = True
                self._sound_probe_monotonic = time.monotonic()
            else:
                self._sound_supported_cache = False
            payload.setdefault("freq_hz", safe_freq)
            payload.setdefault("duration_ms", safe_duration)
            payload.setdefault("volume", safe_volume)
            return payload

    def sound_stop(self) -> Dict[str, Any]:
        with self._lock:
            snippet = self._build_sound_snippet(
                body_lines=[
                    "if hasattr(sound, 'stop'):",
                    "    sound.stop()",
                    "elif hasattr(sound, 'beep'):",
                    "    pass",
                    "else:",
                    "    raise RuntimeError('Missing hub.sound API; firmware/API mismatch.')",
                ]
            )
            payload = self._run_checked(
                snippet=snippet,
                timeout=max(2.0, self._command_timeout),
                success_payload={"stopped": True},
                failure_default="USB sound_stop failed",
                fail_key="stopped",
            )
            if bool(payload.get("stopped", False)):
                self._sound_supported_cache = True
                self._sound_probe_monotonic = time.monotonic()
            return payload

    def _score_stop_lines(self) -> List[str]:
        return [
            "try:",
            "    motor.stop(_p)",
            "except Exception:",
            "    try:",
            "        motor.run(_p, 0)",
            "    except Exception:",
            "        try:",
            "            motor.set_duty_cycle(_p, 0)",
            "        except Exception:",
            "            pass",
        ]

    def _encode_score_action(self, action: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        action_type = str(action.get("type", "")).strip().lower()
        if action_type == "run":
            speed = self._clip_speed(action.get("speed", 0.0))
            return {
                "type": "run",
                "velocity": self._velocity_units(speed),
            }
        if action_type == "stop":
            return {
                "type": "stop",
                "stop_action": self._normalize_stop_action(action.get("stop_action", "coast")),
            }
        if action_type == "beep":
            return {
                "type": "beep",
                "freq_hz": self._clamp_freq_hz(int(action.get("freq_hz", action.get("freq", 440)))),
                "duration_ms": self._clamp_duration_ms(
                    int(action.get("duration_ms", action.get("duration", 120)))
                ),
                "volume": self._clamp_volume(int(action.get("volume", 60))),
            }
        return None

    def _build_score_batch_snippet(
        self,
        *,
        port_letter: str,
        actions: List[Dict[str, Any]],
        initial_delay_ms: int,
    ) -> str:
        safe_initial_delay_ms = max(0, int(initial_delay_ms))
        has_beep = any(item.get("type") == "beep" for item in actions)

        lines: List[str] = [
            "import time",
            "import motor",
            "from hub import port",
            f"_letter = {port_letter!r}",
            "_p = getattr(port, _letter)",
            f"_has_beep = {bool(has_beep)}",
            "_snd = None",
            "if _has_beep:",
            "    from hub import sound as _snd",
            "_events = [",
        ]

        for row in actions:
            t_rel_ms = int(row["t_rel_ms"])
            kind = str(row["type"])
            if kind == "run":
                velocity = int(row["velocity"])
                lines.append(f"    ({t_rel_ms}, 'run', {velocity}, 0, 0, ''),")
            elif kind == "stop":
                stop_action = self._normalize_stop_action(row.get("stop_action", "coast"))
                lines.append(
                    f"    ({t_rel_ms}, 'stop', 0, 0, 0, {stop_action!r}),"
                )
            elif kind == "beep":
                freq_hz = int(row["freq_hz"])
                duration_ms = int(row["duration_ms"])
                volume = int(row["volume"])
                lines.append(
                    f"    ({t_rel_ms}, 'beep', {freq_hz}, {duration_ms}, {volume}, ''),"
                )

        lines.extend(
            [
                "]",
                "_t0 = time.ticks_ms()",
                f"_initial_delay_ms = {safe_initial_delay_ms}",
                "if _initial_delay_ms > 0:",
                "    _first_target = time.ticks_add(_t0, _initial_delay_ms)",
                "    while time.ticks_diff(_first_target, time.ticks_ms()) > 0:",
                "        pass",
                "for _ev in _events:",
                "    _target = time.ticks_add(_t0, _initial_delay_ms + _ev[0])",
                "    while time.ticks_diff(_target, time.ticks_ms()) > 0:",
                "        pass",
                "    _kind = _ev[1]",
                "    if _kind == 'run':",
                "        motor.run(_p, int(_ev[2]))",
                "    elif _kind == 'stop':",
            ]
        )
        lines.extend([f"        {line}" if line else "" for line in self._score_stop_lines()])
        lines.extend(
            [
                "    elif _kind == 'beep':",
                "        if _snd is None:",
                "            raise RuntimeError('Missing hub.sound API; firmware/API mismatch.')",
                "        _snd.beep(int(_ev[2]), int(_ev[3]), int(_ev[4]))",
                "",
                f"print({USB_OK_MARKER!r})",
            ]
        )

        wrapped: List[str] = ["try:"]
        for line in lines:
            if line:
                wrapped.append(f"    {line}")
            else:
                wrapped.append("")
        wrapped.extend(
            [
                "except Exception as _e:",
                f"    print({USB_ERR_MARKER!r} + ' ' + repr(_e))",
            ]
        )
        return self._prepare_snippet(wrapped)

    def _dispatch_score_action(self, action: Dict[str, Any]) -> Dict[str, Any]:
        action_type = str(action.get("type", "")).strip().lower()
        port = str(action.get("port", self._motor_port)).strip().upper() or self._motor_port
        stop_action = self._normalize_stop_action(action.get("stop_action", "coast"))
        speed = self._clip_speed(action.get("speed", 0.0))
        if action_type == "run":
            return self.run(speed=speed, duration=0.0, port=port)
        if action_type == "stop":
            return self.stop(port=port, stop_action=stop_action)
        if action_type == "beep":
            return self.sound_beep(
                freq_hz=int(action.get("freq_hz", action.get("freq", 440))),
                duration_ms=int(action.get("duration_ms", action.get("duration", 120))),
                volume=int(action.get("volume", 60)),
            )
        if action_type == "run_for_degrees":
            return self.run_for_degrees(
                port=port,
                speed=speed,
                degrees=int(action.get("degrees", 0)),
                stop_action=stop_action,
            )
        if action_type == "run_to_absolute_position":
            return self.run_to_absolute(
                port=port,
                speed=speed,
                position_degrees=int(action.get("position_degrees", 0)),
                stop_action=stop_action,
            )
        if action_type == "run_to_relative_position":
            return self.run_to_relative(
                port=port,
                speed=speed,
                degrees=int(action.get("degrees", 0)),
                stop_action=stop_action,
            )
        if action_type == "reset_relative_position":
            return self.reset_relative(port=port)
        if action_type == "set_duty_cycle":
            return self.set_duty_cycle(port=port, speed=speed)
        return {"accepted": False, "error": f"unsupported score action type '{action_type}'"}

    def _score_stop_worker_locked(self, join_timeout: float = 1.0) -> bool:
        thread = self._score_thread
        self._score_stop_event.set()
        worker_stopped = True
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=max(0.1, float(join_timeout)))
            worker_stopped = not thread.is_alive()
        self._score_thread = None if worker_stopped else thread
        self._score_playing = False
        return worker_stopped

    @staticmethod
    def _score_action_success(result: Dict[str, Any]) -> bool:
        if "accepted" in result:
            return bool(result.get("accepted", False))
        if "stopped" in result:
            return bool(result.get("stopped", False))
        if "ok" in result:
            return bool(result.get("ok", False))
        return False

    def _group_score_events(self, events: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        groups: List[Dict[str, Any]] = []
        current_t: Optional[int] = None
        current_events: List[Dict[str, Any]] = []
        for row in events:
            t_rel_ms = int(row.get("t_rel_ms", 0))
            if current_t is None:
                current_t = t_rel_ms
            if t_rel_ms != current_t:
                groups.append({"t_rel_ms": current_t, "events": current_events})
                current_t = t_rel_ms
                current_events = []
            current_events.append(row)
        if current_t is not None:
            groups.append({"t_rel_ms": current_t, "events": current_events})
        return groups

    def _windowize_score_groups(self, groups: List[Dict[str, Any]]) -> List[List[Dict[str, Any]]]:
        windows: List[List[Dict[str, Any]]] = []
        group_index = 0
        while group_index < len(groups):
            window = [groups[group_index]]
            base_t = int(groups[group_index]["t_rel_ms"])
            scan_index = group_index + 1
            while scan_index < len(groups):
                next_t = int(groups[scan_index]["t_rel_ms"])
                span_ms = next_t - base_t
                if span_ms > SCORE_BATCH_MAX_SPAN_MS:
                    break
                event_count = sum(len(g["events"]) for g in window) + len(groups[scan_index]["events"])
                if event_count > SCORE_BATCH_MAX_EVENTS:
                    break
                window.append(groups[scan_index])
                scan_index += 1
            windows.append(window)
            group_index = scan_index
        return windows

    def _preflight_score_batch_compile(
        self,
        *,
        port_letter: str,
        events: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        groups = self._group_score_events(events)
        windows = self._windowize_score_groups(groups)

        batch_windows = 0
        batch_events = 0
        for window_index, window in enumerate(windows):
            base_t = int(window[0]["t_rel_ms"])
            encoded_actions: List[Dict[str, Any]] = []
            fallback_needed = False
            for group in window:
                rel_t = max(0, int(group["t_rel_ms"]) - base_t)
                for event in group["events"]:
                    action = dict(event.get("action", {}))
                    encoded = self._encode_score_action(action)
                    if encoded is None:
                        fallback_needed = True
                        break
                    encoded["t_rel_ms"] = rel_t
                    encoded_actions.append(encoded)
                if fallback_needed:
                    break

            if fallback_needed or not encoded_actions:
                continue

            snippet = self._build_score_batch_snippet(
                port_letter=port_letter,
                actions=encoded_actions,
                initial_delay_ms=0,
            )
            compile_check = self._compile_check_snippet(
                snippet,
                label=f"score_batch_window_{window_index}",
            )
            self._write_score_batch_artifacts(
                snippet=snippet,
                encoded_actions=encoded_actions,
                compile_check=compile_check,
                reason="preflight",
            )
            if not bool(compile_check.get("ok", False)):
                return {
                    "ok": False,
                    "batch_mode_used": False,
                    "event_count": len(events),
                    "batch_event_count": batch_events + len(encoded_actions),
                    "batch_window_count": batch_windows + 1,
                    "compile_check": compile_check,
                    "error": "batch snippet compile failed before execution",
                }

            batch_windows += 1
            batch_events += len(encoded_actions)

        return {
            "ok": True,
            "batch_mode_used": batch_windows > 0,
            "event_count": len(events),
            "batch_event_count": batch_events,
            "batch_window_count": batch_windows,
            "compile_check": {"ok": True, "label": "score_batch_preflight"},
        }

    def _run_score_window_fallback(
        self,
        *,
        window: List[Dict[str, Any]],
        start_monotonic: float,
    ) -> Tuple[bool, float]:
        fallback_start_monotonic = time.monotonic()
        for group in window:
            group_t = int(group["t_rel_ms"])
            group_scheduled = start_monotonic + (group_t / 1000.0)
            while not self._score_stop_event.is_set():
                remaining = group_scheduled - time.monotonic()
                if remaining <= 0.0:
                    break
                time.sleep(min(0.01, remaining))
            if self._score_stop_event.is_set():
                return False, fallback_start_monotonic

            for event in group["events"]:
                action = dict(event.get("action", {}))
                action_type = str(action.get("type", "")).strip().lower()
                result = self._dispatch_score_action(action)
                if not self._score_action_success(result):
                    error_text = str(result.get("error", "score fallback action failed"))
                    if action_type == "beep":
                        # Keep motor timing running even when speaker APIs are unavailable.
                        logging.warning(
                            "[SPIKE_USB] Ignoring beep failure in score fallback: %s",
                            error_text,
                        )
                        continue
                    self._score_last_error = error_text
                    logging.warning(
                        "[SPIKE_USB] score fallback action failed (type=%s): %s",
                        action_type or "<unknown>",
                        error_text,
                    )
                    return False, fallback_start_monotonic
        return True, fallback_start_monotonic

    def _score_worker(
        self,
        *,
        session_id: int,
        events: List[Dict[str, Any]],
        start_monotonic: float,
        port_letter: str,
    ) -> None:
        self._timing.start_session(label=f"spike_usb_score_{session_id}")
        groups = self._group_score_events(events)
        windows = self._windowize_score_groups(groups)
        batch_disabled = False
        for window_index, window in enumerate(windows):
            if self._score_stop_event.is_set():
                break

            base_t = int(window[0]["t_rel_ms"])
            first_group_time = start_monotonic + (base_t / 1000.0)
            while not self._score_stop_event.is_set():
                remaining = first_group_time - time.monotonic() - SCORE_SCHEDULER_LEAD_SEC
                if remaining <= 0.0:
                    break
                time.sleep(min(0.01, remaining))
            if self._score_stop_event.is_set():
                break

            now = time.monotonic()
            initial_delay_ms = int(max(0.0, (first_group_time - now) * 1000.0))
            encoded_actions: List[Dict[str, Any]] = []
            fallback_needed = False
            for group in window:
                group_t = int(group["t_rel_ms"])
                rel_t = max(0, group_t - base_t)
                for event in group["events"]:
                    action = dict(event.get("action", {}))
                    encoded = self._encode_score_action(action)
                    if encoded is None:
                        fallback_needed = True
                        break
                    encoded["t_rel_ms"] = rel_t
                    encoded_actions.append(encoded)
                if fallback_needed:
                    break

            success = True
            batch_start_monotonic = time.monotonic()
            if (not batch_disabled) and (not fallback_needed) and encoded_actions:
                span_ms = int(window[-1]["t_rel_ms"]) - base_t
                timeout = max(2.0, (initial_delay_ms + span_ms) / 1000.0 + 2.0)
                snippet = self._build_score_batch_snippet(
                    port_letter=port_letter,
                    actions=encoded_actions,
                    initial_delay_ms=initial_delay_ms,
                )
                compile_check = self._compile_check_snippet(
                    snippet,
                    label=f"score_batch_worker_session_{session_id}",
                )
                self._write_score_batch_artifacts(
                    snippet=snippet,
                    encoded_actions=encoded_actions,
                    compile_check=compile_check,
                    reason="worker_window",
                )
                if not bool(compile_check.get("ok", False)):
                    error = (
                        f"{compile_check.get('error', 'compile check failed')} "
                        f"(line={compile_check.get('line', 0)})"
                    ).strip()
                    batch_disabled = True
                    with self._score_lock:
                        self._score_batch_disabled = True
                        self._score_batch_disabled_reason = error
                        self._score_compile_check = dict(compile_check)
                    logging.warning("[SPIKE_USB] score batch compile check failed: %s", error)
                    success, batch_start_monotonic = self._run_score_window_fallback(
                        window=window,
                        start_monotonic=start_monotonic,
                    )
                    if not success and not self._score_last_error:
                        self._score_last_error = error

                with self._lock:
                    result = self._execute(
                        snippet=snippet,
                        timeout=timeout,
                        retry_on_syntax_error=True,
                    )
                if not self._successful(result):
                    error = self._extract_error_from_result(result, default="score batch failed")
                    logging.warning("[SPIKE_USB] score batch failed: %s", error)
                    self._write_score_batch_artifacts(
                        snippet=snippet,
                        encoded_actions=encoded_actions,
                        compile_check=compile_check,
                        reason=f"runtime_failure:{error}",
                    )
                    if self._debug_repl_snippets:
                        logging.warning("[SPIKE_USB] score batch snippet:\n%s", snippet)
                    if (
                        self._is_syntax_error_text(error)
                        or "raw repl sync failed" in error.lower()
                    ):
                        batch_disabled = True
                        reason = error
                        with self._score_lock:
                            self._score_batch_disabled = True
                            self._score_batch_disabled_reason = reason
                        logging.warning(
                            "[SPIKE_USB] Disabling batch mode for remaining score events in this session."
                        )
                    success, batch_start_monotonic = self._run_score_window_fallback(
                        window=window,
                        start_monotonic=start_monotonic,
                    )
                    if not success:
                        if not self._score_last_error:
                            self._score_last_error = error
                    else:
                        # Batch path failed but fallback recovered this window.
                        self._score_last_error = ""
            else:
                success, batch_start_monotonic = self._run_score_window_fallback(
                    window=window,
                    start_monotonic=start_monotonic,
                )

            for group in window:
                group_t = int(group["t_rel_ms"])
                scheduled_group = start_monotonic + (group_t / 1000.0)
                estimated_actual_group = batch_start_monotonic + ((group_t - base_t + initial_delay_ms) / 1000.0)
                pair_key = f"t{group_t}"
                for event in group["events"]:
                    action = dict(event.get("action", {}))
                    event_id = str(event.get("event_id", ""))
                    self._timing.record_event(
                        event_id=event_id or f"evt_{window_index:04d}",
                        event_type=str(action.get("type", "")),
                        scheduled_time_monotonic=scheduled_group,
                        actual_time_monotonic=estimated_actual_group,
                        pair_key=pair_key,
                    )
                    with self._score_lock:
                        self._score_current_index += 1

            if not success:
                break

        with self._score_lock:
            self._score_playing = False
        try:
            self.stop(port=port_letter, stop_action="coast")
        except Exception:  # noqa: BLE001
            pass
        try:
            self.sound_stop()
        except Exception:  # noqa: BLE001
            pass
        self._timing.finish_session()

    def score_play(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        raw_payload = dict(payload or {})
        try:
            events, metadata = normalize_score_payload(raw_payload)
        except Exception as exc:  # noqa: BLE001
            self._score_last_error = str(exc)
            return {"accepted": False, "error": f"invalid score payload: {exc}"}

        if not events:
            return {"accepted": False, "error": "score has no events"}

        port_letter = self._normalize_port(str(raw_payload.get("port", metadata.get("port", self._motor_port))))
        preflight = self._preflight_score_batch_compile(
            port_letter=port_letter,
            events=events,
        )
        compile_check = dict(preflight.get("compile_check", {"ok": True}))
        if not bool(preflight.get("ok", False)):
            error_msg = str(preflight.get("error", "batch preflight failed"))
            with self._score_lock:
                self._score_playing = False
                self._score_total_events = len(events)
                self._score_current_index = 0
                self._score_last_error = error_msg
                self._score_batch_mode_used = False
                self._score_batch_disabled = True
                self._score_batch_disabled_reason = error_msg
                self._score_compile_check = dict(compile_check)
            return {
                "accepted": False,
                "error": error_msg,
                "event_count": len(events),
                "batch_mode_used": False,
                "compile_check": compile_check,
            }

        batch_mode_used = bool(preflight.get("batch_mode_used", False))
        start_mode = str(metadata.get("start_mode", "immediate")).strip().lower()
        start_time_ms = int(metadata.get("start_time_ms", 0))
        now = time.monotonic()
        if start_mode == "at_time" and start_time_ms > 0:
            start_monotonic = now + (start_time_ms / 1000.0)
        else:
            start_monotonic = now

        with self._score_lock:
            previous_stopped = self._score_stop_worker_locked(join_timeout=1.0)
            if not previous_stopped:
                self._score_last_error = "previous score worker did not stop within timeout"
                self._score_batch_mode_used = False
                self._score_batch_disabled = True
                self._score_batch_disabled_reason = self._score_last_error
                self._score_compile_check = dict(compile_check)
                return {
                    "accepted": False,
                    "error": self._score_last_error,
                    "event_count": len(events),
                    "batch_mode_used": False,
                    "compile_check": compile_check,
                }
            self._score_stop_event = threading.Event()
            self._score_session_id += 1
            session_id = self._score_session_id
            self._score_playing = True
            self._score_total_events = len(events)
            self._score_current_index = 0
            self._score_started_monotonic = start_monotonic
            self._score_expected_end_monotonic = (
                start_monotonic + (float(events[-1].get("t_rel_ms", 0)) / 1000.0)
            )
            self._score_last_error = ""
            self._score_batch_mode_used = bool(batch_mode_used)
            self._score_batch_disabled = False
            self._score_batch_disabled_reason = ""
            self._score_compile_check = dict(compile_check)
            worker = threading.Thread(
                target=self._score_worker,
                kwargs={
                    "session_id": session_id,
                    "events": events,
                    "start_monotonic": start_monotonic,
                    "port_letter": port_letter,
                },
                name=f"spike_usb_score_{session_id}",
                daemon=True,
            )
            self._score_thread = worker
            worker.start()

        return {
            "accepted": True,
            "event_count": len(events),
            "source": metadata.get("source", "events"),
            "start_mode": start_mode,
            "port": port_letter,
            "batch_mode_used": bool(batch_mode_used),
            "compile_check": compile_check,
        }

    def score_stop(self) -> Dict[str, Any]:
        with self._score_lock:
            worker_stopped = self._score_stop_worker_locked(join_timeout=1.2)
            if not worker_stopped:
                self._score_last_error = "score worker did not stop within timeout"
                self._score_batch_disabled = True
                self._score_batch_disabled_reason = self._score_last_error
                return {"stopped": False, "error": self._score_last_error}
        stop_result = self.stop(port=self._motor_port, stop_action="coast")
        try:
            self.sound_stop()
        except Exception:  # noqa: BLE001
            pass
        self._timing.finish_session()
        return {"stopped": bool(stop_result.get("stopped", False))}

    def score_status(self) -> Dict[str, Any]:
        with self._score_lock:
            playing = bool(self._score_playing)
            current_index = int(self._score_current_index)
            total = int(self._score_total_events)
            started = float(self._score_started_monotonic)
            expected_end = float(self._score_expected_end_monotonic)
            last_error = str(self._score_last_error or "")
            batch_mode_used = bool(self._score_batch_mode_used)
            batch_disabled = bool(self._score_batch_disabled)
            batch_disabled_reason = str(self._score_batch_disabled_reason or "")
            compile_check = dict(self._score_compile_check or {"ok": True})

        timing = self._timing.snapshot()
        return {
            "ok": True,
            "playing": playing,
            "current_index": current_index,
            "total_events": total,
            "start_time_monotonic": started,
            "expected_end_monotonic": expected_end,
            "last_error": last_error,
            "batch_mode_used": batch_mode_used,
            "batch_disabled": batch_disabled,
            "batch_disabled_reason": batch_disabled_reason,
            "compile_check": compile_check,
            "delta_ms": timing.get("delta_ms", {}),
            "pair_delta_ms": timing.get("pair_delta_ms", {}),
            "dropped_events": timing.get("dropped_events", 0),
        }

    def debug_timing(self) -> Dict[str, Any]:
        snapshot = self._timing.snapshot()
        return {
            "ok": True,
            **snapshot,
            "score_status": self.score_status(),
        }

    def motor_status(self, port: str = "A") -> Dict[str, Any]:
        with self._lock:
            port_letter = self._normalize_port(port)
            body_lines = [
                "_status = {}",
                "_status['port'] = _letter",
                "if hasattr(motor, 'relative_position'):",
                "    try:",
                "        _status['relative_position'] = motor.relative_position(_p)",
                "    except Exception:",
                "        pass",
                "if hasattr(motor, 'absolute_position'):",
                "    try:",
                "        _status['absolute_position'] = motor.absolute_position(_p)",
                "    except Exception:",
                "        pass",
                "if hasattr(motor, 'velocity'):",
                "    try:",
                "        _status['velocity'] = motor.velocity(_p)",
                "    except Exception:",
                "        pass",
                f"print({USB_STATUS_MARKER!r} + repr(_status))",
            ]
            snippet = self._build_core_snippet(port_letter=port_letter, body_lines=body_lines)
            result = self._execute(
                snippet=snippet,
                timeout=max(2.0, self._command_timeout),
            )
            if not self._successful(result):
                self._spike_connected = False
                error = self._extract_error_from_result(result, default="USB motor_status failed")
                self._record_error(error)
                payload: Dict[str, Any] = {"ok": False, "error": error}
                if self._debug_repl_snippets:
                    payload["snippet"] = self._trim_text(snippet, limit=3000)
                    payload["stdout"] = self._trim_text(result.stdout)
                    payload["stderr"] = self._trim_text(result.stderr)
                return payload

            status_payload: Dict[str, Any] = {}
            for line in (result.stdout or "").splitlines():
                if USB_STATUS_MARKER in line:
                    raw = line.split(USB_STATUS_MARKER, 1)[1].strip()
                    try:
                        parsed = ast.literal_eval(raw)
                        if isinstance(parsed, dict):
                            status_payload = parsed
                    except Exception:  # noqa: BLE001
                        status_payload = {}
                    break

            self._spike_connected = True
            self._last_error = ""
            return {
                "ok": True,
                "state": self._state,
                "last_speed": self._last_speed,
                "timestamp": self._timestamp,
                **status_payload,
            }

    def _minimal_repl_check(self) -> Tuple[bool, RawExecResult]:
        result = self._execute(
            snippet=f"print({USB_OK_MARKER!r})\n",
            timeout=max(self._prompt_timeout + 1.0, 2.0),
        )
        repl_ok = self._successful(result)
        return repl_ok, result

    def get_state(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "state": self._state,
                "last_speed": self._last_speed,
                "timestamp": self._timestamp,
            }

    def health(self) -> Dict[str, Any]:
        with self._lock:
            repl_ok, result = self._minimal_repl_check()
            self._spike_connected = repl_ok
            sound_supported = repl_ok and self._probe_sound_supported(force=False)

            if repl_ok:
                self._last_error = ""
            else:
                error = self._extract_error_from_result(result, default="raw repl health check failed")
                self._record_error(error)

            payload: Dict[str, Any] = {
                "ok": True,
                "backend": self.name,
                "spike_connected": repl_ok,
                "repl_ok": repl_ok,
                "sound_supported": bool(sound_supported),
            }
            if self._resolved_port:
                payload["serial_port"] = self._resolved_port
            if self._last_error:
                payload["detail"] = self._last_error
            payload["score_playing"] = bool(self.score_status().get("playing", False))
            if self._debug_repl_snippets and (not repl_ok):
                payload["stdout"] = self._trim_text(result.stdout)
                payload["stderr"] = self._trim_text(result.stderr)
            return payload

    def close(self) -> None:
        try:
            self.score_stop()
        except KeyboardInterrupt:
            return
        except Exception:  # noqa: BLE001
            pass
        try:
            self.stop()
        except KeyboardInterrupt:
            return
        except Exception:  # noqa: BLE001
            pass
        try:
            self.sound_stop()
        except KeyboardInterrupt:
            return
        except Exception:  # noqa: BLE001
            pass
        self._reset_client()
