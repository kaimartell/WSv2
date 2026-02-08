import ast
import glob
import logging
import re
import threading
import textwrap
import time
from typing import Any, Dict, List, Optional, Tuple

from host_agent.serial_repl import RawExecResult, SerialMicroPythonClient


USB_OK_MARKER = "__SPIKE_USB_OK__"
USB_ERR_MARKER = "__SPIKE_USB_ERR__"
USB_STATUS_MARKER = "__SPIKE_USB_STATUS__"
STOP_SPEED_EPSILON = 1e-3

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
        **_: Any,
    ) -> None:
        self._serial_port_config = (serial_port or "auto").strip()
        self._baud = int(baud)
        self._motor_port = (motor_port or "A").strip().upper()
        self._prompt_timeout = max(0.5, float(prompt_timeout))
        self._command_timeout = max(1.0, float(command_timeout))
        self._sync_retries = max(2, int(sync_retries))

        self._lock = threading.Lock()
        self._state = "idle"
        self._last_speed = 0.0
        self._timestamp = time.time()
        self._spike_connected = False
        self._last_error = ""
        self._resolved_port = ""

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

    def _execute(self, snippet: str, timeout: Optional[float] = None) -> RawExecResult:
        port, error = self._resolve_serial_port()
        if port is None:
            return RawExecResult(ok=False, error=error)

        try:
            with SerialMicroPythonClient(
                port=port,
                baud=self._baud,
                prompt_timeout=self._prompt_timeout,
                exec_timeout=self._command_timeout,
                sync_retries=self._sync_retries,
            ) as client:
                result = client.execute_raw(snippet, timeout=timeout)
        except Exception as exc:  # noqa: BLE001
            result = RawExecResult(ok=False, error=str(exc))

        if not result.ok and "raw repl sync failed" in (result.error or "").lower():
            result.error = (
                "raw repl sync failed; reset hub and close other apps that may hold the serial port"
            )

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

    def _build_core_snippet(self, *, port_letter: str, body: str) -> str:
        core = f"""
try:
    import motor
except Exception as _e:
    raise RuntimeError("Not running LEGO SPIKE firmware / knowledge base API. import motor failed: " + repr(_e))

try:
    from hub import port
except Exception as _e:
    raise RuntimeError("Not running LEGO SPIKE firmware / knowledge base API. from hub import port failed: " + repr(_e))

_letter = {port_letter!r}
_available = []
for _name in dir(port):
    if len(_name) == 1 and _name >= "A" and _name <= "Z":
        _available.append(_name)
if _letter not in _available:
    raise RuntimeError("Invalid motor port " + _letter + " available=" + repr(_available))

_p = getattr(port, _letter)
{body}
print({USB_OK_MARKER!r})
"""
        wrapped = f"""
try:
{textwrap.indent(textwrap.dedent(core).strip(), "    ")}
except Exception as _e:
    print({USB_ERR_MARKER!r} + " " + repr(_e))
"""
        return textwrap.dedent(wrapped).strip() + "\n"

    def _stop_block(self, stop_action: str) -> str:
        normalized = str(stop_action or "coast").strip().lower()
        return textwrap.dedent(
            f"""
_stop_action = {normalized!r}
if _stop_action == "coast":
    try:
        motor.run(_p, 0)
    except Exception as _e1:
        try:
            motor.stop(_p)
        except Exception as _e2:
            raise RuntimeError("coast stop failed: " + repr(_e1) + "; " + repr(_e2))
else:
    try:
        motor.stop(_p)
    except Exception as _e1:
        try:
            motor.run(_p, 0)
        except Exception as _e2:
            try:
                motor.set_duty_cycle(_p, 0)
            except Exception as _e3:
                raise RuntimeError(
                    "motor stop fallbacks failed: "
                    + repr(_e1)
                    + "; "
                    + repr(_e2)
                    + "; "
                    + repr(_e3)
                )
"""
        ).strip()

    def _set_idle(self) -> None:
        self._state = "idle"
        self._last_speed = 0.0
        self._timestamp = time.time()

    def _set_running(self, speed: float) -> None:
        self._state = "running"
        self._last_speed = float(speed)
        self._timestamp = time.time()

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
        payload = {fail_key: False, "error": error}
        payload["stdout"] = self._trim_text(result.stdout)
        payload["stderr"] = self._trim_text(result.stderr)
        return payload

    def run_with_diagnostics(self, speed: float, duration: float, port: str = "") -> Dict[str, Any]:
        with self._lock:
            safe_speed = self._clip_speed(speed)
            advisory_duration = max(0.0, float(duration))
            port_letter = self._normalize_port(port)
            velocity = self._velocity_units(safe_speed)

            snippet = self._build_core_snippet(
                port_letter=port_letter,
                body=textwrap.dedent(
                    f"""
_velocity = {velocity}
motor.run(_p, _velocity)
"""
                ).strip(),
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
                body=self._stop_block(stop_action),
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

            body = textwrap.dedent(
                f"""
_velocity = {velocity}
_degrees = {move_degrees}
if hasattr(motor, "run_for_degrees"):
    try:
        motor.run_for_degrees(_p, _degrees, _velocity)
    except TypeError:
        motor.run_for_degrees(_p, _degrees, _velocity, {str(stop_action).strip().lower()!r})
elif hasattr(motor, "run_angle"):
    motor.run_angle(_p, _velocity, _degrees)
else:
    raise RuntimeError("No supported API for run_for_degrees (expected motor.run_for_degrees or motor.run_angle)")
{self._stop_block(stop_action)}
"""
            ).strip()

            payload = self._run_checked(
                snippet=self._build_core_snippet(port_letter=port_letter, body=body),
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

            body = textwrap.dedent(
                f"""
_velocity = {velocity}
_target = {target}
if hasattr(motor, "run_to_absolute_position"):
    motor.run_to_absolute_position(_p, _target, _velocity)
elif hasattr(motor, "run_to_position"):
    motor.run_to_position(_p, _target, _velocity)
elif hasattr(motor, "run_target"):
    motor.run_target(_p, _velocity, _target)
else:
    raise RuntimeError(
        "No supported API for run_to_absolute_position (expected motor.run_to_absolute_position, run_to_position, or run_target)"
    )
{self._stop_block(stop_action)}
"""
            ).strip()

            payload = self._run_checked(
                snippet=self._build_core_snippet(port_letter=port_letter, body=body),
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

            body = textwrap.dedent(
                f"""
_velocity = {velocity}
_delta = {delta}
if hasattr(motor, "run_to_relative_position"):
    motor.run_to_relative_position(_p, _delta, _velocity)
elif hasattr(motor, "run_for_degrees"):
    motor.run_for_degrees(_p, _delta, _velocity)
elif hasattr(motor, "run_angle"):
    motor.run_angle(_p, _velocity, _delta)
else:
    raise RuntimeError(
        "No supported API for run_to_relative_position (expected motor.run_to_relative_position, run_for_degrees, or run_angle)"
    )
{self._stop_block(stop_action)}
"""
            ).strip()

            payload = self._run_checked(
                snippet=self._build_core_snippet(port_letter=port_letter, body=body),
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
            body = textwrap.dedent(
                """
if hasattr(motor, "reset_relative_position"):
    try:
        motor.reset_relative_position(_p, 0)
    except TypeError:
        motor.reset_relative_position(_p)
elif hasattr(motor, "set_relative_position"):
    motor.set_relative_position(_p, 0)
else:
    raise RuntimeError("No supported API for reset_relative_position")
"""
            ).strip()

            payload = self._run_checked(
                snippet=self._build_core_snippet(port_letter=port_letter, body=body),
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

            body = textwrap.dedent(
                f"""
_duty = {duty}
if hasattr(motor, "set_duty_cycle"):
    motor.set_duty_cycle(_p, _duty)
elif hasattr(motor, "run"):
    motor.run(_p, int(_duty * 10))
else:
    raise RuntimeError("No supported API for set_duty_cycle")
"""
            ).strip()
            payload = self._run_checked(
                snippet=self._build_core_snippet(port_letter=port_letter, body=body),
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

    def motor_status(self, port: str = "A") -> Dict[str, Any]:
        with self._lock:
            port_letter = self._normalize_port(port)
            body = textwrap.dedent(
                f"""
_status = {{}}
_status["port"] = _letter
if hasattr(motor, "relative_position"):
    try:
        _status["relative_position"] = motor.relative_position(_p)
    except Exception:
        pass
if hasattr(motor, "absolute_position"):
    try:
        _status["absolute_position"] = motor.absolute_position(_p)
    except Exception:
        pass
if hasattr(motor, "velocity"):
    try:
        _status["velocity"] = motor.velocity(_p)
    except Exception:
        pass
print({USB_STATUS_MARKER!r} + repr(_status))
"""
            ).strip()
            result = self._execute(
                snippet=self._build_core_snippet(port_letter=port_letter, body=body),
                timeout=max(2.0, self._command_timeout),
            )
            if not self._successful(result):
                self._spike_connected = False
                error = self._extract_error_from_result(result, default="USB motor_status failed")
                self._record_error(error)
                return {"ok": False, "error": error}

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
            snippet=f"print({USB_OK_MARKER!r})",
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
            }
            if self._resolved_port:
                payload["serial_port"] = self._resolved_port
            if self._last_error:
                payload["detail"] = self._last_error
            return payload

    def close(self) -> None:
        try:
            self.stop()
        except Exception:  # noqa: BLE001
            pass
