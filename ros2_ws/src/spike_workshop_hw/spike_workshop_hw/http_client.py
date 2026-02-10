import http.client
import json
import threading
import time
from typing import Any, Dict, Optional, Tuple
from urllib import error, request


class HostAgentHttpClient:
    """Small HTTP client for host-agent communication."""

    def __init__(self, base_url: str, timeout: float = 1.5) -> None:
        self._base_url = base_url.rstrip("/")
        self._timeout = max(0.1, float(timeout))
        self._lock = threading.Lock()
        self._last_error: str = ""
        self._last_health_latency_ms: Optional[float] = None

    def set_base_url(self, base_url: str) -> None:
        self._base_url = base_url.rstrip("/")

    def set_timeout(self, timeout: float) -> None:
        self._timeout = max(0.1, float(timeout))

    def get_base_url(self) -> str:
        return self._base_url

    def get_last_error(self) -> str:
        with self._lock:
            return self._last_error

    def get_last_health_latency_ms(self) -> Optional[float]:
        with self._lock:
            return self._last_health_latency_ms

    def run_motor(
        self,
        speed: float,
        duration: float,
        *,
        port: str = "",
        timeout_sec: Optional[float] = None,
    ) -> Dict[str, Any]:
        response, _meta = self.run_motor_with_meta(
            speed=speed,
            duration=duration,
            port=port,
            timeout_sec=timeout_sec,
        )
        return response

    def run_motor_with_meta(
        self,
        speed: float,
        duration: float,
        *,
        port: str = "",
        timeout_sec: Optional[float] = None,
    ) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        payload: Dict[str, Any] = {
            "speed": float(speed),
            "duration": max(0.0, float(duration)),
        }
        normalized_port = str(port or "").strip().upper()
        if normalized_port:
            payload["port"] = normalized_port
        return self._post_with_meta(
            "/motor/run",
            payload,
            default_failure={"accepted": False},
            timeout_sec=timeout_sec,
        )

    def stop_motor(
        self,
        *,
        port: str = "",
        stop_action: str = "",
        timeout_sec: Optional[float] = None,
    ) -> Dict[str, Any]:
        response, _meta = self.stop_motor_with_meta(
            port=port,
            stop_action=stop_action,
            timeout_sec=timeout_sec,
        )
        return response

    def stop_motor_with_meta(
        self,
        *,
        port: str = "",
        stop_action: str = "",
        timeout_sec: Optional[float] = None,
    ) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        payload: Dict[str, Any] = {}
        normalized_port = str(port or "").strip().upper()
        if normalized_port:
            payload["port"] = normalized_port
        normalized_stop_action = str(stop_action or "").strip().lower()
        if normalized_stop_action:
            payload["stop_action"] = normalized_stop_action
        return self._post_with_meta(
            "/motor/stop",
            payload,
            default_failure={"stopped": False},
            timeout_sec=timeout_sec,
        )

    def run_for_degrees_with_meta(
        self,
        *,
        port: str,
        speed: float,
        degrees: int,
        stop_action: str = "",
        timeout_sec: Optional[float] = None,
    ) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        payload: Dict[str, Any] = {
            "port": str(port).strip().upper(),
            "speed": float(speed),
            "degrees": int(degrees),
        }
        normalized_stop_action = str(stop_action or "").strip().lower()
        if normalized_stop_action:
            payload["stop_action"] = normalized_stop_action
        return self._post_with_meta(
            "/motor/run_for_degrees",
            payload,
            default_failure={"accepted": False},
            timeout_sec=timeout_sec,
        )

    def run_to_absolute_with_meta(
        self,
        *,
        port: str,
        speed: float,
        position_degrees: int,
        stop_action: str = "",
        timeout_sec: Optional[float] = None,
    ) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        payload: Dict[str, Any] = {
            "port": str(port).strip().upper(),
            "speed": float(speed),
            "position_degrees": int(position_degrees),
        }
        normalized_stop_action = str(stop_action or "").strip().lower()
        if normalized_stop_action:
            payload["stop_action"] = normalized_stop_action
        return self._post_with_meta(
            "/motor/run_to_absolute",
            payload,
            default_failure={"accepted": False},
            timeout_sec=timeout_sec,
        )

    def run_to_relative_with_meta(
        self,
        *,
        port: str,
        speed: float,
        degrees: int,
        stop_action: str = "",
        timeout_sec: Optional[float] = None,
    ) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        payload: Dict[str, Any] = {
            "port": str(port).strip().upper(),
            "speed": float(speed),
            "degrees": int(degrees),
        }
        normalized_stop_action = str(stop_action or "").strip().lower()
        if normalized_stop_action:
            payload["stop_action"] = normalized_stop_action
        return self._post_with_meta(
            "/motor/run_to_relative",
            payload,
            default_failure={"accepted": False},
            timeout_sec=timeout_sec,
        )

    def reset_relative_with_meta(
        self,
        *,
        port: str,
        timeout_sec: Optional[float] = None,
    ) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        payload = {"port": str(port).strip().upper()}
        return self._post_with_meta(
            "/motor/reset_relative",
            payload,
            default_failure={"accepted": False},
            timeout_sec=timeout_sec,
        )

    def set_duty_cycle_with_meta(
        self,
        *,
        port: str,
        speed: float,
        timeout_sec: Optional[float] = None,
    ) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        payload = {
            "port": str(port).strip().upper(),
            "speed": float(speed),
        }
        return self._post_with_meta(
            "/motor/set_duty_cycle",
            payload,
            default_failure={"accepted": False},
            timeout_sec=timeout_sec,
        )

    def get_motor_status_with_meta(
        self,
        *,
        port: str = "",
        timeout_sec: Optional[float] = None,
    ) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        payload: Dict[str, Any] = {}
        normalized_port = str(port or "").strip().upper()
        if normalized_port:
            payload["port"] = normalized_port
        return self._post_with_meta(
            "/motor/status",
            payload,
            default_failure={"ok": False},
            timeout_sec=timeout_sec,
        )

    def sound_beep(
        self,
        *,
        freq_hz: int,
        duration_ms: int,
        volume: int,
        timeout_sec: Optional[float] = None,
    ) -> Dict[str, Any]:
        response, _meta = self.sound_beep_with_meta(
            freq_hz=freq_hz,
            duration_ms=duration_ms,
            volume=volume,
            timeout_sec=timeout_sec,
        )
        return response

    def sound_beep_with_meta(
        self,
        *,
        freq_hz: int,
        duration_ms: int,
        volume: int,
        timeout_sec: Optional[float] = None,
    ) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        payload = {
            "freq_hz": int(freq_hz),
            "duration_ms": int(duration_ms),
            "volume": int(volume),
        }
        return self._post_with_meta(
            "/sound/beep",
            payload,
            default_failure={"accepted": False},
            timeout_sec=timeout_sec,
        )

    def sound_stop(
        self,
        *,
        timeout_sec: Optional[float] = None,
    ) -> Dict[str, Any]:
        response, _meta = self.sound_stop_with_meta(timeout_sec=timeout_sec)
        return response

    def sound_stop_with_meta(
        self,
        *,
        timeout_sec: Optional[float] = None,
    ) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        return self._post_with_meta(
            "/sound/stop",
            {},
            default_failure={"stopped": False},
            timeout_sec=timeout_sec,
        )

    def play_score_with_meta(
        self,
        payload: Dict[str, Any],
        *,
        timeout_sec: Optional[float] = None,
    ) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        safe_payload = dict(payload or {})
        return self._post_with_meta(
            "/score/play",
            safe_payload,
            default_failure={"accepted": False},
            timeout_sec=timeout_sec,
        )

    def stop_score_with_meta(
        self,
        *,
        timeout_sec: Optional[float] = None,
    ) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        return self._post_with_meta(
            "/score/stop",
            {},
            default_failure={"stopped": False},
            timeout_sec=timeout_sec,
        )

    def get_score_status_with_meta(
        self,
        *,
        timeout_sec: Optional[float] = None,
    ) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        response, meta = self._request("GET", "/score/status", None, timeout_sec=timeout_sec)
        self._record_error(meta.get("error", ""))
        if response is None:
            return {"ok": False, "playing": False}, meta
        return response, meta

    def get_debug_timing_with_meta(
        self,
        *,
        timeout_sec: Optional[float] = None,
    ) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        response, meta = self._request("GET", "/debug/timing", None, timeout_sec=timeout_sec)
        self._record_error(meta.get("error", ""))
        if response is None:
            return {"ok": False}, meta
        return response, meta

    def get_state(self) -> Dict[str, Any]:
        response, _meta = self.get_state_with_meta()
        return response

    def get_state_with_meta(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        response, meta = self._request("GET", "/state", None)
        self._record_error(meta.get("error", ""))
        if response is None:
            return {"state": "unreachable", "last_speed": 0.0}, meta
        return response, meta

    def get_health(self) -> Dict[str, Any]:
        response, _meta = self.get_health_with_meta()
        return response

    def get_health_with_meta(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        response, meta = self._request("GET", "/health", None)
        self._record_error(meta.get("error", ""))
        with self._lock:
            self._last_health_latency_ms = meta.get("latency_ms")
        if response is None:
            return {
                "ok": False,
                "backend": "unreachable",
                "spike_connected": False,
                "sound_supported": False,
            }, meta
        return response, meta

    def _post_with_meta(
        self,
        path: str,
        payload: Dict[str, Any],
        *,
        default_failure: Dict[str, Any],
        timeout_sec: Optional[float] = None,
    ) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        response, meta = self._request("POST", path, payload, timeout_sec=timeout_sec)
        self._record_error(meta.get("error", ""))
        if response is None:
            result = dict(default_failure)
            result["error"] = meta.get("error", "unreachable")
            return result, meta
        return response, meta

    def _record_error(self, message: str) -> None:
        with self._lock:
            self._last_error = str(message or "")

    def _request(
        self,
        method: str,
        path: str,
        payload: Optional[Dict[str, Any]],
        *,
        timeout_sec: Optional[float] = None,
    ) -> Tuple[Optional[Dict[str, Any]], Dict[str, Any]]:
        start = time.perf_counter()
        url = f"{self._base_url}{path}"
        headers = {"Accept": "application/json"}
        body = None

        if payload is not None:
            headers["Content-Type"] = "application/json"
            body = json.dumps(payload).encode("utf-8")

        req = request.Request(url=url, data=body, method=method, headers=headers)
        timeout = self._timeout if timeout_sec is None else max(0.1, float(timeout_sec))

        try:
            with request.urlopen(req, timeout=timeout) as resp:
                raw = resp.read().decode("utf-8")
            meta = {
                "ok": True,
                "latency_ms": (time.perf_counter() - start) * 1000.0,
                "error": "",
            }
        except error.HTTPError as exc:
            raw = ""
            try:
                raw = exc.read().decode("utf-8")
            except Exception:  # noqa: BLE001
                raw = ""
            meta = {
                "ok": True,
                "latency_ms": (time.perf_counter() - start) * 1000.0,
                "error": f"HTTP {exc.code} {exc.reason}",
            }
            if not raw:
                return {"error": meta["error"]}, meta
        except (
            error.URLError,
            TimeoutError,
            ValueError,
            http.client.RemoteDisconnected,
            ConnectionResetError,
            OSError,
        ) as exc:
            meta = {
                "ok": False,
                "latency_ms": (time.perf_counter() - start) * 1000.0,
                "error": str(exc) or exc.__class__.__name__,
            }
            return None, meta

        if not raw:
            return {}, meta

        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError:
            return {"raw": raw}, meta

        if isinstance(parsed, dict):
            return parsed, meta
        return {"raw": parsed}, meta
