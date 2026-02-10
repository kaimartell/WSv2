import logging
import threading
import time
from typing import Any, Dict

from host_agent.score_timeline import normalize_score_payload
from host_agent.timing_telemetry import TimingTelemetry


class MockBackend:
    """Mock backend that simulates a motor run window and never hard-fails."""

    name = "mock"

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._running_until = 0.0
        self._last_speed = 0.0
        self._timestamp = time.time()
        self._score_lock = threading.Lock()
        self._score_thread: threading.Thread | None = None
        self._score_stop_event = threading.Event()
        self._score_playing = False
        self._score_current_index = 0
        self._score_total_events = 0
        self._score_started_monotonic = 0.0
        self._score_expected_end_monotonic = 0.0
        self._score_last_error = ""
        self._score_session_id = 0
        self._timing = TimingTelemetry(max_events=4000)

    def run(self, speed: float, duration: float, port: str = "A") -> Dict[str, Any]:
        with self._lock:
            safe_speed = float(speed)
            safe_duration = max(0.0, float(duration))
            self._last_speed = safe_speed
            self._running_until = time.monotonic() + safe_duration
            self._timestamp = time.time()

        logging.info(
            "[MOCK] motor/run port=%s speed=%.3f duration=%.3f",
            port,
            safe_speed,
            safe_duration,
        )
        return {"accepted": True}

    def stop(self, port: str = "A", stop_action: str = "coast") -> Dict[str, Any]:
        with self._lock:
            self._running_until = 0.0
            self._last_speed = 0.0
            self._timestamp = time.time()

        logging.info("[MOCK] motor/stop port=%s stop_action=%s", port, stop_action)
        return {"stopped": True}

    def _stop_score_worker_locked(self, join_timeout: float = 0.5) -> None:
        thread = self._score_thread
        self._score_stop_event.set()
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=max(0.1, float(join_timeout)))
        self._score_thread = None
        self._score_playing = False

    def _apply_score_event(self, action: Dict[str, Any]) -> None:
        action_type = str(action.get("type", "")).strip().lower()
        port = str(action.get("port", "A")).strip().upper() or "A"
        if action_type == "run":
            speed = float(action.get("speed", 0.0))
            self.run(speed=speed, duration=0.3, port=port)
            return
        if action_type == "stop":
            self.stop(port=port, stop_action=str(action.get("stop_action", "coast")))
            return
        if action_type == "beep":
            self.sound_beep(
                freq_hz=int(action.get("freq_hz", 440)),
                duration_ms=int(action.get("duration_ms", 120)),
                volume=int(action.get("volume", 60)),
            )
            return
        if action_type == "run_for_degrees":
            self.run_for_degrees(
                port=port,
                speed=float(action.get("speed", 0.0)),
                degrees=int(action.get("degrees", 0)),
                stop_action=str(action.get("stop_action", "coast")),
            )
            return
        if action_type == "run_to_absolute_position":
            self.run_to_absolute(
                port=port,
                speed=float(action.get("speed", 0.0)),
                position_degrees=int(action.get("position_degrees", 0)),
                stop_action=str(action.get("stop_action", "coast")),
            )
            return
        if action_type == "run_to_relative_position":
            self.run_to_relative(
                port=port,
                speed=float(action.get("speed", 0.0)),
                degrees=int(action.get("degrees", 0)),
                stop_action=str(action.get("stop_action", "coast")),
            )
            return
        if action_type == "set_duty_cycle":
            self.set_duty_cycle(port=port, speed=float(action.get("speed", 0.0)))
            return

    def _score_worker(self, session_id: int, events: list[dict[str, Any]], start_monotonic: float) -> None:
        self._timing.start_session(label=f"mock_score_{session_id}")
        for index, row in enumerate(events, start=1):
            if self._score_stop_event.is_set():
                break

            scheduled = start_monotonic + (float(row.get("t_rel_ms", 0)) / 1000.0)
            while not self._score_stop_event.is_set():
                remaining = scheduled - time.monotonic()
                if remaining <= 0.0:
                    break
                time.sleep(min(0.01, remaining))
            if self._score_stop_event.is_set():
                break

            actual = time.monotonic()
            action = dict(row.get("action", {}))
            action_type = str(action.get("type", "")).strip().lower()
            pair_key = f"t{int(row.get('t_rel_ms', 0))}"
            try:
                self._apply_score_event(action)
            except Exception as exc:  # noqa: BLE001
                self._score_last_error = str(exc)
                logging.warning("[MOCK] score event failed: %s", exc)

            self._timing.record_event(
                event_id=str(row.get("event_id", f"evt_{index:04d}")),
                event_type=action_type,
                scheduled_time_monotonic=scheduled,
                actual_time_monotonic=actual,
                pair_key=pair_key,
            )
            with self._score_lock:
                self._score_current_index = index

        self.stop()
        self._timing.finish_session()
        with self._score_lock:
            self._score_playing = False

    def score_play(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        try:
            events, metadata = normalize_score_payload(dict(payload or {}))
        except Exception as exc:  # noqa: BLE001
            self._score_last_error = str(exc)
            return {"accepted": False, "error": f"invalid score payload: {exc}"}

        if not events:
            return {"accepted": False, "error": "score has no events"}

        start_mode = str(metadata.get("start_mode", "immediate")).strip().lower()
        start_time_ms = int(metadata.get("start_time_ms", 0))
        now = time.monotonic()
        if start_mode == "at_time" and start_time_ms > 0:
            start_monotonic = now + (start_time_ms / 1000.0)
        else:
            start_monotonic = now

        with self._score_lock:
            self._stop_score_worker_locked(join_timeout=0.5)
            self._score_stop_event = threading.Event()
            self._score_session_id += 1
            session_id = self._score_session_id
            self._score_playing = True
            self._score_current_index = 0
            self._score_total_events = len(events)
            self._score_last_error = ""
            self._score_started_monotonic = start_monotonic
            self._score_expected_end_monotonic = (
                start_monotonic + (float(events[-1].get("t_rel_ms", 0)) / 1000.0)
            )
            worker = threading.Thread(
                target=self._score_worker,
                args=(session_id, events, start_monotonic),
                name=f"mock_score_{session_id}",
                daemon=True,
            )
            self._score_thread = worker
            worker.start()

        return {
            "accepted": True,
            "event_count": len(events),
            "source": metadata.get("source", "events"),
            "start_mode": start_mode,
        }

    def score_stop(self) -> Dict[str, Any]:
        with self._score_lock:
            self._stop_score_worker_locked(join_timeout=0.8)
        self.stop()
        self._timing.finish_session()
        return {"stopped": True}

    def score_status(self) -> Dict[str, Any]:
        with self._score_lock:
            playing = bool(self._score_playing)
            current_index = int(self._score_current_index)
            total = int(self._score_total_events)
            started = float(self._score_started_monotonic)
            expected_end = float(self._score_expected_end_monotonic)
            last_error = str(self._score_last_error or "")

        timing = self._timing.snapshot()
        return {
            "ok": True,
            "playing": playing,
            "current_index": current_index,
            "total_events": total,
            "start_time_monotonic": started,
            "expected_end_monotonic": expected_end,
            "last_error": last_error,
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

    def reset_relative(self, port: str = "A") -> Dict[str, Any]:
        logging.info("[MOCK] motor/reset_relative port=%s", port)
        return {"accepted": True}

    def run_for_degrees(
        self,
        port: str,
        speed: float,
        degrees: int,
        stop_action: str = "coast",
    ) -> Dict[str, Any]:
        simulated = max(0.05, min(2.0, abs(float(degrees)) / 900.0))
        self.run(speed=speed, duration=simulated, port=port)
        self.stop(port=port, stop_action=stop_action)
        logging.info(
            "[MOCK] motor/run_for_degrees port=%s speed=%.3f degrees=%d stop_action=%s",
            port,
            speed,
            degrees,
            stop_action,
        )
        return {"accepted": True}

    def run_to_absolute(
        self,
        port: str,
        speed: float,
        position_degrees: int,
        stop_action: str = "coast",
    ) -> Dict[str, Any]:
        simulated = max(0.05, min(2.0, abs(int(position_degrees)) / 1000.0))
        self.run(speed=speed, duration=simulated, port=port)
        self.stop(port=port, stop_action=stop_action)
        logging.info(
            "[MOCK] motor/run_to_absolute port=%s speed=%.3f position=%d stop_action=%s",
            port,
            speed,
            position_degrees,
            stop_action,
        )
        return {"accepted": True}

    def run_to_relative(
        self,
        port: str,
        speed: float,
        degrees: int,
        stop_action: str = "coast",
    ) -> Dict[str, Any]:
        return self.run_for_degrees(
            port=port,
            speed=speed,
            degrees=degrees,
            stop_action=stop_action,
        )

    def set_duty_cycle(self, port: str, speed: float) -> Dict[str, Any]:
        duty = max(-1.0, min(1.0, float(speed)))
        with self._lock:
            self._last_speed = duty
            self._running_until = time.monotonic() + 0.5
            self._timestamp = time.time()
        logging.info("[MOCK] motor/set_duty_cycle port=%s duty=%.3f", port, duty)
        return {"accepted": True}

    def motor_status(self, port: str = "A") -> Dict[str, Any]:
        state = self.get_state()
        return {"ok": True, "port": port, **state}

    def sound_beep(
        self,
        freq_hz: int,
        duration_ms: int,
        volume: int,
    ) -> Dict[str, Any]:
        safe_freq = int(freq_hz)
        safe_duration = int(duration_ms)
        safe_volume = int(volume)
        logging.info(
            "[MOCK] sound/beep freq_hz=%d duration_ms=%d volume=%d",
            safe_freq,
            safe_duration,
            safe_volume,
        )
        return {
            "accepted": True,
            "freq_hz": safe_freq,
            "duration_ms": safe_duration,
            "volume": safe_volume,
        }

    def sound_stop(self) -> Dict[str, Any]:
        logging.info("[MOCK] sound/stop")
        return {"stopped": True}

    def get_state(self) -> Dict[str, Any]:
        with self._lock:
            running = time.monotonic() < self._running_until
            state = "running" if running else "idle"
            return {
                "state": state,
                "last_speed": self._last_speed,
                "timestamp": self._timestamp,
            }

    def health(self) -> Dict[str, Any]:
        score_status = self.score_status()
        return {
            "ok": True,
            "backend": self.name,
            "spike_connected": True,
            "sound_supported": True,
            "score_playing": bool(score_status.get("playing", False)),
        }

    def close(self) -> None:
        self.score_stop()
        self.stop()
