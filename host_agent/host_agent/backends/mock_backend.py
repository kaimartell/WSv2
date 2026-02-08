import logging
import threading
import time
from typing import Any, Dict


class MockBackend:
    """Mock backend that simulates a motor run window and never hard-fails."""

    name = "mock"

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._running_until = 0.0
        self._last_speed = 0.0
        self._timestamp = time.time()

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
        return {"ok": True, "backend": self.name, "spike_connected": True}

    def close(self) -> None:
        self.stop()
