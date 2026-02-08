import asyncio
import logging
import threading
import time
from typing import Any, Dict, List, Optional


# LEGO Wireless Protocol (LWP3) service/characteristic UUIDs used by SPIKE hubs.
LWP_SERVICE_UUID = "00001623-1212-efde-1623-785feabcd123"
LWP_CHARACTERISTIC_UUID = "00001624-1212-efde-1623-785feabcd123"

# Typical SPIKE Prime external motor port mapping.
PORT_ID_MAP = {
    "A": 0x00,
    "B": 0x01,
    "C": 0x02,
    "D": 0x03,
    "E": 0x04,
    "F": 0x05,
}


def _run_coro(coro):
    loop = asyncio.new_event_loop()
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


def discover_spike_ble_devices(timeout: float = 4.0, name_filter: str = "") -> List[Dict[str, Any]]:
    try:
        from bleak import BleakScanner  # type: ignore
    except Exception as exc:  # noqa: BLE001
        return [{"error": f"BLE discovery unavailable: {exc}"}]

    async def _discover() -> List[Dict[str, Any]]:
        devices = await BleakScanner.discover(timeout=max(0.5, float(timeout)))
        lowered_filter = name_filter.strip().lower()

        rows: List[Dict[str, Any]] = []
        for dev in devices:
            name = dev.name or ""
            if lowered_filter and lowered_filter not in name.lower() and lowered_filter not in dev.address.lower():
                continue
            rows.append(
                {
                    "name": name,
                    "address": dev.address,
                    "rssi": getattr(dev, "rssi", None),
                }
            )

        rows.sort(key=lambda item: ((item.get("name") or "").lower(), item.get("address") or ""))
        return rows

    try:
        return _run_coro(_discover())
    except Exception as exc:  # noqa: BLE001
        return [{"error": f"BLE discovery failed: {exc}"}]


class SpikeBleBackend:
    """Optional real backend using BLE + LEGO Wireless Protocol (LWP3)."""

    name = "spike_ble"

    def __init__(
        self,
        hub_name: str = "SPIKE",
        hub_address: str = "",
        motor_port: str = "A",
        scan_timeout: float = 4.0,
        connect_timeout: float = 6.0,
    ) -> None:
        self._hub_name = hub_name.strip() or "SPIKE"
        self._hub_address = hub_address.strip()
        self._motor_port = motor_port.strip().upper() or "A"
        self._scan_timeout = max(0.5, float(scan_timeout))
        self._connect_timeout = max(1.0, float(connect_timeout))

        self._lock = threading.Lock()
        self._cancel_event = threading.Event()
        self._worker: Optional[threading.Thread] = None

        self._state = "idle"
        self._last_speed = 0.0
        self._running_until = 0.0
        self._timestamp = time.time()
        self._spike_connected = False
        self._command_id = 0

        self._warned_tags: set[str] = set()

        self._bleak_scanner_cls = None
        self._bleak_client_cls = None
        self._dependency_error: Optional[str] = None
        self._load_dependencies()

    def _load_dependencies(self) -> None:
        try:
            from bleak import BleakClient, BleakScanner  # type: ignore

            self._bleak_client_cls = BleakClient
            self._bleak_scanner_cls = BleakScanner
        except Exception as exc:  # noqa: BLE001
            self._dependency_error = str(exc)
            self._warn_once(
                "ble_dep",
                "spike_ble backend is available but BLE dependencies are missing. "
                "Install with: pip install bleak",
            )

    def _warn_once(self, tag: str, message: str) -> None:
        with self._lock:
            if tag in self._warned_tags:
                return
            self._warned_tags.add(tag)
        logging.warning(message)

    def _port_id(self) -> int:
        port_id = PORT_ID_MAP.get(self._motor_port)
        if port_id is not None:
            return port_id
        self._warn_once("bad_port", f"Unknown motor port '{self._motor_port}', defaulting to A")
        return PORT_ID_MAP["A"]

    def _build_start_power_packet(self, speed: float) -> bytes:
        # Port Output Command (0x81) / StartPower (0x01)
        power = int(max(-1.0, min(1.0, float(speed))) * 100.0)
        return bytes(
            [
                0x08,  # message length
                0x00,  # hub id
                0x81,  # port output command
                self._port_id(),
                0x11,  # execute immediately
                0x51,  # command feedback
                0x01,  # StartPower subcommand
                power & 0xFF,
            ]
        )

    def _set_connected(self, connected: bool) -> None:
        with self._lock:
            self._spike_connected = connected

    def _set_state(self, *, state: Optional[str] = None, speed: Optional[float] = None) -> None:
        with self._lock:
            if state is not None:
                self._state = state
            if speed is not None:
                self._last_speed = float(speed)
            self._timestamp = time.time()

    def _get_target_address(self, timeout: Optional[float] = None) -> Optional[str]:
        if self._hub_address:
            return self._hub_address

        if self._bleak_scanner_cls is None:
            return None

        scan_timeout = self._scan_timeout if timeout is None else max(0.5, float(timeout))

        async def _scan_for_hub() -> Optional[str]:
            devices = await self._bleak_scanner_cls.discover(timeout=scan_timeout)
            target_name = self._hub_name.lower()
            for dev in devices:
                name = (dev.name or "").lower()
                if target_name in name:
                    return dev.address
            return None

        try:
            return _run_coro(_scan_for_hub())
        except Exception as exc:  # noqa: BLE001
            self._warn_once("scan_fail", f"BLE scan failed: {exc}")
            return None

    def _write_packet(self, packet: bytes) -> bool:
        if self._dependency_error is not None:
            return False

        address = self._get_target_address()
        if not address:
            self._warn_once(
                "hub_not_found",
                f"SPIKE hub not found (hub_name='{self._hub_name}'). Use --hub-name/--hub-address.",
            )
            self._set_connected(False)
            return False

        async def _connect_write_disconnect() -> bool:
            client = self._bleak_client_cls(address, timeout=self._connect_timeout)
            try:
                await asyncio.wait_for(client.connect(), timeout=self._connect_timeout)
                try:
                    await client.write_gatt_char(LWP_CHARACTERISTIC_UUID, packet, response=False)
                except Exception:
                    await client.write_gatt_char(LWP_CHARACTERISTIC_UUID, packet, response=True)
                return True
            finally:
                try:
                    if client.is_connected:
                        await client.disconnect()
                except Exception:  # noqa: BLE001
                    pass

        try:
            ok = bool(_run_coro(_connect_write_disconnect()))
            self._set_connected(ok)
            return ok
        except Exception as exc:  # noqa: BLE001
            self._warn_once("write_fail", f"Failed to send BLE motor command: {exc}")
            self._set_connected(False)
            return False

    def _run_worker(self, cmd_id: int, speed: float, duration: float, cancel_event: threading.Event) -> None:
        started = self._write_packet(self._build_start_power_packet(speed))
        if not started:
            self._set_state(state="idle", speed=0.0)
            return

        deadline = time.monotonic() + max(0.0, float(duration))
        while time.monotonic() < deadline:
            if cancel_event.wait(timeout=0.05):
                break

        self._write_packet(self._build_start_power_packet(0.0))
        with self._lock:
            if cmd_id == self._command_id:
                self._state = "idle"
                self._last_speed = 0.0
                self._running_until = 0.0
                self._timestamp = time.time()

    def run(self, speed: float, duration: float, port: str = "") -> Dict[str, Any]:
        if port:
            self._motor_port = str(port).strip().upper() or self._motor_port
        safe_speed = max(-1.0, min(1.0, float(speed)))
        safe_duration = max(0.0, float(duration))

        prev_worker: Optional[threading.Thread]
        with self._lock:
            self._cancel_event.set()
            prev_worker = self._worker

            self._cancel_event = threading.Event()
            self._command_id += 1
            current_cmd_id = self._command_id
            self._state = "running"
            self._last_speed = safe_speed
            self._running_until = time.monotonic() + safe_duration
            self._timestamp = time.time()

            worker = threading.Thread(
                target=self._run_worker,
                args=(current_cmd_id, safe_speed, safe_duration, self._cancel_event),
                daemon=True,
            )
            self._worker = worker
            worker.start()

        if prev_worker is not None and prev_worker.is_alive():
            prev_worker.join(timeout=0.2)

        logging.info(
            "[SPIKE_BLE] motor/run speed=%.3f duration=%.3f hub_name=%s hub_address=%s port=%s",
            safe_speed,
            safe_duration,
            self._hub_name,
            self._hub_address or "<scan>",
            self._motor_port,
        )
        return {"accepted": True}

    def stop(self, port: str = "", stop_action: str = "coast") -> Dict[str, Any]:
        if port:
            self._motor_port = str(port).strip().upper() or self._motor_port
        worker: Optional[threading.Thread]
        with self._lock:
            self._cancel_event.set()
            worker = self._worker
            self._state = "idle"
            self._last_speed = 0.0
            self._running_until = 0.0
            self._timestamp = time.time()

        if worker is not None and worker.is_alive():
            worker.join(timeout=0.5)

        self._write_packet(self._build_start_power_packet(0.0))
        logging.info("[SPIKE_BLE] motor/stop stop_action=%s", stop_action)
        return {"stopped": True}

    def reset_relative(self, port: str = "A") -> Dict[str, Any]:
        return {"accepted": False, "error": "reset_relative not implemented for spike_ble"}

    def run_for_degrees(
        self,
        port: str,
        speed: float,
        degrees: int,
        stop_action: str = "coast",
    ) -> Dict[str, Any]:
        # BLE fallback approximates by running for a short estimated duration.
        estimated_duration = max(0.1, min(4.0, abs(int(degrees)) / 700.0))
        run_result = self.run(speed=speed, duration=estimated_duration, port=port)
        if not bool(run_result.get("accepted", False)):
            return run_result
        self.stop(port=port, stop_action=stop_action)
        return {"accepted": True, "note": "approximate run_for_degrees over BLE"}

    def run_to_absolute(
        self,
        port: str,
        speed: float,
        position_degrees: int,
        stop_action: str = "coast",
    ) -> Dict[str, Any]:
        estimated_duration = max(0.1, min(4.0, abs(int(position_degrees)) / 800.0))
        run_result = self.run(speed=speed, duration=estimated_duration, port=port)
        if not bool(run_result.get("accepted", False)):
            return run_result
        self.stop(port=port, stop_action=stop_action)
        return {"accepted": True, "note": "approximate run_to_absolute over BLE"}

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
        return self.run(speed=float(speed), duration=0.2, port=port)

    def motor_status(self, port: str = "A") -> Dict[str, Any]:
        state = self.get_state()
        return {"ok": True, "port": port, **state}

    def _probe_connection(self) -> bool:
        if self._dependency_error is not None:
            return False

        address = self._get_target_address(timeout=min(2.0, self._scan_timeout))
        if not address:
            return False

        async def _probe() -> bool:
            client = self._bleak_client_cls(address, timeout=self._connect_timeout)
            try:
                await asyncio.wait_for(client.connect(), timeout=self._connect_timeout)
                return bool(client.is_connected)
            finally:
                try:
                    if client.is_connected:
                        await client.disconnect()
                except Exception:  # noqa: BLE001
                    pass

        try:
            return bool(_run_coro(_probe()))
        except Exception:  # noqa: BLE001
            return False

    def get_state(self) -> Dict[str, Any]:
        with self._lock:
            running = time.monotonic() < self._running_until and self._state == "running"
            state = "running" if running else "idle"
            return {
                "state": state,
                "last_speed": self._last_speed,
                "timestamp": self._timestamp,
            }

    def health(self) -> Dict[str, Any]:
        connected = self._probe_connection()
        self._set_connected(connected)

        response: Dict[str, Any] = {
            "ok": True,
            "backend": self.name,
            "spike_connected": connected,
        }
        if self._dependency_error is not None:
            response["detail"] = f"BLE dependency missing: {self._dependency_error}"
        return response

    def close(self) -> None:
        try:
            self.stop()
        except Exception:  # noqa: BLE001
            pass
