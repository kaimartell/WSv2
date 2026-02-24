from __future__ import annotations

import json
import sys
from typing import Any, Dict, List

for _path in ("py", "./py"):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from js import console, document  # noqa: E402

import components  # noqa: E402
import rosbridge  # noqa: E402
from state import DEFAULT_ROSBRIDGE_URL, ui_state  # noqa: E402


def _read_number(element_id: str, fallback: float) -> float:
    raw = document.getElementById(element_id).value
    try:
        return float(raw)
    except Exception:
        return float(fallback)


def _set_inputs(speed: float, duration: float) -> None:
    document.getElementById("speed-input").value = str(speed)
    document.getElementById("duration-input").value = str(duration)


def _set_connection_state(payload: Dict[str, Any]) -> None:
    phase = str(payload.get("phase", "idle") or "idle")
    message = str(payload.get("message", "") or "")
    active_url = str(payload.get("url", "") or "")
    requested_url = str(payload.get("requestedUrl", "") or "")

    if active_url:
        ui_state.ros_url = active_url
    elif requested_url:
        ui_state.ros_url = requested_url
    elif not ui_state.ros_url:
        ui_state.ros_url = DEFAULT_ROSBRIDGE_URL

    ui_state.connected = bool(payload.get("connected", False))

    if phase == "connected":
        ui_state.conn_kind = "connected"
        ui_state.conn_text = message or "Connected"
    elif phase == "connecting":
        ui_state.conn_kind = "connecting"
        ui_state.conn_text = message or "Connecting..."
    elif phase == "error":
        ui_state.conn_kind = "error"
        ui_state.conn_text = message or "Connection error"
    else:
        ui_state.conn_kind = ""
        ui_state.conn_text = message or "Disconnected"

    components.render_connection(ui_state)


def _on_connection_state(payload_json: str) -> None:
    try:
        payload = json.loads(str(payload_json))
    except Exception as exc:
        ui_state.set_error(f"Connection state parse error: {exc}")
        components.render_error(ui_state)
        return
    _set_connection_state(payload)


def _on_error(error_text: str) -> None:
    ui_state.set_error(str(error_text))
    components.render_error(ui_state)


def _on_debug(message: str) -> None:
    ui_state.push_ui_log(str(message))
    components.render_ui_log(ui_state)


def _on_status_message(status_text: str) -> None:
    ui_state.push_status(str(status_text))
    components.render_status(ui_state)


def _on_topics_response(topics_json: str, error_text: str) -> None:
    err = str(error_text or "").strip()
    if err:
        ui_state.set_error(err)
        components.render_error(ui_state)

    try:
        parsed = json.loads(str(topics_json or "[]"))
        topics: List[str] = [str(item) for item in (parsed or [])]
    except Exception as exc:
        ui_state.set_error(f"Topic list parse error: {exc}")
        components.render_error(ui_state)
        return

    ui_state.set_topics(topics)
    components.render_topics(ui_state)


def _on_log_message(log_text: str) -> None:
    ui_state.push_log(str(log_text))
    components.render_logs(ui_state)


def _publish_cmd(speed: float, duration: float) -> None:
    payload = {
        "speed": float(speed),
        "duration": max(0.0, float(duration)),
    }
    success = rosbridge.publish_spike_cmd(payload)
    if success:
        ui_state.last_cmd = json.dumps(payload, separators=(",", ":"))
        components.render_last_command(ui_state)


# py-click handlers

def connect_ros(*args):
    url = str(document.getElementById("ws-url").value or "").strip() or DEFAULT_ROSBRIDGE_URL
    rosbridge.connect(url)


def disconnect_ros(*args):
    rosbridge.disconnect()


def list_topics(*args):
    rosbridge.get_topics(_on_topics_response)


def send_run(*args):
    speed = _read_number("speed-input", 0.4)
    duration = max(0.0, _read_number("duration-input", 0.5))
    _publish_cmd(speed, duration)


def send_stop(*args):
    _publish_cmd(0.0, 0.0)


def preset_forward_1s(*args):
    _set_inputs(0.5, 1.0)
    _publish_cmd(0.5, 1.0)


def preset_pulse(*args):
    _set_inputs(0.7, 0.2)
    _publish_cmd(0.7, 0.2)


def subscribe_rosout(*args):
    ui_state.logs_source_text = "Log source: /rosout (rcl_interfaces/Log)"
    ui_state.clear_logs()
    components.render_logs(ui_state)
    rosbridge.subscribe_log_topic("/rosout", "rcl_interfaces/Log", _on_log_message)


def subscribe_ui_log(*args):
    ui_state.logs_source_text = "Log source: /ui/log (std_msgs/String)"
    ui_state.clear_logs()
    components.render_logs(ui_state)
    rosbridge.subscribe_log_topic("/ui/log", "std_msgs/String", _on_log_message)


def unsubscribe_logs(*args):
    rosbridge.unsubscribe_log_topic()
    ui_state.logs_source_text = "Log source: none"
    components.render_logs(ui_state)


def _initialize() -> None:
    components.render_all(ui_state)

    rosbridge.set_connection_state_handler(_on_connection_state)
    rosbridge.set_error_handler(_on_error)
    rosbridge.set_debug_handler(_on_debug)
    rosbridge.subscribe_status(_on_status_message)

    js_state = rosbridge.get_state()
    if js_state:
        _set_connection_state(
            {
                "phase": js_state.get("phase", "idle"),
                "connected": js_state.get("connected", False),
                "requestedUrl": js_state.get("requestedUrl", DEFAULT_ROSBRIDGE_URL),
                "url": js_state.get("activeUrl", "") or js_state.get("requestedUrl", DEFAULT_ROSBRIDGE_URL),
                "message": "Connected" if js_state.get("connected", False) else "Disconnected",
            }
        )
        last_error = str(js_state.get("lastError", "") or "")
        if last_error:
            ui_state.set_error(last_error)
            components.render_error(ui_state)

    ui_state.push_ui_log("PyScript app initialized.")
    components.render_ui_log(ui_state)


try:
    _initialize()
except Exception as exc:
    console.error("[pyscript-app] initialization failed", exc)
    ui_state.set_error(f"UI init failed: {exc}")
    components.render_error(ui_state)
    ui_state.push_ui_log(f"UI init failed: {exc}")
    components.render_ui_log(ui_state)
