from __future__ import annotations

import json
from typing import Any, Callable, Dict, Optional

from js import JSON as JSJSON
from js import window
from pyodide.ffi import create_proxy


Callback = Callable[..., Any]

_proxies: Dict[str, Any] = {}


def _client():
    return window.rosClient


def _destroy_proxy(name: str) -> None:
    proxy = _proxies.pop(name, None)
    if proxy is None:
        return
    try:
        proxy.destroy()
    except Exception:
        pass


def _set_persistent_proxy(name: str, callback: Optional[Callback]) -> None:
    _destroy_proxy(name)
    if callback is None:
        return
    _proxies[name] = create_proxy(callback)


def _js_bool(value: Any) -> bool:
    try:
        return bool(value)
    except Exception:
        return False


def set_connection_state_handler(callback: Optional[Callable[[str], Any]]) -> bool:
    if callback is None:
        _destroy_proxy("connection_state")
        return _js_bool(_client().setConnectionStateCallback(None))
    _set_persistent_proxy("connection_state", callback)
    return _js_bool(_client().setConnectionStateCallback(_proxies["connection_state"]))


def set_error_handler(callback: Optional[Callable[[str], Any]]) -> bool:
    if callback is None:
        _destroy_proxy("error")
        return _js_bool(_client().setErrorCallback(None))
    _set_persistent_proxy("error", callback)
    return _js_bool(_client().setErrorCallback(_proxies["error"]))


def set_debug_handler(callback: Optional[Callable[[str], Any]]) -> bool:
    if callback is None:
        _destroy_proxy("debug")
        return _js_bool(_client().setDebugCallback(None))
    _set_persistent_proxy("debug", callback)
    return _js_bool(_client().setDebugCallback(_proxies["debug"]))


def connect(url: str) -> bool:
    return _js_bool(_client().connect(str(url)))


def disconnect() -> bool:
    return _js_bool(_client().disconnect())


def is_connected() -> bool:
    return _js_bool(_client().isConnected())


def publish_spike_cmd(payload_obj: Dict[str, Any]) -> bool:
    payload_js = JSJSON.parse(json.dumps(payload_obj))
    return _js_bool(_client().publishSpikeCmd(payload_js))


def subscribe_status(callback: Callable[[str], Any]) -> bool:
    _set_persistent_proxy("status", callback)
    return _js_bool(_client().subscribeStatus(_proxies["status"]))


def unsubscribe_status() -> bool:
    _destroy_proxy("status")
    return _js_bool(_client().unsubscribeStatus())


def get_topics(callback: Callable[[str, str], Any]) -> bool:
    holder: Dict[str, Any] = {}

    def _wrapped(topics_json: str, error_text: str) -> None:
        try:
            callback(str(topics_json), str(error_text))
        finally:
            proxy = holder.pop("proxy", None)
            if proxy is not None:
                try:
                    proxy.destroy()
                except Exception:
                    pass

    proxy = create_proxy(_wrapped)
    holder["proxy"] = proxy
    return _js_bool(_client().getTopics(proxy))


def subscribe_log_topic(topic_name: str, message_type: str, callback: Callable[[str], Any]) -> bool:
    _set_persistent_proxy("log", callback)
    return _js_bool(
        _client().subscribeLogTopic(str(topic_name), str(message_type), _proxies["log"])
    )


def unsubscribe_log_topic() -> bool:
    _destroy_proxy("log")
    return _js_bool(_client().unsubscribeLogTopic())


def get_state() -> Dict[str, Any]:
    raw = _client().getState()
    try:
        return {
            "phase": str(raw.phase),
            "connected": bool(raw.connected),
            "requestedUrl": str(raw.requestedUrl),
            "activeUrl": str(raw.activeUrl),
            "lastError": str(raw.lastError),
            "autoFallbackUsed": bool(raw.autoFallbackUsed),
        }
    except Exception:
        return {}
