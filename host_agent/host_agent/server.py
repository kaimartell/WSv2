import json
import inspect
import logging
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Optional
from urllib.parse import urlparse

from host_agent.backends import create_backend


logging.basicConfig(level=logging.INFO, format="[host_agent] %(asctime)s %(levelname)s: %(message)s")


class HostAgentHTTPServer(ThreadingHTTPServer):
    def __init__(
        self,
        server_address: tuple[str, int],
        backend_name: str,
        backend_kwargs: Optional[Dict[str, Any]] = None,
    ) -> None:
        super().__init__(server_address, HostAgentRequestHandler)
        self.backend = create_backend(backend_name, **(backend_kwargs or {}))

    def backend_health(self) -> Dict[str, Any]:
        if hasattr(self.backend, "health"):
            health = self.backend.health()
            if isinstance(health, dict):
                return health
        return {
            "ok": True,
            "backend": getattr(self.backend, "name", "unknown"),
            "spike_connected": True,
        }

    def close_backend(self) -> None:
        if hasattr(self.backend, "close"):
            try:
                self.backend.close()
            except Exception:  # noqa: BLE001
                logging.exception("Backend close() failed")


class HostAgentRequestHandler(BaseHTTPRequestHandler):
    server: HostAgentHTTPServer

    def _invoke_backend(self, method_name: str, **kwargs: Any) -> Any:
        method = getattr(self.server.backend, method_name, None)
        if method is None:
            raise AttributeError(f"backend does not implement {method_name}()")

        signature = inspect.signature(method)
        accepts_var_kwargs = any(
            param.kind == inspect.Parameter.VAR_KEYWORD
            for param in signature.parameters.values()
        )
        if accepts_var_kwargs:
            return method(**kwargs)

        accepted_names = set(signature.parameters.keys())
        filtered = {key: value for key, value in kwargs.items() if key in accepted_names}
        return method(**filtered)

    @staticmethod
    def _response_from_result(result: Any, *, true_key: str, false_default: Dict[str, Any]) -> Dict[str, Any]:
        if isinstance(result, dict):
            payload = dict(result)
            if true_key not in payload:
                payload[true_key] = bool(payload.get("ok", False))
            return payload
        payload = dict(false_default)
        payload[true_key] = bool(result)
        return payload

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        try:
            if path == "/health":
                self._write_json(HTTPStatus.OK, self.server.backend_health())
                return

            if path == "/state":
                self._write_json(HTTPStatus.OK, self.server.backend.get_state())
                return

            self._write_json(HTTPStatus.NOT_FOUND, {"error": "not_found"})
        except Exception as exc:  # noqa: BLE001
            logging.exception("GET %s failed", path)
            self._write_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": str(exc)})

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        try:
            payload = self._read_json_body()
        except ValueError as exc:
            self._write_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
            return

        try:
            if path == "/motor/run":
                speed = float(payload.get("speed", 0.0))
                duration = float(payload.get("duration", 0.0))
                port = str(payload.get("port", "")).strip().upper()
                try:
                    result = self._invoke_backend("run", speed=speed, duration=duration, port=port)
                except Exception:  # noqa: BLE001
                    logging.exception("Backend run() failed")
                    self._write_json(
                        HTTPStatus.OK, {"accepted": False, "error": "backend_run_exception"}
                    )
                    return

                response = self._response_from_result(
                    result,
                    true_key="accepted",
                    false_default={"accepted": False},
                )
                self._write_json(HTTPStatus.OK, response)
                return

            if path == "/motor/stop":
                port = str(payload.get("port", "")).strip().upper()
                stop_action = str(payload.get("stop_action", "")).strip().lower()
                try:
                    result = self._invoke_backend(
                        "stop",
                        port=port,
                        stop_action=stop_action,
                    )
                except Exception:  # noqa: BLE001
                    logging.exception("Backend stop() failed")
                    self._write_json(
                        HTTPStatus.OK, {"stopped": False, "error": "backend_stop_exception"}
                    )
                    return

                response = self._response_from_result(
                    result,
                    true_key="stopped",
                    false_default={"stopped": False},
                )
                self._write_json(HTTPStatus.OK, response)
                return

            if path == "/motor/reset_relative":
                port = str(payload.get("port", "A")).strip().upper() or "A"
                try:
                    result = self._invoke_backend("reset_relative", port=port)
                except Exception:  # noqa: BLE001
                    logging.exception("Backend reset_relative() failed")
                    self._write_json(
                        HTTPStatus.OK,
                        {"accepted": False, "error": "backend_reset_relative_exception"},
                    )
                    return

                response = self._response_from_result(
                    result,
                    true_key="accepted",
                    false_default={"accepted": False},
                )
                self._write_json(HTTPStatus.OK, response)
                return

            if path == "/motor/run_for_degrees":
                port = str(payload.get("port", "A")).strip().upper() or "A"
                speed = float(payload.get("speed", 0.0))
                degrees = int(payload.get("degrees", 0))
                stop_action = str(payload.get("stop_action", "")).strip().lower()
                try:
                    result = self._invoke_backend(
                        "run_for_degrees",
                        port=port,
                        speed=speed,
                        degrees=degrees,
                        stop_action=stop_action,
                    )
                except Exception:  # noqa: BLE001
                    logging.exception("Backend run_for_degrees() failed")
                    self._write_json(
                        HTTPStatus.OK,
                        {"accepted": False, "error": "backend_run_for_degrees_exception"},
                    )
                    return

                response = self._response_from_result(
                    result,
                    true_key="accepted",
                    false_default={"accepted": False},
                )
                self._write_json(HTTPStatus.OK, response)
                return

            if path == "/motor/run_to_absolute":
                port = str(payload.get("port", "A")).strip().upper() or "A"
                speed = float(payload.get("speed", 0.0))
                position_degrees = int(payload.get("position_degrees", 0))
                stop_action = str(payload.get("stop_action", "")).strip().lower()
                try:
                    result = self._invoke_backend(
                        "run_to_absolute",
                        port=port,
                        speed=speed,
                        position_degrees=position_degrees,
                        stop_action=stop_action,
                    )
                except Exception:  # noqa: BLE001
                    logging.exception("Backend run_to_absolute() failed")
                    self._write_json(
                        HTTPStatus.OK,
                        {"accepted": False, "error": "backend_run_to_absolute_exception"},
                    )
                    return

                response = self._response_from_result(
                    result,
                    true_key="accepted",
                    false_default={"accepted": False},
                )
                self._write_json(HTTPStatus.OK, response)
                return

            if path == "/motor/run_to_relative":
                port = str(payload.get("port", "A")).strip().upper() or "A"
                speed = float(payload.get("speed", 0.0))
                degrees = int(payload.get("degrees", 0))
                stop_action = str(payload.get("stop_action", "")).strip().lower()
                try:
                    result = self._invoke_backend(
                        "run_to_relative",
                        port=port,
                        speed=speed,
                        degrees=degrees,
                        stop_action=stop_action,
                    )
                except Exception:  # noqa: BLE001
                    logging.exception("Backend run_to_relative() failed")
                    self._write_json(
                        HTTPStatus.OK,
                        {"accepted": False, "error": "backend_run_to_relative_exception"},
                    )
                    return

                response = self._response_from_result(
                    result,
                    true_key="accepted",
                    false_default={"accepted": False},
                )
                self._write_json(HTTPStatus.OK, response)
                return

            if path == "/motor/set_duty_cycle":
                port = str(payload.get("port", "A")).strip().upper() or "A"
                speed = float(payload.get("speed", 0.0))
                try:
                    result = self._invoke_backend(
                        "set_duty_cycle",
                        port=port,
                        speed=speed,
                    )
                except Exception:  # noqa: BLE001
                    logging.exception("Backend set_duty_cycle() failed")
                    self._write_json(
                        HTTPStatus.OK,
                        {"accepted": False, "error": "backend_set_duty_cycle_exception"},
                    )
                    return

                response = self._response_from_result(
                    result,
                    true_key="accepted",
                    false_default={"accepted": False},
                )
                self._write_json(HTTPStatus.OK, response)
                return

            if path == "/motor/status":
                port = str(payload.get("port", "A")).strip().upper() or "A"
                try:
                    result = self._invoke_backend("motor_status", port=port)
                except Exception:  # noqa: BLE001
                    logging.exception("Backend motor_status() failed")
                    self._write_json(
                        HTTPStatus.OK,
                        {"ok": False, "error": "backend_motor_status_exception"},
                    )
                    return

                if isinstance(result, dict):
                    self._write_json(HTTPStatus.OK, result)
                else:
                    self._write_json(HTTPStatus.OK, {"ok": bool(result)})
                return

            self._write_json(HTTPStatus.NOT_FOUND, {"error": "not_found"})
        except Exception as exc:  # noqa: BLE001
            logging.exception("POST %s failed", path)
            self._write_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": str(exc)})

    def _read_json_body(self) -> Dict[str, Any]:
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0:
            return {}

        body = self.rfile.read(content_length)
        if not body:
            return {}

        parsed = json.loads(body.decode("utf-8"))
        if isinstance(parsed, dict):
            return parsed
        raise ValueError("JSON body must be an object")

    def _write_json(self, status: HTTPStatus, payload: Dict[str, Any]) -> None:
        raw = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def log_message(self, format: str, *args: Any) -> None:  # noqa: A003
        logging.info("%s - %s", self.address_string(), format % args)

    def log_error(self, format: str, *args: Any) -> None:  # noqa: A003
        # Keep noisy TLS/garbage probe traffic out of workshop INFO logs.
        logging.debug("%s - %s", self.address_string(), format % args)


def run_server(
    host: str,
    port: int,
    backend_name: str,
    backend_kwargs: Optional[Dict[str, Any]] = None,
) -> None:
    server = HostAgentHTTPServer((host, port), backend_name=backend_name, backend_kwargs=backend_kwargs)
    logging.info("Starting host agent on http://%s:%s with backend=%s", host, port, backend_name)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logging.info("Shutting down host agent")
    finally:
        server.close_backend()
        server.shutdown()
        server.server_close()
