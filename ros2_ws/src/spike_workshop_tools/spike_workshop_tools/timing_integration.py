#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any, Dict, Optional
from urllib import error, request

import rclpy
from rclpy.node import Node
from spike_workshop_interfaces.srv import GenerateScore, PlayPattern
from std_msgs.msg import Empty
from std_srvs.srv import Trigger


class IntegrationNode(Node):
    def __init__(self) -> None:
        super().__init__("timing_integration_runner")
        self._done_received = False
        self._done_subscription = self.create_subscription(Empty, "/done", self._on_done, 10)

    def _on_done(self, _msg: Empty) -> None:
        self._done_received = True

    def reset_done(self) -> None:
        self._done_received = False

    def done_received(self) -> bool:
        return self._done_received


def _http_get_json(url: str, timeout_sec: float) -> Dict[str, Any]:
    req = request.Request(url=url, method="GET", headers={"Accept": "application/json"})
    try:
        with request.urlopen(req, timeout=max(0.1, float(timeout_sec))) as resp:
            payload = resp.read().decode("utf-8")
        data = json.loads(payload) if payload else {}
        if isinstance(data, dict):
            return data
        return {"ok": False, "error": "non-object JSON response"}
    except error.HTTPError as exc:
        return {"ok": False, "error": f"HTTP {exc.code}"}
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "error": str(exc)}


def _call_service(
    node: Node,
    *,
    service_name: str,
    service_type: Any,
    request_msg: Any,
    timeout_sec: float,
) -> Any:
    client = node.create_client(service_type, service_name)
    if not client.wait_for_service(timeout_sec=max(0.1, float(timeout_sec))):
        raise RuntimeError(f"service unavailable: {service_name}")

    future = client.call_async(request_msg)
    rclpy.spin_until_future_complete(node, future, timeout_sec=max(0.1, float(timeout_sec)))
    if not future.done():
        raise TimeoutError(f"service call timed out: {service_name}")
    result = future.result()
    if result is None:
        raise RuntimeError(f"service call failed: {service_name}")
    return result


def _default_score_name() -> str:
    return f"timing_score_{time.strftime('%Y%m%d_%H%M%S')}"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Container-side ROS integration check: generate score, play with /instrument/play_score, "
            "then fetch host-agent /debug/timing and save a JSON report."
        )
    )
    parser.add_argument("--name", default="", help="Score/pattern name (default: auto timestamp)")
    parser.add_argument("--bpm", type=float, default=120.0, help="Score BPM")
    parser.add_argument("--repeats", type=int, default=1, help="Score repeats")
    parser.add_argument("--speed", type=float, default=0.5, help="Motor speed magnitude")
    parser.add_argument(
        "--motor",
        default="F S F S | F S F S | B S B S | B S B S",
        help="Motor score lane string (16 beats / 4 bars)",
    )
    parser.add_argument(
        "--melody",
        default="C4 - E4 - | G4 - C5 - | - - - - | - - - -",
        help="Melody lane string (16 beats / 4 bars, '-' for rest)",
    )
    parser.add_argument("--volume", type=int, default=60, help="Beep volume [0..100]")
    parser.add_argument(
        "--host-agent-url",
        default="http://host.docker.internal:8000",
        help="Host agent base URL for /debug/timing",
    )
    parser.add_argument(
        "--service-timeout-sec",
        type=float,
        default=8.0,
        help="ROS service call timeout",
    )
    parser.add_argument(
        "--wait-timeout-sec",
        type=float,
        default=0.0,
        help="Override playback wait timeout. Default estimates from bpm/repeats.",
    )
    parser.add_argument(
        "--output-dir",
        default="/tmp",
        help="Directory for JSON report",
    )
    parser.add_argument(
        "--stop-on-timeout",
        action="store_true",
        help="If /done is not observed before timeout, call /instrument/stop.",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    score_name = (args.name or "").strip() or _default_score_name()
    output_dir = Path(args.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = IntegrationNode()
    report: Dict[str, Any] = {
        "generated_at_unix": time.time(),
        "score_name": score_name,
        "params": {
            "bpm": float(args.bpm),
            "repeats": int(args.repeats),
            "speed": float(args.speed),
            "motor": str(args.motor),
            "melody": str(args.melody),
            "volume": int(args.volume),
            "host_agent_url": str(args.host_agent_url).rstrip("/"),
        },
    }

    stop_sent = False
    try:
        ping_req = Trigger.Request()
        ping_resp = _call_service(
            node,
            service_name="/spike/ping",
            service_type=Trigger,
            request_msg=ping_req,
            timeout_sec=args.service_timeout_sec,
        )
        report["ping"] = {"success": bool(ping_resp.success), "message": str(ping_resp.message)}

        gen_req = GenerateScore.Request()
        gen_req.name = score_name
        gen_req.bpm = float(args.bpm)
        gen_req.repeats = int(args.repeats)
        gen_req.speed = float(args.speed)
        gen_req.motor = str(args.motor)
        gen_req.melody = str(args.melody)
        gen_req.volume = int(args.volume)
        gen_resp = _call_service(
            node,
            service_name="/instrument/generate_score",
            service_type=GenerateScore,
            request_msg=gen_req,
            timeout_sec=args.service_timeout_sec,
        )
        report["generate_score"] = {
            "ok": bool(gen_resp.ok),
            "message": str(gen_resp.message),
            "output_path": str(gen_resp.output_path),
            "pattern_name": str(gen_resp.pattern_name),
        }
        if not gen_resp.ok:
            raise RuntimeError(f"/instrument/generate_score failed: {gen_resp.message}")

        play_req = PlayPattern.Request()
        play_req.pattern_name = str(gen_resp.pattern_name or score_name)
        play_req.pattern_path = ""

        node.reset_done()
        play_resp = _call_service(
            node,
            service_name="/instrument/play_score",
            service_type=PlayPattern,
            request_msg=play_req,
            timeout_sec=max(10.0, float(args.service_timeout_sec)),
        )
        report["play_score"] = {
            "accepted": bool(play_resp.accepted),
            "message": str(play_resp.message),
        }
        if not play_resp.accepted:
            raise RuntimeError(f"/instrument/play_score rejected: {play_resp.message}")

        if float(args.wait_timeout_sec) > 0.0:
            wait_timeout = float(args.wait_timeout_sec)
        else:
            beat_sec = 60.0 / max(1.0, float(args.bpm))
            wait_timeout = beat_sec * 16.0 * max(1, int(args.repeats)) + 5.0
        deadline = time.monotonic() + wait_timeout
        while time.monotonic() < deadline:
            if node.done_received():
                break
            rclpy.spin_once(node, timeout_sec=0.1)

        completed = node.done_received()
        report["done_received"] = bool(completed)
        report["wait_timeout_sec"] = float(wait_timeout)

        if (not completed) and args.stop_on_timeout:
            stop_req = Trigger.Request()
            stop_resp = _call_service(
                node,
                service_name="/instrument/stop",
                service_type=Trigger,
                request_msg=stop_req,
                timeout_sec=args.service_timeout_sec,
            )
            stop_sent = True
            report["stop_after_timeout"] = {
                "success": bool(stop_resp.success),
                "message": str(stop_resp.message),
            }

        debug_timing = _http_get_json(
            f"{str(args.host_agent_url).rstrip('/')}/debug/timing",
            timeout_sec=5.0,
        )
        report["debug_timing"] = debug_timing
        if "delta_ms" in debug_timing and isinstance(debug_timing.get("delta_ms"), dict):
            report["summary"] = {
                "delta_p95_ms": debug_timing["delta_ms"].get("p95", 0.0),
                "pair_delta_p95_ms": (
                    debug_timing.get("pair_delta_ms", {}).get("p95", 0.0)
                    if isinstance(debug_timing.get("pair_delta_ms"), dict)
                    else 0.0
                ),
                "event_count": debug_timing.get("event_count", 0),
                "dropped_events": debug_timing.get("dropped_events", 0),
            }

    except Exception as exc:  # noqa: BLE001
        report["ok"] = False
        report["error"] = str(exc)
    finally:
        if (not stop_sent) and args.stop_on_timeout:
            try:
                stop_req = Trigger.Request()
                stop_resp = _call_service(
                    node,
                    service_name="/instrument/stop",
                    service_type=Trigger,
                    request_msg=stop_req,
                    timeout_sec=2.0,
                )
                report.setdefault(
                    "final_stop",
                    {"success": bool(stop_resp.success), "message": str(stop_resp.message)},
                )
            except Exception:  # noqa: BLE001
                pass
        report.setdefault("ok", True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        out_path = output_dir / f"ros_timing_report_{stamp}.json"
        out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"Saved report: {out_path}")
        if "summary" in report:
            summary = report["summary"]
            print(
                "delta_p95_ms="
                f"{summary.get('delta_p95_ms', 0.0)} "
                f"pair_delta_p95_ms={summary.get('pair_delta_p95_ms', 0.0)} "
                f"event_count={summary.get('event_count', 0)} "
                f"dropped_events={summary.get('dropped_events', 0)}"
            )
        if not report.get("ok", False):
            print(f"ERROR: {report.get('error', 'unknown failure')}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
