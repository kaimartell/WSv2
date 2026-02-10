from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path
from typing import Any, Dict, List, Tuple
from urllib import error, request

from host_agent.score_timeline import normalize_score_payload


def _http_request(
    *,
    method: str,
    url: str,
    payload: Dict[str, Any] | None,
    timeout_sec: float = 5.0,
) -> Tuple[Dict[str, Any], float, str]:
    start = time.monotonic()
    data = None
    headers = {"Accept": "application/json"}
    if payload is not None:
        headers["Content-Type"] = "application/json"
        data = json.dumps(payload).encode("utf-8")

    req = request.Request(url=url, data=data, method=method, headers=headers)
    try:
        with request.urlopen(req, timeout=timeout_sec) as resp:
            raw = resp.read().decode("utf-8")
        latency_ms = (time.monotonic() - start) * 1000.0
        parsed = json.loads(raw) if raw else {}
        if isinstance(parsed, dict):
            return parsed, latency_ms, ""
        return {"raw": parsed}, latency_ms, ""
    except error.HTTPError as exc:
        latency_ms = (time.monotonic() - start) * 1000.0
        body = ""
        try:
            body = exc.read().decode("utf-8")
        except Exception:  # noqa: BLE001
            body = ""
        return {"error": f"HTTP {exc.code}", "body": body}, latency_ms, f"HTTP {exc.code}"
    except Exception as exc:  # noqa: BLE001
        latency_ms = (time.monotonic() - start) * 1000.0
        return {"error": str(exc)}, latency_ms, str(exc)


def _stats(values: List[float]) -> Dict[str, float]:
    if not values:
        return {"count": 0.0, "mean": 0.0, "p95": 0.0, "max": 0.0}
    ordered = sorted(values)
    if len(ordered) == 1:
        p95 = ordered[0]
    else:
        rank = 0.95 * (len(ordered) - 1)
        lo = int(rank)
        hi = min(len(ordered) - 1, lo + 1)
        frac = rank - lo
        p95 = ordered[lo] * (1.0 - frac) + ordered[hi] * frac
    return {
        "count": float(len(ordered)),
        "mean": float(statistics.fmean(ordered)),
        "p95": float(p95),
        "max": float(ordered[-1]),
    }


def _summarize_local_timing(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    deltas = [float(row["delta_ms"]) for row in rows]
    pair_map: Dict[str, List[float]] = {}
    for row in rows:
        pair_key = str(row.get("pair_key", ""))
        if not pair_key:
            continue
        pair_map.setdefault(pair_key, []).append(float(row["actual"]))
    pair_deltas = []
    for vals in pair_map.values():
        if len(vals) >= 2:
            pair_deltas.append((max(vals) - min(vals)) * 1000.0)
    return {
        "event_count": len(rows),
        "delta_ms": _stats(deltas),
        "pair_delta_ms": _stats(pair_deltas),
        "recent_events": rows[-50:],
    }


def _wait_until(monotonic_target: float) -> None:
    while True:
        remain = monotonic_target - time.monotonic()
        if remain <= 0.0:
            return
        time.sleep(min(0.005, remain))


def _run_stream_trial(base_url: str, score_payload: Dict[str, Any]) -> Dict[str, Any]:
    events, _meta = normalize_score_payload(score_payload)
    rows: List[Dict[str, Any]] = []
    start = time.monotonic() + 0.2

    for event in events:
        t_rel_ms = int(event.get("t_rel_ms", 0))
        scheduled = start + (t_rel_ms / 1000.0)
        _wait_until(scheduled)
        actual = time.monotonic()
        action = dict(event.get("action", {}))
        action_type = str(action.get("type", "")).strip().lower()
        pair_key = f"t{t_rel_ms}"

        if action_type == "run":
            path = "/motor/run"
            payload = {
                "speed": float(action.get("speed", 0.0)),
                "duration": 0.0,
                "port": str(action.get("port", "A")).strip().upper() or "A",
            }
        elif action_type == "stop":
            path = "/motor/stop"
            payload = {
                "port": str(action.get("port", "A")).strip().upper() or "A",
                "stop_action": str(action.get("stop_action", "coast")).strip().lower() or "coast",
            }
        elif action_type == "beep":
            path = "/sound/beep"
            payload = {
                "freq_hz": int(action.get("freq_hz", 440)),
                "duration_ms": int(action.get("duration_ms", 120)),
                "volume": int(action.get("volume", 60)),
            }
        else:
            continue

        response, _latency_ms, error_msg = _http_request(
            method="POST",
            url=f"{base_url}{path}",
            payload=payload,
            timeout_sec=5.0,
        )
        accepted = bool(
            response.get("accepted", response.get("stopped", response.get("ok", False)))
        )
        rows.append(
            {
                "event_id": str(event.get("event_id", "")),
                "event_type": action_type,
                "scheduled": scheduled,
                "actual": actual,
                "delta_ms": (actual - scheduled) * 1000.0,
                "pair_key": pair_key,
                "accepted": accepted,
                "error": error_msg or str(response.get("error", "")),
            }
        )

    return _summarize_local_timing(rows)


def _run_host_score_trial(base_url: str, score_payload: Dict[str, Any]) -> Dict[str, Any]:
    stop_url = f"{base_url}/score/stop"
    _http_request(method="POST", url=stop_url, payload={}, timeout_sec=2.0)

    play_resp, _latency_ms, play_err = _http_request(
        method="POST",
        url=f"{base_url}/score/play",
        payload=score_payload,
        timeout_sec=10.0,
    )
    if play_err or not bool(play_resp.get("accepted", False)):
        return {"error": play_err or str(play_resp.get("error", "score/play rejected")), "ok": False}

    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        status, _latency_ms, _status_err = _http_request(
            method="GET",
            url=f"{base_url}/score/status",
            payload=None,
            timeout_sec=3.0,
        )
        if not bool(status.get("playing", False)):
            break
        time.sleep(0.05)

    debug_timing, _latency_ms, debug_err = _http_request(
        method="GET",
        url=f"{base_url}/debug/timing",
        payload=None,
        timeout_sec=5.0,
    )
    if debug_err:
        return {"error": debug_err, "ok": False}
    debug_timing["ok"] = bool(debug_timing.get("ok", True))
    return debug_timing


def _recommendation(stream_summary: Dict[str, Any], host_summary: Dict[str, Any]) -> str:
    stream_p95 = float(stream_summary.get("delta_ms", {}).get("p95", 0.0))
    host_p95 = float(host_summary.get("delta_ms", {}).get("p95", 0.0))
    improvement = stream_p95 - host_p95
    if improvement > 0.0:
        return f"host_score improves p95 by {improvement:.2f} ms"
    if improvement < 0.0:
        return f"stream is currently better by {-improvement:.2f} ms p95"
    return "host_score and stream p95 are equivalent in this run"


def _score_cases() -> List[Tuple[str, Dict[str, Any]]]:
    return [
        (
            "motor_only_120bpm_2bars",
            {
                "port": "A",
                "bpm": 120.0,
                "repeats": 1,
                "speed": 0.5,
                "motor": "F S F S | F S F S | S S S S | S S S S",
                "melody": "",
                "volume": 60,
            },
        ),
        (
            "melody_only_120bpm",
            {
                "port": "A",
                "bpm": 120.0,
                "repeats": 1,
                "speed": 0.5,
                "motor": "S S S S | S S S S | S S S S | S S S S",
                "melody": "C4 D4 E4 F4 | G4 A4 B4 C5 | C5 B4 A4 G4 | F4 E4 D4 C4",
                "volume": 60,
            },
        ),
        (
            "combined_90bpm",
            {
                "port": "A",
                "bpm": 90.0,
                "repeats": 1,
                "speed": 0.6,
                "motor": "F S F S | B S B S | F S F S | B S B S",
                "melody": "C4 - E4 - | G4 - E4 - | C5 - B4 - | A4 - G4 -",
                "volume": 60,
            },
        ),
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Timing benchmark for stream vs host_score execution.")
    parser.add_argument("--host", default="127.0.0.1", help="host_agent bind host")
    parser.add_argument("--port", type=int, default=8000, help="host_agent port")
    parser.add_argument("--trials", type=int, default=3, help="Trials per score per mode")
    parser.add_argument(
        "--auto-iterate",
        action="store_true",
        help="Run both modes and print recommendation (default behavior).",
    )
    parser.add_argument(
        "--mode",
        choices=["both", "stream", "host_score"],
        default="both",
        help="Limit benchmark to one mode.",
    )
    parser.add_argument("--output-dir", default="artifacts", help="Directory for JSON reports.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    base_url = f"http://{args.host}:{args.port}"
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    report: Dict[str, Any] = {
        "generated_at_unix": time.time(),
        "base_url": base_url,
        "trials": int(args.trials),
        "cases": [],
    }

    health, _latency, health_err = _http_request(
        method="GET", url=f"{base_url}/health", payload=None, timeout_sec=5.0
    )
    report["health"] = health
    if health_err:
        report["error"] = health_err
        print(f"ERROR: health check failed: {health_err}")
        return

    cases = _score_cases()
    for case_name, payload in cases:
        case_row: Dict[str, Any] = {"name": case_name}
        print(f"\n== {case_name} ==")

        if args.mode in {"both", "stream"}:
            stream_trials: List[Dict[str, Any]] = []
            for _idx in range(max(1, int(args.trials))):
                trial = _run_stream_trial(base_url, payload)
                stream_trials.append(trial)
                time.sleep(0.2)
            stream_p95_values = [float(t.get("delta_ms", {}).get("p95", 0.0)) for t in stream_trials]
            pair_p95_values = [float(t.get("pair_delta_ms", {}).get("p95", 0.0)) for t in stream_trials]
            case_row["stream"] = {
                "trials": stream_trials,
                "summary": {
                    "delta_ms": _stats(stream_p95_values),
                    "pair_delta_ms": _stats(pair_p95_values),
                },
            }
            print(
                "stream p95(ms): "
                f"{case_row['stream']['summary']['delta_ms']['mean']:.2f}"
            )

        if args.mode in {"both", "host_score"}:
            host_trials: List[Dict[str, Any]] = []
            for _idx in range(max(1, int(args.trials))):
                trial = _run_host_score_trial(base_url, payload)
                host_trials.append(trial)
                time.sleep(0.2)
            host_p95_values = [float(t.get("delta_ms", {}).get("p95", 0.0)) for t in host_trials if isinstance(t, dict)]
            host_pair_p95_values = [
                float(t.get("pair_delta_ms", {}).get("p95", 0.0)) for t in host_trials if isinstance(t, dict)
            ]
            case_row["host_score"] = {
                "trials": host_trials,
                "summary": {
                    "delta_ms": _stats(host_p95_values),
                    "pair_delta_ms": _stats(host_pair_p95_values),
                },
            }
            print(
                "host_score p95(ms): "
                f"{case_row['host_score']['summary']['delta_ms']['mean']:.2f}"
            )

        if args.mode == "both" and "stream" in case_row and "host_score" in case_row:
            stream_summary = {
                "delta_ms": {
                    "p95": case_row["stream"]["summary"]["delta_ms"]["mean"],
                }
            }
            host_summary = {
                "delta_ms": {
                    "p95": case_row["host_score"]["summary"]["delta_ms"]["mean"],
                }
            }
            case_row["recommendation"] = _recommendation(stream_summary, host_summary)
            print(case_row["recommendation"])

        report["cases"].append(case_row)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_path = output_dir / f"timing_report_{stamp}.json"
    out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"\nSaved report: {out_path}")


if __name__ == "__main__":
    main()
