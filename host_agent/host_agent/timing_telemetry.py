from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import Any, Deque, Dict, List, Optional


def _percentile(values: List[float], pct: float) -> float:
    if not values:
        return 0.0
    if len(values) == 1:
        return float(values[0])
    safe_pct = min(100.0, max(0.0, float(pct)))
    rank = (safe_pct / 100.0) * (len(values) - 1)
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return float(values[lo])
    w = rank - lo
    return float(values[lo] * (1.0 - w) + values[hi] * w)


def _stats(values: List[float]) -> Dict[str, Any]:
    if not values:
        return {
            "count": 0,
            "mean": 0.0,
            "p95": 0.0,
            "max": 0.0,
        }
    ordered = sorted(float(v) for v in values)
    return {
        "count": len(ordered),
        "mean": float(sum(ordered) / len(ordered)),
        "p95": float(_percentile(ordered, 95.0)),
        "max": float(ordered[-1]),
    }


class TimingTelemetry:
    """In-memory ring buffer for score execution timing diagnostics."""

    def __init__(self, max_events: int = 2000) -> None:
        self._max_events = max(10, int(max_events))
        self._lock = threading.Lock()
        self._events: Deque[Dict[str, Any]] = deque(maxlen=self._max_events)
        self._dropped_events = 0
        self._session_started_monotonic = 0.0
        self._session_label = ""
        self._session_active = False

    def start_session(self, label: str) -> None:
        with self._lock:
            self._events.clear()
            self._dropped_events = 0
            self._session_started_monotonic = time.monotonic()
            self._session_label = str(label or "")
            self._session_active = True

    def finish_session(self) -> None:
        with self._lock:
            self._session_active = False

    def record_dropped(self, count: int = 1) -> None:
        with self._lock:
            self._dropped_events += max(0, int(count))

    def record_event(
        self,
        *,
        event_id: str,
        event_type: str,
        scheduled_time_monotonic: float,
        actual_time_monotonic: float,
        pair_key: str = "",
        detail: str = "",
    ) -> None:
        scheduled = float(scheduled_time_monotonic)
        actual = float(actual_time_monotonic)
        delta_ms = (actual - scheduled) * 1000.0
        row = {
            "event_id": str(event_id),
            "event_type": str(event_type),
            "scheduled_time_monotonic": scheduled,
            "actual_time_monotonic": actual,
            "delta_ms": delta_ms,
            "pair_key": str(pair_key or ""),
            "detail": str(detail or ""),
        }
        with self._lock:
            before = len(self._events)
            self._events.append(row)
            if len(self._events) == before and len(self._events) == self._events.maxlen:
                self._dropped_events += 1

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            rows = list(self._events)
            dropped = int(self._dropped_events)
            started = float(self._session_started_monotonic)
            label = self._session_label
            active = bool(self._session_active)

        deltas = [float(row.get("delta_ms", 0.0)) for row in rows]
        pair_map: Dict[str, List[float]] = {}
        for row in rows:
            pair_key = str(row.get("pair_key", "")).strip()
            if not pair_key:
                continue
            pair_map.setdefault(pair_key, []).append(float(row.get("actual_time_monotonic", 0.0)))

        pair_deltas_ms: List[float] = []
        for values in pair_map.values():
            if len(values) < 2:
                continue
            pair_deltas_ms.append((max(values) - min(values)) * 1000.0)

        if rows:
            execution_duration_ms = (
                (max(float(row["actual_time_monotonic"]) for row in rows)
                 - min(float(row["scheduled_time_monotonic"]) for row in rows))
                * 1000.0
            )
        else:
            execution_duration_ms = 0.0

        return {
            "session_label": label,
            "session_active": active,
            "session_started_monotonic": started,
            "event_count": len(rows),
            "dropped_events": dropped,
            "delta_ms": _stats(deltas),
            "pair_delta_ms": _stats(pair_deltas_ms),
            "execution_duration_ms": execution_duration_ms,
            "recent_events": rows[-50:],
        }
