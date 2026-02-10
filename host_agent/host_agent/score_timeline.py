from __future__ import annotations

import re
from typing import Any, Dict, List, Optional, Tuple


SCORE_BEATS = 16
DEFAULT_BEEP_DURATION_MS = 100
DEFAULT_VOLUME = 60
DEFAULT_SPEED = 0.5

NOTE_OFFSETS = {
    "C": 0,
    "D": 2,
    "E": 4,
    "F": 5,
    "G": 7,
    "A": 9,
    "B": 11,
}

NOTE_TOKEN_RE = re.compile(r"^([A-Ga-g])([#b]?)(-?\d)$")

MOTOR_REMAP = {
    "F": "F",
    "FORWARD": "F",
    "+": "F",
    "B": "B",
    "BACK": "B",
    "BACKWARD": "B",
    "REV": "B",
    "REVERSE": "B",
    "S": "S",
    "STOP": "S",
    "REST": "S",
    "R": "S",
    "0": "S",
    "-": "S",
}


def _to_int(value: Any, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return int(default)


def _to_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _split_bars(raw: str) -> List[str]:
    parts = [part.strip() for part in str(raw or "").split("|")]
    parts = [part for part in parts if part]
    if not parts:
        return [""]
    return parts[:4]


def _tokenize_motor_bar(bar: str) -> List[str]:
    raw = bar.strip()
    if not raw:
        return []
    if any(ch.isspace() for ch in raw) or "," in raw:
        return [item for item in re.split(r"[,\s]+", raw) if item]
    compact = raw.replace("-", "")
    if compact and all(ch.upper() in {"F", "B", "S"} for ch in compact):
        return [ch for ch in compact]
    if "-" in raw:
        return [item for item in raw.split("-") if item]
    return [raw]


def _tokenize_melody_bar(bar: str) -> List[str]:
    raw = bar.strip()
    if not raw:
        return []
    if any(ch.isspace() for ch in raw) or "," in raw:
        return [item for item in re.split(r"[,\s]+", raw) if item]
    if "-" in raw:
        parts = raw.split("-")
        result: List[str] = []
        for part in parts:
            if part.strip():
                result.append(part.strip())
            else:
                result.append("-")
        return result
    return [raw]


def _parse_note_or_freq(token: str) -> Optional[int]:
    raw = str(token or "").strip()
    if not raw:
        return None
    upper = raw.upper()
    if upper in {"-", "REST", "R"}:
        return None
    try:
        return int(round(float(raw)))
    except ValueError:
        pass

    match = NOTE_TOKEN_RE.match(raw)
    if not match:
        raise ValueError(
            f"unsupported melody token '{token}'; use note names (C4/A#4) or frequency Hz."
        )
    note = match.group(1).upper()
    accidental = match.group(2)
    octave = int(match.group(3))
    if octave < 0 or octave > 8:
        raise ValueError(f"octave out of range in '{token}'")
    semitone = NOTE_OFFSETS[note]
    if accidental == "#":
        semitone += 1
    elif accidental == "b":
        semitone -= 1
    midi = (octave + 1) * 12 + semitone
    freq = int(round(440.0 * (2.0 ** ((midi - 69) / 12.0))))
    return freq


def parse_motor_lane(score: str) -> List[str]:
    raw = str(score or "").strip()
    if not raw:
        return ["S"] * SCORE_BEATS
    tokens: List[str] = []
    for bar in _split_bars(raw):
        tokens.extend(_tokenize_motor_bar(bar))

    lane: List[str] = []
    for idx, token in enumerate(tokens, start=1):
        key = token.strip().upper()
        mapped = MOTOR_REMAP.get(key)
        if mapped is None:
            raise ValueError(f"motor token #{idx}='{token}' invalid; use F/B/S")
        lane.append(mapped)

    if len(lane) < SCORE_BEATS:
        lane.extend(["S"] * (SCORE_BEATS - len(lane)))
    return lane[:SCORE_BEATS]


def parse_melody_lane(score: str) -> List[Optional[int]]:
    raw = str(score or "").strip()
    if not raw:
        return [None] * SCORE_BEATS
    tokens: List[str] = []
    for bar in _split_bars(raw):
        tokens.extend(_tokenize_melody_bar(bar))

    lane: List[Optional[int]] = []
    for idx, token in enumerate(tokens, start=1):
        freq = _parse_note_or_freq(token)
        if freq is None:
            lane.append(None)
            continue
        if freq < 50 or freq > 5000:
            raise ValueError(f"melody token #{idx}='{token}' resolves to {freq}Hz; expected 50..5000")
        lane.append(freq)

    if len(lane) < SCORE_BEATS:
        lane.extend([None] * (SCORE_BEATS - len(lane)))
    return lane[:SCORE_BEATS]


def _normalize_explicit_events(raw_events: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    normalized: List[Dict[str, Any]] = []
    for idx, item in enumerate(raw_events):
        if not isinstance(item, dict):
            raise ValueError(f"events[{idx}] must be an object")
        t_rel_ms = _to_int(item.get("t_rel_ms", 0), 0)
        if t_rel_ms < 0:
            raise ValueError(f"events[{idx}] has negative t_rel_ms")
        if "action" in item and isinstance(item["action"], dict):
            action = dict(item["action"])
        else:
            action = dict(item)
            action.pop("t_rel_ms", None)
            action.pop("event_id", None)
        event_type = str(action.get("type", "")).strip().lower()
        if not event_type:
            raise ValueError(f"events[{idx}] missing action type")
        action["type"] = event_type
        event_id = str(item.get("event_id", f"evt_{idx+1:04d}"))
        normalized.append(
            {
                "event_id": event_id,
                "t_rel_ms": int(t_rel_ms),
                "action": action,
            }
        )
    normalized.sort(key=lambda row: row["t_rel_ms"])
    return normalized


def _build_events_from_score_lanes(payload: Dict[str, Any]) -> List[Dict[str, Any]]:
    port = str(payload.get("port", "A")).strip().upper() or "A"
    bpm = _to_float(payload.get("bpm", 120.0), 120.0)
    if bpm <= 0.0:
        raise ValueError("bpm must be > 0")
    beat_ms = 60000.0 / bpm
    repeats = max(1, _to_int(payload.get("repeats", 1), 1))
    speed = abs(_to_float(payload.get("speed", DEFAULT_SPEED), DEFAULT_SPEED))
    if speed <= 0.0:
        speed = DEFAULT_SPEED
    speed = min(1.0, speed)
    volume = _to_int(payload.get("volume", DEFAULT_VOLUME), DEFAULT_VOLUME)
    volume = max(0, min(100, volume))
    beep_duration_ms = _to_int(payload.get("beep_duration_ms", DEFAULT_BEEP_DURATION_MS), DEFAULT_BEEP_DURATION_MS)
    beep_duration_ms = max(10, min(5000, beep_duration_ms))

    motor_lane = parse_motor_lane(str(payload.get("motor", "")))
    melody_lane = parse_melody_lane(str(payload.get("melody", "")))

    rows: List[Dict[str, Any]] = []
    event_idx = 0
    t_cursor_ms = 0.0
    for _ in range(repeats):
        for beat in range(SCORE_BEATS):
            t_rel_ms = int(round(t_cursor_ms))
            token = motor_lane[beat]
            if token == "F":
                event_idx += 1
                rows.append(
                    {
                        "event_id": f"evt_{event_idx:04d}",
                        "t_rel_ms": t_rel_ms,
                        "action": {"type": "run", "speed": float(speed), "port": port},
                    }
                )
                event_idx += 1
                rows.append(
                    {
                        "event_id": f"evt_{event_idx:04d}",
                        "t_rel_ms": int(round(t_cursor_ms + beat_ms)),
                        "action": {"type": "stop", "port": port, "stop_action": "coast"},
                    }
                )
            elif token == "B":
                event_idx += 1
                rows.append(
                    {
                        "event_id": f"evt_{event_idx:04d}",
                        "t_rel_ms": t_rel_ms,
                        "action": {"type": "run", "speed": -float(speed), "port": port},
                    }
                )
                event_idx += 1
                rows.append(
                    {
                        "event_id": f"evt_{event_idx:04d}",
                        "t_rel_ms": int(round(t_cursor_ms + beat_ms)),
                        "action": {"type": "stop", "port": port, "stop_action": "coast"},
                    }
                )
            else:
                event_idx += 1
                rows.append(
                    {
                        "event_id": f"evt_{event_idx:04d}",
                        "t_rel_ms": t_rel_ms,
                        "action": {"type": "stop", "port": port, "stop_action": "coast"},
                    }
                )

            freq = melody_lane[beat]
            if freq is not None:
                event_idx += 1
                rows.append(
                    {
                        "event_id": f"evt_{event_idx:04d}",
                        "t_rel_ms": t_rel_ms,
                        "action": {
                            "type": "beep",
                            "freq_hz": int(freq),
                            "duration_ms": int(beep_duration_ms),
                            "volume": int(volume),
                        },
                    }
                )
            t_cursor_ms += beat_ms

    rows.sort(key=lambda row: row["t_rel_ms"])
    return rows


def normalize_score_payload(payload: Dict[str, Any]) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    raw_events = payload.get("events")
    if isinstance(raw_events, list) and raw_events:
        events = _normalize_explicit_events(raw_events)
        metadata = {
            "source": "events",
            "port": str(payload.get("port", "A")).strip().upper() or "A",
            "event_count": len(events),
            "start_mode": str(payload.get("start_mode", "immediate")).strip().lower() or "immediate",
            "start_time_ms": _to_int(payload.get("start_time_ms", 0), 0),
        }
        return events, metadata

    events = _build_events_from_score_lanes(payload)
    metadata = {
        "source": "lanes",
        "port": str(payload.get("port", "A")).strip().upper() or "A",
        "event_count": len(events),
        "bpm": _to_float(payload.get("bpm", 120.0), 120.0),
        "repeats": max(1, _to_int(payload.get("repeats", 1), 1)),
        "start_mode": str(payload.get("start_mode", "immediate")).strip().lower() or "immediate",
        "start_time_ms": _to_int(payload.get("start_time_ms", 0), 0),
    }
    return events, metadata
