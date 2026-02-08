from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Tuple

import yaml


VALID_PORTS = {"A", "B", "C", "D", "E", "F"}
VALID_STOP_ACTIONS = {"coast", "brake", "hold"}
VALID_DIRECTIONS = {"cw", "ccw"}

TOP_LEVEL_KEYS = {"name", "version", "defaults", "steps"}
DEFAULT_KEYS = {"motor_port", "stop_action"}

STEP_SPECS: Dict[str, Tuple[set[str], set[str]]] = {
    "run": (
        {"type", "velocity"},
        {"port", "direction", "comment"},
    ),
    "stop": (
        {"type"},
        {"port", "stop_action", "comment"},
    ),
    "sleep": (
        {"type", "duration_sec"},
        {"comment"},
    ),
    "run_for_time": (
        {"type", "velocity", "duration_sec"},
        {"port", "direction", "stop_action", "comment"},
    ),
    "run_for_degrees": (
        {"type", "velocity", "degrees"},
        {"port", "direction", "stop_action", "comment"},
    ),
    "run_to_absolute_position": (
        {"type", "velocity", "position_degrees"},
        {"port", "direction", "stop_action", "comment"},
    ),
    "run_to_relative_position": (
        {"type", "velocity", "degrees"},
        {"port", "direction", "stop_action", "comment"},
    ),
    "reset_relative_position": (
        {"type"},
        {"port", "comment"},
    ),
    "set_duty_cycle": (
        {"type", "velocity"},
        {"port", "direction", "comment"},
    ),
}


class PatternValidationError(ValueError):
    """Raised when a pattern YAML file violates schema rules."""


def _error(message: str, step_index: int | None = None) -> PatternValidationError:
    if step_index is None:
        return PatternValidationError(message)
    return PatternValidationError(f"step[{step_index}]: {message}")


def _validate_port(raw: Any, *, default: str, step_index: int | None = None) -> str:
    if raw is None:
        return default
    port = str(raw).strip().upper()
    if port not in VALID_PORTS:
        raise _error(f"invalid port '{raw}', expected one of {sorted(VALID_PORTS)}", step_index)
    return port


def _validate_stop_action(raw: Any, *, default: str, step_index: int | None = None) -> str:
    if raw is None:
        return default
    action = str(raw).strip().lower()
    if action not in VALID_STOP_ACTIONS:
        raise _error(
            f"invalid stop_action '{raw}', expected one of {sorted(VALID_STOP_ACTIONS)}",
            step_index,
        )
    return action


def _validate_direction(raw: Any, *, step_index: int | None = None) -> str | None:
    if raw is None:
        return None
    direction = str(raw).strip().lower()
    if direction not in VALID_DIRECTIONS:
        raise _error(
            f"invalid direction '{raw}', expected one of {sorted(VALID_DIRECTIONS)}",
            step_index,
        )
    return direction


def _validate_float(
    value: Any, *, name: str, minimum: float | None = None, maximum: float | None = None, step_index: int
) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise _error(f"'{name}' must be numeric", step_index) from None
    if minimum is not None and number < minimum:
        raise _error(f"'{name}' must be >= {minimum}", step_index)
    if maximum is not None and number > maximum:
        raise _error(f"'{name}' must be <= {maximum}", step_index)
    return number


def _validate_int(value: Any, *, name: str, step_index: int) -> int:
    try:
        number = int(value)
    except (TypeError, ValueError):
        raise _error(f"'{name}' must be an integer", step_index) from None
    return number


def _validate_velocity(raw: Any, *, step_index: int) -> float:
    return _validate_float(raw, name="velocity", minimum=-1.0, maximum=1.0, step_index=step_index)


def _apply_direction(velocity: float, direction: str | None) -> float:
    if direction == "cw":
        return abs(velocity)
    if direction == "ccw":
        return -abs(velocity)
    return velocity


def _normalize_step(
    raw_step: Dict[str, Any],
    *,
    step_index: int,
    default_port: str,
    default_stop_action: str,
) -> Dict[str, Any]:
    if not isinstance(raw_step, dict):
        raise _error("step must be a mapping/object", step_index)

    raw_type = raw_step.get("type")
    step_type = str(raw_type).strip() if raw_type is not None else ""
    if not step_type:
        raise _error("missing required key 'type'", step_index)
    if step_type not in STEP_SPECS:
        raise _error(
            f"unsupported type '{step_type}', expected one of {sorted(STEP_SPECS)}",
            step_index,
        )

    required, optional = STEP_SPECS[step_type]
    allowed = required | optional

    unknown_keys = sorted(set(raw_step.keys()) - allowed)
    if unknown_keys:
        raise _error(
            f"unknown keys {unknown_keys}; allowed keys for '{step_type}': {sorted(allowed)}",
            step_index,
        )

    missing = sorted(key for key in required if key not in raw_step)
    if missing:
        raise _error(f"missing required keys {missing}", step_index)

    normalized: Dict[str, Any] = {"type": step_type}

    if "comment" in raw_step:
        normalized["comment"] = str(raw_step["comment"])

    if step_type in {
        "run",
        "run_for_time",
        "run_for_degrees",
        "run_to_absolute_position",
        "run_to_relative_position",
        "set_duty_cycle",
    }:
        velocity = _validate_velocity(raw_step.get("velocity"), step_index=step_index)
        direction = _validate_direction(raw_step.get("direction"), step_index=step_index)
        velocity = _apply_direction(velocity, direction)
        normalized["velocity"] = velocity
        if direction is not None:
            normalized["direction"] = direction
        normalized["port"] = _validate_port(
            raw_step.get("port"), default=default_port, step_index=step_index
        )

    if step_type in {"stop", "run_for_time", "run_for_degrees", "run_to_absolute_position", "run_to_relative_position"}:
        normalized["stop_action"] = _validate_stop_action(
            raw_step.get("stop_action"),
            default=default_stop_action,
            step_index=step_index,
        )

    if step_type == "stop":
        normalized["port"] = _validate_port(
            raw_step.get("port"), default=default_port, step_index=step_index
        )

    if step_type == "sleep":
        normalized["duration_sec"] = _validate_float(
            raw_step.get("duration_sec"),
            name="duration_sec",
            minimum=0.0,
            step_index=step_index,
        )

    if step_type == "run_for_time":
        normalized["duration_sec"] = _validate_float(
            raw_step.get("duration_sec"),
            name="duration_sec",
            minimum=0.0,
            step_index=step_index,
        )

    if step_type in {"run_for_degrees", "run_to_relative_position"}:
        degrees = _validate_int(raw_step.get("degrees"), name="degrees", step_index=step_index)
        if degrees == 0:
            raise _error("'degrees' cannot be 0", step_index)
        normalized["degrees"] = degrees

    if step_type == "run_to_absolute_position":
        normalized["position_degrees"] = _validate_int(
            raw_step.get("position_degrees"),
            name="position_degrees",
            step_index=step_index,
        )

    if step_type == "reset_relative_position":
        normalized["port"] = _validate_port(
            raw_step.get("port"), default=default_port, step_index=step_index
        )

    return normalized


def load_and_validate_pattern(path: str) -> Dict[str, Any]:
    pattern_path = Path(path).expanduser()
    if not pattern_path.exists():
        raise PatternValidationError(f"pattern file not found: {pattern_path}")
    if not pattern_path.is_file():
        raise PatternValidationError(f"pattern path is not a file: {pattern_path}")

    try:
        raw = yaml.safe_load(pattern_path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise PatternValidationError(f"invalid YAML: {exc}") from exc

    if not isinstance(raw, dict):
        raise PatternValidationError("pattern root must be a mapping/object")

    unknown_top = sorted(set(raw.keys()) - TOP_LEVEL_KEYS)
    if unknown_top:
        raise PatternValidationError(
            f"unknown top-level keys {unknown_top}; expected keys: {sorted(TOP_LEVEL_KEYS)}"
        )

    name = str(raw.get("name", "")).strip()
    if not name:
        raise PatternValidationError("missing required top-level key 'name'")

    try:
        version = int(raw.get("version"))
    except (TypeError, ValueError):
        raise PatternValidationError("missing or invalid top-level key 'version' (expected int)") from None

    defaults_raw = raw.get("defaults", {})
    if defaults_raw is None:
        defaults_raw = {}
    if not isinstance(defaults_raw, dict):
        raise PatternValidationError("'defaults' must be a mapping/object")

    unknown_defaults = sorted(set(defaults_raw.keys()) - DEFAULT_KEYS)
    if unknown_defaults:
        raise PatternValidationError(
            f"unknown defaults keys {unknown_defaults}; allowed keys: {sorted(DEFAULT_KEYS)}"
        )

    default_port = _validate_port(defaults_raw.get("motor_port"), default="A")
    default_stop_action = _validate_stop_action(defaults_raw.get("stop_action"), default="coast")

    steps_raw = raw.get("steps")
    if not isinstance(steps_raw, list):
        raise PatternValidationError("missing or invalid top-level key 'steps' (expected list)")
    if not steps_raw:
        raise PatternValidationError("'steps' must contain at least one step")

    normalized_steps: List[Dict[str, Any]] = []
    for idx, step in enumerate(steps_raw):
        normalized_steps.append(
            _normalize_step(
                step,
                step_index=idx,
                default_port=default_port,
                default_stop_action=default_stop_action,
            )
        )

    return {
        "name": name,
        "version": version,
        "defaults": {
            "motor_port": default_port,
            "stop_action": default_stop_action,
        },
        "steps": normalized_steps,
        "path": str(pattern_path),
    }
