#!/usr/bin/env python3
import argparse
import datetime as dt
from pathlib import Path
from typing import Any, Dict, List


STEP_TYPES = [
    "run",
    "stop",
    "sleep",
    "run_for_time",
    "run_for_degrees",
    "run_to_absolute_position",
    "run_to_relative_position",
    "reset_relative_position",
    "set_duty_cycle",
]


def _quote(text: str) -> str:
    escaped = str(text).replace('\\', '\\\\').replace('"', '\\"')
    return f'"{escaped}"'


def _fmt_scalar(value: Any) -> str:
    if isinstance(value, str):
        return _quote(value)
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        if value.is_integer():
            return f"{value:.1f}"
        return f"{value:.6f}".rstrip("0").rstrip(".")
    return _quote(str(value))


def dump_pattern_yaml(pattern: Dict[str, Any]) -> str:
    lines: List[str] = []
    lines.append(f"name: {_fmt_scalar(pattern['name'])}")
    lines.append(f"version: {int(pattern.get('version', 1))}")
    lines.append("defaults:")
    defaults = pattern.get("defaults", {})
    lines.append(f"  motor_port: {_fmt_scalar(defaults.get('motor_port', 'A'))}")
    lines.append(f"  stop_action: {_fmt_scalar(defaults.get('stop_action', 'coast'))}")
    lines.append("steps:")

    ordered_keys = [
        "type",
        "port",
        "velocity",
        "duration_sec",
        "degrees",
        "position_degrees",
        "direction",
        "stop_action",
        "comment",
    ]

    for step in pattern.get("steps", []):
        lines.append(f"  - type: {_fmt_scalar(step['type'])}")
        for key in ordered_keys[1:]:
            if key in step:
                lines.append(f"    {key}: {_fmt_scalar(step[key])}")

    return "\n".join(lines) + "\n"


def _default_name(prefix: str) -> str:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{prefix}_{stamp}"


def _build_pattern(name: str, port: str, stop_action: str, steps: List[Dict[str, Any]]) -> Dict[str, Any]:
    return {
        "name": name,
        "version": 1,
        "defaults": {
            "motor_port": port,
            "stop_action": stop_action,
        },
        "steps": steps,
    }


def preset_pulse(
    *,
    name: str,
    port: str,
    stop_action: str,
    speed: float,
    duration: float,
    repeats: int,
    off_time: float,
) -> Dict[str, Any]:
    steps: List[Dict[str, Any]] = []
    count = max(1, int(repeats))
    for idx in range(count):
        steps.append({"type": "run_for_time", "velocity": speed, "duration_sec": duration})
        if idx < count - 1 and off_time > 0.0:
            steps.append({"type": "sleep", "duration_sec": off_time})
    return _build_pattern(name=name, port=port, stop_action=stop_action, steps=steps)


def preset_metronome(
    *,
    name: str,
    port: str,
    stop_action: str,
    speed: float,
    bpm: float,
    repeats: int,
    duty_sec: float,
) -> Dict[str, Any]:
    safe_bpm = max(1.0, float(bpm))
    beat = 60.0 / safe_bpm
    on_time = max(0.02, min(float(duty_sec), beat))
    off_time = max(0.0, beat - on_time)
    steps: List[Dict[str, Any]] = []
    for idx in range(max(1, int(repeats))):
        steps.append({"type": "run_for_time", "velocity": speed, "duration_sec": on_time})
        if idx < int(repeats) - 1 and off_time > 0.0:
            steps.append({"type": "sleep", "duration_sec": off_time})
    return _build_pattern(name=name, port=port, stop_action=stop_action, steps=steps)


def preset_sweep(
    *,
    name: str,
    port: str,
    stop_action: str,
    amplitude: float,
    duration: float,
    steps_count: int,
) -> Dict[str, Any]:
    count = max(2, int(steps_count))
    per_step = max(0.05, float(duration) / count)
    amp = abs(float(amplitude))
    steps: List[Dict[str, Any]] = []
    for idx in range(count):
        ratio = idx / (count - 1)
        velocity = -amp + (2.0 * amp * ratio)
        steps.append({"type": "run_for_time", "velocity": velocity, "duration_sec": per_step})
    return _build_pattern(name=name, port=port, stop_action=stop_action, steps=steps)


def preset_dance(*, name: str, port: str, stop_action: str, amplitude: float) -> Dict[str, Any]:
    amp = max(0.2, min(1.0, abs(float(amplitude))))
    steps: List[Dict[str, Any]] = [
        {"type": "run_for_time", "velocity": amp, "duration_sec": 0.25},
        {"type": "sleep", "duration_sec": 0.08},
        {"type": "run_for_time", "velocity": -amp, "duration_sec": 0.25},
        {"type": "sleep", "duration_sec": 0.08},
        {"type": "run_for_degrees", "velocity": amp * 0.8, "degrees": 180, "stop_action": "hold"},
        {"type": "sleep", "duration_sec": 0.08},
        {"type": "run_for_degrees", "velocity": -amp * 0.8, "degrees": 180, "stop_action": "hold"},
    ]
    return _build_pattern(name=name, port=port, stop_action=stop_action, steps=steps)


def preset_clock(*, name: str, port: str, stop_action: str) -> Dict[str, Any]:
    steps: List[Dict[str, Any]] = [
        {"type": "reset_relative_position"},
        {"type": "run_to_absolute_position", "velocity": 0.45, "position_degrees": 0, "stop_action": "hold"},
        {"type": "sleep", "duration_sec": 0.2},
        {"type": "run_to_absolute_position", "velocity": 0.45, "position_degrees": 90, "stop_action": "hold"},
        {"type": "sleep", "duration_sec": 0.2},
        {"type": "run_to_absolute_position", "velocity": 0.45, "position_degrees": 180, "stop_action": "hold"},
        {"type": "sleep", "duration_sec": 0.2},
        {"type": "run_to_absolute_position", "velocity": 0.45, "position_degrees": 270, "stop_action": "hold"},
        {"type": "sleep", "duration_sec": 0.2},
        {"type": "run_to_absolute_position", "velocity": 0.45, "position_degrees": 360, "stop_action": "hold"},
    ]
    return _build_pattern(name=name, port=port, stop_action=stop_action, steps=steps)


def preset_bounce(*, name: str, port: str, stop_action: str, degrees: int) -> Dict[str, Any]:
    amount = max(30, abs(int(degrees)))
    steps: List[Dict[str, Any]] = [
        {"type": "reset_relative_position"},
        {"type": "run_to_relative_position", "velocity": 0.45, "degrees": amount, "stop_action": "hold"},
        {"type": "sleep", "duration_sec": 0.15},
        {
            "type": "run_to_relative_position",
            "velocity": 0.45,
            "degrees": -2 * amount,
            "stop_action": "hold",
        },
        {"type": "sleep", "duration_sec": 0.15},
        {"type": "run_to_relative_position", "velocity": 0.45, "degrees": amount, "stop_action": "hold"},
    ]
    return _build_pattern(name=name, port=port, stop_action=stop_action, steps=steps)


def ask(prompt: str, default: str = "") -> str:
    suffix = f" [{default}]" if default else ""
    raw = input(f"{prompt}{suffix}: ").strip()
    return raw if raw else default


def ask_float(prompt: str, default: float) -> float:
    while True:
        raw = ask(prompt, str(default))
        try:
            return float(raw)
        except ValueError:
            print("Please enter a number.")


def ask_int(prompt: str, default: int) -> int:
    while True:
        raw = ask(prompt, str(default))
        try:
            return int(raw)
        except ValueError:
            print("Please enter an integer.")


def ask_choice(prompt: str, options: List[str], default: str) -> str:
    lower = {item.lower(): item for item in options}
    while True:
        raw = ask(prompt + " (" + ", ".join(options) + ")", default).strip().lower()
        if raw in lower:
            return lower[raw]
        print(f"Choose one of: {', '.join(options)}")


def interactive_manual_steps() -> List[Dict[str, Any]]:
    steps: List[Dict[str, Any]] = []
    print("Add steps one by one. Type index or name to choose step type.")
    for idx, step_type in enumerate(STEP_TYPES, start=1):
        print(f"  {idx}. {step_type}")

    while True:
        choice = ask("Step type", "run_for_time")
        if choice.isdigit() and 1 <= int(choice) <= len(STEP_TYPES):
            step_type = STEP_TYPES[int(choice) - 1]
        else:
            step_type = choice.strip().lower()
        if step_type not in STEP_TYPES:
            print("Unknown step type. Try again.")
            continue

        step: Dict[str, Any] = {"type": step_type}
        if step_type in {
            "run",
            "run_for_time",
            "run_for_degrees",
            "run_to_absolute_position",
            "run_to_relative_position",
            "set_duty_cycle",
        }:
            step["velocity"] = ask_float("velocity (-1..1)", 0.4)

        if step_type in {"run_for_time", "sleep"}:
            step["duration_sec"] = max(0.0, ask_float("duration_sec", 0.3))

        if step_type in {"run_for_degrees", "run_to_relative_position"}:
            step["degrees"] = ask_int("degrees", 180)

        if step_type == "run_to_absolute_position":
            step["position_degrees"] = ask_int("position_degrees", 180)

        if step_type in {"run_for_time", "run_for_degrees", "run_to_absolute_position", "run_to_relative_position", "stop"}:
            step["stop_action"] = ask_choice("stop_action", ["coast", "brake", "hold"], "coast")

        override = ask("Override port for this step? (blank keeps default)", "").upper().strip()
        if override:
            step["port"] = override

        comment = ask("Comment (optional)", "")
        if comment:
            step["comment"] = comment

        steps.append(step)

        add_more = ask_choice("Add another step", ["yes", "no"], "yes")
        if add_more == "no":
            break

    return steps


def resolve_output_path(path_arg: str, pattern_name: str) -> Path:
    if path_arg:
        return Path(path_arg).expanduser()
    return Path("patterns/user") / f"{pattern_name}.yaml"


def to_container_pattern_path(path: Path) -> str:
    parts = list(path.resolve().parts)
    if "patterns" in parts:
        index = parts.index("patterns")
        suffix = "/".join(parts[index + 1 :])
        return f"/patterns/{suffix}"
    return str(path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate workshop motor pattern YAML without coding")
    parser.add_argument(
        "--preset",
        choices=["pulse", "metronome", "sweep", "dance", "clock", "bounce"],
        help="Generate a preset pattern (non-interactive)",
    )
    parser.add_argument("--name", default="", help="Pattern name")
    parser.add_argument("--port", default="A", help="Default motor port A-F")
    parser.add_argument("--stop-action", default="coast", choices=["coast", "brake", "hold"])
    parser.add_argument("--speed", type=float, default=0.4)
    parser.add_argument("--duration", type=float, default=0.3)
    parser.add_argument("--repeats", type=int, default=6)
    parser.add_argument("--bpm", type=float, default=90.0)
    parser.add_argument("--amplitude", type=float, default=0.7)
    parser.add_argument("--degrees", type=int, default=120)
    parser.add_argument("--off-time", type=float, default=0.2)
    parser.add_argument("--out", default="", help="Output YAML path")
    return parser.parse_args()


def build_from_preset(args: argparse.Namespace) -> Dict[str, Any]:
    name = args.name.strip() or _default_name(args.preset)
    port = (args.port or "A").strip().upper()
    stop_action = args.stop_action

    if args.preset == "pulse":
        return preset_pulse(
            name=name,
            port=port,
            stop_action=stop_action,
            speed=args.speed,
            duration=args.duration,
            repeats=args.repeats,
            off_time=args.off_time,
        )
    if args.preset == "metronome":
        return preset_metronome(
            name=name,
            port=port,
            stop_action=stop_action,
            speed=args.speed,
            bpm=args.bpm,
            repeats=args.repeats,
            duty_sec=args.duration,
        )
    if args.preset == "sweep":
        return preset_sweep(
            name=name,
            port=port,
            stop_action=stop_action,
            amplitude=args.amplitude,
            duration=max(0.5, args.duration),
            steps_count=max(2, args.repeats),
        )
    if args.preset == "dance":
        return preset_dance(
            name=name,
            port=port,
            stop_action=stop_action,
            amplitude=args.amplitude,
        )
    if args.preset == "clock":
        return preset_clock(name=name, port=port, stop_action=stop_action)
    if args.preset == "bounce":
        return preset_bounce(
            name=name,
            port=port,
            stop_action=stop_action,
            degrees=args.degrees,
        )

    raise ValueError(f"Unsupported preset: {args.preset}")


def build_interactive() -> Dict[str, Any]:
    preset = ask_choice(
        "Choose template",
        ["pulse", "metronome", "sweep", "dance", "clock", "bounce", "manual"],
        "pulse",
    )

    if preset != "manual":
        args = argparse.Namespace(
            preset=preset,
            name=ask("Pattern name", _default_name(preset)),
            port=ask("Default motor port", "A").upper(),
            stop_action=ask_choice("Default stop_action", ["coast", "brake", "hold"], "coast"),
            speed=ask_float("speed (-1..1)", 0.4),
            duration=ask_float("duration_sec", 0.3),
            repeats=ask_int("repeats / steps", 6),
            bpm=ask_float("bpm (metronome only)", 90.0),
            amplitude=ask_float("amplitude", 0.7),
            degrees=ask_int("degrees (bounce only)", 120),
            off_time=ask_float("off_time (pulse only)", 0.2),
        )
        return build_from_preset(args)

    name = ask("Pattern name", _default_name("manual"))
    port = ask("Default motor port", "A").upper()
    stop_action = ask_choice("Default stop_action", ["coast", "brake", "hold"], "coast")
    steps = interactive_manual_steps()
    return _build_pattern(name=name, port=port, stop_action=stop_action, steps=steps)


def main() -> None:
    args = parse_args()
    pattern = build_from_preset(args) if args.preset else build_interactive()

    output = resolve_output_path(args.out, pattern_name=pattern["name"])
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(dump_pattern_yaml(pattern), encoding="utf-8")

    print(f"Wrote pattern: {output}")
    container_path = to_container_pattern_path(output)
    print("Play command:")
    print(
        "ros2 launch spike_workshop_instrument instrument.launch.py "
        f"mode:=pattern pattern_file:={container_path} queue_policy:=fifo"
    )


if __name__ == "__main__":
    main()
