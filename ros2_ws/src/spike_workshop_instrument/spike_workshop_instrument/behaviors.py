import random
from typing import Dict, List

from spike_workshop_instrument.sequence_loader import load_sequence_file


Step = Dict[str, float]


def build_steps(
    *,
    mode: str,
    speed: float,
    duration: float,
    repeats: int,
    bpm: float,
    amplitude: float,
    sweep_steps: int,
    seed: int,
    sequence_file: str,
) -> List[Step]:
    normalized = mode.strip().lower()

    if normalized == "pulse":
        return _build_pulse(speed=speed, duration=duration, repeats=repeats)
    if normalized == "metronome":
        return _build_metronome(speed=speed, duration=duration, bpm=bpm)
    if normalized == "sweep":
        return _build_sweep(amplitude=amplitude, duration=duration, sweep_steps=sweep_steps)
    if normalized == "random_wiggle":
        return _build_random_wiggle(
            amplitude=amplitude,
            duration=duration,
            repeats=repeats,
            seed=seed,
        )
    if normalized == "sequence":
        _, steps = load_sequence_file(sequence_file)
        return steps

    raise ValueError(
        f"Unsupported mode '{mode}'. Expected one of: pulse, sweep, metronome, random_wiggle, sequence"
    )


def _build_pulse(*, speed: float, duration: float, repeats: int) -> List[Step]:
    pulse_count = max(1, int(repeats))
    on_duration = max(0.0, float(duration))
    gap_duration = 0.2

    steps: List[Step] = []
    for _ in range(pulse_count):
        steps.append({"speed": float(speed), "duration": on_duration})
        steps.append({"speed": 0.0, "duration": gap_duration})
    return steps


def _build_metronome(*, speed: float, duration: float, bpm: float) -> List[Step]:
    tempo = max(1.0, float(bpm))
    beat_period = 60.0 / tempo

    requested_on = max(0.0, float(duration))
    if requested_on == 0.0:
        on_duration = min(0.1, beat_period * 0.3)
    else:
        on_duration = min(requested_on, beat_period)
    off_duration = max(0.0, beat_period - on_duration)

    # Metronome continuity is implemented by timer logic in instrument_node.
    # Keep this helper to represent one beat cycle.
    steps: List[Step] = [{"speed": float(speed), "duration": on_duration}]
    if off_duration > 0.0:
        steps.append({"speed": 0.0, "duration": off_duration})
    return steps


def _build_sweep(*, amplitude: float, duration: float, sweep_steps: int) -> List[Step]:
    step_count = max(2, int(sweep_steps))
    total_duration = max(0.0, float(duration))
    per_step_duration = total_duration / step_count if total_duration > 0.0 else 0.1
    amp = abs(float(amplitude))

    steps: List[Step] = []
    for idx in range(step_count):
        ratio = idx / (step_count - 1)
        step_speed = -amp + (2.0 * amp * ratio)
        steps.append({"speed": step_speed, "duration": per_step_duration})

    steps.append({"speed": 0.0, "duration": 0.1})
    return steps


def _build_random_wiggle(*, amplitude: float, duration: float, repeats: int, seed: int) -> List[Step]:
    move_count = max(1, int(repeats))
    total_duration = max(0.0, float(duration))
    per_step_duration = total_duration / move_count if total_duration > 0.0 else 0.1
    amp = abs(float(amplitude))

    rng = random.Random(None if int(seed) == 0 else int(seed))
    steps: List[Step] = []

    for _ in range(move_count):
        steps.append({"speed": rng.uniform(-amp, amp), "duration": per_step_duration})

    steps.append({"speed": 0.0, "duration": 0.1})
    return steps
