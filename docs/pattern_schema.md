# Pattern Schema (YAML)

Each pattern file uses this shape:

```yaml
name: "my_pattern"
version: 1
defaults:
  motor_port: "A"
  stop_action: "coast"  # coast|brake|hold
steps:
  - type: "run_for_time"
    velocity: 0.5
    duration_sec: 0.4
```

## Supported Step Types

- `run`: set velocity and keep running until another step changes it.
- `stop`: stop motor with optional `stop_action`.
- `sleep`: wait without changing motor state.
- `run_for_time`: run velocity for `duration_sec`, then stop.
- `run_for_degrees`: run velocity for `degrees`, then stop.
- `run_to_absolute_position`: move to absolute encoder position.
- `run_to_relative_position`: move relative encoder amount.
- `reset_relative_position`: reset relative encoder to zero.
- `set_duty_cycle`: best-effort duty cycle command.

## Canonical Units

- `velocity`: float in `[-1.0, 1.0]`.
- `degrees`: integer encoder degrees.
- `position_degrees`: integer absolute encoder position.
- `duration_sec`: float seconds.
- `direction`: optional `cw|ccw` (applies sign to velocity).

## Validation Rules

- Unknown top-level keys are rejected.
- Unknown step keys are rejected.
- Each step must include exactly the required keys for its type.
- Errors include the failing step index (`step[<index>]`) with reason.

## Required Keys Per Step Type

- `run`: `type`, `velocity`
- `stop`: `type`
- `sleep`: `type`, `duration_sec`
- `run_for_time`: `type`, `velocity`, `duration_sec`
- `run_for_degrees`: `type`, `velocity`, `degrees`
- `run_to_absolute_position`: `type`, `velocity`, `position_degrees`
- `run_to_relative_position`: `type`, `velocity`, `degrees`
- `reset_relative_position`: `type`
- `set_duty_cycle`: `type`, `velocity`

## Optional Keys

- `port` (`A`..`F`) step-level override.
- `stop_action` (`coast|brake|hold`) on stop-related steps.
- `direction` (`cw|ccw`) on velocity steps.
- `comment` free-form string.
