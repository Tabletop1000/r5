"""Motor test maths (spec RB-9, RB-10). Plain Python so it can be unit-tested."""

from __future__ import annotations

from dataclasses import dataclass

LEFT, RIGHT, BOTH = 0, 1, 2


@dataclass(frozen=True)
class MotorTestLimits:
    wheel_separation_m: float
    wheel_speed_mps: float
    duration_s: float


def wheel_speeds(side: int, forward: bool, speed_mps: float) -> tuple[float, float]:
    """Ground speed of the left and right wheels for a test of one side, or both."""
    v = speed_mps if forward else -speed_mps
    if side == LEFT:
        return v, 0.0
    if side == RIGHT:
        return 0.0, v
    if side == BOTH:
        return v, v
    raise ValueError(f"Unknown side {side}")


def twist_for(left_mps: float, right_mps: float, wheel_separation_m: float) -> tuple[float, float]:
    """Linear (m/s) and angular (rad/s) velocity that make a differential drive give these wheel speeds."""
    linear = (left_mps + right_mps) / 2.0
    angular = (right_mps - left_mps) / wheel_separation_m
    return linear, angular


def side_name(side: int) -> str:
    return {LEFT: "Left", RIGHT: "Right", BOTH: "Both"}[side]


def evaluate(
    side: int, commanded: tuple[float, float], measured: tuple[float, float] | None
) -> tuple[bool, str]:
    """Pass if each tested side turned the right way at no less than half the commanded speed."""
    if measured is None:
        return False, "No wheel speed readings arrived during the test (check /joint_states)."
    problems = []
    for name, cmd, meas in (("Left", commanded[0], measured[0]), ("Right", commanded[1], measured[1])):
        if cmd == 0.0:
            continue
        if meas * cmd <= 0.0:
            problems.append(f"{name} wheels didn't turn the commanded way ({meas:+.3f} m/s).")
        elif abs(meas) < 0.5 * abs(cmd):
            problems.append(
                f"{name} wheels reached only {abs(meas):.3f} of {abs(cmd):.3f} m/s."
            )
    if problems:
        return False, " ".join(problems)
    what = "All wheels" if side == BOTH else f"{side_name(side)}-side wheels"
    return True, f"{what} turned as commanded."
