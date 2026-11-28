"""R5's speed and acceleration limits, from r5's config/limits.yaml, for launch files.

Parameter files write a limit as $(var <name>), for example

    linear.x.max_velocity: $(var max_linear_speed)
    linear.x.min_velocity: -$(var max_linear_speed)

and a launch file passes them through fill():

    from r5_limits import fill
    Node(..., parameters=[fill(path_to_yaml)])

fill() writes a copy with the values filled in and returns its path, because some readers (the
controller spawner, gz_ros2_control) open the file themselves. Other $(...) substitutions are left
for launch to resolve.
"""

import math
import os
import re
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory

NAMES = (
    "wheel_max_velocity",
    "max_linear_speed",
    "max_angular_speed",
    "max_linear_accel",
    "max_angular_accel",
)
_PLACEHOLDER = re.compile(r"\$\(var ([A-Za-z_][A-Za-z0-9_]*)\)")


class LimitsError(RuntimeError):
    pass


def _share(*parts: str) -> str:
    return os.path.join(get_package_share_directory("r5"), *parts)


def wheel_geometry() -> tuple[float, float]:
    """(wheel_radius, wheel_separation) in metres, from diffbot_controllers.yaml."""
    return _wheel_geometry()


def _wheel_geometry() -> tuple[float, float]:
    """(wheel_radius, wheel_separation) from diffbot_controllers.yaml."""
    with open(_share("config", "diffbot_controllers.yaml")) as f:
        params = yaml.safe_load(f)["diffbot_base_controller"]["ros__parameters"]
    return float(params["wheel_radius"]), float(params["wheel_separation"])


def load(path: str | None = None) -> dict[str, float]:
    """The limits, checked. Raises LimitsError with a plain reason if they can't be used."""
    path = path or _share("config", "limits.yaml")
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    limits = {}
    for name in NAMES:
        value = data.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise LimitsError(f"{path}: {name} must be a positive number, not {value!r}.")
        limits[name] = float(value)
    unknown = sorted(set(data) - set(NAMES))
    if unknown:
        raise LimitsError(f"{path}: unknown limits {', '.join(unknown)}.")

    radius, separation = _wheel_geometry()
    wheel_top = limits["wheel_max_velocity"] * radius
    if limits["max_linear_speed"] > wheel_top + 1e-9:
        raise LimitsError(
            f"{path}: max_linear_speed {limits['max_linear_speed']} m/s needs the wheels faster than "
            f"wheel_max_velocity allows ({wheel_top:.2f} m/s with {radius} m wheels)."
        )
    if limits["max_angular_speed"] * separation / 2 > wheel_top + 1e-9:
        raise LimitsError(
            f"{path}: max_angular_speed {limits['max_angular_speed']} rad/s needs the wheels faster "
            f"than wheel_max_velocity allows ({wheel_top / (separation / 2):.2f} rad/s at most)."
        )
    return limits


def fill(path: str, limits: dict[str, float] | None = None) -> str:
    """A copy of the parameter file at path with $(var <limit>) filled in. Returns its path."""
    limits = limits or load()
    with open(path) as f:
        text = f.read()

    def value(match: re.Match) -> str:
        name = match.group(1)
        if name not in limits:
            # Not a limit: leave it for launch's own substitution.
            return match.group(0)
        return repr(limits[name])

    filled = _PLACEHOLDER.sub(value, text)
    out = tempfile.NamedTemporaryFile(
        "w", prefix=f"r5_{os.path.splitext(os.path.basename(path))[0]}_", suffix=".yaml", delete=False
    )
    with out:
        out.write(filled)
    return out.name
