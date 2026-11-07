"""Spray job helpers (spec AS-5 to AS-9). Plain Python, no ROS, so they can be unit-tested."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .geo import XY


@dataclass(frozen=True)
class Affine2D:
    """map = A · local + t. Built from three points, so it copes with any rotation or scale
    between the field's local frame and the robot's map frame."""

    a: float
    b: float
    c: float
    d: float
    tx: float
    ty: float

    @classmethod
    def from_points(cls, origin: XY, east: XY, north: XY, step_m: float) -> Affine2D:
        """origin, east, north: where local (0,0), (step,0) and (0,step) land in the map frame."""
        a = (east[0] - origin[0]) / step_m
        c = (east[1] - origin[1]) / step_m
        b = (north[0] - origin[0]) / step_m
        d = (north[1] - origin[1]) / step_m
        return cls(a, b, c, d, origin[0], origin[1])

    def apply(self, p: XY) -> XY:
        return (self.a * p[0] + self.b * p[1] + self.tx, self.c * p[0] + self.d * p[1] + self.ty)

    def scale_error(self) -> float:
        """How far the transform is from a pure rotation (0 is perfect)."""
        return max(abs(math.hypot(self.a, self.c) - 1), abs(math.hypot(self.b, self.d) - 1))


def along_track(p: XY, start: XY, end: XY) -> tuple[float, float]:
    """(distance along start→end, signed distance to the left of it) of point p."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy) or 1e-9
    ux, uy = dx / length, dy / length
    rx, ry = p[0] - start[0], p[1] - start[1]
    return rx * ux + ry * uy, ux * ry - uy * rx


def extend_line(start: XY, end: XY, before_m: float, after_m: float) -> list[XY]:
    """A row with a lead-in before its start and a lead-out after its end, for the controller."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy) or 1e-9
    ux, uy = dx / length, dy / length
    pts = [(start[0] - ux * before_m, start[1] - uy * before_m), start, end, (end[0] + ux * after_m, end[1] + uy * after_m)]
    # Densify: Nav2's controller prunes paths by distance and works best with closely spaced points.
    out: list[XY] = [pts[0]]
    for a, b in zip(pts, pts[1:]):
        n = max(1, int(math.dist(a, b) / 0.25))
        for i in range(1, n + 1):
            out.append((a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n))
    return out


def heading(a: XY, b: XY) -> float:
    return math.atan2(b[1] - a[1], b[0] - a[0])


def flow_for_speed(rate_l_per_m2: float, boom_width_m: float, speed_mps: float) -> float:
    """Pump flow in L/min that applies the rate at the robot's actual ground speed."""
    return rate_l_per_m2 * boom_width_m * max(0.0, speed_mps) * 60.0


class Tank:
    """Liquid left, estimated from flow and spraying time (no tank sensor yet, spec AS-9)."""

    def __init__(self, capacity_l: float, reserve_fraction: float):
        self.capacity = capacity_l
        self.reserve = capacity_l * reserve_fraction
        self.remaining = capacity_l
        self.used_total = 0.0

    def use(self, flow_l_min: float, dt_s: float) -> None:
        litres = max(0.0, flow_l_min) * dt_s / 60.0
        self.remaining = max(0.0, self.remaining - litres)
        self.used_total += litres

    def needs_refill_for(self, litres: float) -> bool:
        return self.remaining - litres < self.reserve

    def refill(self) -> None:
        self.remaining = self.capacity


MAX_HEADING_ERROR = math.radians(25)


def angle_difference(a: float, b: float) -> float:
    """Smallest absolute difference between two angles, radians."""
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def sprayer_should_be_on(s: float, offset: float, length: float, boom_width: float, heading_error: float = 0.0) -> bool:
    """On only while the robot is driving along the row: between its start and end, within half a
    boom width (plus 25 cm) of it, and pointing along it. Without the last two, the sprayer would
    flick on while the robot turns in beside the next row."""
    return (
        0.0 <= s <= length
        and abs(offset) <= boom_width / 2 + 0.25
        and abs(heading_error) <= MAX_HEADING_ERROR
    )


def format_duration(seconds: float) -> str:
    minutes = max(0, round(seconds / 60))
    return f"{minutes // 60} h {minutes % 60:02d} min" if minutes >= 60 else f"{minutes} min"


def tilt_deg(x: float, y: float, z: float, w: float) -> float:
    """How far the robot leans from level, in degrees, whichever way, from an orientation quaternion.

    It's the angle between the robot's up axis and vertical, so the IMU must be mounted level.
    """
    norm = x * x + y * y + z * z + w * w
    if norm <= 0:
        return 0.0
    cos_tilt = 1.0 - 2.0 * (x * x + y * y) / norm
    return math.degrees(math.acos(max(-1.0, min(1.0, cos_tilt))))


class Episode:
    """A condition, such as a low battery, that counts once it has held for hold_s.

    It ends only when the clear condition holds, which is set a little apart from the start
    condition so a reading near the limit doesn't start a new episode every few seconds. Episodes
    are numbered, so the code acting on them can tell a new one from the one it has handled.
    """

    def __init__(self, hold_s: float):
        self.hold_s = hold_s
        self.since: float | None = None
        self.active = False
        self.number = 0

    def update(self, over: bool, clear: bool, now: float) -> None:
        if self.active:
            if clear:
                self.active = False
                self.since = None
            return
        if not over:
            self.since = None
            return
        if self.since is None:
            self.since = now
        if now - self.since >= self.hold_s:
            self.active = True
            self.number += 1
