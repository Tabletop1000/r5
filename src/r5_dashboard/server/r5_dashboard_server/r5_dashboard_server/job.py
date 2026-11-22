"""Spray job helpers (spec AS-5 to AS-9). Plain Python, no ROS, so they can be unit-tested."""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

from .geo import XY, point_in_ring, segments_intersect

# How far outside and inside the field the robot lines up before and after driving through the
# gate. More than Nav2's xy_goal_tolerance (1.5 m), so it is square to the gate when it goes
# through.
GATE_APPROACH_M = 3.0
# How far outside the boundary's corners the robot keeps when driving around the field.
ROUTE_MARGIN_M = 2.0


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


def gate_waypoints(post_a: XY, post_b: XY, boundary: list[XY], distance_m: float = GATE_APPROACH_M) -> tuple[XY, XY]:
    """(outside, inside): points distance_m either side of the middle of the gate, square to it.
    The robot drives a straight line between them to go through the gate."""
    mx, my = (post_a[0] + post_b[0]) / 2, (post_a[1] + post_b[1]) / 2
    dx, dy = post_b[0] - post_a[0], post_b[1] - post_a[1]
    length = math.hypot(dx, dy) or 1e-9
    nx, ny = -dy / length, dx / length  # square to the gate
    if not point_in_ring((mx + nx * 0.1, my + ny * 0.1), boundary):
        nx, ny = -nx, -ny  # point (nx, ny) into the field
    return (mx - nx * distance_m, my - ny * distance_m), (mx + nx * distance_m, my + ny * distance_m)


def crosses_field(p: XY, q: XY, ring: list[XY]) -> bool:
    """True if the straight line p→q touches the boundary or runs inside the field."""
    n = len(ring)
    if any(segments_intersect(p, q, ring[i], ring[(i + 1) % n]) for i in range(n)):
        return True
    return point_in_ring(((p[0] + q[0]) / 2, (p[1] + q[1]) / 2), ring)


def _corner_waypoints(ring: list[XY], margin_m: float) -> list[XY]:
    """A point margin_m outside each corner of the boundary, along the corner's bisector."""
    n = len(ring)
    area2 = sum(ring[i][0] * ring[(i + 1) % n][1] - ring[(i + 1) % n][0] * ring[i][1] for i in range(n))
    sign = 1.0 if area2 > 0 else -1.0  # anticlockwise rings have their outside on the right

    def outward(a: XY, b: XY) -> XY:
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy) or 1e-9
        return sign * dy / length, -sign * dx / length

    out = []
    for i in range(n):
        n1 = outward(ring[i - 1], ring[i])
        n2 = outward(ring[i], ring[(i + 1) % n])
        bx, by = n1[0] + n2[0], n1[1] + n2[1]
        length = math.hypot(bx, by)
        if length < 1e-9:
            continue  # the boundary doubles back on itself here
        bx, by = bx / length, by / length
        # Further out at sharp corners, so the edges either side stay margin_m away. Capped.
        reach = margin_m / max(n1[0] * bx + n1[1] * by, 0.35)
        p = (ring[i][0] + bx * reach, ring[i][1] + by * reach)
        if not point_in_ring(p, ring):
            out.append(p)
    return out


def route_around(start: XY, goal: XY, ring: list[XY], margin_m: float = ROUTE_MARGIN_M) -> list[XY]:
    """The shortest way from start to goal that stays outside the field, through points just outside
    its corners. Just [start, goal] if the straight line doesn't cross the field, or if there's no
    way around (start or goal inside it, for example)."""
    if not crosses_field(start, goal, ring):
        return [start, goal]
    nodes = [start, goal, *_corner_waypoints(ring, margin_m)]
    best = {0: 0.0}
    previous: dict[int, int] = {}
    queue = [(0.0, 0)]
    done = set()
    while queue:
        dist, i = heapq.heappop(queue)
        if i in done:
            continue
        done.add(i)
        if i == 1:
            path = [1]
            while path[-1] != 0:
                path.append(previous[path[-1]])
            return [nodes[k] for k in reversed(path)]
        for j in range(len(nodes)):
            if j in done or crosses_field(nodes[i], nodes[j], ring):
                continue
            d = dist + math.dist(nodes[i], nodes[j])
            if d < best.get(j, math.inf):
                best[j] = d
                previous[j] = i
                heapq.heappush(queue, (d, j))
    return [start, goal]


def densify(points: list[XY], spacing_m: float = 0.25) -> list[XY]:
    """The same polyline with points at most spacing_m apart, for Nav2's controller."""
    out: list[XY] = points[:1]
    for a, b in zip(points, points[1:]):
        n = max(1, math.ceil(math.dist(a, b) / spacing_m))
        out.extend((a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(1, n + 1))
    return out


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
