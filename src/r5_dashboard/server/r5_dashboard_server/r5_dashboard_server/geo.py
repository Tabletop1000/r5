"""Small geometry helpers for fields (spec FM-4, FM-6). Plain Python, no ROS.

Fields are a few hundred metres across, so positions are projected onto a flat local grid in metres
(equirectangular, centred on the shape). The error from this is far below GPS accuracy at that size.
web/src/fields/geo.ts mirrors these rules so the dashboard can warn before saving.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

EARTH_RADIUS_M = 6_371_008.8

LatLon = tuple[float, float]  # (latitude, longitude) in degrees
XY = tuple[float, float]  # metres east, north


class LocalProjection:
    """Flat x/y in metres around an origin. Accurate to well under a centimetre over a few km."""

    def __init__(self, origin: LatLon):
        self.lat0, self.lon0 = origin
        self.kx = math.radians(1) * EARTH_RADIUS_M * math.cos(math.radians(self.lat0))
        self.ky = math.radians(1) * EARTH_RADIUS_M

    @classmethod
    def around(cls, points: Sequence[LatLon]) -> LocalProjection:
        lat = sum(p[0] for p in points) / len(points)
        lon = sum(p[1] for p in points) / len(points)
        return cls((lat, lon))

    def to_xy(self, p: LatLon) -> XY:
        return ((p[1] - self.lon0) * self.kx, (p[0] - self.lat0) * self.ky)

    def to_latlon(self, xy: XY) -> LatLon:
        return (self.lat0 + xy[1] / self.ky, self.lon0 + xy[0] / self.kx)


def ring_area_m2(ring_xy: Sequence[XY]) -> float:
    """Unsigned area of a closed ring (first point not repeated)."""
    n = len(ring_xy)
    s = 0.0
    for i in range(n):
        x1, y1 = ring_xy[i]
        x2, y2 = ring_xy[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return abs(s) / 2.0


def path_length_m(points_xy: Sequence[XY]) -> float:
    return sum(math.dist(points_xy[i], points_xy[i + 1]) for i in range(len(points_xy) - 1))


def _orient(a: XY, b: XY, c: XY) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a: XY, b: XY, p: XY) -> bool:
    return min(a[0], b[0]) <= p[0] <= max(a[0], b[0]) and min(a[1], b[1]) <= p[1] <= max(a[1], b[1])


def segments_intersect(p1: XY, p2: XY, q1: XY, q2: XY) -> bool:
    d1, d2 = _orient(q1, q2, p1), _orient(q1, q2, p2)
    d3, d4 = _orient(p1, p2, q1), _orient(p1, p2, q2)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)) and d1 and d2 and d3 and d4:
        return True
    return (
        (d1 == 0 and _on_segment(q1, q2, p1))
        or (d2 == 0 and _on_segment(q1, q2, p2))
        or (d3 == 0 and _on_segment(p1, p2, q1))
        or (d4 == 0 and _on_segment(p1, p2, q2))
    )


def _edges(ring: Sequence[XY]):
    n = len(ring)
    return [(ring[i], ring[(i + 1) % n]) for i in range(n)]


def ring_self_intersects(ring: Sequence[XY]) -> bool:
    """True if any two non-neighbouring edges of the closed ring touch or cross."""
    edges = _edges(ring)
    n = len(edges)
    for i in range(n):
        for j in range(i + 1, n):
            if j == i + 1 or (i == 0 and j == n - 1):
                continue  # neighbours share a corner
            if segments_intersect(*edges[i], *edges[j]):
                return True
    return False


def rings_cross(a: Sequence[XY], b: Sequence[XY]) -> bool:
    return any(segments_intersect(*e, *f) for e in _edges(a) for f in _edges(b))


def point_in_ring(p: XY, ring: Sequence[XY]) -> bool:
    inside = False
    n = len(ring)
    for i in range(n):
        (x1, y1), (x2, y2) = ring[i], ring[(i + 1) % n]
        if (y1 > p[1]) != (y2 > p[1]):
            x = x1 + (p[1] - y1) * (x2 - x1) / (y2 - y1)
            if p[0] < x:
                inside = not inside
    return inside


def ring_inside(inner: Sequence[XY], outer: Sequence[XY]) -> bool:
    return all(point_in_ring(p, outer) for p in inner) and not rings_cross(inner, outer)


def simplify(points: Sequence[XY], tolerance: float) -> list[XY]:
    """Douglas-Peucker: drop points while staying within tolerance of the original line."""
    if len(points) < 3:
        return list(points)

    def dist_to_segment(p: XY, a: XY, b: XY) -> float:
        dx, dy = b[0] - a[0], b[1] - a[1]
        if dx == dy == 0:
            return math.dist(p, a)
        t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)))
        return math.dist(p, (a[0] + t * dx, a[1] + t * dy))

    keep = [False] * len(points)
    keep[0] = keep[-1] = True
    stack = [(0, len(points) - 1)]
    while stack:
        start, end = stack.pop()
        worst, index = 0.0, -1
        for i in range(start + 1, end):
            d = dist_to_segment(points[i], points[start], points[end])
            if d > worst:
                worst, index = d, i
        if index >= 0 and worst > tolerance:
            keep[index] = True
            stack += [(start, index), (index, end)]
    return [p for p, k in zip(points, keep) if k]


def close_track(track: Sequence[XY], tolerance: float) -> list[XY]:
    """Turn a driven track into a simplified ring (first point not repeated).

    The track is closed by joining its end to its start, then simplified. Repeated end points are
    dropped.
    """
    pts = list(track)
    if len(pts) >= 2 and math.dist(pts[0], pts[-1]) < 1e-6:
        pts.pop()
    if len(pts) < 3:
        return pts
    ring = simplify(pts + [pts[0]], tolerance)[:-1]
    return ring
