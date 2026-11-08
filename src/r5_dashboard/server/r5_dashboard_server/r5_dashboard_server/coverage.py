"""Coverage planning for area spraying, with Fields2Cover (spec 4.2, AS-1 to AS-4, RI-1, RI-2).

Works in a flat local frame in metres around the field (geo.LocalProjection). The job manager turns
the result into the robot's map frame when it runs the job.

Fields2Cover steps: headland (constant width, also around keep-out zones), Boustrophedon
decomposition and swaths. The rest is done here, because Fields2Cover's own angle search and route
planner take several seconds each on a 4 ha field (more on the robot's computer):

- The best row direction is the one across which the field is narrowest (fewest rows), found
  directly from the outline.
- Within each cell, rows go back and forth with a Dubins turn between neighbours.
- Cells (made by keep-outs or a concave outline) are visited nearest first. Each join is a direct
  turn if that stays inside the field and clear of keep-outs, otherwise it follows the headland
  near the field edge. If neither is clear, Fields2Cover's full route planner is used instead.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .fields import Field
from .geo import XY, LocalProjection, point_in_ring, segments_intersect
from .settings import UNITS_METRIC, UNITS_US
from .units import format_flow, format_speed

SWATH, TURN = 0, 1


class PlanError(ValueError):
    pass


@dataclass
class SpraySettings:
    field_id: str
    application_rate_l_per_m2: float
    boom_width_m: float
    speed_kmh: float
    tank_capacity_l: float
    auto_angle: bool = True
    row_angle_deg: float = 0.0  # clockwise from north


@dataclass(frozen=True)
class RobotLimits:
    max_speed_kmh: float = 2.16  # Nav2 desired_linear_vel 0.6 m/s
    pump_max_flow_l_min: float = 10.0
    tank_reserve_fraction: float = 0.05
    turn_radius_m: float = 1.0
    # Nav2's xy_goal_tolerance. Swaths are extended past their ends by this much plus a margin, so
    # the controller doesn't stop short of the end of a row.
    goal_tolerance_m: float = 1.5
    # Transit speed (Settings page), to estimate trips home to refill.
    travel_speed_mps: float = 0.3
    # Turn speed (Settings page), to estimate the time spent turning between rows.
    turn_speed_mps: float = 0.3
    # Row directions tried when choosing the best one, besides the outline's own edge directions.
    angle_step_deg: float = 1.0


@dataclass
class Section:
    kind: int
    points: list[XY]

    @property
    def length(self) -> float:
        return sum(math.dist(self.points[i], self.points[i + 1]) for i in range(len(self.points) - 1))


@dataclass
class CoveragePlan:
    projection: LocalProjection
    sections: list[Section]
    row_angle_deg: float
    swath_count: int
    area_sprayed_m2: float
    spray_length_m: float
    path_length_m: float
    duration_s: float
    liquid_needed_l: float
    refills: int
    flow_l_min: float
    warnings: list[str] = field(default_factory=list)


def flow_l_min(rate_l_per_m2: float, boom_width_m: float, speed_kmh: float) -> float:
    """Spec AS-1: flow (L/min) = rate (L/m²) × boom width (m) × speed (km/h) × 1000 / 60."""
    return rate_l_per_m2 * boom_width_m * speed_kmh * 1000.0 / 60.0


def max_speed_for_pump(rate_l_per_m2: float, boom_width_m: float, pump_max_l_min: float) -> float:
    return pump_max_l_min * 60.0 / (rate_l_per_m2 * boom_width_m * 1000.0)


def check_settings(s: SpraySettings, limits: RobotLimits, units: str = UNITS_METRIC) -> None:
    def positive(value: float, what: str):
        if not (math.isfinite(value) and value > 0):
            raise PlanError(f"Enter a {what} greater than zero.")

    positive(s.application_rate_l_per_m2, "application rate")
    positive(s.boom_width_m, "boom width")
    positive(s.speed_kmh, "travel speed")
    positive(s.tank_capacity_l, "tank capacity")
    if s.application_rate_l_per_m2 > 1.0:
        raise PlanError("The application rate looks too high: at most 1 L/m² (10,000 L/ha).")
    if s.boom_width_m > 10:
        raise PlanError("The boom width can be at most 32.8 ft." if units == UNITS_US else "The boom width can be at most 10 m.")
    if s.speed_kmh > limits.max_speed_kmh + 1e-9:
        raise PlanError(f"The robot's top speed is {format_speed(limits.max_speed_kmh, units)}.")
    if not s.auto_angle and not math.isfinite(s.row_angle_deg):
        raise PlanError("Enter a row direction, or choose Auto.")
    flow = flow_l_min(s.application_rate_l_per_m2, s.boom_width_m, s.speed_kmh)
    if flow > limits.pump_max_flow_l_min + 1e-9:
        top = max_speed_for_pump(s.application_rate_l_per_m2, s.boom_width_m, limits.pump_max_flow_l_min)
        raise PlanError(
            f"That needs {format_flow(flow, units)} but the pump gives at most "
            f"{format_flow(limits.pump_max_flow_l_min, units)}. "
            f"Lower the speed to {format_speed(math.floor(top * 100 + 1e-6) / 100, units)} or less."
        )


def liquid_plan(swath_liquid: list[float], tank_l: float, reserve_fraction: float) -> int:
    """Number of trips home to refill, deciding before each swath as the job manager does."""
    usable = tank_l * (1 - reserve_fraction)
    remaining = tank_l
    refills = 0
    for need in swath_liquid:
        if need > usable:
            raise PlanError("One row needs more liquid than a full tank. Lower the rate or the row length.")
        if remaining - need < tank_l * reserve_fraction:
            refills += 1
            remaining = tank_l
        remaining -= need
    return refills


def _f2c():
    try:
        import fields2cover
    except ImportError as exc:  # pragma: no cover - depends on the system
        raise PlanError("Fields2Cover isn't installed on the robot (ros-jazzy-fields2cover).") from exc
    return fields2cover


def _ring(f2c, points: list[XY]):
    ring = f2c.LinearRing()
    for x, y in list(points) + [points[0]]:
        ring.addPoint(f2c.Point(x, y))
    return ring


def _path_points(f2c, path) -> list[tuple[int, XY, XY]]:
    """(kind, start, end) of each Fields2Cover path state."""
    out = []
    for i in range(path.size()):
        state = path.getState(i)
        kind = SWATH if state.type == f2c.PathSectionType_SWATH else TURN
        p = state.point
        start = (p.getX(), p.getY())
        sign = -1.0 if state.dir == f2c.PathDirection_BACKWARD else 1.0
        end = (start[0] + sign * state.len * math.cos(state.angle), start[1] + sign * state.len * math.sin(state.angle))
        out.append((kind, start, end))
    return out


def _thin(points: list[XY], spacing: float = 0.25) -> list[XY]:
    """Turns are finely discretised; a point every 25 cm is plenty for Nav2."""
    if len(points) <= 2:
        return points
    kept = [points[0]]
    for pt in points[1:-1]:
        if math.dist(pt, kept[-1]) >= spacing:
            kept.append(pt)
    kept.append(points[-1])
    return kept


def _sections_from_states(states) -> list[Section]:
    """Group consecutive path states of the same kind into sections."""
    sections: list[Section] = []
    for kind, start, end in states:
        if sections and sections[-1].kind == kind:
            sections[-1].points.append(end)
        else:
            sections.append(Section(kind, [start, end]))
    for sec in sections:
        if sec.kind == TURN:
            sec.points = _thin(sec.points)
    return [sec for sec in sections if len(sec.points) >= 2 and sec.length > 1e-3]


def _sections_from_path(f2c, path) -> list[Section]:
    return _sections_from_states(_path_points(f2c, path))


def best_row_angle(outline: list[XY], step_deg: float) -> float:
    """Row direction (radians from +x, 0–π) needing the fewest rows: the one across which the
    field is narrowest. The narrowest direction always runs along an edge of the convex outline,
    so every edge direction is tried as well as a regular sweep."""
    candidates = {round(math.atan2(b[1] - a[1], b[0] - a[0]) % math.pi, 9) for a, b in zip(outline, outline[1:] + outline[:1])}
    candidates |= {math.radians(d) for d in _frange(0.0, 180.0, step_deg)}

    def width_across(angle: float) -> float:
        nx, ny = -math.sin(angle), math.cos(angle)
        proj = [x * nx + y * ny for x, y in outline]
        return max(proj) - min(proj)

    return min(sorted(candidates), key=lambda a: (round(width_across(a), 6), a))


def _frange(start: float, stop: float, step: float):
    v = start
    while v < stop:
        yield v
        v += step


class _Obstacles:
    """Where turns and joins may go: inside the field and outside every keep-out zone."""

    def __init__(self, outer: list[XY], zones: list[list[XY]], clearance: float):
        self.outer = outer
        self.zones = [_grow(z, clearance) for z in zones]

    def clear(self, points: list[XY]) -> bool:
        for p in points:
            if not point_in_ring(p, self.outer) or any(point_in_ring(p, z) for z in self.zones):
                return False
        for a, b in zip(points, points[1:]):
            for ring in [self.outer, *self.zones]:
                for c, d in zip(ring, ring[1:] + ring[:1]):
                    if segments_intersect(a, b, c, d):
                        return False
        return True


def _grow(ring: list[XY], d: float) -> list[XY]:
    """Push each corner of a small polygon away from its centre by d (a simple, safe enlargement)."""
    cx = sum(p[0] for p in ring) / len(ring)
    cy = sum(p[1] for p in ring) / len(ring)
    out = []
    for x, y in ring:
        r = math.hypot(x - cx, y - cy) or 1.0
        out.append((x + (x - cx) / r * d, y + (y - cy) / r * d))
    return out


def _along_ring(ring: list[XY], a: XY, b: XY) -> list[XY]:
    """Shorter way round a ring from the point nearest a to the point nearest b."""

    def nearest(p: XY) -> tuple[int, XY]:
        best = (math.inf, 0, ring[0])
        for i, (c, d) in enumerate(zip(ring, ring[1:] + ring[:1])):
            dx, dy = d[0] - c[0], d[1] - c[1]
            t = max(0.0, min(1.0, ((p[0] - c[0]) * dx + (p[1] - c[1]) * dy) / ((dx * dx + dy * dy) or 1e-9)))
            q = (c[0] + t * dx, c[1] + t * dy)
            dist = math.dist(p, q)
            if dist < best[0]:
                best = (dist, i, q)
        return best[1], best[2]

    ia, qa = nearest(a)
    ib, qb = nearest(b)
    n = len(ring)
    forward = [qa] + [ring[(ia + 1 + k) % n] for k in range((ib - ia) % n)] + [qb]
    backward = [qa] + [ring[(ia - k) % n] for k in range((ia - ib) % n)] + [qb]
    length = lambda pts: sum(math.dist(p, q) for p, q in zip(pts, pts[1:]))  # noqa: E731
    return min(forward, backward, key=length)


def _rows_of(f2c, swaths) -> list[tuple[XY, XY]]:
    ordered = f2c.RP_Boustrophedon().genSortedSwaths(swaths)
    rows = []
    for i in range(ordered.size()):
        a, b = ordered.at(i).startPoint(), ordered.at(i).endPoint()
        rows.append(((a.getX(), a.getY()), (b.getX(), b.getY())))
    return rows


def _turn(f2c, robot, dubins, start: XY, start_heading: float, end: XY, end_heading: float) -> list[tuple[int, XY, XY]]:
    path = dubins.createTurn(robot, f2c.Point(*start), start_heading % (2 * math.pi), f2c.Point(*end), end_heading % (2 * math.pi))
    return [(TURN, s, e) for _, s, e in _path_points(f2c, path)]


def _heading(a: XY, b: XY) -> float:
    return math.atan2(b[1] - a[1], b[0] - a[0])


class _NeedsFullPlanner(Exception):
    pass


def _route_cells(f2c, robot, swaths_by_cells, ring: list[XY], obstacles: _Obstacles) -> list[Section]:
    """Visit cells nearest first, back and forth within each (see module notes)."""
    dubins = f2c.PP_DubinsCurves()
    cells = [_rows_of(f2c, swaths_by_cells.at(i)) for i in range(swaths_by_cells.size())]
    cells = [c for c in cells if c]
    states: list[tuple[int, XY, XY]] = []
    pos: XY | None = None
    prev_row: tuple[XY, XY] | None = None
    while cells:
        options = []  # (distance, cell index, rows in driving order)
        for i, rows in enumerate(cells):
            for order in (rows, [(b, a) for a, b in reversed(rows)]):
                options.append((0.0 if pos is None else math.dist(pos, order[0][0]), i, order))
        _, index, rows = min(options, key=lambda o: o[0])
        cells.pop(index)
        for k, (a, b) in enumerate(rows):
            if prev_row is not None:
                turn = _turn(f2c, robot, dubins, prev_row[1], _heading(*prev_row), a, _heading(a, b))
                points = [turn[0][1]] + [e for _, _, e in turn] if turn else [prev_row[1], a]
                if k == 0 and not obstacles.clear(points):
                    # Joining two cells: follow the headland near the field edge instead.
                    points = [prev_row[1], *_along_ring(ring, prev_row[1], a), a]
                    if not obstacles.clear(points):
                        raise _NeedsFullPlanner
                    states += [(TURN, p, q) for p, q in zip(points, points[1:])]
                else:
                    states += turn
            states.append((SWATH, a, b))
            prev_row = (a, b)
            pos = b
    return _sections_from_states(states)


def plan_coverage(
    fld: Field, s: SpraySettings, limits: RobotLimits, home: tuple[float, float] | None = None, units: str = UNITS_METRIC
) -> CoveragePlan:
    """units only changes the units in error messages."""
    check_settings(s, limits, units)
    f2c = _f2c()
    proj = LocalProjection.around(fld.boundary)
    outer = [proj.to_xy(p) for p in fld.boundary]
    cell = f2c.Cell(_ring(f2c, outer))
    for zone in fld.keep_out:
        cell.addRing(_ring(f2c, [proj.to_xy(p) for p in zone]))
    cells = f2c.Cells(cell)

    warnings: list[str] = []
    width = s.boom_width_m
    headland = fld.headland_width_m
    if headland < width / 2 + limits.turn_radius_m:
        warnings.append(
            f"The headland ({headland:.1f} m) is narrow for turning. Turns may run past the field edge; "
            f"{width / 2 + limits.turn_radius_m:.1f} m or more is safer."
        )

    robot = f2c.Robot(width, width)
    robot.setMinTurningRadius(limits.turn_radius_m)
    hg = f2c.HG_Const_gen()
    try:
        inner = hg.generateHeadlands(cells, headland) if headland > 0 else cells
        route_ring = hg.generateHeadlands(cells, headland / 2) if headland > 0 else cells
        if inner.area() < width * width:
            raise PlanError("After taking off the headland there is nothing left to spray.")

        if s.auto_angle:
            angle = best_row_angle(outer, limits.angle_step_deg)
        else:
            angle = math.radians(90.0 - s.row_angle_deg) % math.pi

        decomp = f2c.DECOMP_Boustrophedon()
        decomp.setSplitAngle(angle)
        work = decomp.decompose(inner)
        swaths = f2c.SG_BruteForce().generateSwaths(angle, width, work)
        ring_cells = route_ring.getGeometry(0)
        ring = [(ring_cells.getExteriorRing().getGeometry(i).getX(), ring_cells.getExteriorRing().getGeometry(i).getY())
                for i in range(ring_cells.getExteriorRing().size() - 1)]
        zones = [[proj.to_xy(p) for p in z] for z in fld.keep_out]
        try:
            sections = _route_cells(f2c, robot, swaths, ring, _Obstacles(outer, zones, width / 2))
        except _NeedsFullPlanner:
            route = f2c.RP_RoutePlannerBase().genRoute(route_ring, swaths)
            path = f2c.PP_PathPlanning().planPath(robot, route, f2c.PP_DubinsCurves())
            sections = _sections_from_path(f2c, path)
    except PlanError:
        raise
    except Exception as exc:  # Fields2Cover raises various C++ errors through SWIG
        raise PlanError(f"The coverage planner couldn't plan this field: {exc}") from exc

    swaths_only = [x for x in sections if x.kind == SWATH]
    if not swaths_only:
        raise PlanError("No rows fit in the field at this boom width.")

    spray_len = sum(x.length for x in swaths_only)
    path_len = sum(x.length for x in sections)
    area = spray_len * width
    liquid = area * s.application_rate_l_per_m2
    swath_liquid = [x.length * width * s.application_rate_l_per_m2 for x in swaths_only]
    refills = liquid_plan(swath_liquid, s.tank_capacity_l, limits.tank_reserve_fraction)

    flow = flow_l_min(s.application_rate_l_per_m2, width, s.speed_kmh)
    speed = s.speed_kmh / 3.6
    duration = spray_len / speed + (path_len - spray_len) / limits.turn_speed_mps
    if refills:
        if home is None:
            warnings.append("Set the home point: the job needs refills.")
        else:
            hx, hy = proj.to_xy(home)
            far = max(math.hypot(px - hx, py - hy) for px, py in outer)
            duration += refills * 2 * far / limits.travel_speed_mps
    bearing = (90.0 - math.degrees(angle)) % 180.0
    return CoveragePlan(
        projection=proj,
        sections=sections,
        row_angle_deg=bearing,
        swath_count=len(swaths_only),
        area_sprayed_m2=area,
        spray_length_m=spray_len,
        path_length_m=path_len,
        duration_s=duration,
        liquid_needed_l=liquid,
        refills=refills,
        flow_l_min=flow,
        warnings=warnings,
    )
