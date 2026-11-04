import math

import pytest

fields2cover = pytest.importorskip("fields2cover")

from r5_dashboard_server.coverage import SWATH, TURN, PlanError, RobotLimits, SpraySettings, plan_coverage  # noqa: E402
from r5_dashboard_server.fields import Field  # noqa: E402
from r5_dashboard_server.geo import LocalProjection, point_in_ring  # noqa: E402

P = LocalProjection((-33.86, 151.21))


def ll(pts):
    return [P.to_latlon(p) for p in pts]


def field(keep_out=(), headland=3.0, boundary=((0, 0), (60, 0), (60, 40), (0, 40))):
    return Field(id="f", name="Test", boundary=ll(list(boundary)), keep_out=[ll(list(z)) for z in keep_out], headland_width_m=headland)


def settings(**kw):
    base = dict(field_id="f", application_rate_l_per_m2=0.02, boom_width_m=1.5, speed_kmh=1.0, tank_capacity_l=50.0)
    base.update(kw)
    return SpraySettings(**base)


def row_bearing(plan):
    sw = next(s for s in plan.sections if s.kind == SWATH)
    (x0, y0), (x1, y1) = sw.points[0], sw.points[-1]
    return math.degrees(math.atan2(x1 - x0, y1 - y0)) % 180


def test_auto_angle_picks_fewest_rows():
    plan = plan_coverage(field(), settings(), RobotLimits())
    # A 60 × 40 m field: rows along the 60 m side need fewer passes.
    assert plan.row_angle_deg == pytest.approx(90)
    assert row_bearing(plan) == pytest.approx(90, abs=0.5)
    assert plan.swath_count == math.ceil((40 - 6) / 1.5)
    # Rows alternate with turns.
    kinds = [s.kind for s in plan.sections]
    assert kinds[0] == SWATH and all(a != b for a, b in zip(kinds, kinds[1:]))


@pytest.mark.parametrize("angle", [0, 45, 135])
def test_fixed_row_direction(angle):
    plan = plan_coverage(field(), settings(auto_angle=False, row_angle_deg=angle), RobotLimits())
    assert row_bearing(plan) == pytest.approx(angle, abs=0.5)


def test_summary_numbers():
    plan = plan_coverage(field(), settings(), RobotLimits())
    inner = (60 - 6) * (40 - 6)
    assert plan.area_sprayed_m2 == pytest.approx(inner, rel=0.08)
    assert plan.liquid_needed_l == pytest.approx(plan.area_sprayed_m2 * 0.02)
    assert plan.flow_l_min == pytest.approx(0.5)
    assert plan.refills == 0
    assert plan.duration_s > plan.spray_length_m / (1.0 / 3.6)


def test_rows_avoid_keep_out_zone():
    zone = [(25, 15), (35, 15), (35, 25), (25, 25)]
    plan = plan_coverage(field(keep_out=[zone]), settings(), RobotLimits())
    for s in plan.sections:
        if s.kind == SWATH:
            for t in (0.25, 0.5, 0.75):
                x = s.points[0][0] + t * (s.points[-1][0] - s.points[0][0])
                y = s.points[0][1] + t * (s.points[-1][1] - s.points[0][1])
                assert not point_in_ring((x, y), zone)


def test_refills_and_home_warning():
    plan = plan_coverage(field(), settings(application_rate_l_per_m2=0.1, tank_capacity_l=40.0), RobotLimits(pump_max_flow_l_min=10))
    assert plan.refills >= 3
    assert any("home point" in w for w in plan.warnings)
    with_home = plan_coverage(field(), settings(application_rate_l_per_m2=0.1, tank_capacity_l=40.0), RobotLimits(), home=P.to_latlon((-10, 0)))
    assert with_home.duration_s > plan.duration_s


def test_pump_limit_and_narrow_headland():
    with pytest.raises(PlanError, match="Lower the speed to 0.50 km/h"):
        plan_coverage(field(), settings(application_rate_l_per_m2=0.2), RobotLimits(pump_max_flow_l_min=2.5))
    plan = plan_coverage(field(headland=1.0), settings(), RobotLimits())
    assert any("narrow" in w for w in plan.warnings)


def test_field_too_small():
    with pytest.raises(PlanError, match="nothing left"):
        plan_coverage(field(boundary=((0, 0), (5, 0), (5, 5), (0, 5)), headland=2.4), settings(), RobotLimits())


def test_single_cell_rows_go_back_and_forth():
    plan = plan_coverage(field(), settings(), RobotLimits())
    rows = [sec for sec in plan.sections if sec.kind == SWATH]
    for a, b in zip(rows, rows[1:]):
        # Each row starts next to where the previous one ended (one boom width over).
        assert math.dist(a.points[-1], b.points[0]) == pytest.approx(1.5, abs=0.2)
        # ...and runs the other way.
        da = (a.points[-1][0] - a.points[0][0], a.points[-1][1] - a.points[0][1])
        db = (b.points[-1][0] - b.points[0][0], b.points[-1][1] - b.points[0][1])
        assert da[0] * db[0] + da[1] * db[1] < 0


def test_large_field_plans_quickly():
    import time

    big = field(boundary=((0, 0), (250, 0), (240, 160), (-10, 150)), headland=2.5)  # about 4 ha
    t = time.perf_counter()
    plan = plan_coverage(big, settings(), RobotLimits())
    assert time.perf_counter() - t < 3.0
    assert plan.swath_count > 90


@pytest.mark.parametrize(
    "zones",
    [
        [[(25, 15), (35, 15), (35, 25), (25, 25)]],
        [[(12, 12), (20, 12), (20, 20)], [(40, 18), (50, 18), (50, 28), (40, 28)]],
    ],
)
def test_turns_and_joins_stay_in_field_and_out_of_keep_outs(zones):
    plan = plan_coverage(field(keep_out=zones), settings(), RobotLimits())
    # Into the plan's own frame (centred on the field).
    to_plan = lambda pts: [plan.projection.to_xy(P.to_latlon(q)) for q in pts]  # noqa: E731
    outer = to_plan([(0, 0), (60, 0), (60, 40), (0, 40)])
    zones = [to_plan(z) for z in zones]
    for sec in plan.sections:
        pts = sec.points
        dense = [(a[0] + (b[0] - a[0]) * t / 4, a[1] + (b[1] - a[1]) * t / 4) for a, b in zip(pts, pts[1:]) for t in range(5)]
        for p in dense:
            assert point_in_ring(p, outer), f"{sec.kind} leaves the field at {p}"
            for z in zones:
                assert not point_in_ring(p, z), f"{sec.kind} enters a keep-out at {p}"
    # Every row of every cell is driven exactly once.
    assert plan.swath_count == sum(1 for s in plan.sections if s.kind == SWATH)
