import math

import pytest

from r5_dashboard_server.coverage import (
    PlanError,
    RobotLimits,
    SpraySettings,
    check_settings,
    flow_l_min,
    liquid_plan,
    max_speed_for_pump,
)
from r5_dashboard_server.job import (
    Affine2D,
    Tank,
    along_track,
    angle_difference,
    extend_line,
    flow_for_speed,
    format_duration,
    crosses_field,
    densify,
    gate_waypoints,
    route_around,
    heading,
    sprayer_should_be_on,
)


def test_flow_matches_spec_formula():
    # 0.02 L/m² × 1.5 m × 1.0 km/h × 1000 / 60 = 0.5 L/min
    assert flow_l_min(0.02, 1.5, 1.0) == pytest.approx(0.5)
    # Same thing from ground speed in m/s.
    assert flow_for_speed(0.02, 1.5, 1.0 / 3.6) == pytest.approx(0.5)
    assert max_speed_for_pump(0.02, 1.5, 0.5) == pytest.approx(1.0)


@pytest.mark.parametrize(
    "change,reason",
    [
        ({"application_rate_l_per_m2": 0}, "application rate"),
        ({"application_rate_l_per_m2": 2}, "too high"),
        ({"boom_width_m": -1}, "boom width"),
        ({"speed_kmh": 2.0}, "top speed"),
        ({"tank_capacity_l": float("nan")}, "tank capacity"),
        ({"auto_angle": False, "row_angle_deg": float("nan")}, "row direction"),
    ],
)
def test_settings_are_checked(change, reason):
    s = SpraySettings("f", 0.02, 1.5, 1.0, 50.0)
    for k, v in change.items():
        setattr(s, k, v)
    with pytest.raises(PlanError, match=reason):
        check_settings(s, RobotLimits())


def test_refill_count_decided_before_each_row():
    # 50 L tank, 5% reserve: rows of 20 L → 20, 20, refill, 20, 20, refill, 20
    assert liquid_plan([20.0] * 5, 50.0, 0.05) == 2
    assert liquid_plan([10.0] * 4, 50.0, 0.05) == 0
    with pytest.raises(PlanError, match="more liquid than a full tank"):
        liquid_plan([60.0], 50.0, 0.05)


def test_tank():
    t = Tank(50.0, 0.05)
    t.use(30.0, 60.0)  # 30 L
    assert t.remaining == pytest.approx(20.0) and t.used_total == pytest.approx(30.0)
    assert not t.needs_refill_for(17.0)
    assert t.needs_refill_for(18.0)  # would dip below the 2.5 L reserve
    t.refill()
    assert t.remaining == 50.0 and t.used_total == pytest.approx(30.0)


def test_affine_from_points_handles_rotation_and_offset():
    angle = math.radians(30)
    def truth(p):
        return (math.cos(angle) * p[0] - math.sin(angle) * p[1] + 5, math.sin(angle) * p[0] + math.cos(angle) * p[1] - 3)
    t = Affine2D.from_points(truth((0, 0)), truth((100, 0)), truth((0, 100)), 100)
    assert t.apply((12.0, -7.0)) == pytest.approx(truth((12.0, -7.0)))
    assert t.scale_error() < 1e-9


def test_along_track_and_sprayer_window():
    s, off = along_track((5, 1), (0, 0), (10, 0))
    assert (s, off) == pytest.approx((5, 1))
    assert sprayer_should_be_on(5, 0.2, 10, 1.5)
    assert not sprayer_should_be_on(-0.1, 0, 10, 1.5)  # before the row
    assert not sprayer_should_be_on(10.1, 0, 10, 1.5)  # past the end
    assert not sprayer_should_be_on(5, 1.2, 10, 1.5)  # beside the row (next row over is 1.5 m away)
    assert not sprayer_should_be_on(5, 0.1, 10, 1.5, math.radians(40))  # still turning onto it
    assert sprayer_should_be_on(5, 0.1, 10, 1.5, math.radians(10))
    assert angle_difference(math.radians(350), math.radians(10)) == pytest.approx(math.radians(20))


def test_extend_line_adds_lead_in_and_out_and_densifies():
    pts = extend_line((0, 0), (10, 0), 2.0, 1.5)
    assert pts[0] == pytest.approx((-2, 0)) and pts[-1] == pytest.approx((11.5, 0))
    assert all(math.dist(a, b) <= 0.26 for a, b in zip(pts, pts[1:]))
    assert heading((0, 0), (0, 1)) == pytest.approx(math.pi / 2)


def test_format_duration():
    assert format_duration(30) == "0 min" or format_duration(30) == "1 min"
    assert format_duration(45 * 60) == "45 min"
    assert format_duration(135 * 60) == "2 h 15 min"


def test_gate_waypoints_are_square_to_the_gate_either_side():
    square = [(0, 0), (100, 0), (100, 100), (0, 100)]
    outside, inside = gate_waypoints((40, 0), (44, 0), square, 3.0)
    assert outside == pytest.approx((42, -3)) and inside == pytest.approx((42, 3))
    # The order of the posts doesn't matter.
    assert gate_waypoints((44, 0), (40, 0), square, 3.0) == (pytest.approx((42, -3)), pytest.approx((42, 3)))
    outside, inside = gate_waypoints((100, 50), (100, 54), square, 3.0)
    assert outside == pytest.approx((103, 52)) and inside == pytest.approx((97, 52))


def test_route_around_goes_round_the_field_not_across_it():
    square = [(0, 0), (100, 0), (100, 100), (0, 100)]
    # Clear line: straight there.
    assert route_around((-10, -10), (50, -5), square) == [(-10, -10), (50, -5)]
    # Home south-west, gate on the north side: round the west side, not across.
    route = route_around((-10, -10), (50, 103), square)
    assert route[0] == (-10, -10) and route[-1] == (50, 103)
    # Up the west side to just outside the north-west corner, then along the north side.
    assert route[1:-1] == [pytest.approx((-2, 102))]
    assert all(not crosses_field(a, b, square) for a, b in zip(route, route[1:]))
    # Every waypoint keeps its distance from the boundary.
    assert all(min(abs(p[0]), abs(p[0] - 100), abs(p[1]), abs(p[1] - 100)) >= 1.99 for p in route[1:-1])


def test_route_around_a_notch():
    # A U-shaped field, gate at the bottom of the notch, home below the field.
    u = [(0, 0), (100, 0), (100, 100), (60, 100), (60, 40), (40, 40), (40, 100), (0, 100)]
    route = route_around((50, -10), (50, 43), u)
    assert route[-1] == (50, 43)
    assert all(not crosses_field(a, b, u) for a, b in zip(route, route[1:]))


def test_route_around_gives_up_from_inside():
    square = [(0, 0), (100, 0), (100, 100), (0, 100)]
    assert route_around((50, 50), (50, 150), square) == [(50, 50), (50, 150)]


def test_densify():
    pts = densify([(0, 0), (1, 0), (1, 0.5)], 0.25)
    assert pts[0] == (0, 0) and pts[-1] == (1, 0.5) and len(pts) == 7
