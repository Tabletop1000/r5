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
        ({"speed_kmh": 3.0}, "top speed"),
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
