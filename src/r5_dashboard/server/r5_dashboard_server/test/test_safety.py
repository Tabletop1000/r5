import math

import pytest

from r5_dashboard_server.job import Episode, tilt_deg
from r5_dashboard_server.settings import BY_KEY, DEVICE_TIME_ZONE, check_changes, time_zone_choices
from r5_dashboard_server.units import format_area, format_flow, format_speed, format_volume


def quat_about_x(deg):
    half = math.radians(deg) / 2
    return math.sin(half), 0.0, 0.0, math.cos(half)


def test_tilt_is_zero_when_level_whatever_the_heading():
    assert tilt_deg(0, 0, 0, 1) == pytest.approx(0)
    half = math.radians(70) / 2
    assert tilt_deg(0, 0, math.sin(half), math.cos(half)) == pytest.approx(0, abs=1e-6)


def test_tilt_matches_a_roll_or_pitch():
    assert tilt_deg(*quat_about_x(20)) == pytest.approx(20)
    half = math.radians(-12) / 2
    assert tilt_deg(0, math.sin(half), 0, math.cos(half)) == pytest.approx(12)


def test_tilt_copes_with_a_zero_quaternion():
    assert tilt_deg(0, 0, 0, 0) == 0.0


def test_episode_needs_the_condition_to_hold():
    e = Episode(hold_s=10)
    e.update(True, False, 0)
    e.update(False, True, 5)  # a short dip doesn't count
    e.update(True, False, 6)
    e.update(True, False, 15)
    assert not e.active
    e.update(True, False, 16)
    assert e.active and e.number == 1


def test_episode_ends_only_when_clear():
    e = Episode(hold_s=0)
    e.update(True, False, 0)
    assert e.active
    e.update(False, False, 1)  # between the limits: still on
    assert e.active
    e.update(False, True, 2)
    assert not e.active
    e.update(True, False, 3)
    assert e.number == 2


def test_units():
    assert format_speed(1.609344, "us") == "1.00 mph"
    assert format_speed(1.0, "metric") == "1.00 km/h"
    assert format_area(40468.564224, "us") == "10.00 acres"
    assert format_area(25_000, "metric") == "2.50 ha"
    assert format_area(500, "metric") == "500 m²"
    assert format_volume(37.85411784, "us") == "10.0 gal"
    assert format_flow(3.785411784, "us") == "1.00 gal/min"


def test_time_zones():
    values = [v for v, _ in time_zone_choices()]
    assert values[0] == DEVICE_TIME_ZONE and "UTC" in values
    assert BY_KEY["time_zone"].default == DEVICE_TIME_ZONE
    assert check_changes({"time_zone": values[-1]}) == {"time_zone": values[-1]}


def test_new_speed_settings_are_unit_aware():
    for key in ("max_speed_kmh", "transit_speed_kmh", "turn_speed_kmh"):
        assert BY_KEY[key].to_json()["quantity"] == "speed"
