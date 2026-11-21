"""config/limits.yaml and r5_limits, which fills the limits into parameter files."""

import os
import re

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from r5_limits import LimitsError, fill, load

SHARE = get_package_share_directory("r5")


def write(tmp_path, text, name="limits.yaml"):
    path = tmp_path / name
    path.write_text(text)
    return str(path)


GOOD = """
wheel_max_velocity: 6.0
max_linear_speed: 0.6
max_angular_speed: 1.0
max_linear_accel: 1.0
max_angular_accel: 1.0
"""


def test_the_installed_limits_are_valid():
    limits = load()
    assert set(limits) == {
        "wheel_max_velocity", "max_linear_speed", "max_angular_speed", "max_linear_accel", "max_angular_accel",
    }


def test_the_controllers_file_is_filled_completely():
    path = fill(os.path.join(SHARE, "config", "diffbot_controllers.yaml"))
    try:
        text = open(path).read()
        assert not re.search(r"\$\(var \w+\)", text)
        params = yaml.safe_load(text)["diffbot_base_controller"]["ros__parameters"]
        limits = load()
        assert params["linear.x.max_velocity"] == limits["max_linear_speed"]
        assert params["linear.x.min_velocity"] == -limits["max_linear_speed"]
        assert params["angular.z.min_acceleration"] == -limits["max_angular_accel"]
    finally:
        os.unlink(path)


def test_fill_leaves_other_substitutions(tmp_path):
    limits = load(write(tmp_path, GOOD))
    source = write(tmp_path, "a: [$(var max_linear_speed), -$(var max_angular_speed)]\nb: $(var other)\n", "p.yaml")
    path = fill(source, limits)
    try:
        assert open(path).read() == "a: [0.6, -1.0]\nb: $(var other)\n"
    finally:
        os.unlink(path)


@pytest.mark.parametrize(
    "change, reason",
    [
        ("max_linear_speed: 0.7", "faster than wheel_max_velocity allows"),
        ("max_angular_speed: 40.0", "faster than wheel_max_velocity allows"),
        ("max_linear_speed: -1", "positive number"),
        ("max_linear_speed: fast", "positive number"),
        ("max_linear_accel: .nan", "positive number"),
        ("top_speed: 1", "unknown limits top_speed"),
    ],
)
def test_bad_limits_are_refused(tmp_path, change, reason):
    key = change.split(":")[0]
    lines = [line for line in GOOD.strip().splitlines() if not line.startswith(key + ":")]
    with pytest.raises(LimitsError, match=reason):
        load(write(tmp_path, "\n".join(lines + [change]) + "\n"))


def test_a_missing_limit_is_refused(tmp_path):
    with pytest.raises(LimitsError, match="max_angular_accel"):
        load(write(tmp_path, GOOD.replace("max_angular_accel: 1.0", "")))
