import pytest

from r5_dashboard_server.motor_test import BOTH, LEFT, RIGHT, evaluate, twist_for, wheel_speeds


def test_wheel_speeds():
    assert wheel_speeds(LEFT, True, 0.06) == (0.06, 0.0)
    assert wheel_speeds(RIGHT, False, 0.06) == (0.0, -0.06)
    assert wheel_speeds(BOTH, True, 0.06) == (0.06, 0.06)
    with pytest.raises(ValueError):
        wheel_speeds(7, True, 0.06)


@pytest.mark.parametrize("left,right", [(0.06, 0.0), (0.0, -0.06), (0.06, 0.06), (-0.06, -0.06)])
def test_twist_gives_back_the_wheel_speeds(left, right):
    b = 0.36
    v, w = twist_for(left, right, b)
    assert v - w * b / 2 == pytest.approx(left)
    assert v + w * b / 2 == pytest.approx(right)


def test_evaluate():
    ok, _ = evaluate(BOTH, (0.06, 0.06), (0.055, 0.058))
    assert ok
    ok, msg = evaluate(LEFT, (0.06, 0.0), (0.01, 0.0))
    assert not ok and "Left" in msg
    ok, msg = evaluate(RIGHT, (0.0, 0.06), (0.0, -0.05))
    assert not ok and "commanded way" in msg
    ok, msg = evaluate(BOTH, (0.06, 0.06), None)
    assert not ok and "joint_states" in msg
