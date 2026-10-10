"""Frame conversions in offboard_adapter.py: NED/FRD (PX4) vs ENU/FLU (ROS)."""
import importlib.util
import math
import pathlib

import pytest

spec = importlib.util.spec_from_file_location(
    "adapter", pathlib.Path(__file__).parents[1] / "scripts" / "offboard_adapter.py")
a = importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)


def yaw_q(yaw):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def test_nose_north_is_enu_yaw_90():
    assert a.yaw_enu(a.att_ned_to_enu((1.0, 0.0, 0.0, 0.0))) == pytest.approx(math.pi / 2)


def test_nose_east_is_enu_yaw_0():
    # PX4 yaw +90 deg (NED, clockwise from north) = facing east.
    assert a.yaw_enu(a.att_ned_to_enu(yaw_q(math.pi / 2))) == pytest.approx(0.0, abs=1e-9)


def test_right_wing_down_is_positive_roll_both_ways():
    # FRD roll +0.2 (right side down) is FLU roll +0.2 too.
    q = a.att_ned_to_enu((math.cos(0.1), math.sin(0.1), 0.0, 0.0))
    right = a.rotate(q, (0.0, -1.0, 0.0))  # FLU right wing, in ENU
    assert right[2] < 0


def test_world_ned_round_trip_and_offset():
    o = a.Origin(1.0, -2.0, 0.05)  # PX4 booted at world (1, -2), resting 5 cm up
    # 2 m above the world ground, at the boot point: PX4 NED (0, 0, -1.95).
    assert o.world_to_ned((1.0, -2.0, 2.0)) == pytest.approx((0.0, 0.0, -1.95))
    # 3 m east of the boot point is NED east.
    assert o.world_to_ned((4.0, -2.0, 0.05)) == pytest.approx((0.0, 3.0, 0.0))
    p = (3.0, -4.0, 2.5)
    assert o.ned_to_world(o.world_to_ned(p)) == pytest.approx(p)
