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
    ref = a.Ref(47.0, 8.0, 0.0)
    ref.set(47.0 + math.degrees(10.0 / a.EARTH_RADIUS), 8.0, 0.75)  # origin 10 m north, 0.75 m up
    assert ref.offset == pytest.approx((0.0, 10.0, 0.75))
    # A point 2 m above the world origin is 10 m south of EKF2's origin, 1.25 m up.
    assert ref.world_to_ned((0.0, 0.0, 2.0)) == pytest.approx((-10.0, 0.0, -1.25))
    p = (3.0, -4.0, 2.5)
    assert ref.ned_to_world(ref.world_to_ned(p)) == pytest.approx(p)
