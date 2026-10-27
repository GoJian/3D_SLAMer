import math

from x650_sim.mission import Waypoint, parse_faults, reached, square_plan


def test_square_plan_shape_and_faults():
    p = square_plan(altitude_m=2.0, side_m=3.0, faults={1: 'jump 0.5'})
    assert len(p.waypoints) == 6
    assert all(w.z == -2.0 for w in p.waypoints)          # NED/FRD: up is negative z
    assert (p.waypoints[1].x, p.waypoints[1].y) == (3.0, 0.0)
    assert p.waypoints[1].fault == 'jump 0.5'
    assert p.waypoints[0].fault is None


def test_parse_faults():
    assert parse_faults('1:jump 0.5; 3:reset') == {1: 'jump 0.5', 3: 'reset'}
    assert parse_faults('') == {}


def test_reached():
    w = Waypoint(1.0, 1.0, -2.0, 0.0)
    assert reached((1.1, 1.0, -2.0), w, 0.3)
    assert not reached((1.5, 1.0, -2.0), w, 0.3)
    assert math.isclose(square_plan().waypoints[2].yaw, math.pi / 2)
