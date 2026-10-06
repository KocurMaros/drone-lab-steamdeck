"""Square demo planning (dronelab.demos.square) - pure geometry, no ROS."""
import math

import yaml

from dronelab.config import DEFAULT_CONFIG, Config
from dronelab.demos.square import clearance, fence_from_status, fit_altitude, plan_square


def fence(profile=None):
    ds = Config(yaml.safe_load(open(DEFAULT_CONFIG))).drone(11, profile)
    return fence_from_status(ds.fence.describe())


def is_square(c, side):
    assert c[0] == c[-1] and len(c) == 5
    for a, b in zip(c, c[1:]):
        assert math.isclose(math.dist(a, b), side, abs_tol=1e-9)


def test_indoor_square_fits_from_origin_and_altitude_is_lowered():
    f = fence()                                   # x [-0.5, 4], y [-0.5, 6], z [0.5, 2.5]
    c = plan_square(f, (0.0, 0.0), 3.0, 0.5)
    is_square(c, 3.0)
    assert min(clearance(f, p) for p in c) >= 0.5
    assert c[0] == (0.0, 0.0)                      # starts where it took off
    assert fit_altitude(f, 3.0) == 2.2             # 3 m does not fit under the 2.5 m ceiling


def test_indoor_square_mirrors_near_a_wall_and_refuses_when_too_big():
    f = fence()
    c = plan_square(f, (3.5, 5.5), 3.0, 0.5)       # near the +x/+y corner -> goes to -x/-y
    is_square(c, 3.0)
    assert all(x <= 3.5 and y <= 5.5 for x, y in c)
    assert plan_square(f, (1.0, 1.0), 6.0, 0.5) is None


def test_outdoor_polygon_and_sim():
    f = fence("outdoor_field")
    assert f.kind == "global"
    c = plan_square(f, (0.0, 0.0), 3.0, 1.0)        # plane origin = polygon centre
    is_square(c, 3.0)
    assert fit_altitude(f, 3.0) == 3.2             # alt_rel [3, 30]: at least just above the floor
    s = fence("sim_arena")
    assert fit_altitude(s, 3.0) == 3.0
    is_square(plan_square(s, (0.0, 0.0), 3.0, 0.5), 3.0)


def test_heights_are_relative_to_the_takeoff_point():
    from dronelab.demos.square import plan_height
    f = fence()                                    # indoor: z [0.5, 2.5]
    assert plan_height(f, 0.05, 3.0) == 2.5 - 0.3 - 0.05
    # drifted height reference: on the floor it reads 2.0 m -> refuse instead of flying 0.2 m above the floor
    import pytest
    with pytest.raises(ValueError, match="height reference"):
        plan_height(f, 2.0, 3.0)
    assert abs(plan_height(f, -0.4, 1.0) - 1.1) < 1e-9   # reads 0.4 m low: climbs 1.1 m to reach the fence floor + 0.2
