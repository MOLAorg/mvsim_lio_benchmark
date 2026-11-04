"""open_field: 400 x 400 m of gently rolling grassland (+-4 m) with very few
features: a dozen poles, a fence line, three lone trees, hay bales and a
small shed. Stresses horizontal degeneracy: the ground constrains z, roll and
pitch, but x, y and yaw rest on a handful of distant objects."""
import math

import numpy as np

from mvsim_lio import props
from mvsim_lio.traj import P, Spin
from mvsim_lio.world import Terrain, World

SEED = 101


def ground(x, y):
    return (2.5 * np.sin(x / 45.0) * np.cos(y / 60.0) + 1.5 * np.sin((x + y) / 80.0)
            + 0.3 * np.sin(x / 9.0 + 1.0) * np.sin(y / 11.0))


def make_world():
    rng = np.random.default_rng(SEED)
    w = World('open_field', __doc__, extent=(-200, 200, -200, 200))
    w.set_terrain(Terrain(ground, -200, 200, -200, 200, 1.0, color='#93b070'))
    g = lambda x, y: float(ground(x, y))  # noqa: E731
    # Sparse poles (utility / survey poles)
    for _ in range(12):
        x, y = rng.uniform(-170, 170, size=2)
        if abs(y - 20) < 8:
            continue
        w.cylinder(x, y, g(x, y) - 0.3, 0.12, rng.uniform(3, 7), color='#806a50')
    # Fence line along y = 60, from x = -150 to 50
    for k in range(8):
        x0 = -150 + 25 * k
        props.fence(w, x0, 60, x0 + 25, 60, g, spacing=3.0)
    # Lone trees
    for x, y in ((-60, -40), (90, 70), (30, -120)):
        props.tree(w, rng, x, y, g(x, y), height=9, crown=3.2)
    # Hay bales (cylinders lying down)
    for _ in range(10):
        x, y = rng.uniform(40, 120), rng.uniform(-60, 10)
        w.cylinder(x, y, g(x, y) + 0.7, 0.7, 1.2, yaw=rng.uniform(0, math.pi),
                   pitch=math.pi / 2, color='#d8c070')
    # Small shed
    w.box(-110, -90, g(-110, -90) - 0.3, 4.0, 3.0, 2.8, 0.3, '#a05040')
    return w


def trajectories(world):
    # A long lap through the field (UGV, 2 m/s), and a faster zig-zag with
    # tight turns and two in-place spins.
    lap = [P(-120, 20, v=2.0), P(-60, 30), P(0, 10), P(60, 25), P(120, 0), P(140, -60),
           P(100, -120), P(30, -140), P(-40, -110), P(-100, -130), P(-150, -60), P(-130, 0),
           P(-120, 20)]
    zig = [P(-80, -20, v=3.0)]
    for k in range(8):
        zig.append(P(-60 + 20 * k, -20 + (25 if k % 2 == 0 else -25)))
    zig += [Spin(360, 6.0), P(100, 40), P(40, 48), P(-40, 40), Spin(-270, 5.0), P(-80, -20)]
    return {
        'lap': dict(items=lap, platform='ugv', seed=1,
                    description='UGV lap through the field, 2 m/s'),
        'zigzag': dict(items=zig, platform='ugv', seed=2,
                       description='UGV zig-zag at 3 m/s, tight turns and in-place spins'),
    }
