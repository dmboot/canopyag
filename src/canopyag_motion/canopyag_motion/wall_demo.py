"""The demo path (scripts/demo.py): fruit on a wall beside the gantry.

The wall stands WALL_Y from the gantry, along x, on the +y side (side +1)
or the -y side (side -1, everything mirrored). The fruit are TOUCHES points
on it: ROWS rows from the carriage's lowest height to its highest, two per
row, zig-zag over COLUMNS. The carriage goes up row by row; the crate
follows it CRATE_BELOW lower, slower, and is in place by the time the first
fruit of the row is dropped.

Every fruit: in front of it (APPROACH off the wall), in slowly until the
gripper tip touches the wall, then the drop - link 2 turns fast to 180 deg,
folded under link 1 - and on to the next. The gripper points to the wall the
whole way; joint_3 follows (motion_server.py).

Below on the right side (+y); the left side is the mirror. The wall is
not in the robot model, the planner does not see it: the path keeps the arm
off it by its shape.

  near  x < NEAR_X: the elbow out, link 2 pointing to the wall (joint_2 > 0)
        and joint_1 small (-20 .. 45 deg); link 2 does the turning. The drop
        folds it on, to joint_2 = +180, away from the wall.
  far   x >= NEAR_X: there joint_1 would have to turn below -20 deg, so link 1
        swings out to the wall instead (joint_1 30 .. 65 deg) and link 2 points
        back (joint_2 < 0). The drop folds it the other way, to -180.

Link 2 can never point straight at the wall on the way: with joint_1 >= -10
deg the tip would go through it. Between near and far (and from / to rest)
it turns past that with joint_1 at PASS[0] = -20 deg, 6 cm off the wall: via
PASS (straight out, joint_2 = -20) and, towards near, UNFOLD.
"""

import math
from dataclasses import dataclass

import numpy as np

WALL_Y = 0.40           # m, the wall's distance from the gantry (world y = 0)
APPROACH = 0.06         # m, off the wall before going in
COLUMNS = (0.30, 0.465, 0.635, 0.80)   # world x of the fruit (m)
NEAR_X = 0.60           # x below: near, above: far (see above)
ROWS = 5                # the full z range, bottom to top
PICK_SPEED = 0.4        # of the demo speed, for going in
CRATE_BELOW = 0.03      # m, crate goal under the carriage (> motion.yaml crate.gap)
# (joint_1, joint_2) of the turn past the wall.
PASS = (math.radians(-20.0), math.radians(-20.0))
UNFOLD = (math.radians(-20.0), math.radians(160.0))
DEG = math.degrees


@dataclass
class Step:
    name: str
    joints: list            # [z_carriage, joint_1, joint_2]; joint_3 follows the heading
    crate: float            # z_crate goal
    speed: float = 1.0
    wait_crate: bool = False  # done when the crate is there too, not only the arm
    touch: int = -1         # the fruit this step touches the wall at, or -1
    drop: bool = False      # the fruit goes into the crate at the end of this step


def touches(model, gap, side=1):
    """[(x, y, z) of the gripper tip, z_carriage] for every fruit, in path order."""
    (zlo, zhi), (clo, _) = model.limits("z_carriage"), model.limits("z_crate")
    zlo = max(zlo, clo + gap)
    out = []
    for r, zc in enumerate(np.linspace(zlo, zhi, ROWS)):
        cols = (0, 2) if r % 2 == 0 else (3, 1)       # zig-zag, near / far each row
        for c in cols:
            out.append(((COLUMNS[c], side * WALL_Y, float(zc) + model.tool_z0), float(zc)))
    return out


def arm(model, point, side, near):
    """[z_carriage, joint_1, joint_2] with the tip at `point`, the gripper to
    the wall: near the elbow-out solution, far the elbow-in one."""
    for q in model.inverse(point, side * math.pi / 2):
        q = {"z_carriage": q[0], "joint_1": q[1], "joint_2": q[2], "joint_3": q[3],
             "z_crate": 0.0}
        if (side * q["joint_2"] > 0.0) == near and not model.colliding_pairs(q):
            return [float(q["z_carriage"]), float(q["joint_1"]), float(q["joint_2"])]
    raise ValueError(f"demo fruit ({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}) is out of "
                     f"reach {'near' if near else 'far'} (elbow {'out' if near else 'in'})")


def path(model, gap, side=1):
    """The whole demo from rest to rest, as Steps. rest is [z low, 0, 0]."""
    s = float(side)
    fruit = touches(model, gap, side)
    z0 = fruit[0][1]
    steps = []

    def crate_for(zc):
        return max(zc - CRATE_BELOW, model.limits("z_crate")[0])

    def go(name, zc, j1, j2, **kw):          # j1, j2 for the right side
        steps.append(Step(name, [zc, s * j1, s * j2], crate_for(zc), **kw))

    where = "rest"
    for i, (point, zc) in enumerate(fruit):
        x = point[0]
        near = x < NEAR_X
        reg = "near" if near else "far"
        if where != reg and where != "near":   # from far or rest: the turn past the wall
            go(f"fruit{i}_pass", zc, *PASS)
            if near:
                go(f"fruit{i}_unfold", zc, *UNFOLD)
        elif where == "near" and not near:     # near -> far: the drop went to PASS[0]
            go(f"fruit{i}_pass", zc, *PASS)
        off = (x, point[1] - s * APPROACH, point[2])
        a = arm(model, off, side, near)
        p = arm(model, point, side, near)
        steps.append(Step(f"fruit{i}_approach", a, crate_for(zc)))
        steps.append(Step(f"fruit{i}_pick", p, crate_for(zc), speed=PICK_SPEED, touch=i))
        nxt = fruit[i + 1][0][0] < NEAR_X if i + 1 < len(fruit) else None
        # Fold link 2 under link 1, away from the wall. Leaving near (to far
        # or rest), at PASS[0] already.
        j1 = s * p[1] if near and nxt or not near else PASS[0]
        go(f"fruit{i}_drop", zc, j1, math.pi if near else -math.pi, wait_crate=True,
           drop=True)
        where = reg
    go("rest_pass", fruit[-1][1], *PASS)
    steps.append(Step("rest", [z0, 0.0, 0.0], crate_for(z0), wait_crate=True))
    return steps


def describe(steps):
    """One line per step (for a log)."""
    return [f"{st.name:18s} z {st.joints[0]:.3f} joint_1 {DEG(st.joints[1]):6.1f} joint_2 "
            f"{DEG(st.joints[2]):6.1f} crate {st.crate:.3f}" for st in steps]
