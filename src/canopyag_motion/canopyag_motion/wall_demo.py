"""The demo path (scripts/demo.py): fruit off a wall beside the gantry, each
one placed on its own spot in a basket that rides on the crate.

The wall stands WALL_Y from the gantry, along x, on the +y side (side +1) or
the -y side (side -1, everything mirrored). The gripper points to the wall the
whole way (joint_3 follows, motion_server.py).

The fruit cover the carriage's whole range: LEVELS heights from its lowest to
its highest on the way up, and the LEVELS - 1 heights halfway between them on
the way down, so the path starts and ends low and never travels empty. The x
of the fruit runs through COLUMNS.

Every fruit, three or four moves:
  approach  in front of it, APPROACH off the wall (the carriage goes to the
            fruit's height on the way)
  pick      in, until the gripper tip touches the wall (PICK_SPEED)
  back      out to the approach point again - only where folding straight
            from the wall would sweep the gripper through it
  place     link 1 points forward (|joint_1| <= PLACE_JOINT_1), link 2 folded
            back under it (|joint_2| >= PLACE_FOLD), the tip over the fruit's
            own spot in the basket. The spots are a grid (BASKET_X x
            BASKET_Y), filled in SPOT_ORDER: anywhere in the basket, not one
            drop point.

The basket is not in the robot model: it hangs on the crate in front of the
gantry, under the folded arm, its rim just under the gripper with the crate
`gap` under the carriage (basket()). The crate goes to CRATE_BELOW under the
carriage, and on the way down ahead to the next fruit's height, so it never
holds the carriage up. Nothing waits for it: wherever it is, it is under the
gripper.

The wall is not in the robot model either, the planner does not see it. Each
move here is planned the way motion_server.py will plan it, for every arm
solution (elbow either way) of approach, pick, (back) and place; the fastest
combination that keeps the arm off the wall wins (wall_reach).
"""

import itertools
import math
from dataclasses import dataclass

import numpy as np

from .model import CRATE, JOINTS, WRIST

WALL_Y = 0.45           # m, the wall's distance from the gantry (world y = 0)
APPROACH = 0.06         # m, off the wall before going in
COLUMNS = (0.30, 0.75, 0.45, 0.60)     # world x of the fruit (m), in turn; past 0.75
#                                        unfolding to it sweeps the gripper through the wall
LEVELS = 5              # fruit heights on the way up (and LEVELS - 1 down)
PICK_SPEED = 0.5        # of the demo speed, for going in
CRATE_BELOW = 0.03      # m, crate goal under the carriage (> motion.yaml crate.gap)
# The basket spots: world x and y (y on the +y side, mirrored) of the tip.
# All must be in reach folded (_folded); 4 cm apart, the fruit are 4 cm.
BASKET_X = (0.38, 0.42, 0.46)
BASKET_Y = (0.24, 0.28, 0.32)
SPOT_ORDER = (4, 0, 8, 2, 6, 5, 3, 7, 1)   # index = ix * len(BASKET_Y) + iy
BASKET_WALL = 0.03      # m, basket wall to the outer spots
BASKET_DEPTH = 0.12     # m
BASKET_CLEARANCE = 0.005  # m, rim under the gripper at the closest the crate comes
PLACE_JOINT_1 = math.radians(35.0)
PLACE_FOLD = math.radians(130.0)
WALL_SLACK = 0.002      # m, how far past WALL_Y the plan may reach (the touch is at it)
DEG = math.degrees


@dataclass
class Step:
    name: str
    joints: list            # [z_carriage, joint_1, joint_2]; joint_3 follows the heading
    crate: float            # z_crate goal
    speed: float = 1.0
    wait_crate: bool = False  # done when the crate is there too, not only the arm
    touch: int = -1         # the fruit this step touches the wall at, or -1
    drop: bool = False      # the fruit goes into the basket at the end of this step
    place: tuple = None     # world (x, y, z) of the tip at the drop


def heights(model, gap):
    """z_carriage of every fruit: LEVELS up from the lowest to the highest,
    then the LEVELS - 1 halfway heights down."""
    (zlo, zhi), (clo, _) = model.limits("z_carriage"), model.limits(CRATE)
    up = np.linspace(max(zlo, clo + gap), zhi, LEVELS)
    return [float(z) for z in up] + [float(z) for z in ((up[:-1] + up[1:]) / 2)[::-1]]


def touches(model, gap, side=1):
    """[(x, y, z) of the gripper tip at the wall, z_carriage] for every fruit,
    in path order."""
    return [((COLUMNS[i % len(COLUMNS)], side * WALL_Y, zc + model.tool_z0), zc)
            for i, zc in enumerate(heights(model, gap))]


def spots(side=1):
    """World (x, y) of the tip over every basket spot, in the order they fill."""
    grid = [(x, side * y) for x in BASKET_X for y in BASKET_Y]
    return [grid[i] for i in SPOT_ORDER]


def crate_frame(model):
    """(name, x, y) of the crate link: the basket's frame. Its x and y never
    change, z_crate only moves it up and down."""
    link = model.joints[CRATE]["child"]
    x, y, _, _ = (float(v) for v in model.link_poses({})[link])
    return link, x, y


def basket(model, gap, side=1):
    """The basket in the crate link's frame: (centre (x, y, z), size (x, y, z),
    z of the floor). Its rim sits BASKET_CLEARANCE under the gripper when the
    crate is as high as it may come (gap under the carriage)."""
    crate_link, cx, cy = crate_frame(model)
    cz = float(model.link_poses({})[crate_link][2])
    world, _ = model._world_boxes({})
    _, _, gz, _, gh = world[model.wrist_link]
    bottom = float(gz[0] - (gh[2] - model.margin))         # gripper box, carriage at 0
    rim = bottom + gap - BASKET_CLEARANCE - cz
    xs, ys = BASKET_X, [side * y for y in BASKET_Y]
    size = (max(xs) - min(xs) + 2 * BASKET_WALL, max(ys) - min(ys) + 2 * BASKET_WALL,
            BASKET_DEPTH)
    centre = ((max(xs) + min(xs)) / 2 - cx, (max(ys) + min(ys)) / 2 - cy,
              rim - BASKET_DEPTH / 2)
    return centre, size, rim - BASKET_DEPTH


def wall_reach(model, q, side=1):
    """Per sample of q (joint -> array): how far (m) the arm reaches towards
    the wall's side - the gripper tip and every corner of the collision boxes
    (without the margin) of everything joint_1 turns."""
    world, n = model._world_boxes(q)
    reach = [side * model.tool_position(q)[1]]
    moving = {model.joints[j]["child"] for j in ("joint_1", "joint_2", WRIST)}
    for link in moving & set(world):
        _, y, _, yaw, h = world[link]
        h = h - model.margin
        for sx in (-1, 1):
            for sy in (-1, 1):
                reach.append(side * (y + np.sin(yaw) * sx * h[0] + np.cos(yaw) * sy * h[1]))
    return np.max([np.broadcast_to(r, (n,)) for r in reach], axis=0)


def _plan(model, planner, start, step, side):
    """Plan `step` from `start` as motion_server.py does: (plan, the state the
    next goal starts from), or None if it fails or reaches past the wall."""
    try:
        plan = planner.to_joints(start, step.joints, step.speed, heading=side, crate=step.crate)
    except ValueError:
        return None
    if "stops first" in plan.notes:
        return None
    t = np.linspace(0.0, plan.duration, max(int(plan.duration / 0.005), 1) + 1)
    s = plan.sample(t)
    if wall_reach(model, {j: s[j][0] for j in JOINTS}, side).max() > WALL_Y + WALL_SLACK:
        return None
    end = plan.duration if step.wait_crate else plan.arm_duration
    return plan, end, plan.raw_state(end)


def _arms(model, point, side, keep=lambda j1, j2: True):
    """[z_carriage, joint_1, joint_2] options with the tip at `point`, the
    gripper to the wall, clear of the robot's own boxes."""
    out = []
    for q in model.inverse(point, side * math.pi / 2):
        qd = {"z_carriage": q[0], "joint_1": q[1], "joint_2": q[2], WRIST: q[3], CRATE: 0.0}
        if keep(q[1], q[2]) and not model.colliding_pairs(qd):
            out.append([float(q[0]), float(q[1]), float(q[2])])
    return out


def _folded(side):
    """Link 1 forward, link 2 back under it. Always folded the same way, the
    way the picks bend (joint_2 towards the wall's side): joint_2 cannot turn
    past 180 deg, so the other fold is 360 - 2 * 130 deg away, link 2
    sweeping through the wall on the way."""
    return lambda j1, j2: abs(j1) <= PLACE_JOINT_1 + 1e-9 and side * j2 >= PLACE_FOLD - 1e-9


def path(model, planner, side=1):
    """The whole demo from rest to rest, as Steps. rest: low, folded over the
    basket's centre (_folded)."""
    fruit = touches(model, planner.gap, side)
    places = spots(side)
    if len(places) < len(fruit):
        raise ValueError(f"{len(fruit)} fruit but only {len(places)} basket spots")
    z0 = fruit[0][1]
    clo = model.limits(CRATE)[0]
    zs = [zc for _, zc in fruit for _ in range(3)] + [z0]

    def crate_for(k):        # step k: under the carriage now and over the next fruit
        return max(min(zs[k:k + 4]) - CRATE_BELOW, clo)

    # rest: folded over the basket's centre.
    centre = (float(np.mean(BASKET_X)), side * float(np.mean(BASKET_Y)), z0 + model.tool_z0)
    folds = _arms(model, centre, side, _folded(side))
    if not folds:
        raise ValueError("the basket's centre is out of reach folded")
    rest = Step("rest", folds[0], crate_for(len(zs) - 1), wait_crate=True)
    state = {"z_carriage": (z0, 0.0), "joint_1": (rest.joints[1], 0.0),
             "joint_2": (rest.joints[2], 0.0), CRATE: (rest.crate, 0.0),
             WRIST: (float(model.wrist_for(side * math.pi / 2, *rest.joints[1:])), 0.0)}
    steps = []
    for i, (point, zc) in enumerate(fruit):
        k = 3 * i
        off = (point[0], point[1] - side * APPROACH, point[2])
        place = (places[i][0], places[i][1], point[2])
        folded = _arms(model, place, side, _folded(side))
        if not folded:
            raise ValueError(f"basket spot ({place[0]:.3f}, {place[1]:.3f}) is out of reach "
                             f"folded (|joint_1| <= {DEG(PLACE_JOINT_1):.0f}, |joint_2| >= "
                             f"{DEG(PLACE_FOLD):.0f} deg)")
        best = None
        for a in _arms(model, off, side):
            # The pick: the same elbow as the approach, the closest solution.
            picks = _arms(model, point, side)
            if not picks:
                break
            p = min(picks, key=lambda q: abs(q[1] - a[1]) + abs(q[2] - a[2]))
            for f, back in itertools.product(folded, (False, True)):
                trio = [Step(f"fruit{i}_approach", a, crate_for(k)),
                        Step(f"fruit{i}_pick", p, crate_for(k + 1), speed=PICK_SPEED, touch=i)]
                if back:
                    trio.append(Step(f"fruit{i}_back", a, crate_for(k + 2)))
                trio.append(Step(f"fruit{i}_place", f, crate_for(k + 2), drop=True, place=place))
                st, total = state, 0.0
                for step in trio:
                    r = _plan(model, planner, st, step, side)
                    if r is None:
                        break
                    total += r[1]
                    st = r[2]
                else:
                    if best is None or total < best[0]:
                        best = (total, trio, st)
        if best is None:
            raise ValueError(f"fruit {i} at ({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}): "
                             f"no approach, pick and place that stays off the wall")
        steps += best[1]
        state = best[2]
    if _plan(model, planner, state, rest, side) is None:
        raise ValueError("no way back to rest that stays off the wall")
    return steps + [rest]


def describe(steps):
    """One line per step (for a log)."""
    return [f"{st.name:18s} z {st.joints[0]:.3f} joint_1 {DEG(st.joints[1]):6.1f} joint_2 "
            f"{DEG(st.joints[2]):6.1f} crate {st.crate:.3f}" for st in steps]
