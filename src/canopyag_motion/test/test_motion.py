"""canopyag_motion without ROS: kinematics, profiles, planner.

Uses the real robot_parameters.yaml and motion.yaml, so a CAD re-import or a
limit change that breaks planning shows up here.

Run standalone: pytest src/canopyag_motion/test
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

import canopyag_motion  # noqa: E402
from canopyag_motion import JOINTS, PlanError  # noqa: E402
from canopyag_motion.profiles import min_time  # noqa: E402

PARAMS = PKG.parent / "canopyag_description" / "config" / "robot_parameters.yaml"
CONFIG = PKG / "config" / "motion.yaml"
DEG = math.radians


@pytest.fixture(scope="module")
def loaded():
    return canopyag_motion.load(PARAMS, CONFIG)


@pytest.fixture(scope="module")
def model(loaded):
    return loaded[0]


@pytest.fixture(scope="module")
def planner(loaded):
    return loaded[1]


def rest(z=0.05, j1=0.0, j2=0.0, crate=0.0):
    return {"z_carriage": (z, 0.0), "joint_1": (j1, 0.0), "joint_2": (j2, 0.0), "z_crate": (crate, 0.0)}


def check_plan(model, planner, plan, start):
    """Starts at start, ends at rest at its goal, within the limits, and never
    collides - checked 4x finer than the planner does."""
    t = np.linspace(0.0, plan.duration, max(int(plan.duration / planner.check_dt * 4), 2) + 1)
    s = plan.sample(t)
    for j in JOINTS:
        p, v, a = s[j]
        vmax, amax = planner.limits[j]
        assert abs(p[0] - start[j][0]) < 1e-9 and abs(v[0] - start[j][1]) < 1e-9, j
        assert np.all(np.abs(v) <= vmax + 1e-6), j
        assert np.all(np.abs(a) <= amax + 1e-6), j
        assert abs(v[-1]) < 1e-9, j
        lo, hi = model.limits(j)
        assert np.all(p >= lo - 1e-6) and np.all(p <= hi + 1e-6), j
    for j, g in zip(("z_carriage", "joint_1", "joint_2"), plan.goal):
        assert abs(s[j][0][-1] - g) < 1e-9, j
    assert not model.collisions({j: s[j][0] for j in JOINTS}).any()
    gap = s["z_carriage"][0] - s["z_crate"][0]
    g0 = start["z_carriage"][0] - start["z_crate"][0]
    assert np.all(gap >= min(planner.gap, g0) - 1e-6)


# ---------------------------------------------------------------- kinematics

def test_inverse_hits_the_point(model):
    rng = np.random.default_rng(1)
    hit = 0
    for _ in range(300):
        p = np.array([rng.uniform(-0.5, 0.9), rng.uniform(-0.7, 0.7), rng.uniform(0.3, 1.4)])
        for q in model.inverse(p):
            hit += 1
            got = model.tool_position(dict(zip(("z_carriage", "joint_1", "joint_2"), q)))
            assert np.allclose(got, p, atol=1e-9)
            for j, v in zip(("z_carriage", "joint_1", "joint_2"), q):
                lo, hi = model.limits(j)
                assert lo - 1e-9 <= v <= hi + 1e-9
    assert hit > 100


def test_out_of_reach(model):
    far = model.j1_xy + [model.link1 + model.link2 + 0.01, 0.0]
    assert model.inverse([far[0], far[1], 0.8]) == []
    assert model.inverse([0.5, 0.5, 5.0]) == []


def test_joint_2_zero_has_both_turns(model):
    """joint_2 0..2 pi: arm straight out is joint_2 = 0 and = 2 pi."""
    p = model.tool_position({"z_carriage": 0.5, "joint_1": 0.3, "joint_2": 0.0})
    j2 = sorted(round(q[2], 6) for q in model.inverse(p))
    assert j2[0] == 0.0 and abs(j2[-1] - 2 * math.pi) < 1e-3


def test_collision_boxes(model):
    def hit(j1, j2):
        return bool(model.collisions({"z_carriage": 0.5, "z_crate": 0.3,
                                      "joint_1": DEG(j1), "joint_2": DEG(j2)})[0])
    assert not hit(0, 0)
    assert not hit(0, 180)        # link 2 folded under link 1, arm in the middle
    assert not hit(60, 15)        # picking on the right
    assert hit(60, 165)           # folding out there sweeps through the carriage
    assert hit(120, 180)          # ... and at 120 deg through the gantry
    assert hit(120, 0)            # mid_link reaches the gantry at the joint_1 limit
    # the crate and the carriage are kept apart by the gap, not the boxes
    assert ("carriage", "crate") not in model.pairs
    assert not model.collisions({"z_carriage": 0.0, "z_crate": 0.0})[0]


# ---------------------------------------------------------------- profiles

@pytest.mark.parametrize("p0,v0,p1", [
    (0.0, 0.0, 2.0), (0.0, 0.0, 0.1), (1.0, 0.0, -3.0),   # trapezoid, triangle, negative
    (0.0, 2.0, 3.0), (0.0, 2.0, 0.2), (0.0, -2.0, 1.0),   # moving: on, overshoot, away
    (0.0, 5.0, 4.0), (0.5, 0.0, 0.5),                      # too fast, already there
])
def test_min_time(p0, v0, p1):
    vmax, amax = 3.5, 5.0
    pr = min_time(p0, v0, p1, vmax, amax)
    t = np.linspace(0, pr.duration, 2001)
    p, v, a = pr.sample(t)
    assert abs(p[0] - p0) < 1e-12 and abs(v[0] - v0) < 1e-12
    assert abs(p[-1] - p1) < 1e-9 and abs(v[-1]) < 1e-12
    assert np.all(np.abs(a) <= amax + 1e-9)
    assert np.all(np.abs(v) <= max(vmax, abs(v0)) + 1e-9)
    # velocity is the integral of acceleration, position of velocity
    assert np.allclose(np.diff(v), 0.5 * (a[1:] + a[:-1]) * np.diff(t), atol=1e-2)
    assert np.allclose(np.diff(p), 0.5 * (v[1:] + v[:-1]) * np.diff(t), atol=1e-4)


def test_min_time_is_the_textbook_minimum():
    # from rest, d = 2, vmax 1, amax 2: 0.5 s up, 1.5 s cruise, 0.5 s down
    assert min_time(0.0, 0.0, 2.0, 1.0, 2.0).duration == pytest.approx(2.5)
    # short: triangle, t = 2 sqrt(d / a)
    assert min_time(0.0, 0.0, 0.18, 1.0, 2.0).duration == pytest.approx(2 * math.sqrt(0.09))


def test_after_and_delayed():
    pr = min_time(0.0, 0.0, 2.0, 1.0, 2.0)
    later = pr.delayed(0.3)
    assert later.duration == pytest.approx(pr.duration + 0.3)
    assert later.sample(1.3)[0] == pytest.approx(pr.sample(1.0)[0])
    rest_ = pr.after(1.0)
    assert rest_.sample(0.5)[0] == pytest.approx(pr.sample(1.5)[0])
    assert rest_.duration == pytest.approx(pr.duration - 1.0)


# ---------------------------------------------------------------- planner

FRUIT = [(0.48, 0.58, 0.60), (0.28, 0.64, 0.70), (0.48, -0.58, 0.90), (0.28, -0.64, 1.00)]


@pytest.mark.parametrize("point", FRUIT)
def test_plan_to_point(model, planner, point):
    start = rest()
    plan = planner.to_point(start, point)
    check_plan(model, planner, plan, start)
    q = dict(zip(("z_carriage", "joint_1", "joint_2"), plan.goal))
    assert np.allclose(model.tool_position(q), point, atol=1e-9)


@pytest.mark.parametrize("side", [1, -1])
def test_pick_drop_next(model, planner, side):
    """The demo's moves: pick, fold link 2 under link 1, next fruit - all
    joints at once, collision free."""
    start = rest()
    for x, y, z in FRUIT[:2]:
        pick = planner.to_point(start, (x, side * y, z))
        check_plan(model, planner, pick, start)
        start = {j: (p.end, 0.0) for j, p in pick.profiles.items()}
        j2 = pick.goal[2] - math.pi if pick.goal[2] > math.pi else pick.goal[2] + math.pi
        drop = planner.to_joints(start, (pick.goal[0], 0.0, j2))
        check_plan(model, planner, drop, start)
        start = {j: (p.end, 0.0) for j, p in drop.profiles.items()}


def test_simultaneous(planner):
    """No wait needed: every joint starts at once and the move takes as long
    as its slowest joint alone."""
    start = rest()
    plan = planner.to_joints(start, (0.3, DEG(40), DEG(30)))
    slowest = max(min_time(*start[j], g, *planner.limits[j]).duration
                  for j, g in zip(("z_carriage", "joint_1", "joint_2"), (0.3, DEG(40), DEG(30))))
    assert plan.duration == pytest.approx(slowest)
    for j in ("z_carriage", "joint_1", "joint_2"):
        assert plan.profiles[j].phases[0][1] != 0.0, f"{j} does not start at once"


def test_wait_instead_of_collision(model, planner):
    """Folded in the middle, then out to the side: link 1 is faster, and
    swinging out while link 2 is still folded sweeps it through the carriage -
    joint_1 waits a little for joint_2."""
    start = rest(z=0.4, j1=0.0, j2=DEG(190), crate=0.25)
    plan = planner.to_joints(start, (0.4, DEG(90), DEG(20)))
    check_plan(model, planner, plan, start)
    assert any("joint_1 waits" in n for n in plan.notes)
    # and it is still a simultaneous move, not one joint after the other
    assert plan.duration < sum(min_time(*start[j], g, *planner.limits[j]).duration
                               for j, g in (("joint_1", DEG(90)), ("joint_2", DEG(20))))


def test_stop_on_the_way(model, planner):
    """Link 2 from 15 to 210 deg with link 1 out at 60 deg, staying there:
    no wait helps, it swings in, turns, swings out."""
    start = rest(z=0.4, j1=DEG(60), j2=DEG(15), crate=0.25)
    plan = planner.to_joints(start, (0.4, DEG(60), DEG(210)))
    check_plan(model, planner, plan, start)
    assert any("stops at joint_1" in n for n in plan.notes)


def test_goal_in_collision(planner):
    with pytest.raises(PlanError, match="collides at the goal"):
        planner.to_joints(rest(), (0.5, DEG(60), DEG(165)))


def test_crate_keeps_its_gap(model, planner):
    # from home (gap 0) up: the crate follows; down from high: the crate goes first
    start = rest(z=0.0)
    up = planner.to_joints(start, (1.0, 0.0, 0.0))
    check_plan(model, planner, up, start)
    assert up.profiles["z_crate"].end == pytest.approx(1.0 - planner.crate_trail)
    start = {j: (p.end, 0.0) for j, p in up.profiles.items()}
    down = planner.to_joints(start, (0.1, 0.0, 0.0))
    check_plan(model, planner, down, start)
    with pytest.raises(PlanError, match="too low"):
        planner.to_joints(rest(), (0.0, 0.0, 0.0))


def test_new_goal_while_moving(model, planner):
    """Replacing a running motion: starts from its position and speed."""
    first = planner.to_point(rest(), FRUIT[0])
    for t in (0.3, 0.8, 1.2):
        start = first.state(t)
        assert any(abs(v) > 0.1 for _, v in start.values())
        plan = planner.to_point(start, FRUIT[3])
        check_plan(model, planner, plan, start)
        stop = planner.stop(start)
        check_plan(model, planner, stop, start)
