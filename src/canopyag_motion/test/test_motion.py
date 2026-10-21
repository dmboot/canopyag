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


def rest(z=0.05, j1=0.0, j2=0.0, crate=0.0, j3=0.0):
    return {"z_carriage": (z, 0.0), "joint_1": (j1, 0.0), "joint_2": (j2, 0.0), "z_crate": (crate, 0.0),
            "joint_3": (j3, 0.0)}


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
    for j, g in zip(("z_carriage", "joint_1", "joint_2", "joint_3"), plan.goal):
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
        p = np.array([rng.uniform(-0.5, 0.9), rng.uniform(-0.7, 0.7), rng.uniform(0.2, 0.75)])
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


def test_reach_box(model):
    lo, hi = model.reach_box()
    assert np.all(lo < hi)
    # Every point inverse() finds a free solution for lies inside the box.
    rng = np.random.default_rng(0)
    for p in lo - 0.1 + (hi - lo + 0.2) * rng.random((300, 3)):
        free = [q for q in model.inverse(p)
                if not model.colliding_pairs({"z_carriage": q[0], "joint_1": q[1],
                                              "joint_2": q[2], "z_crate": 0.0})]
        if free:
            assert np.all(p >= lo - 0.01) and np.all(p <= hi + 0.01), p


def test_joint_2_folded_has_both_turns(model):
    """joint_2 -pi..pi: link 2 folded under link 1 is joint_2 = -pi and = pi."""
    p = model.tool_position({"z_carriage": 0.3, "joint_1": 0.3, "joint_2": math.pi})
    j2 = sorted(round(q[2], 6) for q in model.inverse(p))
    assert abs(j2[0] + math.pi) < 1e-3 and abs(j2[-1] - math.pi) < 1e-3


def test_joint_3_does_not_move_the_tool_point(model):
    """The tool point is on joint_3's axis: turning the gripper only turns it."""
    q = {"z_carriage": 0.2, "joint_1": 0.4, "joint_2": -0.7}
    p = model.tool_position(q)
    for j3 in (-2.0, 1.0, math.pi):
        assert np.allclose(model.tool_position({**q, "joint_3": j3}), p, atol=1e-12)


def test_collision_boxes(model):
    def hit(j1, j2, j3=0):
        return bool(model.collisions({"z_carriage": 0.3, "z_crate": 0.15, "joint_1": DEG(j1),
                                      "joint_2": DEG(j2), "joint_3": DEG(j3)})[0])
    assert not hit(0, 0)
    assert not hit(0, 180)        # link 2 folded under link 1, arm in the middle
    assert not hit(60, 15)        # picking on the right
    assert hit(60, 165)           # folding out there swings the gripper into the gantry
    assert not hit(60, 165, 180)  # ... unless the gripper is turned round
    assert hit(120, 180)          # at 120 deg link 2 reaches the gantry too
    assert hit(120, 0)            # mid_link reaches the gantry at the joint_1 limit
    # the crate and the carriage are kept apart by the gap, not the boxes
    assert {"carriage", "crate"} not in [{a, b} for a, b, _ in model.pairs]
    assert not model.collisions({"z_carriage": 0.0, "z_crate": 0.0})[0]


def test_gripper_turns_freely_at_rest(model):
    """The gripper passes 2.5 cm under link 1. Both ride the carriage and never
    move up or down relative to each other, so the margin is not added in z
    between them - it is between the arm and the crate, which rides its own rail."""
    q = {"z_carriage": 0.05, "z_crate": 0.0, "joint_1": 0.0, "joint_2": 0.0}
    for j3 in np.radians(np.arange(-180, 181, 10)):
        assert not model.colliding_pairs({**q, "joint_3": j3}), math.degrees(j3)
    dz = {frozenset((a, b)): d for a, b, d in model.pairs}
    assert dz[frozenset(("end_effector", "mid_link"))] == pytest.approx(2 * model.margin)
    assert dz[frozenset(("crate", "end_effector"))] == 0.0


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

FRUIT = [(0.46, 0.55, 0.30), (0.28, 0.60, 0.36), (0.46, -0.55, 0.45), (0.28, -0.60, 0.51)]


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
        j2 = pick.goal[2] - math.pi if pick.goal[2] > 0.0 else pick.goal[2] + math.pi
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
    """Gripper turned 90 deg, out to the side with link 2 folding back: link 1
    is faster, and swinging out before link 2 has folded sweeps the gripper
    through the gantry - joint_1 waits a little for joint_2."""
    start = rest(z=0.3, j1=DEG(-40), j2=DEG(30), crate=0.15, j3=DEG(90))
    plan = planner.to_joints(start, (0.3, DEG(80), DEG(170)))
    check_plan(model, planner, plan, start)
    assert any("joint_1 waits" in n for n in plan.notes)
    # and it is still a simultaneous move, not one joint after the other
    assert plan.duration < sum(min_time(*start[j], g, *planner.limits[j]).duration
                               for j, g in (("joint_1", DEG(80)), ("joint_2", DEG(170))))


def test_stop_on_the_way(model, planner):
    """Gripper turned 90 deg, link 2 from -80 to 180 deg with link 1 out at
    70..80 deg: no wait helps, it swings in, turns, swings out."""
    start = rest(z=0.3, j1=DEG(80), j2=DEG(-80), crate=0.15, j3=DEG(90))
    plan = planner.to_joints(start, (0.3, DEG(70), DEG(180)))
    check_plan(model, planner, plan, start)
    assert any("stops at joint_1" in n for n in plan.notes)


def test_joint_3(model, planner):
    """A point goal leaves the gripper where it is; a fourth joint value turns it."""
    start = rest(j3=0.7)
    plan = planner.to_point(start, FRUIT[0])
    check_plan(model, planner, plan, start)
    assert plan.goal[3] == pytest.approx(0.7)
    plan = planner.to_joints(start, (0.2, DEG(40), DEG(30), DEG(-120)))
    check_plan(model, planner, plan, start)
    assert plan.profiles["joint_3"].end == pytest.approx(DEG(-120))
    with pytest.raises(PlanError, match="joint goal is"):
        planner.to_joints(start, (0.2, 0.0))


def test_out_of_reach_says_why(model, planner):
    far = model.j1_xy + [model.link1 + model.link2 + 0.01, 0.0]
    with pytest.raises(PlanError, match="from joint_1"):
        planner.to_point(rest(), [far[0], far[1], 0.5])
    # Within reach, but straight behind joint_1: past its limit.
    with pytest.raises(PlanError, match="joint_1 does not turn that far"):
        planner.to_point(rest(), [model.j1_xy[0] - 0.3, model.j1_xy[1], 0.5])


def test_goal_in_collision(planner):
    with pytest.raises(PlanError, match="collides at the goal"):
        planner.to_joints(rest(), (0.3, DEG(60), DEG(165)))


def test_crate_keeps_its_gap(model, planner):
    # from home (gap 0) up: the crate follows; down from high: the crate goes first
    start = rest(z=0.0)
    up = planner.to_joints(start, (0.4, 0.0, 0.0))
    check_plan(model, planner, up, start)
    assert up.profiles["z_crate"].end == pytest.approx(0.4 - planner.crate_trail)
    start = {j: (p.end, 0.0) for j, p in up.profiles.items()}
    down = planner.to_joints(start, (0.1, 0.0, 0.0))
    check_plan(model, planner, down, start)
    with pytest.raises(PlanError, match="too low"):
        planner.to_joints(rest(), (0.0, 0.0, 0.0))


def test_new_goal_while_moving(model, planner):
    """Replacing a running motion: starts from its position and speed."""
    first = planner.to_point(rest(), FRUIT[0])
    for f in (0.25, 0.5, 0.75):
        start = first.state(f * first.duration)
        assert any(abs(v) > 0.1 for _, v in start.values())
        plan = planner.to_point(start, FRUIT[3])
        check_plan(model, planner, plan, start)
        stop = planner.stop(start)
        check_plan(model, planner, stop, start)
