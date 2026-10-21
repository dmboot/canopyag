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
import yaml

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

import canopyag_motion  # noqa: E402
from canopyag_motion import ARM, HEADING, JOINTS, WRIST, Model, ModelError, PlanError  # noqa: E402
from canopyag_motion.profiles import min_time  # noqa: E402

PARAMS = PKG.parent / "canopyag_description" / "config" / "robot_parameters.yaml"
CONFIG = PKG / "config" / "motion.yaml"
DEG = math.radians
PLUS_Y, MINUS_Y = math.pi / 2, -math.pi / 2


@pytest.fixture(scope="module")
def loaded():
    return canopyag_motion.load(PARAMS, CONFIG)


@pytest.fixture(scope="module")
def model(loaded):
    return loaded[0]


@pytest.fixture(scope="module")
def planner(loaded):
    return loaded[1]


def model_with(change):
    """A Model of robot_parameters.yaml after change(params)."""
    params = yaml.safe_load(PARAMS.read_text())
    cfg = yaml.safe_load(CONFIG.read_text())
    change(params)
    return Model(params, cfg["tool"]["link"], cfg["tool"]["xyz"], cfg["collision"]["margin"],
                 cfg["collision"]["ignore"])


def planner_for(model, planner):
    return canopyag_motion.Planner(model, planner.limits, gap=planner.gap,
                                   crate_trail=planner.crate_trail)


def rest(z=0.05, j1=0.0, j2=0.0, crate=0.0, j3=0.0):
    return {"z_carriage": (z, 0.0), "joint_1": (j1, 0.0), "joint_2": (j2, 0.0), "z_crate": (crate, 0.0),
            "joint_3": (j3, 0.0)}


def pointing(model, heading, z=0.05, j1=0.0, j2=0.0, crate=0.0):
    """rest(), with joint_3 turning the gripper to heading (the turn nearest 0)."""
    j3 = min(model.turns(WRIST, float(model.wrist_for(heading, j1, j2))), key=abs)
    return rest(z, j1, j2, crate, j3)


def same_angle(a, b):
    return np.allclose(np.cos(np.asarray(a) - np.asarray(b)), 1.0, atol=1e-12)


def ends(plan):
    return {j: (v, 0.0) for j, (v, _) in plan.state(plan.duration).items()}


def check_plan(model, planner, plan, start, heading=None):
    """Starts at start, ends at rest at its goal, within the limits (joint_3
    too: it follows from the others), and never collides - checked 4x finer
    than the planner does. joint_3 holds the gripper at the plan's heading,
    and with `heading` given that is it all the way."""
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
    for j, g in zip(ARM + (WRIST,), plan.goal):
        assert abs(s[j][0][-1] - g) < 1e-9, j
    q = {j: s[j][0] for j in JOINTS}
    assert not model.collisions(q).any()
    gap = s["z_carriage"][0] - s["z_crate"][0]
    g0 = start["z_carriage"][0] - start["z_crate"][0]
    assert np.all(gap >= min(planner.gap, g0) - 1e-6)
    # The gripper's world yaw, from the link poses, is the plan's heading.
    yaw = model.link_poses(q)[model.tool_link][3]
    assert np.allclose(yaw, s[HEADING][0], atol=1e-9)
    if heading is not None:
        assert same_angle(yaw, heading)
    return s


# ---------------------------------------------------------------- kinematics

def test_inverse_round_trip(model):
    """The tip on the point, the gripper to +y or -y, every joint in its limits."""
    rng = np.random.default_rng(1)
    hit = 0
    for _ in range(300):
        p = np.array([rng.uniform(-0.5, 0.9), rng.uniform(-0.8, 0.8), rng.uniform(0.2, 0.75)])
        for heading in (PLUS_Y, MINUS_Y):
            for q in model.inverse(p, heading):
                hit += 1
                joints = dict(zip(ARM + (WRIST,), q))
                assert np.allclose(model.tool_position(joints), p, atol=1e-9)
                assert same_angle(model.heading_of(q[1], q[2], q[3]), heading)
                assert same_angle(model.link_poses(joints)[model.tool_link][3], heading)
                for j, v in joints.items():
                    lo, hi = model.limits(j)
                    assert lo - 1e-9 <= v <= hi + 1e-9, j
    assert hit > 200


def test_the_wrist_is_behind_the_tip(model):
    """The wrist axis sits the tip offset back from the point, against the heading."""
    p = np.array([0.55, 0.15, 0.40])
    for heading in (PLUS_Y, MINUS_Y):
        q = model.inverse(p, heading)[0]
        w = model.wrist_position(dict(zip(ARM + (WRIST,), q)))
        tip = math.hypot(model.tip[0], model.tip[1])
        assert np.allclose(w[:2], p[:2] - tip * np.array([0.0, math.sin(heading)]), atol=1e-9)


def test_out_of_reach(model):
    far = model.j1_xy + [model.link1 + model.link2 + 0.3, 0.0]
    assert model.inverse([far[0], far[1], 0.5], PLUS_Y) == []
    assert model.inverse([0.5, 0.5, 5.0], PLUS_Y) == []


def test_reach_box(model):
    lo, hi = model.reach_box()
    assert np.all(lo < hi)
    # Every point inverse() finds a free solution for lies inside the box.
    rng = np.random.default_rng(0)
    for p in lo - 0.1 + (hi - lo + 0.2) * rng.random((300, 3)):
        free = [q for h in (PLUS_Y, MINUS_Y) for q in model.inverse(p, h)
                if not model.colliding_pairs({"z_carriage": q[0], "joint_1": q[1],
                                              "joint_2": q[2], WRIST: q[3], "z_crate": 0.0})]
        if free:
            assert np.all(p >= lo - 0.01) and np.all(p <= hi + 0.01), p


def test_joint_2_folded_has_both_turns(model):
    """joint_2 -pi..pi: link 2 folded under link 1 is joint_2 = -pi and = pi."""
    q = pointing(model, PLUS_Y, z=0.3, j1=0.3, j2=math.pi)
    p = model.tool_position({j: v for j, (v, _) in q.items()})
    j2 = sorted(round(g[2], 6) for g in model.inverse(p, PLUS_Y))
    assert abs(j2[0] + math.pi) < 1e-3 and abs(j2[-1] - math.pi) < 1e-3


def test_joint_3_turns_the_tip_about_its_axis(model):
    q = {"z_carriage": 0.2, "joint_1": 0.4, "joint_2": -0.7}
    w = model.wrist_position(q)
    r = math.hypot(model.tip[0], model.tip[1])
    for j3 in (-2.0, 1.0, math.pi):
        tip = model.tool_position({**q, WRIST: j3})
        assert math.hypot(*(tip[:2] - w[:2])) == pytest.approx(r)
        assert same_angle(math.atan2(*(tip[:2] - w[:2])[::-1]), model.heading_of(0.4, -0.7, j3))


def test_only_the_planned_chain_must_be_vertical():
    """A pitch after the wrist may tilt (it is frozen at 0) and its link's box
    is checked like the others; a tilted arm joint is refused."""
    def add_pitch(params):
        params["links"]["finger"] = {"collision_box": {"size": [0.05, 0.03, 0.03],
                                                       "xyz": [0.03, 0.0, 0.0]}}
        params["joints"]["wrist_pitch"] = {
            "type": "revolute", "parent": "end_effector", "child": "finger",
            "xyz": [0.2, 0.0, -0.01], "rpy": [0.0, 0.0, 0.0], "axis": [0.0, 1.0, 0.0],
            "limit": {"lower": -1.0, "upper": 1.0, "effort": 1.0, "velocity": 1.0}}
    m = model_with(add_pitch)
    assert m.frozen == ["wrist_pitch"]
    assert any("finger" in (a, b) and "gantry" in (a, b) for a, b, _ in m.pairs)
    q = {"z_carriage": 0.2, "joint_1": 0.4, "joint_2": -0.7, WRIST: 0.3}
    assert np.allclose(m.link_poses(q)["finger"], m.link_poses({**q, "wrist_pitch": 0.8})["finger"])

    def tilt(params):
        params["joints"]["joint_1"]["axis"] = [0.0, 1.0, 0.0]
    with pytest.raises(ModelError, match="joint_1.*not vertical"):
        model_with(tilt)


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
    """The gripper passes 2.5 cm under link 1 and the carriage. They all ride
    the carriage and never move up or down relative to each other, so the
    margin is not added in z between them and those pairs are never checked;
    it is between the arm and the crate, which rides its own rail."""
    q = {"z_carriage": 0.05, "z_crate": 0.0, "joint_1": 0.0, "joint_2": 0.0}
    for j3 in np.radians(np.arange(-270, 271, 10)):
        assert not model.colliding_pairs({**q, "joint_3": j3}), math.degrees(j3)
    dz = {frozenset((a, b)): d for a, b, d in model.pairs}
    for link in ("mid_link", "carriage"):
        assert frozenset(("end_effector", link)) not in dz
    assert dz[frozenset(("crate", "end_effector"))] == 0.0
    assert frozenset(("end_effector", "gantry")) in dz


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


def test_stretched():
    """k times slower on the same path: velocity / k, acceleration / k^2."""
    pr, k = min_time(0.0, 0.0, 2.0, 1.0, 2.0), 1.6
    slow = pr.stretched(k)
    assert slow.duration == pytest.approx(k * pr.duration)
    t = np.linspace(0.0, pr.duration, 50)
    p, v, a = pr.sample(t)
    ps, vs, as_ = slow.sample(k * t)
    assert np.allclose(ps, p) and np.allclose(vs, v / k) and np.allclose(as_, a / k ** 2)


# ---------------------------------------------------------------- planner

FRUIT = [(0.46, 0.55, 0.30), (0.28, 0.60, 0.36), (0.46, -0.55, 0.45), (0.28, -0.60, 0.51)]


@pytest.mark.parametrize("point", FRUIT)
def test_plan_to_point(model, planner, point):
    """From home (gripper along link 2) the gripper turns to the fruit's side
    on the way and points there at the end, tip on the fruit."""
    start = rest()
    plan = planner.to_point(start, point)
    side = math.copysign(math.pi / 2, point[1])
    check_plan(model, planner, plan, start)
    assert same_angle(plan.profiles[HEADING].end, side)
    q = dict(zip(ARM + (WRIST,), plan.goal))
    assert np.allclose(model.tool_position(q), point, atol=1e-9)


@pytest.mark.parametrize("side", [1, -1])
def test_pick_drop_next(model, planner, side):
    """The demo's moves: approach, pick, fold link 2 under link 1, next fruit -
    all joints at once, collision free, the gripper to the fruit side all the
    way once it is there."""
    heading = side * math.pi / 2
    start = pointing(model, heading)
    for x, y, z in FRUIT[:2]:
        for point in ((x, side * (y - 0.06), z), (x, side * y, z)):
            pick = planner.to_point(start, point, heading=side)
            check_plan(model, planner, pick, start, heading)
            start = ends(pick)
        j2 = pick.goal[2] - math.pi if pick.goal[2] > 0.0 else pick.goal[2] + math.pi
        drop = planner.to_joints(start, (pick.goal[0], 0.0, j2), heading=side)
        check_plan(model, planner, drop, start, heading)
        start = ends(drop)


def test_joint_3_holds_the_heading(model, planner):
    """Along a sampled plan joint_3 = heading - joint_1 - joint_2 (minus the
    fixed turns), and the gripper does not turn in the world."""
    start = pointing(model, PLUS_Y, z=0.2, j1=DEG(30), j2=DEG(-40), crate=0.05)
    plan = planner.to_point(start, FRUIT[1])
    s = check_plan(model, planner, plan, start, PLUS_Y)
    assert np.allclose(s[HEADING][0], s[HEADING][0][0]) and np.allclose(s[HEADING][1], 0.0)
    assert np.allclose(s[WRIST][0], model.wrist_for(s[HEADING][0], s["joint_1"][0], s["joint_2"][0]))
    assert np.ptp(s[WRIST][0]) > 2.0        # joint_3 turned most of a half turn


def test_simultaneous(model, planner):
    """No wait needed: every joint starts at once and the move takes as long
    as its slowest joint alone."""
    start = pointing(model, PLUS_Y)
    plan = planner.to_joints(start, (0.3, DEG(40), DEG(30)), heading=1)
    check_plan(model, planner, plan, start, PLUS_Y)
    slowest = max(min_time(*start[j], g, *planner.limits[j]).duration
                  for j, g in zip(ARM, (0.3, DEG(40), DEG(30))))
    assert plan.duration == pytest.approx(slowest)
    for j in ARM:
        assert plan.profiles[j].phases[0][1] != 0.0, f"{j} does not start at once"


def test_wait_instead_of_collision(model, planner):
    """Out to the side, link 2 folding all the way back: link 1 is faster, and
    swinging out before link 2 has folded sweeps the gripper through the
    gantry - joint_1 waits a little for joint_2."""
    start = pointing(model, PLUS_Y, z=0.3, j1=DEG(-40), crate=0.15)
    plan = planner.to_joints(start, (0.3, DEG(80), DEG(180)), heading=1)
    check_plan(model, planner, plan, start, PLUS_Y)
    assert any("joint_1 waits" in n for n in plan.notes)
    # and it is still a simultaneous move, not one joint after the other
    assert plan.duration < sum(min_time(*start[j], g, *planner.limits[j]).duration
                               for j, g in (("joint_1", DEG(80)), ("joint_2", DEG(180))))


def test_stop_on_the_way(model, planner):
    """Link 1 out at -80 deg, link 2 from -60 to -180 deg: no wait helps, it
    swings in, turns, swings out."""
    start = pointing(model, PLUS_Y, z=0.3, j1=DEG(-80), j2=DEG(-60), crate=0.15)
    plan = planner.to_joints(start, (0.3, DEG(-80), DEG(-180)), heading=1)
    check_plan(model, planner, plan, start, PLUS_Y)
    assert any("stops at joint_1" in n for n in plan.notes)


def test_side_switch(model, planner):
    """Picked on +y, then a fruit on -y: the heading turns half a turn, all
    in one move, collision free; joint_3 runs within its limits."""
    start = ends(planner.to_point(rest(), FRUIT[0]))
    plan = planner.to_point(start, FRUIT[2])
    s = check_plan(model, planner, plan, start)
    h = s[HEADING][0]
    assert same_angle(h[0], PLUS_Y) and same_angle(h[-1], MINUS_Y)
    assert abs(h[-1] - h[0]) == pytest.approx(math.pi)      # the short way round
    assert np.all(np.diff(h) * np.sign(h[-1] - h[0]) >= -1e-12)
    # The same arm pose, the other way: a joint goal with the heading chosen.
    back = planner.to_joints(ends(plan), plan.goal[:3], heading=1)
    check_plan(model, planner, back, ends(plan))
    assert same_angle(back.profiles[HEADING].end, PLUS_Y)


def test_time_stretch_keeps_joint_3_in_its_limits(model, planner):
    """Across to the other side at full speed, joint_3 would turn faster than
    it may: the whole plan runs slower, by just enough."""
    start = ends(planner.to_point(rest(), FRUIT[1]))
    plan = planner.to_point(start, FRUIT[3])
    note = [n for n in plan.notes if n.startswith("slowed to")]
    assert note, plan.notes
    s = check_plan(model, planner, plan, start)        # within joint_3's limits
    vmax, amax = planner.limits[WRIST]
    t = np.unique(np.concatenate([plan.profiles[j].t for j in ("joint_1", "joint_2", HEADING)]))
    _, v, a = plan.sample(t)[WRIST]
    use = max(np.abs(v).max() / vmax, math.sqrt(np.abs(a).max() / amax))
    assert use == pytest.approx(1.0, abs=1e-6)          # at the limit: no slower than needed
    # every joint slowed alike: the rail too is under its own limits
    assert np.abs(s["z_carriage"][1]).max() < planner.limits["z_carriage"][0] + 1e-9


def test_joint_3_out_of_limits_is_out_of_reach(model, planner):
    """With joint_3 limited to +-0.5 rad the gripper cannot point to +y from
    most arm poses: no solution, and the planner says it is joint_3."""
    def narrow(params):
        params["joints"][WRIST]["limit"].update(lower=-0.5, upper=0.5)
    m = model_with(narrow)
    p = planner_for(m, planner)
    point = FRUIT[0]
    assert m.inverse(point, PLUS_Y) == []
    assert m.inverse(point, PLUS_Y, wrist_limits=False)
    with pytest.raises(PlanError, match="joint_3 does not turn that far"):
        p.to_point(rest(), point, heading=1)
    with pytest.raises(PlanError, match="joint_3 cannot turn the gripper"):
        p.to_joints(rest(), (0.3, DEG(20), DEG(20)), heading=1)
    with pytest.raises(PlanError, match="joint_3 = .* outside its limits"):
        p.to_joints(rest(), (0.3, DEG(20), DEG(20), 1.0))


def test_heading_choice(model, planner):
    """auto: the side the goal is on; or chosen per goal. A fourth joint value
    is joint_3 itself; without it joint_3 points the gripper."""
    on_plus = (0.55, 0.15, 0.4)
    assert planner.heading(0, on_plus[1]) == PLUS_Y
    assert planner.heading(0, -0.15) == MINUS_Y
    assert planner.heading(-1, on_plus[1]) == MINUS_Y
    # on joint_1's axis: the side it points to now
    assert planner.heading(0, model.j1_xy[1], pointing(model, MINUS_Y)) == MINUS_Y
    plan = planner.to_point(rest(), on_plus, heading=-1)
    check_plan(model, planner, plan, rest())
    assert same_angle(plan.profiles[HEADING].end, MINUS_Y)
    assert np.allclose(model.tool_position(dict(zip(ARM + (WRIST,), plan.goal))), on_plus)
    plan = planner.to_joints(rest(), (0.2, DEG(40), DEG(30), DEG(-120)))
    check_plan(model, planner, plan, rest())
    assert plan.goal[3] == pytest.approx(DEG(-120))
    plan = planner.to_joints(rest(), (0.2, DEG(40), DEG(30)), heading=1)
    assert same_angle(model.heading_of(*plan.goal[1:]), PLUS_Y)
    with pytest.raises(PlanError, match="joint goal is"):
        planner.to_joints(rest(), (0.2, 0.0))
    with pytest.raises(PlanError, match="heading must be"):
        planner.to_point(rest(), on_plus, heading=2)


def test_default_heading_from_motion_yaml(model, planner):
    fixed = canopyag_motion.Planner(model, planner.limits, heading="-y")
    assert fixed.heading(0, 0.5) == MINUS_Y and fixed.heading(1, 0.5) == PLUS_Y
    with pytest.raises(ValueError, match="heading must be one of"):
        canopyag_motion.Planner(model, planner.limits, heading="left")


def test_out_of_reach_says_why(model, planner):
    far = model.j1_xy + [model.link1 + model.link2 + 0.3, 0.0]
    with pytest.raises(PlanError, match="from joint_1"):
        planner.to_point(rest(), [far[0], far[1], 0.5])
    # Within reach, but straight behind joint_1: past its limit.
    with pytest.raises(PlanError, match="joint_1 does not turn that far"):
        planner.to_point(rest(), [model.j1_xy[0] - 0.3, model.j1_xy[1] + 0.21, 0.5], heading=1)


def test_goal_in_collision(planner):
    # joint_3 at 0 there points the gripper back into the gantry
    with pytest.raises(PlanError, match="collides at the goal"):
        planner.to_joints(rest(), (0.3, DEG(60), DEG(165), 0.0))


def test_crate_stays_at_zero(model, planner):
    """motion.yaml crate.follow is off: arm moves leave the crate at home."""
    assert not planner.crate_follow
    start = rest(z=0.0)
    for plan, goal in ((planner.to_joints, (0.4, 0.0, 0.0)), (planner.to_point, FRUIT[0]),
                       (planner.to_joints, (0.1, 0.0, 0.0))):
        move = plan(start, goal)
        check_plan(model, planner, move, start)
        assert move.profiles["z_crate"].end == 0.0
        assert move.profiles["z_crate"].duration == 0.0
        start = ends(move)
    with pytest.raises(PlanError, match="too low"):
        planner.to_joints(rest(), (0.0, 0.0, 0.0))


def test_crate_follows_when_told(model, planner):
    """crate.follow on: the crate goes crate_trail below the carriage; down
    from high, the crate goes first."""
    follow = canopyag_motion.Planner(model, planner.limits, gap=planner.gap,
                                     crate_trail=planner.crate_trail, crate_follow=True)
    start = rest(z=0.0)
    up = follow.to_joints(start, (0.4, 0.0, 0.0))
    check_plan(model, follow, up, start)
    assert up.profiles["z_crate"].end == pytest.approx(0.4 - follow.crate_trail)
    start = ends(up)
    down = follow.to_joints(start, (0.1, 0.0, 0.0))
    check_plan(model, follow, down, start)


def test_crate_goal(model, planner):
    """A crate goal (the panel's slider) moves only the crate, and only to
    where it stays the gap below the carriage."""
    start = pointing(model, PLUS_Y, z=0.4)
    arm = np.array([start[j][0] for j in ARM + (WRIST,)])
    up = planner.to_joints(start, arm, crate=0.3)
    check_plan(model, planner, up, start)
    assert up.profiles["z_crate"].end == pytest.approx(0.3)
    for j in ARM + (WRIST,):
        assert np.ptp(up.sample(np.linspace(0, up.duration, 20))[j][0]) == 0.0, j
    with pytest.raises(PlanError, match="too high"):
        planner.to_joints(start, arm, crate=0.4 - planner.gap + 0.001)
    with pytest.raises(PlanError, match="outside its limits"):
        planner.to_joints(start, arm, crate=-0.1)
    # The crate stays where it was put, until the carriage comes down onto it:
    # then it is pushed down, ahead of the carriage.
    start = ends(up)
    pick = planner.to_point(start, FRUIT[1])                   # carriage at 0.1
    check_plan(model, planner, pick, start)
    assert pick.profiles["z_crate"].end == pytest.approx(pick.goal[0] - planner.gap)
    # With an arm goal: both at once.
    both = planner.to_point(rest(z=0.3), FRUIT[0], crate=0.02)
    check_plan(model, planner, both, rest(z=0.3))
    assert both.profiles["z_crate"].end == pytest.approx(0.02)


def test_new_goal_while_moving(model, planner):
    """Replacing a running motion: starts from its position and speed. The
    stop brakes joint_1, joint_2 and the heading together: joint_3 within
    its limits."""
    first = planner.to_point(rest(), FRUIT[0])
    for f in (0.25, 0.5, 0.75):
        start = first.state(f * first.duration)
        assert any(abs(v) > 0.1 for _, v in start.values())
        plan = planner.to_point(start, FRUIT[3])
        check_plan(model, planner, plan, start)
        stop = planner.stop(start)
        check_plan(model, planner, stop, start)
