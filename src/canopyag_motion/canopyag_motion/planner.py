"""Plans a move to a world point or to joint values, as fast as the limits allow.

There are no obstacles yet, only the robot itself, so "planning" is choosing
how the joints move at once:

  * every joint runs its own time-optimal profile, so they all move at the
    same time and each at its own top speed;
  * if that path makes link 2 sweep through the carriage or the gantry, one
    revolute joint waits a little before it starts (joint_1 swings in first,
    then joint_2 turns; or joint_2 folds back first, then joint_1 swings out).
    Of all the waits that clear the collision boxes, the shortest total wins;
  * if no wait helps (link 2 has to turn past 180 deg while joint_1 starts and
    ends far out), the arm goes through a stop at an intermediate joint_1;
  * a point has up to two arm solutions (elbow either way, and joint_2 at 0 or
    2 pi): each is planned and the fastest collision-free one is used.

The carriage and the crate ride the same rail. The crate goes to
`crate_trail` below the carriage's goal (or only as low as needed when
`crate_follow` is off) and the carriage-crate gap never drops below `gap`:
whichever must not catch up waits.
"""

import math
from dataclasses import dataclass, field

import numpy as np

from .model import ARM, CRATE, JOINTS
from .profiles import EPS_V, Profile, hold, min_time


class PlanError(ValueError):
    pass


@dataclass
class Plan:
    profiles: dict                     # joint -> Profile, all starting at t = 0
    duration: float
    goal: np.ndarray                   # [z_carriage, joint_1, joint_2]
    notes: list = field(default_factory=list)

    def sample(self, t):
        """joint -> (position, velocity, acceleration) at t (array)."""
        return {j: p.sample(t) for j, p in self.profiles.items()}

    def state(self, t):
        """joint -> (position, velocity) at time t."""
        out = {}
        for j, p in self.profiles.items():
            pos, vel, _ = p.sample(t)
            out[j] = (float(pos), float(vel))
        return out


class Planner:

    def __init__(self, model, limits, gap=0.02, crate_trail=0.15, crate_follow=True,
                 check_step=0.01, delay_step=0.02):
        """limits: joint -> (max_velocity, max_acceleration). check_step: m any
        point of the arm moves between two collision checks."""
        self.model = model
        self.limits = {j: (float(v), float(a)) for j, (v, a) in limits.items()}
        for j in JOINTS:
            if j not in self.limits:
                raise ValueError(f"no velocity / acceleration limit for {j}")
        self.gap = float(gap)
        self.crate_trail = float(crate_trail)
        self.crate_follow = bool(crate_follow)
        # The fastest any point of the arm moves, at the joint speed limits.
        v1, v2 = self.limits["joint_1"][0], self.limits["joint_2"][0]
        v_arm = max(v1 * model.reach1 + v2 * model.reach2, self.limits["z_carriage"][0])
        self.check_dt = float(check_step) / v_arm
        # Once the arm stands still only the rail moves, much slower.
        v_rail = max(self.limits["z_carriage"][0], self.limits[CRATE][0])
        self.rail_dt = float(check_step) / v_rail
        self.delay_step = float(delay_step)

    # ------------------------------------------------------------------ public

    def to_point(self, start, point, speed=1.0):
        """start: joint -> (position, velocity). Tool point to `point` (world)."""
        return self._from(start, lambda st: self._point(st, point, speed))

    def to_joints(self, start, goal, speed=1.0):
        """Arm to [z_carriage, joint_1, joint_2]."""
        return self._from(start, lambda st: self._joints(st, goal, speed))

    def _from(self, start, plan):
        """plan(start), or - if the robot is moving and there is no way on
        from there - brake every joint to a stop first and plan from that."""
        start = {j: (float(p), float(v)) for j, (p, v) in start.items()}
        try:
            return plan(start)
        except PlanError:
            if all(abs(v) <= EPS_V for _, v in start.values()):
                raise
        stop = self.stop(start)
        after = plan({j: (pr.end, 0.0) for j, pr in stop.profiles.items()})
        profiles = {}
        for j, pr in stop.profiles.items():
            wait = Profile(pr.p0, pr.v0, pr.phases + [(stop.duration - pr.duration, 0.0)], pr.end)
            profiles[j] = wait.then(after.profiles[j])
        return Plan(profiles, stop.duration + after.duration, after.goal,
                    ["stops first"] + after.notes)

    def stop(self, start):
        """Every joint brakes to a stop at its full acceleration."""
        stop = {}
        for j, (p, v) in start.items():
            a = self.limits[j][1]
            stop[j] = min_time(p, v, p + v * abs(v) / (2.0 * a), self.limits[j][0], a)
        rail = {j: stop[j] for j in ("z_carriage", CRATE)}
        if not self._free((stop["joint_1"], stop["joint_2"]), rail):
            raise PlanError("the robot is moving and cannot stop without a collision")
        goal = np.array([stop[j].end for j in ARM])
        return Plan(stop, max(pr.duration for pr in stop.values()), goal, ["stop"])

    def _point(self, start, point, speed):
        goals = self.model.inverse(point)
        if not goals:
            x, y, z = (float(v) for v in point)
            r = math.hypot(x - self.model.j1_xy[0], y - self.model.j1_xy[1])
            lo, hi = abs(self.model.link1 - self.model.link2), self.model.link1 + self.model.link2
            zlo, zhi = self.model.limits("z_carriage")
            raise PlanError(
                f"({x:.3f}, {y:.3f}, {z:.3f}) is out of reach: the tool reaches "
                f"{lo:.3f}..{hi:.3f} m from joint_1 (this is {r:.3f} m) and z "
                f"{zlo + self.model.tool_z0:.3f}..{zhi + self.model.tool_z0:.3f} m")
        return self._best(start, goals, speed)

    def _joints(self, start, goal, speed):
        goal = np.asarray(goal, float)
        if goal.shape != (3,):
            raise PlanError("a joint goal is [z_carriage, joint_1, joint_2]")
        for j, v in zip(ARM, goal):
            lo, hi = self.model.limits(j)
            if not lo - 1e-9 <= v <= hi + 1e-9:
                raise PlanError(f"{j} = {v:.4f} is outside its limits {lo:.4f} .. {hi:.4f}")
        return self._best(start, [goal], speed)

    # ------------------------------------------------------------------ planning

    def _best(self, start, goals, speed):
        if not 0.0 < speed <= 1.0:
            raise PlanError(f"speed must be in (0, 1], got {speed}")
        lim = {j: (v * speed, a * speed) for j, (v, a) in self.limits.items()}
        rail, rail_notes = self._rail(start, goals[0][0], lim)
        why, free = [], []
        for goal in goals:
            q_goal = {"z_carriage": goal[0], "joint_1": goal[1], "joint_2": goal[2],
                      CRATE: rail[CRATE].end}
            hit = self.model.colliding_pairs(q_goal)
            if hit:
                why.append(f"joint_1 {goal[1]:+.3f} joint_2 {goal[2]:.3f}: "
                           f"collides at the goal ({', '.join(hit)})")
            else:
                free.append(goal)
        # All joints at once (with waits) first; a stop on the way only if no
        # arm solution can do without - it is never faster.
        def bound(goal):        # no plan to this goal can be faster than this
            return max(min_time(*start[j], v, *lim[j]).duration
                       for j, v in zip(("joint_1", "joint_2"), goal[1:]))
        free.sort(key=bound)
        best = None
        for method in (self._arm_direct, self._arm_via):
            for goal in free:
                if best is not None and bound(goal) >= best.duration:
                    continue
                arm = method(start, goal, lim, rail)
                if arm is None:
                    continue
                profiles, notes = arm
                profiles.update(rail)
                duration = max(p.duration for p in profiles.values())
                if best is None or duration < best.duration - 1e-6:
                    best = Plan(profiles, duration, np.array(goal, float), rail_notes + notes)
            if best is not None:
                return best
        for goal in free:
            why.append(f"joint_1 {goal[1]:+.3f} joint_2 {goal[2]:.3f}: no collision-free way")
        raise PlanError("no plan: " + "; ".join(why))

    def _times(self, profiles, fine_until=None):
        """Sample times to the end of the longest profile: every check_dt up to
        fine_until (default: the end), every rail_dt after it."""
        end = max(p.duration for p in profiles)
        fine = end if fine_until is None else min(fine_until, end)
        t = np.linspace(0.0, fine, max(int(math.ceil(fine / self.check_dt)), 1) + 1)
        if end > fine:
            n = max(int(math.ceil((end - fine) / self.rail_dt)), 1)
            t = np.concatenate([t, np.linspace(fine, end, n + 1)[1:]])
        return t

    def _free(self, arm, rail):
        """True if arm (joint_1, joint_2 profiles) + rail never collide."""
        t = self._times(list(arm) + list(rail.values()), max(p.duration for p in arm))
        q = {"joint_1": arm[0].sample(t)[0], "joint_2": arm[1].sample(t)[0]}
        for j, p in rail.items():
            q[j] = p.sample(t)[0]
        return not self.model.collisions(q).any()

    def _free_many(self, arms, rail):
        """_free for several (joint_1, joint_2) options at once: bool per option."""
        t = self._times([p for arm in arms for p in arm] + list(rail.values()),
                        max(p.duration for arm in arms for p in arm))
        q = {"joint_1": np.concatenate([a[0].sample(t)[0] for a in arms]),
             "joint_2": np.concatenate([a[1].sample(t)[0] for a in arms])}
        for j, p in rail.items():
            q[j] = np.tile(p.sample(t)[0], len(arms))
        return ~self.model.collisions(q).reshape(len(arms), t.size).any(axis=1)

    def _waits(self, a, b, a_can_wait, b_can_wait):
        """(delay_a, delay_b) options, shortest total time first."""
        longest = max(a.duration, b.duration)
        opts = [(0.0, 0.0)]
        k = 1
        while k * self.delay_step <= longest + self.delay_step:
            d = k * self.delay_step
            if a_can_wait:
                opts.append((d, 0.0))
            if b_can_wait:
                opts.append((0.0, d))
            k += 1
        return sorted(opts, key=lambda o: (max(o[0] + a.duration, o[1] + b.duration), sum(o)))

    def _arm_direct(self, start, goal, lim, rail):
        """joint_1 and joint_2 at once, one of them maybe waiting. Or None."""
        j1, j2 = "joint_1", "joint_2"
        p1 = min_time(*start[j1], goal[1], *lim[j1])
        p2 = min_time(*start[j2], goal[2], *lim[j2])
        rest1, rest2 = abs(start[j1][1]) <= EPS_V, abs(start[j2][1]) <= EPS_V
        opts = self._waits(p1, p2, rest1, rest2)
        for k in range(0, len(opts), 16):      # 16 options per numpy call
            chunk = [(p1.delayed(d1), p2.delayed(d2)) for d1, d2 in opts[k:k + 16]]
            ok = np.flatnonzero(self._free_many(chunk, rail))
            if ok.size == 0:
                continue
            d1, d2 = opts[k + ok[0]]
            arm = chunk[ok[0]]
            notes = []
            if d1 > 0:
                notes.append(f"joint_1 waits {d1:.2f} s for joint_2")
            if d2 > 0:
                notes.append(f"joint_2 waits {d2:.2f} s for joint_1")
            return {j1: arm[0], j2: arm[1]}, notes
        return None

    def _arm_via(self, start, goal, lim, rail):
        """Through a stop at an intermediate joint_1 (joint_2 still where it
        started): first swing there, then the rest with waits. Or None."""
        j1, j2 = "joint_1", "joint_2"
        if abs(start[j1][1]) > EPS_V or abs(start[j2][1]) > EPS_V:
            return None
        lo, hi = self.model.limits(j1)
        best = None
        for via1 in np.radians(np.arange(-180.0, 180.1, 5.0)):
            if not lo <= via1 <= hi:
                continue
            leg1 = (min_time(start[j1][0], 0.0, via1, *lim[j1]), hold(start[j2][0]))
            if not self._free(leg1, self._shift(rail, 0.0)):
                continue
            t1 = leg1[0].duration
            a = min_time(via1, 0.0, goal[1], *lim[j1])
            b = min_time(start[j2][0], 0.0, goal[2], *lim[j2])
            if best is not None and t1 + max(a.duration, b.duration) >= best[0]:
                continue
            for d1, d2 in self._waits(a, b, True, True):
                total = t1 + max(d1 + a.duration, d2 + b.duration)
                if best is not None and total >= best[0]:
                    break
                leg2 = (a.delayed(d1), b.delayed(d2))
                if self._free(leg2, self._shift(rail, t1)):
                    best = (total, leg1, leg2, via1)
                    break
        if best is None:
            return None
        _, leg1, leg2, via1 = best
        t1 = leg1[0].duration
        return ({j1: leg1[0].then(leg2[0]), j2: leg2[1].delayed(t1)},
                [f"stops at joint_1 = {math.degrees(via1):.0f} deg on the way"])

    @staticmethod
    def _shift(rail, t0):
        """The rail profiles as seen from t0 on (to check a later leg)."""
        return {j: p.after(t0) for j, p in rail.items()}

    def _rail(self, start, z_goal, lim):
        """Profiles for z_carriage and z_crate with the gap kept, + notes."""
        zc, cr = "z_carriage", CRATE
        lo, hi = self.model.limits(cr)
        c_now = start[cr][0]
        if self.crate_follow:
            c_goal = min(max(z_goal - self.crate_trail, lo), hi)
        else:
            c_goal = c_now
        c_goal = min(c_goal, z_goal - self.gap)
        if c_goal < lo - 1e-9:
            raise PlanError(
                f"z_carriage {z_goal:.3f} is too low: the crate (lowest {lo:.3f}) has to stay "
                f"{self.gap:.3f} m below it")
        c_goal = max(c_goal, lo)
        pz = min_time(*start[zc], z_goal, *lim[zc])
        pc = min_time(*start[cr], c_goal, *lim[cr])
        gap0 = start[zc][0] - c_now
        need = min(self.gap, gap0) - 1e-6
        for dz, dc in self._waits(pz, pc, abs(start[zc][1]) <= EPS_V, abs(start[cr][1]) <= EPS_V):
            a, b = pz.delayed(dz), pc.delayed(dc)
            t = self._times([a, b])
            if (a.sample(t)[0] - b.sample(t)[0] >= need).all():
                notes = []
                if dz > 0:
                    notes.append(f"carriage waits {dz:.2f} s for the crate")
                return {zc: a, cr: b}, notes
        raise PlanError("cannot keep the carriage-crate gap on the way (moving start?)")
