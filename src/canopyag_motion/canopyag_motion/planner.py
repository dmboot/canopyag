"""Plans a move to a world point or to joint values, as fast as the limits allow.

There are no obstacles yet, only the robot itself, so "planning" is choosing
how the joints move at once:

  * every joint runs its own time-optimal profile, so they all move at the
    same time and each at its own top speed;
  * if that path makes link 2 or the gripper sweep through the carriage or
    the gantry, one revolute joint waits a little before it starts (joint_1
    swings in first, then joint_2 turns; or joint_2 folds back first, then
    joint_1 swings out). Of all the waits that clear the collision boxes, the
    shortest total wins;
  * if no wait helps (link 2 has to turn past 180 deg while joint_1 starts and
    ends far out), the arm goes through a stop at an intermediate joint_1;
  * a point has several arm solutions (elbow either way, joint_2 at -pi or
    pi, joint_3 one turn or the other): each is planned and the fastest
    collision-free one is used.

The gripper points along world +y or -y (the heading: auto = the side of the
goal, or chosen per goal). joint_3 is no profile of its own: at every instant
it is heading - joint_1 - joint_2 (minus the fixed turns), so the gripper
keeps its heading through the whole move. If the heading has to change (the
other side, or a start at any other yaw), the heading itself gets a
time-optimal profile, at joint_3's limits. joint_3's velocity and
acceleration follow from those of joint_1, joint_2 and the heading; if they
come out over joint_3's limits the whole plan runs slower, uniformly in time,
by the smallest factor that fits. A plan whose joint_3 leaves its limits on
the way is not used.

The carriage and the crate ride the same rail. The crate goes where a goal
sends it (`crate`), or else to `crate_trail` below the carriage's goal
(`crate_follow` on) or nowhere (off: it only moves down when the carriage
would come too close). The carriage-crate gap never drops below `gap`:
whichever must not catch up waits.
"""

import itertools
import math
from dataclasses import dataclass, field

import numpy as np

from .model import ARM, CRATE, HEADING, JOINTS, WRIST
from .profiles import EPS_V, Profile, hold, min_time

# MoveTo.heading and motion.yaml heading.default.
HEADINGS = {"auto": 0, "+y": 1, "-y": -1}
# auto: closer than this to joint_1's axis in y, the gripper stays on its side.
SIDE_TIE = 0.01     # m


class PlanError(ValueError):
    pass


@dataclass
class Plan:
    profiles: dict                     # z_carriage, z_crate, joint_1, joint_2, HEADING -> Profile,
                                       # all starting at t = 0
    duration: float
    goal: np.ndarray                   # [z_carriage, joint_1, joint_2, joint_3]
    model: object                      # works out joint_3 from the profiles
    notes: list = field(default_factory=list)

    def sample(self, t):
        """joint -> (position, velocity, acceleration) at t (array), joint_3
        and the HEADING included."""
        out = {j: p.sample(t) for j, p in self.profiles.items()}
        out[WRIST] = self.model.wrist_motion(out[HEADING], out["joint_1"], out["joint_2"])
        return out

    def state(self, t):
        """joint -> (position, velocity) at time t."""
        s = self.sample(t)
        return {j: (float(s[j][0]), float(s[j][1])) for j in JOINTS}


class Planner:

    def __init__(self, model, limits, gap=0.02, crate_trail=0.15, crate_follow=True,
                 check_step=0.01, delay_step=0.02, heading="auto"):
        """limits: joint -> (max_velocity, max_acceleration). check_step: m any
        point of the arm moves between two collision checks. heading: the
        default for goals that do not choose one ("auto", "+y", "-y")."""
        self.model = model
        self.limits = {j: (float(v), float(a)) for j, (v, a) in limits.items()}
        for j in JOINTS:
            if j not in self.limits:
                raise ValueError(f"no velocity / acceleration limit for {j}")
        if heading not in HEADINGS:
            raise ValueError(f"heading must be one of {', '.join(HEADINGS)}, got {heading!r}")
        self.default_heading = HEADINGS[heading]
        self.gap = float(gap)
        self.crate_trail = float(crate_trail)
        self.crate_follow = bool(crate_follow)
        # The fastest any point of the arm moves, at the joint speed limits.
        # The gripper turns in the world at the heading's speed, at most joint_3's.
        v1, v2 = self.limits["joint_1"][0], self.limits["joint_2"][0]
        v_wrist = self.limits[WRIST][0] * model.reach3
        v_arm = max(v1 * model.reach1 + v2 * model.reach2 + v_wrist, self.limits["z_carriage"][0])
        self.check_dt = float(check_step) / v_arm
        # Once the arm stands still only the rail (and the gripper) moves, much slower.
        v_rail = max(self.limits["z_carriage"][0], self.limits[CRATE][0], v_wrist)
        self.rail_dt = float(check_step) / v_rail
        self.delay_step = float(delay_step)

    # ------------------------------------------------------------------ public

    def to_point(self, start, point, speed=1.0, heading=0, crate=None):
        """start: joint -> (position, velocity). Tool point to `point` (world).
        heading: +1 the gripper to +y, -1 to -y, 0 the default (motion.yaml).
        crate: z_crate to go to, or None (see _rail)."""
        return self._from(start, lambda st: self._point(st, point, speed, heading, crate))

    def to_joints(self, start, goal, speed=1.0, heading=0, crate=None):
        """Arm to [z_carriage, joint_1, joint_2, joint_3], or [z_carriage,
        joint_1, joint_2] with joint_3 holding the gripper at `heading`.
        For a crate-only move: the arm's current goal (4 values) and `crate`."""
        return self._from(start, lambda st: self._joints(st, goal, speed, heading, crate))

    def heading(self, choice, y, start=None):
        """World yaw for the gripper, pi/2 (+y) or -pi/2 (-y). choice +1 / -1;
        0 is the default, and auto is the side of joint_1's axis that y (of
        the goal) is on - on the axis, the side the gripper points to now."""
        choice = int(choice) or self.default_heading
        if choice not in (-1, 0, 1):
            raise PlanError(f"heading must be 0 (default), 1 (+y) or -1 (-y), got {choice}")
        if choice == 0:
            dy = float(y) - self.model.j1_xy[1]
            if abs(dy) > SIDE_TIE:
                choice = 1 if dy > 0 else -1
            elif start is not None:
                choice = -1 if math.sin(self._heading_now(start)[0]) < 0 else 1
            else:
                choice = 1
        return math.copysign(math.pi / 2, choice)

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
        after = plan(stop.state(stop.duration))
        profiles = {}
        for j, pr in stop.profiles.items():
            wait = Profile(pr.p0, pr.v0, pr.phases + [(stop.duration - pr.duration, 0.0)], pr.end)
            profiles[j] = wait.then(after.profiles[j])
        return Plan(profiles, stop.duration + after.duration, after.goal, self.model,
                    ["stops first"] + after.notes)

    def stop(self, start):
        """Every joint brakes to a stop, the rail at its full acceleration.
        joint_1, joint_2 and the heading brake together, in the same time, so
        joint_3 slows down evenly too and stays within its limits."""
        stop = {}
        for j in ("z_carriage", CRATE):
            p, v = start[j]
            a = self.limits[j][1]
            stop[j] = min_time(p, v, p + v * abs(v) / (2.0 * a), self.limits[j][0], a)
        arm = {"joint_1": start["joint_1"], "joint_2": start["joint_2"],
               HEADING: self._heading_now(start)}
        brake = max([abs(v) / self.limits[j][1] for j, (_, v) in arm.items() if j != HEADING] +
                    [abs(v) / self.limits[WRIST][1] for v in (arm[HEADING][1], start[WRIST][1])])
        for j, (p, v) in arm.items():
            stop[j] = Profile(p, v, [(brake, -v / brake)], p + 0.5 * v * brake) if brake > 0.0 \
                else hold(p)
        rail = {j: stop[j] for j in ("z_carriage", CRATE, HEADING)}
        if not self._free((stop["joint_1"], stop["joint_2"]), rail):
            raise PlanError("the robot is moving and cannot stop without a collision "
                            "or joint_3 leaving its limits")
        e = {j: pr.end for j, pr in stop.items()}
        goal = np.array([e["z_carriage"], e["joint_1"], e["joint_2"],
                         float(self.model.wrist_for(e[HEADING], e["joint_1"], e["joint_2"]))])
        return Plan(stop, max(pr.duration for pr in stop.values()), goal, self.model, ["stop"])

    def _heading_now(self, start):
        """(heading, its rate) of a start state."""
        (p1, v1), (p2, v2), (p3, v3) = (start[j] for j in ("joint_1", "joint_2", WRIST))
        return (float(self.model.heading_of(p1, p2, p3)),
                float(self.model.heading_rate(v1, v2, v3)))

    def _point(self, start, point, speed, heading, crate):
        x, y, z = (float(v) for v in point)
        psi = self.heading(heading, y, start)
        goals = self.model.inverse(point, psi)
        if not goals:
            side = "+y" if psi > 0 else "-y"
            if self.model.inverse(point, psi, wrist_limits=False):
                lo, hi = self.model.limits(WRIST)
                raise PlanError(f"({x:.3f}, {y:.3f}, {z:.3f}) is out of reach with the gripper "
                                f"to {side}: joint_3 does not turn that far ({lo:.3f} .. {hi:.3f})")
            wx, wy = self.model.wrist_point(point, psi)
            r = math.hypot(wx - self.model.j1_xy[0], wy - self.model.j1_xy[1])
            lo, hi = abs(self.model.link1 - self.model.link2), self.model.link1 + self.model.link2
            zlo, zhi = self.model.limits("z_carriage")
            zlo, zhi = zlo + self.model.tool_z0, zhi + self.model.tool_z0
            if lo <= r <= hi and zlo <= z <= zhi:
                raise PlanError(f"({x:.3f}, {y:.3f}, {z:.3f}) is out of reach with the gripper "
                                f"to {side}: joint_1 does not turn that far")
            raise PlanError(
                f"({x:.3f}, {y:.3f}, {z:.3f}) is out of reach with the gripper to {side}: the "
                f"wrist axis reaches {lo:.3f}..{hi:.3f} m from joint_1 (this needs {r:.3f} m) "
                f"and the tool z {zlo:.3f}..{zhi:.3f} m")
        return self._best(start, goals, speed, crate)

    def _joints(self, start, goal, speed, heading, crate):
        goal = np.asarray(goal, float)
        if goal.shape not in ((3,), (4,)):
            raise PlanError("a joint goal is [z_carriage, joint_1, joint_2] or "
                            "[z_carriage, joint_1, joint_2, joint_3]")
        for j, v in zip(ARM + (WRIST,), goal):
            lo, hi = self.model.limits(j)
            if not lo - 1e-9 <= v <= hi + 1e-9:
                raise PlanError(f"{j} = {v:.4f} is outside its limits {lo:.4f} .. {hi:.4f}")
        if goal.size == 4:
            return self._best(start, [goal], speed, crate)
        wrist = self.model.wrist_position(dict(zip(ARM, goal)))
        psi = self.heading(heading, wrist[1], start)
        yaws = self.model.turns(WRIST, float(self.model.wrist_for(psi, goal[1], goal[2])))
        if not yaws:
            lo, hi = self.model.limits(WRIST)
            raise PlanError(f"joint_3 cannot turn the gripper to {'+y' if psi > 0 else '-y'} "
                            f"there: it would be outside {lo:.3f} .. {hi:.3f}")
        return self._best(start, [np.append(goal, w) for w in yaws], speed, crate)

    # ------------------------------------------------------------------ planning

    def _best(self, start, goals, speed, crate=None):
        """goals: [z_carriage, joint_1, joint_2, joint_3] options; the fastest
        plan to any of them. crate: see _rail."""
        if not 0.0 < speed <= 1.0:
            raise PlanError(f"speed must be in (0, 1], got {speed}")
        lim = {j: (v * speed, a * speed) for j, (v, a) in self.limits.items()}
        rail, rail_notes = self._rail(start, goals[0][0], lim, crate)
        moving = any(abs(v) > EPS_V for _, v in start.values())
        # The heading moves on its own, like the rail: every collision check
        # below samples it with the rail's profiles, joint_3 follows from it.
        psi0, dpsi0 = self._heading_now(start)
        why, free = [], []
        for goal in goals:
            what = f"joint_1 {goal[1]:+.3f} joint_2 {goal[2]:.3f} joint_3 {goal[3]:+.3f}"
            q_goal = {"z_carriage": goal[0], "joint_1": goal[1], "joint_2": goal[2],
                      CRATE: rail[CRATE].end, WRIST: goal[3]}
            hit = self.model.colliding_pairs(q_goal)
            if hit:
                why.append(f"{what}: collides at the goal ({', '.join(hit)})")
                continue
            psi = float(self.model.heading_of(goal[1], goal[2], goal[3]))
            free.append((what, goal, {**rail, HEADING: min_time(psi0, dpsi0, psi, *lim[WRIST])}))

        # All joints at once (with waits) first; a stop on the way only if no
        # arm solution can do without - it is never faster.
        def bound(option):      # no plan to this goal can be faster than this
            _, goal, rail_g = option
            return max([min_time(*start[j], v, *lim[j]).duration
                        for j, v in zip(("joint_1", "joint_2"), goal[1:3])] +
                       [rail_g[HEADING].duration])

        def turn(option):       # of two as fast, the gripper turns the least
            return abs(option[2][HEADING].end - psi0)
        free.sort(key=lambda o: (bound(o), turn(o)))
        # The heading the short way (at most half a turn) first. A goal whose
        # joint_3 only fits the other turn spins the gripper once in the
        # world on the way: only if nothing else works.
        tiers = [([o for o in free if turn(o) <= math.pi + 1e-6], []),
                 ([o for o in free if turn(o) > math.pi + 1e-6],
                  ["the gripper turns a full turn on the way (joint_3 limits)"])]
        best, best_turn, too_fast = None, None, set()
        for (tier, tier_notes), method in itertools.product(
                tiers, (self._arm_direct, self._arm_via)):
            for option in tier:
                what, goal, rail_g = option
                if best is not None and bound(option) >= best.duration:
                    continue
                arm = method(start, goal, lim, rail_g)
                if arm is None:
                    continue
                profiles, notes = arm
                profiles.update(rail_g)
                k = self._slowdown(profiles, lim)
                if k is None:
                    continue
                if k > 1.0:
                    if moving:      # would not start at the robot's speed
                        too_fast.add(what)
                        continue
                    profiles = {j: p.stretched(k) for j, p in profiles.items()}
                    notes = notes + [f"slowed to {100.0 / k:.0f} % for joint_3"]
                duration = max(p.duration for p in profiles.values())
                if best is None or duration < best.duration - 1e-6 or \
                        (duration < best.duration + 1e-6 and turn(option) < best_turn - 1e-6):
                    best = Plan(profiles, duration, np.array(goal, float), self.model,
                                rail_notes + notes + tier_notes)
                    best_turn = turn(option)
            if best is not None:
                return best
        for what, _, _ in free:
            why.append(f"{what}: " + ("joint_3 would be too fast from the moving start"
                                      if what in too_fast else
                                      "no way clear of collisions and joint_3's limits"))
        raise PlanError("no plan: " + "; ".join(why))

    def _slowdown(self, profiles, lim):
        """How many times slower the plan has to run for joint_3 to keep its
        velocity and acceleration limits (1: it does), or None if joint_3
        leaves its position limits on the way. Exact: joint_3's acceleration
        is constant between the phase changes of joint_1, joint_2 and the
        heading, and its position has its extremes there or where its
        velocity crosses zero."""
        parts = [profiles[j] for j in (HEADING, "joint_1", "joint_2")]
        t = np.unique(np.concatenate([p.t for p in parts]))
        p, v, a = self.model.wrist_motion(*(pr.sample(t) for pr in parts))
        va, vb = v[:-1], v[1:]
        cross = va * vb < 0.0
        if cross.any():
            tc = t[:-1][cross] + va[cross] / (va[cross] - vb[cross]) * np.diff(t)[cross]
            p = np.concatenate([p, self.model.wrist_motion(*(pr.sample(tc) for pr in parts))[0]])
        lo, hi = self.model.limits(WRIST)
        if p.min() < lo - 1e-6 or p.max() > hi + 1e-6:
            return None
        vmax, amax = lim[WRIST]
        k = max(1.0, float(np.abs(v).max()) / vmax, math.sqrt(float(np.abs(a).max()) / amax))
        return 1.0 if k < 1.0 + 1e-9 else k

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

    def _ok(self, q):
        """q: joint_1, joint_2, the rail and HEADING -> arrays. Per sample:
        clear of collisions, and joint_3 (which follows) inside its limits."""
        q = dict(q)
        w = self.model.wrist_for(q.pop(HEADING), q["joint_1"], q["joint_2"])
        q[WRIST] = w
        lo, hi = self.model.limits(WRIST)
        return (w >= lo - 1e-6) & (w <= hi + 1e-6) & ~self.model.collisions(q)

    def _free(self, arm, rail):
        """True if arm (joint_1, joint_2 profiles) + rail (incl. HEADING) never
        collide and keep joint_3 inside its limits."""
        return bool(self._free_many([arm], rail)[0])

    def _free_many(self, arms, rail):
        """_free for several (joint_1, joint_2) options at once: bool per option.
        Every 8th sample first: most options collide, and that shows there
        already. Only the rest get every sample."""
        t = self._times([p for arm in arms for p in arm] + list(rail.values()),
                        max(p.duration for arm in arms for p in arm))

        def ok(arms, t):
            q = {"joint_1": np.concatenate([a[0].sample(t)[0] for a in arms]),
                 "joint_2": np.concatenate([a[1].sample(t)[0] for a in arms])}
            for j, p in rail.items():
                q[j] = np.tile(p.sample(t)[0], len(arms))
            return self._ok(q).reshape(len(arms), t.size).all(axis=1)
        free = ok(arms, t[::8])
        rest = np.flatnonzero(free)
        if rest.size:
            free[rest] = ok([arms[i] for i in rest], t)
        return free

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
        b = min_time(start[j2][0], 0.0, goal[2], *lim[j2])
        vias = []                   # (fastest it could be, via1, leg 1, joint_1 in leg 2)
        for via1 in np.radians(np.arange(-180.0, 180.1, 5.0)):
            if lo <= via1 <= hi:
                one = min_time(start[j1][0], 0.0, via1, *lim[j1])
                a = min_time(via1, 0.0, goal[1], *lim[j1])
                vias.append((one.duration + max(a.duration, b.duration), via1, one, a))
        # Most promising first: once one is found, the rest that cannot beat it go.
        for bound, via1, one, a in sorted(vias, key=lambda v: v[0]):
            if best is not None and bound >= best[0]:
                break
            leg1 = (one, hold(start[j2][0]))
            if not self._free(leg1, self._shift(rail, 0.0)):
                continue
            t1 = one.duration
            opts, later = self._waits(a, b, True, True), self._shift(rail, t1)
            for k in range(0, len(opts), 16):      # 16 options per numpy call
                total = [t1 + max(d1 + a.duration, d2 + b.duration) for d1, d2 in opts[k:k + 16]]
                n = sum(best is None or tt < best[0] for tt in total)   # sorted: a prefix
                if n == 0:
                    break
                chunk = [(a.delayed(d1), b.delayed(d2)) for d1, d2 in opts[k:k + n]]
                ok = np.flatnonzero(self._free_many(chunk, later))
                if ok.size:
                    best = (total[ok[0]], leg1, chunk[ok[0]], via1)
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

    def _rail(self, start, z_goal, lim, crate=None):
        """Profiles for z_carriage and z_crate with the gap kept, + notes.
        crate: where the crate goes; None: crate_trail below the carriage
        (crate_follow) or where it is, in both cases lower if the carriage
        would come too close."""
        zc, cr = "z_carriage", CRATE
        lo, hi = self.model.limits(cr)
        c_now = start[cr][0]
        if crate is not None:
            crate = float(crate)
            if not lo - 1e-9 <= crate <= hi + 1e-9:
                raise PlanError(f"z_crate = {crate:.4f} is outside its limits {lo:.4f} .. {hi:.4f}")
            if crate > z_goal - self.gap + 1e-9:
                raise PlanError(f"z_crate {crate:.3f} is too high: it has to stay {self.gap:.3f} m "
                                f"below the carriage (z_carriage goal {z_goal:.3f})")
            c_goal = min(max(crate, lo), hi)
        elif self.crate_follow:
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
