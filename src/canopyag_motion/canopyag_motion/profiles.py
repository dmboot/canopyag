"""Time-optimal motion of one joint under a velocity and an acceleration limit.

A Profile is a run of phases with constant acceleration, starting from a
position and velocity: accelerate at the limit, cruise at the speed limit if
there is room, brake at the limit (bang-coast-bang). From a moving start it
first brakes if it is heading the wrong way or cannot stop in time. That is
the fastest any joint can get there within its limits.

Every joint gets its own profile, so each one moves at its own top speed and
they all move at once; a joint can be told to wait (`delay`) before it starts.
"""

import math

import numpy as np

EPS_P = 1e-9   # position, joint units
EPS_V = 1e-9   # velocity, joint units / s


class Profile:

    def __init__(self, p0, v0, phases, p_end=None):
        """phases: [(duration, acceleration), ...] from (p0, v0)."""
        self.p0, self.v0 = float(p0), float(v0)
        self.phases = [(float(d), float(a)) for d, a in phases if d > 0.0]
        self.t, self.p, self.v = [0.0], [self.p0], [self.v0]
        for d, a in self.phases:
            self.p.append(self.p[-1] + self.v[-1] * d + 0.5 * a * d * d)
            self.v.append(self.v[-1] + a * d)
            self.t.append(self.t[-1] + d)
        if p_end is not None:   # exact end, not the integration's last bit
            self.p[-1] = float(p_end)
        self.v[-1] = 0.0 if self.phases else self.v0
        self.duration = self.t[-1]
        self.end = self.p[-1]

    def delayed(self, delay):
        """The same motion, starting `delay` s later. Only from rest."""
        if delay <= 0.0:
            return self
        if abs(self.v0) > EPS_V:
            raise ValueError("a moving joint cannot wait")
        return Profile(self.p0, 0.0, [(delay, 0.0)] + self.phases, self.end)

    def then(self, other):
        """This motion, then `other` (which must start where this one stops)."""
        return Profile(self.p0, self.v0, self.phases + other.phases, other.end)

    def stretched(self, k):
        """The same path k times slower: every phase k times longer, velocity
        / k, acceleration / k^2."""
        return Profile(self.p0, self.v0 / k, [(d * k, a / (k * k)) for d, a in self.phases],
                       self.end)

    def after(self, t0):
        """What is left of this motion from t0 on."""
        if t0 <= 0.0:
            return self
        p, v, _ = self.sample(t0)
        rest, t = [], 0.0
        for d, a in self.phases:
            if t + d > t0:
                rest.append((min(d, t + d - t0), a))
            t += d
        return Profile(float(p), float(v), rest, self.end)

    def sample(self, t):
        """Position, velocity, acceleration at times t (scalar or array)."""
        t = np.asarray(t, float)
        k = np.clip(np.searchsorted(self.t, t, side="right") - 1, 0, len(self.t) - 1)
        acc = np.array([a for _, a in self.phases] + [0.0])[k]
        dt = np.clip(t - np.array(self.t)[k], 0.0, None)
        v0 = np.array(self.v)[k]
        p = np.array(self.p)[k] + v0 * dt + 0.5 * acc * dt * dt
        v = v0 + acc * dt
        last = k == len(self.t) - 1          # after the end: hold
        p = np.where(last, self.end, p)
        v = np.where(last, 0.0 if self.phases else self.v0, v)
        acc = np.where(last, 0.0, acc)
        return p, v, acc


def min_time(p0, v0, p1, vmax, amax):
    """Fastest motion from (p0, v0) to rest at p1."""
    if vmax <= 0.0 or amax <= 0.0:
        raise ValueError("vmax and amax must be > 0")
    phases = []
    start_p, start_v = p0, v0
    d = p1 - p0
    stop = v0 * abs(v0) / (2.0 * amax)          # signed distance to stand still
    if abs(v0) > EPS_V and (v0 * d <= 0.0 or abs(stop) > abs(d) + EPS_P):
        # Heading away, or too fast to stop in time: brake to rest first.
        phases.append((abs(v0) / amax, -math.copysign(amax, v0)))
        p0, v0 = p0 + stop, 0.0
        d = p1 - p0
    if abs(d) < EPS_P and abs(v0) <= EPS_V:
        return Profile(start_p, start_v, phases, p1)

    s, dist, u = math.copysign(1.0, d), abs(d), abs(v0)
    if u > vmax:                                # faster than allowed: slow down to vmax
        phases.append(((u - vmax) / amax, -s * amax))
        dist -= (u * u - vmax * vmax) / (2.0 * amax)
        u = vmax
    peak = math.sqrt(amax * dist + 0.5 * u * u)
    if peak <= vmax:                            # triangle: never reaches vmax
        phases += [((peak - u) / amax, s * amax), (peak / amax, -s * amax)]
    else:                                       # trapezoid: cruise at vmax
        d_acc = (vmax * vmax - u * u) / (2.0 * amax)
        d_dec = vmax * vmax / (2.0 * amax)
        phases += [((vmax - u) / amax, s * amax),
                   (max(dist - d_acc - d_dec, 0.0) / vmax, 0.0),
                   (vmax / amax, -s * amax)]
    return Profile(start_p, start_v, phases, p1)


def hold(p):
    """Stand still at p."""
    return Profile(p, 0.0, [], p)
