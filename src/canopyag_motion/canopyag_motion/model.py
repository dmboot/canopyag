"""The robot as canopyag_motion sees it: kinematics, inverse kinematics and
collision boxes, all built from canopyag_description's robot_parameters.yaml.

Every joint axis is vertical and every joint origin is a translation plus a
turn about z, so a link's pose in the world is (x, y, z, yaw). That keeps all
of this a few lines of numpy, and every function takes arrays: one call
checks a whole trajectory.

Collision: each link's collision_box, grown by `margin` on every side. Two
boxes touch when their z ranges overlap and their footprints (rectangles
turned by yaw) overlap. Links that are parent and child never count (they
always touch), nor the pairs in `ignore`. Links on the same rail (the same
prismatic joint above them, e.g. everything on the carriage) cannot move up
or down relative to each other, so between them the margin is left out of
the z test: the gripper may pass 2.5 cm under link 1.
"""

import itertools
import math

import numpy as np

ARM = ("z_carriage", "joint_1", "joint_2")
CRATE = "z_crate"
# Turns the gripper about its own vertical axis. Not part of the inverse
# kinematics: the tool point sits on its axis, so it only turns the gripper.
WRIST = "joint_3"
JOINTS = ARM + (CRATE, WRIST)


class ModelError(ValueError):
    pass


class Model:

    def __init__(self, params, tool_link, tool_xyz, margin=0.0, ignore=()):
        """params: robot_parameters.yaml as a dict. tool_xyz: the tool point
        in tool_link's frame. margin: m added to every box on every side.
        ignore: link pairs never checked (besides parent / child)."""
        self.margin = float(margin)
        self.root = params["robot"]["root_link"]
        m = params.get("mount", {})
        self.mount = self._origin("mount", m.get("xyz", [0, 0, 0]), m.get("rpy", [0, 0, 0]))

        self.joints = {}
        for name, j in params["joints"].items():
            axis = j.get("axis", [0.0, 0.0, 1.0])
            if j["type"] != "fixed" and (abs(axis[0]) > 1e-9 or abs(axis[1]) > 1e-9):
                raise ModelError(f"joint {name}: axis {axis} is not vertical")
            lim = j.get("limit", {})
            continuous = j["type"] == "continuous"
            self.joints[name] = dict(
                type=j["type"], parent=j["parent"], child=j["child"],
                origin=self._origin(name, j["xyz"], j.get("rpy", [0, 0, 0])),
                sign=1.0 if j["type"] == "fixed" else math.copysign(1.0, axis[2]),
                lower=-math.inf if continuous else float(lim.get("lower", -math.inf)),
                upper=math.inf if continuous else float(lim.get("upper", math.inf)))
        for name in JOINTS:
            if name not in self.joints:
                raise ModelError(f"robot_parameters.yaml has no joint {name}")

        # Joints in an order where every parent link is placed before its child.
        self.order, placed, todo = [], {self.root}, list(self.joints)
        while todo:
            ready = [n for n in todo if self.joints[n]["parent"] in placed]
            if not ready:
                raise ModelError(f"joints {todo} are not connected to {self.root}")
            for n in ready:
                self.order.append(n)
                placed.add(self.joints[n]["child"])
                todo.remove(n)

        self.boxes = {}
        for link, L in params["links"].items():
            if "collision_box" in L:
                b = L["collision_box"]
                self.boxes[link] = (np.array(b["xyz"], float),
                                    np.array(b["size"], float) / 2.0 + self.margin)
        skip = {frozenset((j["parent"], j["child"])) for j in self.joints.values()}
        skip |= {frozenset(pair) for pair in ignore}
        parent = {j["child"]: (n, j) for n, j in self.joints.items()}

        def rail(link):         # the prismatic joint (or root) the link rides on
            while link in parent and parent[link][1]["type"] != "prismatic":
                link = parent[link][1]["parent"]
            return parent[link][0] if link in parent else self.root
        # (a, b, how much less the z ranges may overlap): 2 * margin on one rail.
        self.pairs = [(a, b, 2.0 * self.margin if rail(a) == rail(b) else 0.0)
                      for a, b in itertools.combinations(sorted(self.boxes), 2)
                      if frozenset((a, b)) not in skip]

        # The arm: z_carriage -> joint_1 -> joint_2 -> tool_link, every one
        # the child of the one before.
        chain = [self.joints[n] for n in ARM]
        if chain[0]["type"] != "prismatic" or chain[1]["parent"] != chain[0]["child"] or \
                chain[2]["parent"] != chain[1]["child"] or chain[2]["child"] != tool_link:
            raise ModelError("expected z_carriage -> joint_1 -> joint_2 -> " + tool_link)
        self.tool_link = tool_link
        self.tool = np.array(tool_xyz, float)

        # Planar 2-link geometry for the inverse kinematics, from the pose at zero.
        zero = self.link_poses({n: 0.0 for n in JOINTS})
        p1 = zero[chain[1]["child"]]                      # mid_link frame, joint_1 = 0
        self.j1_xy = np.array([p1[0], p1[1]])
        self.j1_yaw = float(p1[3])
        o2 = chain[2]["origin"]
        self.link1 = math.hypot(o2[0], o2[1])
        self.alpha1 = math.atan2(o2[1], o2[0])
        self.j2_yaw = float(o2[3])
        self.link2 = math.hypot(self.tool[0], self.tool[1])
        self.alpha2 = math.atan2(self.tool[1], self.tool[0])
        self.tool_z0 = float(self.tool_position({n: 0.0 for n in JOINTS})[2])

        # How far from its joint any part of link 1 / link 2 / the gripper
        # reaches (boxes, the tool point and everything further down the
        # chain): turns a joint speed into a speed of the arm's parts.
        def reach(link):
            r = math.hypot(self.tool[0], self.tool[1]) if link == tool_link else 0.0
            if link in self.boxes:
                c, h = self.boxes[link]
                r = max(r, max(math.hypot(c[0] + sx * h[0], c[1] + sy * h[1])
                               for sx in (-1, 1) for sy in (-1, 1)))
            for j in self.joints.values():
                if j["parent"] == link:
                    r = max(r, math.hypot(j["origin"][0], j["origin"][1]) + reach(j["child"]))
            return r
        self.reach2 = reach(tool_link)
        self.reach1 = reach(chain[1]["child"])
        self.reach3 = reach(self.joints[WRIST]["child"])

    @staticmethod
    def _origin(what, xyz, rpy):
        if abs(rpy[0]) > 1e-9 or abs(rpy[1]) > 1e-9:
            raise ModelError(f"{what}: rpy {rpy} tilts the frame; only a turn about z is supported")
        return (float(xyz[0]), float(xyz[1]), float(xyz[2]), float(rpy[2]))

    # ------------------------------------------------------------------ kinematics

    def limits(self, joint):
        j = self.joints[joint]
        return j["lower"], j["upper"]

    def link_poses(self, q):
        """q: joint -> value or array (missing joints: 0). Link -> (x, y, z, yaw)."""
        def compose(p, o):
            c, s = np.cos(p[3]), np.sin(p[3])
            return (p[0] + c * o[0] - s * o[1], p[1] + s * o[0] + c * o[1], p[2] + o[2], p[3] + o[3])
        poses = {self.root: self.mount}
        for name in self.order:
            j = self.joints[name]
            x, y, z, yaw = compose(poses[j["parent"]], j["origin"])
            v = j["sign"] * np.asarray(q.get(name, 0.0), float)
            if j["type"] == "prismatic":
                z = z + v
            elif j["type"] != "fixed":
                yaw = yaw + v
            poses[j["child"]] = (x, y, z, yaw)
        return poses

    def tool_position(self, q):
        """World (x, y, z) of the tool point."""
        x, y, z, yaw = self.link_poses(q)[self.tool_link]
        c, s = np.cos(yaw), np.sin(yaw)
        return np.array([x + c * self.tool[0] - s * self.tool[1],
                         y + s * self.tool[0] + c * self.tool[1], z + self.tool[2]])

    def inverse(self, point):
        """Arm joints [z_carriage, joint_1, joint_2] that put the tool point at
        `point` (world). Both elbow solutions, each in every equivalent turn
        that fits the joint limits (joint_2 -pi..pi allows two at +-pi).
        Continuous joints come back in (-pi, pi]. Empty if out of reach."""
        zj = self.joints["z_carriage"]
        zc = (float(point[2]) - self.tool_z0) / zj["sign"]
        if not zj["lower"] - 1e-9 <= zc <= zj["upper"] + 1e-9:
            return []
        zc = min(max(zc, zj["lower"]), zj["upper"])
        dx, dy = float(point[0]) - self.j1_xy[0], float(point[1]) - self.j1_xy[1]
        a, b = self.link1, self.link2
        c = (dx * dx + dy * dy - a * a - b * b) / (2 * a * b)
        if c > 1 + 1e-9 or c < -1 - 1e-9:
            return []
        beta = math.acos(min(max(c, -1.0), 1.0))
        s1, s2 = self.joints["joint_1"]["sign"], self.joints["joint_2"]["sign"]
        out = []
        for bend in ((beta,) if beta < 1e-9 or beta > math.pi - 1e-9 else (beta, -beta)):
            phi1 = math.atan2(dy, dx) - math.atan2(b * math.sin(bend), a + b * math.cos(bend))
            j1 = (phi1 - self.j1_yaw - self.alpha1) / s1
            j2 = (bend + self.alpha1 - self.j2_yaw - self.alpha2) / s2
            for v1 in self.turns("joint_1", j1):
                for v2 in self.turns("joint_2", j2):
                    out.append(np.array([zc, v1, v2]))
        return out

    def reach_box(self, n=181):
        """(lo, hi): world corners of the box around every tool point the arm
        reaches without touching itself, rounded out to the cm. Not every
        point inside is reachable."""
        (z_lo, z_hi), (c_lo, _) = self.limits("z_carriage"), self.limits(CRATE)
        j1, j2 = np.meshgrid(np.linspace(*self.limits("joint_1"), n),
                             np.linspace(*self.limits("joint_2"), n))
        q = {"z_carriage": np.full(j1.size, (z_lo + z_hi) / 2), "joint_1": j1.ravel(),
             "joint_2": j2.ravel(), CRATE: np.full(j1.size, c_lo)}
        tool = self.tool_position(q)[:, ~self.collisions(q)]
        lo = np.floor(tool.min(axis=1) * 100) / 100
        hi = np.ceil(tool.max(axis=1) * 100) / 100
        lo[2], hi[2] = z_lo + self.tool_z0, z_hi + self.tool_z0
        return lo, hi

    def turns(self, joint, value, slack=1e-3):
        """value + k * 2 pi for every k that fits the joint's limits. Within
        `slack` (rad) of a limit counts as in, clamped onto it: a limit of
        3.1416 means pi."""
        lo, hi = self.limits(joint)
        v = math.remainder(value, 2 * math.pi)
        if math.isinf(lo) or math.isinf(hi):
            return [v]
        k0 = math.ceil((lo - slack - v) / (2 * math.pi))
        out = []
        while v + k0 * 2 * math.pi <= hi + slack:
            out.append(min(max(v + k0 * 2 * math.pi, lo), hi))
            k0 += 1
        return out

    # ------------------------------------------------------------------ collision

    def collisions(self, q):
        """q: joint -> value or array of N. Bool array (N,): any pair in collision."""
        world, n = self._world_boxes(q)
        hit = np.zeros(n, bool)
        for a, b, dz in self.pairs:
            hit |= self._touch(world[a], world[b], dz)
        return hit

    def colliding_pairs(self, q):
        """For one configuration: the pairs that touch, as 'link/link' (for messages)."""
        world, _ = self._world_boxes(q)
        return [f"{a}/{b}" for a, b, dz in self.pairs if self._touch(world[a], world[b], dz)[0]]

    def _world_boxes(self, q):
        poses = self.link_poses(q)
        n = max((np.size(v) for v in q.values()), default=1)
        world = {}
        for link, (c, h) in self.boxes.items():
            x, y, z, yaw = (np.broadcast_to(np.asarray(v, float), (n,)) for v in poses[link])
            cy, sy = np.cos(yaw), np.sin(yaw)
            world[link] = (x + cy * c[0] - sy * c[1], y + sy * c[0] + cy * c[1], z + c[2], yaw, h)
        return world, n

    @staticmethod
    def _touch(A, B, dz=0.0):
        ax, ay, az, at, ah = A
        bx, by, bz, bt, bh = B
        hit = np.abs(az - bz) < ah[2] + bh[2] - dz
        dx, dy = bx - ax, by - ay
        ua = (np.cos(at), np.sin(at))
        ub = (np.cos(bt), np.sin(bt))
        for nx, ny in (ua, (-ua[1], ua[0]), ub, (-ub[1], ub[0])):
            ra = ah[0] * np.abs(ua[0] * nx + ua[1] * ny) + ah[1] * np.abs(-ua[1] * nx + ua[0] * ny)
            rb = bh[0] * np.abs(ub[0] * nx + ub[1] * ny) + bh[1] * np.abs(-ub[1] * nx + ub[0] * ny)
            hit &= np.abs(dx * nx + dy * ny) < ra + rb
        return hit
