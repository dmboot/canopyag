#!/usr/bin/env python3
"""Hard-coded demo path: the arm picks its way up one side of the gantry, the
crate follows.

    ros2 launch canopyag_moveit_config demo.launch.py [loop:=true] [speed:=0.5] [side:=left]
    ros2 launch canopyag_moveit_config moveit.launch.py demo:=true speed:=0.3   # real robot

`speed` (0..1] scales every move: the arm's velocity scaling per waypoint and
the crate's follow speeds. `side` (right / left, seen from the front of the
gantry) is where the arm picks; see PICKS.

The arm (carriage + two revolutes) goes through waypoints() as point-to-point
moves: a straight line in joint space, all three joints starting and stopping
together, each within its own limit from config/joint_limits.yaml. MoveIt
checks every sample for collisions (the boxes in robot_parameters.yaml,
including the crate) and executes it through arm_controller.

That is what Pilz PTP does too, but Pilz gives every joint in the group the
strictest limit of any of them - the carriage's 0.25 m/s becomes 0.25 rad/s
for the revolutes - which makes it about 6x too slow for this arm.

The crate is not planned. It trails CRATE_TRAIL below the carriage on its own
slow trajectory, straight to crate_controller, and keeps moving while the arm
does. Two rules keep it under the carriage:

  * its target is never above min(carriage now, carriage goal) - CRATE_TRAIL,
    so while it climbs it can only close in on where the carriage will be;
  * before the carriage is sent below the crate + GAP, the crate is brought
    down first and the arm waits for it.

A monitor on /joint_states reports any moment the gap drops below GAP.
"""

import math
import os
import threading
import time

import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ["z_carriage", "joint_1", "joint_2"]

GAP = 0.02               # m, crate top to carriage bottom, never less
CRATE_TRAIL = 0.15       # m, how far below the carriage the crate follows
CRATE_UP_SPEED = 0.03    # m/s, the slow creep
CRATE_DOWN_SPEED = 0.10  # m/s, getting out of the carriage's way
CRATE_MAX = 1.215        # m, z_crate upper limit in robot_parameters.yaml
SAMPLE_DT = 0.02         # s, trajectory point spacing (and collision check step)
CHECKS_IN_FLIGHT = 8     # concurrent /check_state_validity calls

# At home the crate and the carriage touch (gap 0), so the path starts and
# ends at REST, with the carriage lifted clear.
REST = (0.05, 0.0, 0.0)

# The fruit hangs beside the gantry, on ONE side. Per fruit, for the right
# side (seen from the front, joint_1 > 0), in degrees:
#   (carriage height above the level, reach (joint_1, joint_2), pick (joint_1, joint_2))
# reach stops short of the fruit, pick goes in with the arm almost straight.
# The tips are 0.49..0.64 m out to the side, 0.28..0.49 m in front of the
# column (tool point TOOL along endeffector_link).
PICKS = [
    (0.00, (50.0, 30.0), (58.0, 14.0)),
    (0.10, (70.0, 28.0), (78.0, 12.0)),
]
LEVELS = (0.35, 0.65, 0.95)   # m, carriage height of each row of fruit
# Then the arm swings to joint_1 = CARRY and link 2 turns 180 deg, folding
# under link 1 to drop the fruit under itself, and turns back. Link 2 can only
# make that turn with link 1 near the middle: from |joint_1| ~ 30 deg its box
# sweeps through the carriage, and at 120 deg through the gantry. 15 deg keeps
# 8 deg of margin in both joints (checked against the collision boxes).
CARRY = 15.0
DROP_TURN = 180.0
LINK_1, TOOL = 0.37, 0.28     # m, joint_1 -> joint_2, joint_2 -> tool point


def side_pose(j1, j2, side):
    """Right-side (joint_1, joint_2) in degrees -> radians on `side`.

    joint_2 only turns 0..360 deg, so the left side cannot just negate it.
    The left pose keeps joint_2 (the same elbow bend) and turns joint_1 so the
    tool point lands on the mirror image of the right one."""
    a1, a2 = math.radians(j1), math.radians(j2)
    if side == "right":
        return a1, a2
    bend = math.atan2(TOOL * math.sin(a2), LINK_1 + TOOL * math.cos(a2))
    return -(a1 + 2.0 * bend), a2


def waypoints(side):
    """(name, (z_carriage, joint_1, joint_2), velocity scaling) for the whole path.
    The z values are where the carriage is, so the crate follows the climb."""
    s = 1.0 if side == "right" else -1.0
    path = [("rest", REST, 0.5)]
    for i, z0 in enumerate(LEVELS):
        for k, (dz, reach, pick) in enumerate(PICKS):
            z, name = z0 + dz, f"level{i}_fruit{k}"
            j2 = math.radians(pick[1])
            carry = s * math.radians(CARRY)
            path += [
                (f"{name}_reach",  (z, *side_pose(*reach, side)), 0.6),
                (f"{name}_pick",   (z, *side_pose(*pick, side)), 0.3),
                (f"{name}_carry",  (z, carry, j2), 0.6),
                (f"{name}_drop",   (z, carry, j2 + math.radians(DROP_TURN)), 0.8),
                (f"{name}_unfold", (z, carry, j2), 0.8),
            ]
    path += [("back_down", (0.30, 0.0, 0.0), 0.8), ("rest", REST, 0.5)]
    return path


class Shutdown(Exception):
    """rclpy was shut down (Ctrl-C, launch stopping) while we were waiting."""


class DemoPath(Node):

    def __init__(self):
        super().__init__("demo_path")
        self.declare_parameter("loop", False)
        self.side = str(self.declare_parameter("side", "right").value)
        if self.side not in ("right", "left"):
            raise ValueError(f"side must be right or left, got {self.side}")
        self.speed = float(self.declare_parameter("speed", 1.0).value)
        if not 0.0 < self.speed <= 1.0:
            raise ValueError(f"speed must be in (0, 1], got {self.speed}")

        self.execute = ActionClient(self, ExecuteTrajectory, "execute_trajectory")
        self.validity = self.create_client(GetStateValidity, "check_state_validity")
        limits_file = os.path.join(get_package_share_directory("canopyag_moveit_config"),
                                   "config", "joint_limits.yaml")
        with open(limits_file) as fh:
            self.limits = yaml.safe_load(fh)["joint_limits"]
        self.crate = ActionClient(self, FollowJointTrajectory,
                                  "crate_controller/follow_joint_trajectory")
        # Only waited on: move_group sends to arm_controller, but fails
        # outright if it is not up yet.
        self.arm = ActionClient(self, FollowJointTrajectory,
                                "arm_controller/follow_joint_trajectory")
        self.list_controllers = self.create_client(
            ListControllers, "controller_manager/list_controllers")

        self.pos = {}
        self.car_goal = None       # where the carriage is being sent
        self.crate_sent = None     # last crate target sent
        self.gap_armed = False     # set after the first move: home has no gap
        self.min_gap = float("inf")
        self.violations = 0

        self.create_subscription(JointState, "joint_states", self.on_joint_states, 10)
        self.create_timer(0.2, self.follow_crate)

    # ---------------------------------------------------------------- state

    def on_joint_states(self, msg):
        self.pos.update(zip(msg.name, msg.position))
        if "z_carriage" not in self.pos or "z_crate" not in self.pos:
            return
        gap = self.pos["z_carriage"] - self.pos["z_crate"]
        if self.gap_armed:
            self.min_gap = min(self.min_gap, gap)
            if gap < GAP - 1e-3:
                self.violations += 1
                self.get_logger().error(
                    f"crate {gap * 100:.1f} cm below the carriage (min {GAP * 100:.0f} cm)",
                    throttle_duration_sec=1.0)

    # ---------------------------------------------------------------- crate

    def crate_ceiling(self):
        """Highest the crate may go right now."""
        car = self.pos["z_carriage"]
        if self.car_goal is not None:
            car = min(car, self.car_goal)
        return car - GAP

    def follow_crate(self):
        if "z_crate" not in self.pos or not self.crate.server_is_ready():
            return
        target = min(self.crate_ceiling() - (CRATE_TRAIL - GAP), CRATE_MAX)
        target = max(target, 0.0)
        if self.crate_sent is not None and abs(target - self.crate_sent) < 0.002:
            return
        now = self.pos["z_crate"]
        speed = (CRATE_UP_SPEED if target > now else CRATE_DOWN_SPEED) * self.speed
        seconds = max(abs(target - now) / speed, 0.2)

        point = JointTrajectoryPoint(positions=[target])
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int((seconds % 1) * 1e9)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["z_crate"]
        goal.trajectory.points = [point]
        self.crate.send_goal_async(goal)  # replaces the previous one
        self.crate_sent = target

    # ------------------------------------------------------------------ arm

    def move_arm(self, name, pose, scale):
        # Make room first: the crate has to be GAP below where the carriage
        # is going before the carriage starts moving.
        self.car_goal = pose[0]
        deadline = time.time() + 60.0
        while self.pos["z_crate"] > pose[0] - GAP + 1e-3:
            if not rclpy.ok():
                raise Shutdown()
            if time.time() > deadline:
                self.get_logger().error(f"{name}: crate did not clear the way")
                return False
            time.sleep(0.05)

        traj = self.ptp(pose, scale * self.speed)
        blocked = self.first_collision(traj)
        if blocked is not None:
            self.get_logger().error(f"{name}: in collision at t={blocked:.2f} s - stopping")
            return False

        goal = ExecuteTrajectory.Goal()
        goal.trajectory.joint_trajectory = traj
        handle = self.wait(self.execute.send_goal_async(goal))
        if not handle.accepted:
            self.get_logger().error(f"{name}: move_group rejected the trajectory")
            return False
        code = self.wait(handle.get_result_async()).result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"{name}: execution failed, MoveIt error code {code}")
            return False
        self.gap_armed = True
        self.get_logger().info(
            f"{name:22s} carriage {self.pos['z_carriage']:.2f} m  "
            f"crate {self.pos['z_crate']:.2f} m  ({self.duration(traj):.1f} s)")
        return True

    def ptp(self, pose, scale):
        """Straight joint-space move from the current state to `pose`.

        One trapezoidal profile s(t) from 0 to 1 drives every joint, so they
        all start, cruise and stop together. Its peak rate and acceleration
        are the largest that keep every joint inside its own limits."""
        start = [self.pos[j] for j in ARM_JOINTS]
        delta = [g - s0 for g, s0 in zip(pose, start)]
        v, a = math.inf, math.inf   # limits on ds/dt and d2s/dt2
        for j, d in zip(ARM_JOINTS, delta):
            if abs(d) > 1e-9:
                v = min(v, scale * self.limits[j]["max_velocity"] / abs(d))
                a = min(a, scale * self.limits[j]["max_acceleration"] / abs(d))
        if math.isinf(v):  # already there
            v, a = 1.0, 1.0
        if v * v / a > 1.0:  # never reaches cruise: triangular profile
            v = math.sqrt(a)
        t_acc = v / a
        total = 1.0 / v + t_acc

        def profile(t):
            if t < t_acc:
                return 0.5 * a * t * t, a * t, a
            if t < total - t_acc:
                return 0.5 * a * t_acc ** 2 + v * (t - t_acc), v, 0.0
            r = max(total - t, 0.0)
            return 1.0 - 0.5 * a * r * r, a * r, -a

        traj = JointTrajectory(joint_names=list(ARM_JOINTS))
        # Every SAMPLE_DT, then the end exactly once - times must strictly
        # increase or the controller rejects the whole trajectory.
        times = [k * SAMPLE_DT for k in range(1, int(total / SAMPLE_DT) + 1)
                 if k * SAMPLE_DT < total - 1e-6] + [total]
        for t in times:
            sv, sd, sa = profile(t) if t < total else (1.0, 0.0, 0.0)
            pt = JointTrajectoryPoint(
                positions=[s0 + sv * d for s0, d in zip(start, delta)],
                velocities=[sd * d for d in delta],
                accelerations=[sa * d for d in delta])
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t % 1) * 1e9)
            traj.points.append(pt)
        return traj

    def first_collision(self, traj):
        """Time of the first sample MoveIt finds in collision, else None.

        The crate is checked at the highest it can be during the move - it
        may still be creeping up towards its target."""
        crate = max(self.pos["z_crate"], self.crate_sent or 0.0)
        pending = []
        for pt in traj.points:
            # Keep a few requests in flight instead of one round trip per
            # sample - but only a few: the service's request queue holds 10
            # and silently drops the rest.
            if len(pending) >= CHECKS_IN_FLIGHT:
                done_pt, future = pending.pop(0)
                if not self.check(done_pt, future):
                    return self.duration_of(done_pt)
            pending.append((pt, self.validity.call_async(self.validity_request(pt.positions, crate))))
        for pt, future in pending:
            if not self.check(pt, future):
                return self.duration_of(pt)
        return None

    @staticmethod
    def duration_of(pt):
        return pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9

    def duration(self, traj):
        return self.duration_of(traj.points[-1])

    @staticmethod
    def wait(future, timeout=None):
        deadline = None if timeout is None else time.time() + timeout
        while not future.done():
            if not rclpy.ok():
                raise Shutdown()
            if deadline is not None and time.time() > deadline:
                raise TimeoutError("no reply")
            time.sleep(0.01)
        return future.result()

    def check(self, pt, future):
        # A lost reply is not a collision: ask again (DDS can drop a service
        # reply, most often right after the client connects).
        for attempt in range(3):
            try:
                return self.wait(future, timeout=2.0).valid
            except TimeoutError:
                self.get_logger().warn("check_state_validity did not answer, asking again")
                future = self.validity.call_async(self.validity_request(pt.positions))
        self.get_logger().error("check_state_validity did not answer")
        return False

    def validity_request(self, arm_positions, crate=None):
        req = GetStateValidity.Request()
        req.group_name = "arm"
        req.robot_state = RobotState()
        req.robot_state.joint_state.name = list(ARM_JOINTS) + ["z_crate"]
        crate = max(self.pos["z_crate"], self.crate_sent or 0.0) if crate is None else crate
        req.robot_state.joint_state.position = list(arm_positions) + [crate]
        return req

    def validity_answers(self):
        """True once check_state_validity has answered one request. On Humble a
        service can look ready (wait_for_service) before its replies reach us."""
        if "z_carriage" not in self.pos or "z_crate" not in self.pos:
            return False
        try:
            self.wait(self.validity.call_async(
                self.validity_request([self.pos[j] for j in ARM_JOINTS])), timeout=1.0)
            return True
        except TimeoutError:
            return False

    def controllers_active(self):
        """True once arm_controller and crate_controller are both active. On
        the real robot arm_controller starts inactive, on purpose."""
        if not self.list_controllers.service_is_ready():
            return False
        try:
            res = self.wait(self.list_controllers.call_async(ListControllers.Request()), 2.0)
        except TimeoutError:
            return False
        state = {c.name: c.state for c in res.controller}
        missing = [n for n in ("arm_controller", "crate_controller") if state.get(n) != "active"]
        if missing:
            self.get_logger().warn(
                f"waiting for {', '.join(missing)} to be active - with the robot at home: "
                f"ros2 control switch_controllers --activate {' '.join(missing)}",
                throttle_duration_sec=10.0)
        return not missing

    # ------------------------------------------------------------------ run

    def run(self):
        self.get_logger().info(
            f"waiting for move_group and the controllers ... (speed {self.speed:g})")
        for ready in (lambda: self.execute.wait_for_server(timeout_sec=0.5),
                      lambda: self.validity.wait_for_service(timeout_sec=0.5),
                      lambda: self.arm.wait_for_server(timeout_sec=0.5),
                      lambda: self.crate.wait_for_server(timeout_sec=0.5),
                      self.controllers_active,
                      self.validity_answers,
                      lambda: "z_carriage" in self.pos and "z_crate" in self.pos):
            while not ready():
                if not rclpy.ok():
                    raise Shutdown()
                time.sleep(0.1)

        while rclpy.ok():
            for name, pose, scale in waypoints(self.side):
                if not self.move_arm(name, pose, scale):
                    return
            self.get_logger().info(
                f"path done - smallest gap {self.min_gap * 100:.1f} cm, "
                f"{self.violations} gap violations")
            if not self.get_parameter("loop").value:
                return


def main():
    rclpy.init()
    node = DemoPath()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        node.run()
    except (KeyboardInterrupt, Shutdown):
        pass
    finally:
        executor.shutdown(timeout_sec=1.0)
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
