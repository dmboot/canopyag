#!/usr/bin/env python3
"""Demo: pick fruit off a wall beside the gantry and place each one on its own
spot in a basket, as fast as the robot can.

    ros2 launch canopyag_motion demo.launch.py [side:=left] [loop:=true] [speed:=0.5]   # mock
    ros2 launch canopyag_motion motion.launch.py demo:=true speed:=0.3                # real robot

The wall stands 0.45 m from the gantry, on the `side` (right / left, seen
from the front of the gantry: world +y / -y). Nine fruit on it over the
carriage's whole range: five heights on the way up, four in between on the
way down. Every fruit: in front of it, in until the gripper tip touches the
wall, then link 1 forward with link 2 folded back under it to put the fruit
on its spot in the basket - every fruit another spot. The basket rides on the
crate, which keeps under the carriage. The gripper points to the wall the
whole way. The path and why it is shaped the way it is (the arm must stay
off the wall, which the planner does not know about):
canopyag_motion/wall_demo.py.

Publishes demo_markers (visualization_msgs/MarkerArray, latched): the wall,
the basket on the crate, its spots (the next one green) and a red sphere per
fruit - on the wall, in the gripper once picked, on its spot once placed.

`speed` (0..1] scales every move.
`dexterity` (off; demo.launch.py turns it on for mock): at every fruit a short wiggle of the gripper
pitch and a grip, and a release at the drop. Pitch and gripper are not in the
URDF and not planned: they go to their own FollowJointTrajectory controller
(`gripper_controller`, joints `gripper_joints` = [pitch, gripper]). Leave it
off until those motors and that controller exist; without the controller the
demo says so once and goes on without.
"""

import math
import os
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from canopyag_interfaces.action import MoveTo
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

import canopyag_motion
from canopyag_motion import wall_demo

# dexterity: pitch +- this and back (rad), each step this long (s).
WIGGLE = 0.35
WIGGLE_STEP = 0.25
MARKER = 0.04           # m, diameter of the fruit spheres


class Gripper:
    """Pitch and gripper on their own controller, outside motion_server.py."""

    def __init__(self, node, controller, joints, closed, opened):
        self.node = node
        self.joints = list(joints)
        self.closed, self.opened = float(closed), float(opened)
        self.action = f"{controller}/follow_joint_trajectory"
        self.client = ActionClient(node, FollowJointTrajectory, self.action)

    def ready(self):
        return self.client.wait_for_server(timeout_sec=2.0)

    def move(self, steps):
        """steps: [(pitch, grip), ...], WIGGLE_STEP apart. Waits until done."""
        traj = JointTrajectory(joint_names=self.joints)
        for k, (pitch, grip) in enumerate(steps, 1):
            t = k * WIGGLE_STEP
            traj.points.append(JointTrajectoryPoint(
                positions=[float(pitch), float(grip)],
                time_from_start=Duration(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))))
        future = self.client.send_goal_async(FollowJointTrajectory.Goal(trajectory=traj))
        rclpy.spin_until_future_complete(self.node, future)
        handle = future.result()
        if not handle.accepted:
            self.node.get_logger().error("gripper trajectory rejected")
            return False
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result)
        return result.result().result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

    def wiggle_and_grip(self):
        g = self.opened
        return self.move([(WIGGLE, g), (-WIGGLE, g), (0.0, g), (0.0, self.closed)])

    def release(self):
        return self.move([(0.0, self.opened)])


class Demo(Node):

    def __init__(self):
        super().__init__("demo")
        self.loop = bool(self.declare_parameter("loop", False).value)
        self.speed = float(self.declare_parameter("speed", 1.0).value)
        self.side = str(self.declare_parameter("side", "right").value)
        if not 0.0 < self.speed <= 1.0:
            raise ValueError(f"speed must be in (0, 1], got {self.speed}")
        if self.side not in ("right", "left"):
            raise ValueError(f"side must be right or left, got {self.side}")
        self.heading = (MoveTo.Goal.HEADING_POS_Y if self.side == "right" else
                        MoveTo.Goal.HEADING_NEG_Y)
        # The same robot as motion_server.py, for the path's inverse kinematics.
        desc = get_package_share_directory("canopyag_description")
        here = get_package_share_directory("canopyag_motion")
        model, planner, _ = canopyag_motion.load(
            self.declare_parameter(
                "robot_parameters", os.path.join(desc, "config", "robot_parameters.yaml")).value,
            self.declare_parameter(
                "motion_config", os.path.join(here, "config", "motion.yaml")).value)
        s = 1 if self.side == "right" else -1
        self.steps = wall_demo.path(model, planner, s)
        self.fruit = [p for p, _ in wall_demo.touches(model, planner.gap, s)]
        self.spots = wall_demo.spots(s)[:len(self.fruit)]
        self.basket = wall_demo.basket(model, planner.gap, s)
        self.crate_link, self.crate_x, self.crate_y = wall_demo.crate_frame(model)
        self.tool_link, self.tool = model.tool_link, [float(v) for v in model.tool]
        self.where = ["wall"] * len(self.fruit)     # wall, held or basket
        self.frame = self.declare_parameter("frame", "world").value
        self.markers = self.create_publisher(
            MarkerArray, "demo_markers",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.client = ActionClient(self, MoveTo, "move_to")
        self.gripper = None
        if self.declare_parameter("dexterity", False).value:
            self.gripper = Gripper(
                self, self.declare_parameter("gripper_controller", "gripper_controller").value,
                self.declare_parameter("gripper_joints", ["wrist_pitch", "gripper"]).value,
                self.declare_parameter("grip_closed", 1.0).value,
                self.declare_parameter("grip_open", 0.0).value)

    def publish_markers(self):
        """The wall, the basket and its spots, and every fruit where it is now:
        on the wall, in the gripper (tool link frame) or on its spot in the
        basket (crate frame). Frame-locked, so RViz carries the fruit and the
        basket along with the gripper and the crate."""
        out = []

        def add(ns, i, kind, frame, xyz, scale, rgba):
            m = Marker(type=kind, ns=ns, id=i, action=Marker.ADD, frame_locked=True)
            m.header.frame_id = frame
            m.pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            out.append(m)

        xs, ys, zs = zip(*self.fruit)
        thick = 0.01
        side = math.copysign(1.0, ys[0])
        add("wall", 0, Marker.CUBE, self.frame,
            ((min(xs) + max(xs)) / 2, ys[0] + side * thick / 2, (min(zs) + max(zs)) / 2),
            (max(xs) - min(xs) + 0.2, thick, max(zs) - min(zs) + 0.2), (0.6, 0.6, 0.6, 0.3))
        centre, size, floor = self.basket
        add("basket", 0, Marker.CUBE, self.crate_link, centre, size, (0.55, 0.35, 0.15, 0.35))
        nxt = next((i for i, w in enumerate(self.where) if w != "basket"), None)
        for i, (x, y) in enumerate(self.spots):
            spot = (x - self.crate_x, y - self.crate_y)
            add("spot", i, Marker.CYLINDER, self.crate_link, (*spot, floor + 0.001),
                (0.03, 0.03, 0.002), (0.1, 0.8, 0.1, 1.0) if i == nxt else (0.3, 0.3, 0.3, 0.6))
            if self.where[i] == "wall":
                add("fruit", i, Marker.SPHERE, self.frame, self.fruit[i], [MARKER] * 3,
                    (1.0, 0.0, 0.0, 1.0))
            elif self.where[i] == "held":
                add("fruit", i, Marker.SPHERE, self.tool_link, self.tool, [MARKER] * 3,
                    (1.0, 0.0, 0.0, 1.0))
            else:
                add("fruit", i, Marker.SPHERE, self.crate_link, (*spot, floor + MARKER / 2),
                    [MARKER] * 3, (1.0, 0.0, 0.0, 1.0))
        self.markers.publish(MarkerArray(markers=out))

    def move(self, step, quiet=False):
        """Send a wall_demo.Step and wait: until the arm is there, or (wait_crate)
        the crate too. The joints it went to, or None."""
        goal = MoveTo.Goal(speed=step.speed * self.speed, heading=self.heading,
                           joints=[float(v) for v in step.joints], crate=[float(step.crate)],
                           skip_crate_wait=not step.wait_crate)
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        r = result_future.result().result
        if not r.success:
            if not quiet:
                self.get_logger().error(f"{step.name}: {r.message}")
            return None
        self.get_logger().info(
            f"{step.name:18s} {r.duration:4.2f} s   z_carriage {r.joints[0]:.2f} m  joint_1 "
            f"{math.degrees(r.joints[1]):6.1f}  joint_2 {math.degrees(r.joints[2]):6.1f}  "
            f"joint_3 {math.degrees(r.joints[3]):6.1f} deg   crate {step.crate:.2f} m")
        return r.joints

    def hand(self, name, what):
        """Gripper.<what>(), if dexterity is on and its controller runs."""
        if self.gripper is None or getattr(self.gripper, what)():
            return True
        self.get_logger().error(f"{name}: the gripper controller failed")
        return False

    def run(self):
        self.publish_markers()
        self.get_logger().info(f"waiting for motion_server ... (side {self.side}, speed {self.speed:g})")
        self.client.wait_for_server()
        # Right after launch the controllers may still be starting.
        rest = self.steps[-1]
        deadline = time.monotonic() + 30.0
        while self.move(rest, quiet=time.monotonic() < deadline) is None:
            if time.monotonic() > deadline or not rclpy.ok():
                return 1
            time.sleep(1.0)
        if self.gripper is not None and not self.gripper.ready():
            self.get_logger().warning(
                f"dexterity: no {self.gripper.action} action server, so no pitch "
                f"wiggle and grip (pitch and gripper are not in the URDF yet). Going on without.")
            self.gripper = None
        while rclpy.ok():
            t0 = time.monotonic()
            self.where = ["wall"] * len(self.fruit)        # every round a full wall
            self.publish_markers()
            for step in self.steps:
                if self.move(step) is None:
                    return 1
                if step.touch >= 0:
                    if not self.hand(f"{step.name}_grip", "wiggle_and_grip"):
                        return 1
                    self.where[step.touch] = "held"
                    self.publish_markers()
                if step.drop:
                    if not self.hand(f"{step.name}_release", "release"):
                        return 1
                    self.where[self.where.index("held")] = "basket"
                    self.publish_markers()
            self.get_logger().info(f"path done in {time.monotonic() - t0:.1f} s")
            if not self.loop:
                return 0
        return 0


def main():
    rclpy.init()
    node = Demo()
    try:
        return node.run()
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
