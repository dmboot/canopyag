#!/usr/bin/env python3
"""Demo: pick fruit off a wall beside the gantry, as fast as the robot can.

    ros2 launch canopyag_motion demo.launch.py [side:=left] [loop:=true] [speed:=0.5]   # mock
    ros2 launch canopyag_motion motion.launch.py demo:=true speed:=0.3                # real robot

The wall stands 0.40 m from the gantry, on the `side` (right / left, seen
from the front of the gantry: world +y / -y). Ten fruit on it, five rows
from the carriage's lowest to its highest: the carriage goes up row by row,
the crate follows it slower, just under it. Every fruit: in front of it, in
slowly until the gripper tip touches the wall, then link 2 turns fast under
link 1 to drop it in the crate, and on to the next. The gripper points to the
wall the whole way. The path and why it is shaped the way it is (the arm
must stay off the wall, which the planner does not know about):
canopyag_motion/wall_demo.py.

Publishes demo_markers (visualization_msgs/MarkerArray, latched): a red
sphere at every fruit and the wall.

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
        self.steps = wall_demo.path(model, planner.gap, s)
        self.fruit = [p for p, _ in wall_demo.touches(model, planner.gap, s)]
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
        """A red sphere at every fruit, and the wall behind them."""
        out = []
        for i, (x, y, z) in enumerate(self.fruit):
            m = Marker(type=Marker.SPHERE, ns="fruit", id=i, action=Marker.ADD)
            m.pose.position = Point(x=x, y=y, z=z)
            m.scale.x = m.scale.y = m.scale.z = MARKER
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 1.0
            out.append(m)
        xs, ys, zs = zip(*self.fruit)
        wall = Marker(type=Marker.CUBE, ns="wall", id=0, action=Marker.ADD)
        thick = 0.01
        side = math.copysign(1.0, ys[0])
        wall.pose.position = Point(x=(min(xs) + max(xs)) / 2, y=ys[0] + side * thick / 2,
                                   z=(min(zs) + max(zs)) / 2)
        wall.scale.x, wall.scale.y = max(xs) - min(xs) + 0.2, thick
        wall.scale.z = max(zs) - min(zs) + 0.2
        wall.color.r, wall.color.g, wall.color.b, wall.color.a = 0.6, 0.6, 0.6, 0.3
        out.append(wall)
        for m in out:
            m.header.frame_id = self.frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.pose.orientation.w = 1.0
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
            for step in self.steps:
                if self.move(step) is None:
                    return 1
                if step.touch >= 0 and not self.hand(f"{step.name}_grip", "wiggle_and_grip"):
                    return 1
                if step.drop and not self.hand(f"{step.name}_release", "release"):
                    return 1
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
