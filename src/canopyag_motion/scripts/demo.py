#!/usr/bin/env python3
"""Demo: pick fruit up one side of the gantry, as fast as the robot can.

    ros2 launch canopyag_motion demo.launch.py [side:=left] [loop:=true] [speed:=0.5]   # mock
    ros2 launch canopyag_motion motion.launch.py demo:=true speed:=0.3                # real robot

Every fruit: move in front of it, go in (slowly) with the gripper pointing
at it, then drop it - link 2 turns 180 deg and folds under link 1 - and on to
the next one. The fruit are world points of the gripper tip (FRUIT);
motion_server.py works out the joints, moves them all at once, keeps the
gripper pointing to the fruit side the whole way (joint_3) and the crate
under the carriage.

`side` (right / left, seen from the front of the gantry) mirrors FRUIT and
points the gripper that way: right is world +y, left -y.
`speed` (0..1] scales every move.
`dexterity` (off; demo.launch.py turns it on for mock): at every fruit a short wiggle of the gripper
pitch and a grip, and a release at the drop. Pitch and gripper are not in the
URDF and not planned: they go to their own FollowJointTrajectory controller
(`gripper_controller`, joints `gripper_joints` = [pitch, gripper]). Leave it
off until those motors and that controller exist; without the controller the
demo says so once and goes on without.
"""

import math
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from canopyag_interfaces.action import MoveTo
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Gripper tip at every fruit (world, m), right side. The gripper points to
# +y; it goes in along that from APPROACH closer to the gantry.
FRUIT = [
    (0.46, 0.55, 0.30), (0.28, 0.60, 0.36),
    (0.46, 0.55, 0.45), (0.28, 0.60, 0.51),
    (0.46, 0.55, 0.60), (0.28, 0.60, 0.66),
]
APPROACH = 0.06            # m
PICK_SPEED = 0.4           # of `speed`, for going in
# Drop: joint_1 here, link 2 turned 180 deg from its pick angle - folded under
# link 1, the gripper still pointing to the side. joint_3 is not sent: it
# follows from the heading.
DROP_JOINT_1 = 0.0
REST = [0.05, 0.0, 0.0]    # z_carriage, joint_1, joint_2: carriage just clear of the crate
# dexterity: pitch +- this and back (rad), each step this long (s).
WIGGLE = 0.35
WIGGLE_STEP = 0.25


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
        self.client = ActionClient(self, MoveTo, "move_to")
        self.gripper = None
        if self.declare_parameter("dexterity", False).value:
            self.gripper = Gripper(
                self, self.declare_parameter("gripper_controller", "gripper_controller").value,
                self.declare_parameter("gripper_joints", ["wrist_pitch", "gripper"]).value,
                self.declare_parameter("grip_closed", 1.0).value,
                self.declare_parameter("grip_open", 0.0).value)

    def move(self, name, point=None, joints=None, speed=1.0, quiet=False):
        goal = MoveTo.Goal(speed=speed * self.speed, heading=self.heading)
        if joints is not None:
            goal.joints = [float(v) for v in joints]
        else:
            goal.point = Point(x=point[0], y=point[1], z=point[2])
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        r = result_future.result().result
        if not r.success:
            if not quiet:
                self.get_logger().error(f"{name}: {r.message}")
            return None
        self.get_logger().info(
            f"{name:22s} {r.duration:4.2f} s   z_carriage {r.joints[0]:.2f} m  joint_1 "
            f"{math.degrees(r.joints[1]):6.1f}  joint_2 {math.degrees(r.joints[2]):6.1f}  "
            f"joint_3 {math.degrees(r.joints[3]):6.1f} deg")
        return r.joints

    def hand(self, name, what):
        """Gripper.<what>(), if dexterity is on and its controller runs."""
        if self.gripper is None or getattr(self.gripper, what)():
            return True
        self.get_logger().error(f"{name}: the gripper controller failed")
        return False

    def run(self):
        self.get_logger().info(f"waiting for motion_server ... (side {self.side}, speed {self.speed:g})")
        self.client.wait_for_server()
        # Right after launch the controllers may still be starting.
        deadline = time.monotonic() + 30.0
        while self.move("rest", joints=REST, quiet=time.monotonic() < deadline) is None:
            if time.monotonic() > deadline or not rclpy.ok():
                return 1
            time.sleep(1.0)
        if self.gripper is not None and not self.gripper.ready():
            self.get_logger().warning(
                f"dexterity: no {self.gripper.action} action server, so no pitch "
                f"wiggle and grip (pitch and gripper are not in the URDF yet). Going on without.")
            self.gripper = None
        s = 1.0 if self.side == "right" else -1.0
        while rclpy.ok():
            t0 = time.monotonic()
            for i, (x, y, z) in enumerate(FRUIT):
                fruit = (x, s * y, z)
                near = (x, s * (y - APPROACH), z)
                if self.move(f"fruit{i}_approach", point=near) is None:
                    return 1
                picked = self.move(f"fruit{i}_pick", point=fruit, speed=PICK_SPEED)
                if picked is None:
                    return 1
                if not self.hand(f"fruit{i}_grip", "wiggle_and_grip"):
                    return 1
                j2 = picked[2] - math.pi if picked[2] > 0.0 else picked[2] + math.pi
                drop = [picked[0], DROP_JOINT_1, j2]
                if self.move(f"fruit{i}_drop", joints=drop) is None:
                    return 1
                if not self.hand(f"fruit{i}_release", "release"):
                    return 1
            if self.move("rest", joints=REST) is None:
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
