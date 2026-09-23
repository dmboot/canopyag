#!/usr/bin/env python3
"""Hard-coded demo path: the arm works its way up the gantry, the crate follows.

    ros2 launch canopyag_moveit_config demo.launch.py [loop:=true]

The arm (carriage + three revolutes) goes through WAYPOINTS with MoveIt's
Pilz PTP planner: one plan per waypoint, all four joints start and stop
together, collision-checked against the boxes in robot_parameters.yaml.

The crate is not planned. It trails CRATE_TRAIL below the carriage on its own
slow trajectory, straight to crate_controller, and keeps moving while the arm
does. Two rules keep it under the carriage:

  * its target is never above min(carriage now, carriage goal) - CRATE_TRAIL,
    so while it climbs it can only close in on where the carriage will be;
  * before the carriage is sent below the crate + GAP, the crate is brought
    down first and the arm waits for it.

A monitor on /joint_states reports any moment the gap drops below GAP.
"""

import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             MoveItErrorCodes, PlanningOptions)
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = ["z_carriage", "joint_1", "joint_2", "joint_3"]

GAP = 0.02               # m, crate top to carriage bottom, never less
CRATE_TRAIL = 0.15       # m, how far below the carriage the crate follows
CRATE_UP_SPEED = 0.03    # m/s, the slow creep
CRATE_DOWN_SPEED = 0.10  # m/s, getting out of the carriage's way
CRATE_MAX = 1.215        # m, z_crate upper limit in robot_parameters.yaml

# At home the crate and the carriage touch (gap 0), so the path starts and
# ends at REST, with the carriage lifted clear.
REST = (0.05, 0.0, 0.0, 0.0)

# (name, [z_carriage, joint_1, joint_2, joint_3], velocity scaling)
# The z values are where the carriage is, so the crate follows the climb.
WAYPOINTS = [("rest", REST, 0.5)]
for i, z in enumerate((0.35, 0.65, 0.95)):
    WAYPOINTS += [
        (f"level{i}_reach_left",  (z, 0.8, -1.1, 0.6), 0.6),
        (f"level{i}_pick_left",   (z + 0.05, 0.6, -0.6, 1.4), 0.3),
        (f"level{i}_reach_right", (z + 0.05, -0.8, 1.1, -0.6), 0.6),
        (f"level{i}_pick_right",  (z + 0.10, -0.6, 0.6, -1.4), 0.3),
        (f"level{i}_stow",        (z + 0.10, 0.0, 0.0, 0.0), 0.6),
    ]
WAYPOINTS += [("back_down", (0.30, 0.0, 0.0, 0.0), 0.8), ("rest", REST, 0.5)]


class DemoPath(Node):

    def __init__(self):
        super().__init__("demo_path")
        self.declare_parameter("loop", False)

        self.move_group = ActionClient(self, MoveGroup, "move_action")
        self.crate = ActionClient(self, FollowJointTrajectory,
                                  "crate_controller/follow_joint_trajectory")

        self.pos = {}
        self.car_goal = None       # where the carriage is being sent
        self.crate_sent = None     # last crate target sent
        self.gap_armed = False     # the gap only counts once it has opened up
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
        if gap >= GAP:
            self.gap_armed = True
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
        speed = CRATE_UP_SPEED if target > now else CRATE_DOWN_SPEED
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
            if time.time() > deadline:
                self.get_logger().error(f"{name}: crate did not clear the way")
                return False
            time.sleep(0.05)

        req = MotionPlanRequest()
        req.group_name = "arm"
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "PTP"
        req.num_planning_attempts = 1
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = scale
        req.max_acceleration_scaling_factor = scale
        req.start_state.is_diff = True
        req.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=float(p),
                            tolerance_above=1e-3, tolerance_below=1e-3, weight=1.0)
            for j, p in zip(ARM_JOINTS, pose)
        ])]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        handle = self.wait(self.move_group.send_goal_async(goal))
        if not handle.accepted:
            self.get_logger().error(f"{name}: move_group rejected the goal")
            return False
        code = self.wait(handle.get_result_async()).result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"{name}: failed, MoveIt error code {code}")
            return False
        self.get_logger().info(
            f"{name:22s} carriage {self.pos['z_carriage']:.2f} m  "
            f"crate {self.pos['z_crate']:.2f} m")
        return True

    @staticmethod
    def wait(future):
        while not future.done():
            time.sleep(0.01)
        return future.result()

    # ------------------------------------------------------------------ run

    def run(self):
        self.get_logger().info("waiting for move_group and crate_controller ...")
        self.move_group.wait_for_server()
        self.crate.wait_for_server()
        while "z_carriage" not in self.pos or "z_crate" not in self.pos:
            time.sleep(0.1)

        while rclpy.ok():
            for name, pose, scale in WAYPOINTS:
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
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
