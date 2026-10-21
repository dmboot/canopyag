#!/usr/bin/env python3
"""Moves the robot to world points or joint values, as fast as it can.

    ros2 launch canopyag_motion motion.launch.py           # this node
    ros2 run canopyag_motion move_to.py 0.45 0.5 0.5       # a goal from the shell

Goals:
  move_to     (canopyag_interfaces/action/MoveTo)   a point or joint values,
              with feedback and a result
  goal_point  (geometry_msgs/PointStamped)          a point, fire and forget -
              what RViz's "Publish Point" tool sends

Plans with canopyag_motion.Planner (all joints at once, clear of the robot's
own collision boxes, crate kept under the carriage) and sends the trajectory
straight to arm_controller and crate_controller, both starting at the same
instant. A new goal replaces the running one: the new motion starts from
where the old one is at that moment, at its speed. Cancelling the action
brakes the robot to a stop.

Publishes goal_markers (visualization_msgs/MarkerArray): the goal and the
tool point's path, in the world frame.
"""

import os
import threading
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from canopyag_interfaces.action import MoveTo
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, PointStamped
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration as RclDuration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

import tf2_geometry_msgs  # noqa: F401  (registers PointStamped with tf2)
from tf2_ros import Buffer, TransformListener

import canopyag_motion
from canopyag_motion import ARM, CRATE, JOINTS, WRIST, PlanError

CONTROLLERS = {"arm_controller": list(ARM) + [WRIST], "crate_controller": [CRATE]}


class Motion:
    """One planned motion on its way to the controllers."""

    REPLACED = "replaced by a new goal"

    def __init__(self, plan, t0, logger):
        self.plan = plan
        self.logger = logger
        self.t0 = t0                  # rclpy Time of the plan's t = 0
        self.done = threading.Event()
        self.success = False
        self.message = ""
        self.pending = set(CONTROLLERS)

    def finish(self, success, message):
        if not self.done.is_set():
            self.success, self.message = success, message
            self.done.set()
            if not success and message != Motion.REPLACED:
                self.logger.error(message)

    def elapsed(self, now):
        return (now - self.t0).nanoseconds * 1e-9


class MotionServer(Node):

    def __init__(self):
        super().__init__("motion_server")
        desc = get_package_share_directory("canopyag_description")
        here = get_package_share_directory("canopyag_motion")
        params_file = self.declare_parameter(
            "robot_parameters", os.path.join(desc, "config", "robot_parameters.yaml")).value
        config_file = self.declare_parameter(
            "motion_config", os.path.join(here, "config", "motion.yaml")).value
        self.frame = self.declare_parameter("frame", "world").value
        self.model, self.planner, cfg = canopyag_motion.load(params_file, config_file)
        self.start_delay = float(cfg["execution"]["start_delay"])
        self.point_dt = float(cfg["execution"]["point_dt"])

        self.lock = threading.Lock()
        self.current = None
        self.measured = {}
        group = ReentrantCallbackGroup()

        self.create_subscription(JointState, "joint_states", self.on_joint_states,
                                 qos_profile_sensor_data, callback_group=group)
        self.create_subscription(PointStamped, "goal_point", self.on_goal_point, 10,
                                 callback_group=group)
        self.markers = self.create_publisher(MarkerArray, "goal_markers", 10)
        self.jtc = {name: ActionClient(self, FollowJointTrajectory,
                                           f"{name}/follow_joint_trajectory", callback_group=group)
                        for name in CONTROLLERS}
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        ActionServer(self, MoveTo, "move_to", execute_callback=self.execute,
                     goal_callback=lambda _: GoalResponse.ACCEPT,
                     cancel_callback=lambda _: CancelResponse.ACCEPT,
                     callback_group=group)

        lo, hi = self.model.limits("z_carriage")
        self.get_logger().info(
            f"ready: tool point {cfg['tool']['xyz']} on {cfg['tool']['link']}, reaches "
            f"{abs(self.model.link1 - self.model.link2):.2f}..{self.model.link1 + self.model.link2:.2f}"
            f" m around joint_1 at ({self.model.j1_xy[0]:.3f}, {self.model.j1_xy[1]:.3f}), z "
            f"{lo + self.model.tool_z0:.3f}..{hi + self.model.tool_z0:.3f} m")

    # ------------------------------------------------------------------ inputs

    def on_joint_states(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in JOINTS:
                self.measured[name] = pos

    def on_goal_point(self, msg):
        try:
            p = self.to_frame(msg)
            self.start("point", np.array([p.x, p.y, p.z]), 1.0)
        except (PlanError, RuntimeError) as e:
            self.get_logger().error(f"goal_point: {e}")

    def to_frame(self, msg):
        if msg.header.frame_id in ("", self.frame):
            return msg.point
        try:
            return self.tf.transform(msg, self.frame,
                                     timeout=RclDuration(seconds=0.2)).point
        except Exception as e:  # tf2 raises several unrelated types
            raise RuntimeError(f"cannot transform {msg.header.frame_id} -> {self.frame}: {e}")

    def execute(self, handle):
        req = handle.request
        result = MoveTo.Result()
        speed = req.speed if req.speed > 0.0 else 1.0
        try:
            if len(req.joints):
                motion = self.start("joints", np.array(req.joints, float), speed)
            else:
                motion = self.start("point", np.array([req.point.x, req.point.y, req.point.z]), speed)
        except (PlanError, RuntimeError) as e:
            result.message = str(e)
            handle.abort()
            return result
        result.joints = [float(v) for v in motion.plan.goal]
        result.duration = motion.plan.duration
        feedback = MoveTo.Feedback()
        while not motion.done.wait(0.1):
            if handle.is_cancel_requested:
                stop = self.halt(motion)
                if stop is not None:
                    stop.done.wait(10.0)
                result.message = "cancelled: braked to a stop"
                handle.canceled()
                return result
            feedback.time_left = max(motion.plan.duration - motion.elapsed(self.get_clock().now()), 0.0)
            handle.publish_feedback(feedback)
        result.success, result.message = motion.success, motion.message
        if motion.success:
            handle.succeed()
        else:
            handle.abort()
        return result

    # ------------------------------------------------------------------ motion

    def state_at(self, t):
        """joint -> (position, velocity) at time t: from the running motion if
        there is one, otherwise the measured position at rest."""
        cur = self.current
        if cur is not None and not cur.done.is_set():
            return cur.plan.state(cur.elapsed(t))
        missing = [j for j in JOINTS if j not in self.measured]
        if missing:
            raise RuntimeError(f"no /joint_states for {', '.join(missing)} yet")
        return {j: (self.measured[j], 0.0) for j in JOINTS}

    def start(self, kind, goal, speed):
        """Plan and send. Raises PlanError (the running motion goes on)."""
        # Before the clock starts: right after launch a controller may still
        # be coming up.
        for name, client in self.jtc.items():
            if not client.server_is_ready() and not client.wait_for_server(timeout_sec=5.0):
                raise RuntimeError(f"{name} is not running (no action server)")
        with self.lock:
            lead = self.start_delay
            for _ in range(4):
                clock = self.get_clock()
                t0 = clock.now() + RclDuration(seconds=lead)
                moving = self.current is not None and not self.current.done.is_set()
                start = self.state_at(t0)
                tic = time.monotonic()
                if kind == "point":
                    plan = self.planner.to_point(start, goal, speed)
                else:
                    plan = self.planner.to_joints(start, goal, speed)
                took = time.monotonic() - tic
                if not moving:
                    # From rest the start time does not change the plan.
                    t0 = clock.now() + RclDuration(seconds=self.start_delay)
                    break
                if clock.now() + RclDuration(seconds=0.01) < t0:
                    break
                # Planning took longer than the lead: the robot is past the
                # start state already. Again, with more lead.
                lead = 1.5 * took + self.start_delay
            else:
                raise PlanError(f"planning takes too long ({took:.2f} s) to join the running motion")
            what = (f"({goal[0]:.3f}, {goal[1]:.3f}, {goal[2]:.3f})" if kind == "point" else
                    f"joints ({', '.join(f'{v:.3f}' for v in goal)})")
            return self.send(plan, t0, f"{what} [planned in {1000 * took:.0f} ms]",
                             goal if kind == "point" else None)

    def halt(self, motion):
        with self.lock:
            if self.current is not motion or motion.done.is_set():
                return None
            t0 = self.get_clock().now() + RclDuration(seconds=self.start_delay)
            plan = self.planner.stop(motion.plan.state(motion.elapsed(t0)))
            return self.send(plan, t0, "stop", None)

    def send(self, plan, t0, what, point):
        """Make `plan` the running motion (caller holds the lock)."""
        if self.current is not None:
            self.current.finish(False, Motion.REPLACED)
        motion = Motion(plan, t0, self.get_logger())
        self.current = motion
        notes = f" ({'; '.join(plan.notes)})" if plan.notes else ""
        self.get_logger().info(
            f"{what}: joints ({plan.goal[0]:.3f} m, {np.degrees(plan.goal[1]):.1f} deg, "
            f"{np.degrees(plan.goal[2]):.1f} deg, {np.degrees(plan.goal[3]):.1f} deg) "
            f"in {plan.duration:.2f} s{notes}")
        self.publish_markers(plan, point)
        if plan.duration < 1e-6:
            motion.finish(True, "already there")
            return motion

        # Every point_dt, then the end exactly once: times must strictly
        # increase or the controller rejects the whole trajectory.
        t = np.arange(1, int(plan.duration / self.point_dt) + 2) * self.point_dt
        t = np.append(t[t < plan.duration - 1e-6], plan.duration)
        samples = plan.sample(t)
        for name, joints in CONTROLLERS.items():
            traj = JointTrajectory(joint_names=joints)
            traj.header.stamp = t0.to_msg()
            for k, tk in enumerate(t):
                pt = JointTrajectoryPoint(
                    positions=[float(samples[j][0][k]) for j in joints],
                    velocities=[float(samples[j][1][k]) for j in joints],
                    accelerations=[float(samples[j][2][k]) for j in joints])
                sec = int(tk)
                pt.time_from_start = Duration(sec=sec, nanosec=int(round((tk - sec) * 1e9)))
                traj.points.append(pt)
            goal = FollowJointTrajectory.Goal(trajectory=traj)
            self.jtc[name].send_goal_async(goal).add_done_callback(
                lambda f, name=name: self.on_accepted(motion, name, f))
        return motion

    def on_accepted(self, motion, name, future):
        handle = future.result()
        if not handle.accepted:
            motion.finish(False, f"{name} rejected the trajectory: is it active? "
                                 f"ros2 control switch_controllers --activate {name}")
            return
        handle.get_result_async().add_done_callback(
            lambda f: self.on_result(motion, name, f))

    def on_result(self, motion, name, future):
        res = future.result().result
        if res.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            motion.finish(False, f"{name}: error {res.error_code} {res.error_string}".strip())
            return
        motion.pending.discard(name)
        if not motion.pending:
            motion.finish(True, "arrived")

    def publish_markers(self, plan, point):
        t = np.linspace(0.0, plan.duration, 60)
        s = plan.sample(t)
        tool = self.model.tool_position({j: s[j][0] for j in JOINTS})
        path = Marker(type=Marker.LINE_STRIP, ns="tool_path", id=0, action=Marker.ADD)
        path.scale.x = 0.005
        path.color.r, path.color.g, path.color.b, path.color.a = 0.1, 0.6, 1.0, 1.0
        path.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in tool.T]
        goal = Marker(type=Marker.SPHERE, ns="goal", id=0, action=Marker.ADD)
        end = tool[:, -1] if point is None else point
        goal.pose.position = Point(x=float(end[0]), y=float(end[1]), z=float(end[2]))
        goal.pose.orientation.w = 1.0
        goal.scale.x = goal.scale.y = goal.scale.z = 0.03
        goal.color.r, goal.color.g, goal.color.b, goal.color.a = 1.0, 0.4, 0.0, 1.0
        msgs = MarkerArray(markers=[path, goal])
        for m in msgs.markers:
            m.header.frame_id = self.frame
            m.header.stamp = self.get_clock().now().to_msg()
        self.markers.publish(msgs)


def main():
    rclpy.init()
    node = MotionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():      # not just Ctrl+C tearing the context down mid-spin
            raise
    finally:
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
