#!/usr/bin/env python3
"""Demo: pick fruit up one side of the gantry, as fast as the robot can.

    ros2 launch canopyag_motion demo.launch.py [side:=left] [loop:=true] [speed:=0.5]   # mock
    ros2 launch canopyag_motion motion.launch.py demo:=true speed:=0.3                # real robot

Every fruit: move in front of it, go in (slowly), then drop it - link 2 turns
180 deg and folds under link 1 - and on to the next one. The fruit are world
points (FRUIT); motion_server.py works out the joints, moves them all at once
and keeps the crate under the carriage.

`side` (right / left, seen from the front of the gantry) mirrors FRUIT.
`speed` (0..1] scales every move.
"""

import math
import sys
import time

import rclpy
from canopyag_interfaces.action import MoveTo
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.node import Node

# Tool point of every fruit (world, m), right side. The arm reaches in from
# APPROACH closer to the gantry, sideways.
FRUIT = [
    (0.48, 0.58, 0.60), (0.28, 0.64, 0.70),
    (0.48, 0.58, 0.90), (0.28, 0.64, 1.00),
    (0.48, 0.58, 1.20), (0.28, 0.64, 1.30),
]
APPROACH = 0.06            # m
PICK_SPEED = 0.4           # of `speed`, for going in
# Drop: joint_1 here, link 2 turned 180 deg from its pick angle - folded under
# link 1. Folded, link 2's tip is a few cm from the carriage: with the 2 cm
# collision margin (motion.yaml) that only fits with joint_1 within ~5 deg of
# straight ahead.
DROP_JOINT_1 = 0.0
REST = [0.05, 0.0, 0.0]    # z_carriage, joint_1, joint_2: carriage just clear of the crate


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
        self.client = ActionClient(self, MoveTo, "move_to")

    def move(self, name, point=None, joints=None, speed=1.0, quiet=False):
        goal = MoveTo.Goal(speed=speed * self.speed)
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
            f"{math.degrees(r.joints[1]):6.1f}  joint_2 {math.degrees(r.joints[2]):6.1f} deg")
        return r.joints

    def run(self):
        self.get_logger().info(f"waiting for motion_server ... (side {self.side}, speed {self.speed:g})")
        self.client.wait_for_server()
        # Right after launch the controllers may still be starting.
        deadline = time.monotonic() + 30.0
        while self.move("rest", joints=REST, quiet=time.monotonic() < deadline) is None:
            if time.monotonic() > deadline or not rclpy.ok():
                return 1
            time.sleep(1.0)
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
                j2 = picked[2] - math.pi if picked[2] > math.pi else picked[2] + math.pi
                drop = [picked[0], DROP_JOINT_1, j2]
                if self.move(f"fruit{i}_drop", joints=drop) is None:
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
