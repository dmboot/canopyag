#!/usr/bin/env python3
"""Send the robot somewhere and wait until it is there.

    ros2 run canopyag_motion move_to.py 0.45 0.55 0.7             # tool point, world, m
    ros2 run canopyag_motion move_to.py 0.45 0.55 0.7 --speed 0.3
    ros2 run canopyag_motion move_to.py --joints 0.3 60 15         # z_carriage m, joint_1/2 deg
    ros2 run canopyag_motion move_to.py --joints 0.3 1.05 0.26 --rad

Needs motion_server.py running (canopyag_motion motion.launch.py). Ctrl+C
cancels the goal: the robot brakes to a stop.
"""

import argparse
import math
import sys

import rclpy
from canopyag_interfaces.action import MoveTo
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("values", type=float, nargs=3, metavar="V",
                    help="x y z of the tool point (m), or with --joints: z_carriage joint_1 joint_2")
    ap.add_argument("--joints", action="store_true", help="the values are joints, not a point")
    ap.add_argument("--rad", action="store_true", help="with --joints: radians, not degrees")
    ap.add_argument("--speed", type=float, default=1.0, help="0..1 of the limits (default 1)")
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    goal = MoveTo.Goal(speed=args.speed)
    if args.joints:
        z, j1, j2 = args.values
        if not args.rad:
            j1, j2 = math.radians(j1), math.radians(j2)
        goal.joints = [z, j1, j2]
    else:
        goal.point = Point(x=args.values[0], y=args.values[1], z=args.values[2])

    # Ctrl+C must leave the context alive, to cancel the goal.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("move_to")
    client = ActionClient(node, MoveTo, "move_to")
    if not client.wait_for_server(timeout_sec=5.0):
        print("no move_to action server: ros2 launch canopyag_motion motion.launch.py")
        return 1
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    handle = future.result()
    result_future = handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, result_future)
    except KeyboardInterrupt:
        print("cancelling: braking to a stop")
        rclpy.spin_until_future_complete(node, handle.cancel_goal_async(), timeout_sec=5.0)
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)
    res = result_future.result()
    if res is None:
        return 1
    r = res.result
    if r.joints:
        print(f"joints: z_carriage {r.joints[0]:.3f} m, joint_1 {math.degrees(r.joints[1]):.1f} deg, "
              f"joint_2 {math.degrees(r.joints[2]):.1f} deg ({r.duration:.2f} s planned)")
    print(("OK: " if r.success else "FAILED: ") + r.message)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if r.success else 1


if __name__ == "__main__":
    sys.exit(main())
