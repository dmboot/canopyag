#!/usr/bin/env python3
"""
ghost_state.py  --  what the robot is TOLD to do, as joint states for RViz.

Publishes /ghost/joint_states: every joint at its commanded position, taken
from whichever controller commanded it last:
  /arm_controller/controller_state        (reference of the trajectory)
  /crate_controller/controller_state
  /forward_position_controller/commands   (bring-up steps)
Joints nobody has commanded yet sit at their measured position.

hardware.launch.py rviz:=true feeds this to a second robot_state_publisher
with frame_prefix cmd/, drawn see-through on top of the real robot: where the
two differ, the hardware is not following (lag, direction, scaling).
"""

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class GhostState(Node):
    def __init__(self):
        super().__init__("ghost_state")
        # Must match forward_position_controller.joints in controllers.yaml.
        self.forward_joints = self.declare_parameter(
            "forward_joints", ["z_carriage", "joint_1", "joint_2"]).value
        self.commanded = {}
        self.pub = self.create_publisher(JointState, "/ghost/joint_states", 10)
        for ctrl in ("arm_controller", "crate_controller"):
            self.create_subscription(JointTrajectoryControllerState,
                                     f"/{ctrl}/controller_state", self.on_jtc, 10)
        self.create_subscription(Float64MultiArray, "/forward_position_controller/commands",
                                 self.on_forward, 10)
        self.create_subscription(JointState, "/joint_states", self.on_measured,
                                 qos_profile_sensor_data)

    def on_jtc(self, msg):
        # Humble has both `reference` and the deprecated `desired`.
        ref = msg.reference if msg.reference.positions else msg.desired
        self.commanded.update(zip(msg.joint_names, ref.positions))

    def on_forward(self, msg):
        self.commanded.update(zip(self.forward_joints, msg.data))

    def on_measured(self, msg):
        out = JointState()
        out.header.stamp = msg.header.stamp
        out.name = list(msg.name)
        out.position = [self.commanded.get(n, p) for n, p in zip(msg.name, msg.position)]
        self.pub.publish(out)


def main():
    rclpy.init()
    try:
        rclpy.spin(GhostState())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
