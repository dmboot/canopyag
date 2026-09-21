"""Bring up the REAL arm: ros2_control_node + the CAN/serial hardware interfaces.

    sudo ip link set can0 type can bitrate 500000
    sudo ip link set can0 up
    ros2 launch canopyag_bringup hardware.launch.py can_interface:=can0

Note there is no controller_manager node in gazebo.launch.py and there IS one
here: in simulation the gz_ros2_control plugin hosts it inside the sim process.
That asymmetry is the only structural difference between sim and hardware.

Requires the canopyag_hardware package, which does not exist yet.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = FindPackageShare("canopyag_description")

    can_interface = LaunchConfiguration("can_interface")
    can_bitrate = LaunchConfiguration("can_bitrate")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([desc_pkg, "urdf", "canopyag.urdf.xacro"]),
            " sim:=false",
            " can_interface:=", can_interface,
            " can_bitrate:=", can_bitrate,
        ]),
        value_type=str,
    )

    controllers = PathJoinSubstitution([desc_pkg, "config", "controllers.yaml"])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}, controllers],
        output="both",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
        output="screen",
    )

    def spawner(name, *extra):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager", *extra],
            output="screen",
        )

    jsb = spawner("joint_state_broadcaster")

    return LaunchDescription([
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("can_bitrate", default_value="500000"),

        control_node,
        robot_state_publisher,
        jsb,
        # The arm controller is loaded INACTIVE on real hardware. Home the axes
        # against the limit switches first, then:
        #   ros2 control switch_controllers --activate arm_controller
        # Activating a trajectory controller on an unhomed stepper arm means
        # joint 0 is wherever the arm happened to be powered on.
        RegisterEventHandler(OnProcessExit(
            target_action=jsb,
            on_exit=[
                spawner("arm_controller", "--inactive"),
                spawner("gripper_controller"),
                spawner("tool_roll_controller"),
            ],
        )),
    ])
