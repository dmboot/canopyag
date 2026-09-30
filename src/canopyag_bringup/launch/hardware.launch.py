"""Bring up the REAL robot: ros2_control_node + canopyag_hardware on SocketCAN.

    sudo ip link set can0 type can bitrate 500000
    sudo ip link set can0 up
    ros2 launch canopyag_bringup hardware.launch.py                 # can0
    ros2 launch canopyag_bringup hardware.launch.py rviz:=true      # + RViz with the ghost
    ros2 launch canopyag_bringup hardware.launch.py can_interface:=vcan0   # against mks_sim.py

Which joints are real motors and which are virtual (state = command) is set
per joint in canopyag_description/config/hardware.yaml; `hardware_file:=`
points at another copy of it.

ZERO AT STARTUP: every joint's position when the hardware activates is taken
as its initial_position (0.0 = URDF home). Power the drivers on with the
robot at home.

Note there is no controller_manager node in gazebo.launch.py and there IS one
here: in simulation the gz_ros2_control plugin hosts it inside the sim process.
That asymmetry is the only structural difference between sim and hardware.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    AndSubstitution, Command, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = FindPackageShare("canopyag_description")

    can_interface = LaunchConfiguration("can_interface")
    can_bitrate = LaunchConfiguration("can_bitrate")
    rviz = LaunchConfiguration("rviz")
    ghost = LaunchConfiguration("ghost")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([desc_pkg, "urdf", "canopyag.urdf.xacro"]),
            " sim:=false",
            " can_interface:=", can_interface,
            " can_bitrate:=", can_bitrate,
            " hardware_file:=", LaunchConfiguration("hardware_file"),
        ]),
        value_type=str,
    )

    controllers = PathJoinSubstitution([desc_pkg, "config", "controllers.yaml"])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}, controllers],
        # hw_log_level:=debug -> every F5 in counts / motor revs / joint units,
        # frame rates and missed replies every 10 s
        arguments=["--ros-args", "--log-level",
                   ["canopyag.CanopyagSystem:=", LaunchConfiguration("hw_log_level")]],
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

    # The commanded robot, see-through in RViz on top of the measured one:
    # ghost_state.py -> /ghost/joint_states -> robot_state_publisher (cmd/ frames).
    with_ghost = IfCondition(AndSubstitution(rviz, ghost))
    ghost_nodes = [
        Node(
            package="canopyag_hardware",
            executable="ghost_state.py",
            condition=with_ghost,
            output="log",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="ghost",
            parameters=[{"robot_description": robot_description, "frame_prefix": "cmd/",
                         "use_sim_time": False}],
            remappings=[("joint_states", "/ghost/joint_states")],
            condition=with_ghost,
            output="log",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--frame-id", "world", "--child-frame-id", "cmd/world"],
            condition=with_ghost,
            output="log",
        ),
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution(
            [FindPackageShare("canopyag_bringup"), "rviz", "hardware.rviz"])],
        condition=IfCondition(rviz),
        output="log",
    )

    return LaunchDescription([
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("can_bitrate", default_value="500000",
                              description="Informational: the bitrate is set with ip link."),
        DeclareLaunchArgument(
            "hardware_file",
            default_value=PathJoinSubstitution([desc_pkg, "config", "hardware.yaml"]),
            description="Joint <-> motor mapping (CAN IDs, gearing, can/virtual)."),
        DeclareLaunchArgument("hw_log_level", default_value="info",
                              description="Log level of the CAN plugin (debug: every frame)."),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("ghost", default_value="true",
                              description="With rviz: also draw the commanded robot."),

        control_node,
        robot_state_publisher,
        jsb,
        # Both position controllers are loaded INACTIVE on real hardware: the
        # joints are zeroed where the robot was switched on, so check the
        # robot is at home first, then activate ONE of them:
        #   ros2 control switch_controllers --activate forward_position_controller
        #   ros2 control switch_controllers --activate arm_controller
        # They claim the same joints and cannot be active together.
        RegisterEventHandler(OnProcessExit(
            target_action=jsb,
            on_exit=[
                spawner("arm_controller", "--inactive"),
                spawner("forward_position_controller", "--inactive"),
                spawner("crate_controller"),
            ],
        )),
        *ghost_nodes,
        rviz_node,
    ])
