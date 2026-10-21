"""The robot on mock hardware: ros2_control without motors.

    ros2 launch canopyag_bringup mock.launch.py                 # + RViz
    ros2 launch canopyag_bringup mock.launch.py rviz:=false

mock_components/GenericSystem reflects every command straight back as state,
so everything that drives the controllers (canopyag_motion, the sliders, the
forward_position_controller) works as on the robot, minus the physics. The
controllers come up ACTIVE here (arm_controller and crate_controller), unlike
hardware.launch.py: there is no robot to put at home first.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = FindPackageShare("canopyag_description")
    robot_description = ParameterValue(
        Command(["xacro ", PathJoinSubstitution([desc_pkg, "urdf", "canopyag.urdf.xacro"]),
                 " sim:=false mock:=true"]),
        value_type=str)
    controllers = PathJoinSubstitution([desc_pkg, "config", "controllers.yaml"])

    def spawner(name, *extra):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager", *extra],
            output="screen",
        )

    jsb = spawner("joint_state_broadcaster")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description, "use_sim_time": False}],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": robot_description, "use_sim_time": False},
                        controllers],
            output="screen",
        ),
        jsb,
        RegisterEventHandler(OnProcessExit(
            target_action=jsb,
            on_exit=[spawner("arm_controller"), spawner("crate_controller"),
                     spawner("forward_position_controller", "--inactive")],
        )),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution(
                [FindPackageShare("canopyag_bringup"), "rviz", "motion.rviz"])],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="log",
        ),
    ])
