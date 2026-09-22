"""Show the robot in RViz with a slider per joint.

No Gazebo, no controllers - the fast loop for checking that
config/robot_parameters.yaml matches the CAD.

    ros2 launch canopyag_description view_robot.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("canopyag_description")
    gui = LaunchConfiguration("gui")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([pkg, "urdf", "canopyag.urdf.xacro"]),
            " sim:=false",
            " params_file:=", LaunchConfiguration("params_file"),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([pkg, "config", "robot_parameters.yaml"]),
            description="Point this at another yaml to view a different build "
                        "of the robot without touching the installed one.",
        ),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Show the joint_state_publisher slider window.",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(gui),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution([pkg, "rviz", "view_robot.rviz"])],
            output="screen",
        ),
    ])
