"""Show the arm in RViz with sliders for every joint.

No Gazebo, no controllers - this is the fast loop for checking that the
kinematics in config/arm_parameters.yaml actually match the CAD. Drive each
slider to its limit and compare against SolidWorks.

    ros2 launch canopyag_description view_robot.launch.py
    ros2 launch canopyag_description view_robot.launch.py use_meshes:=true
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

    use_meshes = LaunchConfiguration("use_meshes")
    gui = LaunchConfiguration("gui")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([pkg, "urdf", "canopyag.urdf.xacro"]),
            " sim:=false",
            " use_meshes:=", use_meshes,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_meshes", default_value="",
            description="Override options.use_meshes from arm_parameters.yaml "
                        "('true'/'false'; empty means use the yaml value).",
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
