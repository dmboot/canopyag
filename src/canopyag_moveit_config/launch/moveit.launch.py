"""MoveIt (move_group + RViz), WITHOUT ros2_control: plan by hand in RViz.

Attaches to whatever controller manager is already running:

    # the real robot
    ros2 launch canopyag_bringup hardware.launch.py
    ros2 control switch_controllers --activate arm_controller
    ros2 launch canopyag_moveit_config moveit.launch.py

    # mock hardware: demo.launch.py includes this file

move_group only reads the URDF for kinematics and collision geometry, so the
mock variant of the robot description is used here whatever the hardware is.

Goals in world coordinates and the demo path do not use MoveIt: see
canopyag_motion.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def moveit_configs():
    desc_share = get_package_share_directory("canopyag_description")
    return (
        MoveItConfigsBuilder("canopyag", package_name="canopyag_moveit_config")
        .robot_description(
            file_path=os.path.join(desc_share, "urdf", "canopyag.urdf.xacro"),
            mappings={"sim": "false", "mock": "true"},
        )
        .robot_description_semantic(file_path="config/canopyag.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        # The demo does its own point-to-point moves (see demo_path.py); these
        # are for planning by hand in RViz. OMPL is the default because Pilz
        # slows every joint to the carriage's limits.
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "pilz_industrial_motion_planner"],
        )
        .to_moveit_configs()
    )


def generate_launch_description():
    moveit_config = moveit_configs()

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[moveit_config.to_dict(), {"use_sim_time": False}],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution(
            [FindPackageShare("canopyag_moveit_config"), "rviz", "moveit.rviz"])],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="log",
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        move_group,
        rviz,
    ])
