"""MoveIt on mock hardware: plan by hand in RViz.

    ros2 launch canopyag_moveit_config demo.launch.py

The scripted demo path is canopyag_motion's (no MoveIt):
    ros2 launch canopyag_motion demo.launch.py

No Gazebo and no real axes: ros2_control runs mock_components/GenericSystem,
which reflects every command straight back as state. MoveIt itself is
moveit.launch.py, the same file that runs against the real robot.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory("canopyag_description")
    here = os.path.dirname(os.path.abspath(__file__))
    robot_description = {"robot_description": ParameterValue(Command([
        "xacro ", os.path.join(desc_share, "urdf", "canopyag.urdf.xacro"),
        " sim:=false mock:=true"]), value_type=str)}

    controllers = os.path.join(desc_share, "config", "controllers.yaml")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # Mock hardware runs on wall-clock time, not Gazebo's /clock.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers, {"use_sim_time": False}],
        output="screen",
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            output="screen",
        )

    jsb = spawner("joint_state_broadcaster")

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(here, "moveit.launch.py")),
    )

    return LaunchDescription([
        robot_state_publisher,
        control_node,
        jsb,
        RegisterEventHandler(OnProcessExit(
            target_action=jsb,
            on_exit=[spawner("arm_controller"), spawner("crate_controller")],
        )),
        moveit,
    ])
