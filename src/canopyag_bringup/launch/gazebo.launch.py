"""Spawn the canopy.ag SCARA in Gazebo (gz sim Harmonic) with controllers up.

    ros2 launch canopyag_bringup gazebo.launch.py

Then try it:
    ros2 control list_controllers
    ros2 action send_goal /gripper_controller/gripper_cmd \
        control_msgs/action/GripperCommand "{command: {position: 0.03, max_effort: 10.0}}"

Controllers are spawned in sequence via event handlers rather than all at once:
joint_state_broadcaster must be active before the trajectory controllers, or
they come up with no state and immediately fault.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = FindPackageShare("canopyag_description")
    bringup_share = get_package_share_directory("canopyag_bringup")

    world = LaunchConfiguration("world")
    use_meshes = LaunchConfiguration("use_meshes")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([desc_pkg, "urdf", "canopyag.urdf.xacro"]),
            " sim:=true",
            " use_meshes:=", use_meshes,
        ]),
        value_type=str,
    )

    # Gazebo needs to resolve package:// mesh URIs. Pointing the resource path
    # at the parent of the share dir makes package://canopyag_description/...
    # resolvable. Without this, meshes silently render as nothing.
    gz_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.pathsep.join([
            os.path.dirname(get_package_share_directory("canopyag_description")),
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        ]),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ]),
        launch_arguments={"gz_args": ["-r -v4 ", world]}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "canopyag", "-z", "0.0"],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
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
    arm = spawner("arm_controller")
    gripper = spawner("gripper_controller")
    tool_roll = spawner("tool_roll_controller")
    # Loaded but not started - see the note in controllers.yaml.
    fwd = spawner("forward_position_controller", "--inactive")

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(bringup_share, "worlds", "greenhouse.sdf"),
            description="Path to the .sdf world to load.",
        ),
        DeclareLaunchArgument(
            "use_meshes", default_value="",
            description="Override options.use_meshes from arm_parameters.yaml.",
        ),

        gz_resource_path,
        gz_sim,
        clock_bridge,
        robot_state_publisher,
        spawn_robot,

        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm, gripper, tool_roll, fwd])),
    ])
