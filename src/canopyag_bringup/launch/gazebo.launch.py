"""Spawn the robot in Gazebo (gz sim Harmonic) with controllers up.

    ros2 launch canopyag_bringup gazebo.launch.py

Controllers are spawned in sequence via event handlers rather than all at once:
joint_state_broadcaster must be active before the trajectory controllers, or
they come up with no state and immediately fault.

The crate and the carriage are counterweighted on the other side of the
gantry. URDF has no way to say that, so once the robot is spawned this sends
each counterweight to Gazebo as a constant upward force on the joint's child
link (ApplyLinkWrench, loaded by the world). The links keep their real mass
and inertia; only gravity along the axis is offset.
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
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


MODEL_NAME = "canopyag"
GRAVITY = 9.8  # matches the SDF default the world uses


def counterweight_forces(world_name):
    """One `gz topic` call per counterweight in robot_parameters.yaml.

    Persistent wrenches accumulate in ApplyLinkWrench, so each one is sent
    exactly once. `gz topic -p` waits for the subscriber before publishing.
    """
    params = os.path.join(get_package_share_directory("canopyag_description"),
                          "config", "robot_parameters.yaml")
    with open(params) as fh:
        cfg = yaml.safe_load(fh)

    calls = []
    for joint, cw in (cfg.get("counterweights") or {}).items():
        link = cfg["joints"][joint]["child"]
        force = float(cw["mass"]) * GRAVITY
        msg = (f'entity: {{name: "{MODEL_NAME}::{link}", type: LINK}}, '
               f"wrench: {{force: {{x: 0, y: 0, z: {force:.4f}}}}}")
        calls.append(ExecuteProcess(
            cmd=["gz", "topic", "-t", ["/world/", world_name, "/wrench/persistent"],
                 "-m", "gz.msgs.EntityWrench", "-p", msg],
            output="screen",
        ))
    return calls


def generate_launch_description():
    desc_pkg = FindPackageShare("canopyag_description")
    bringup_share = get_package_share_directory("canopyag_bringup")

    world = LaunchConfiguration("world")
    world_name = LaunchConfiguration("world_name")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([desc_pkg, "urdf", "canopyag.urdf.xacro"]),
            " sim:=true",
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
        arguments=["-topic", "robot_description", "-name", MODEL_NAME, "-z", "0.0"],
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
    crate = spawner("crate_controller")
    # Loaded but not started - see the note in controllers.yaml.
    fwd = spawner("forward_position_controller", "--inactive")

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(bringup_share, "worlds", "greenhouse.sdf"),
            description="Path to the .sdf world to load.",
        ),
        DeclareLaunchArgument(
            "world_name", default_value="greenhouse",
            description="<world name=...> inside that file; the counterweight "
                        "forces are sent to /world/<world_name>/wrench/persistent.",
        ),
        gz_resource_path,
        gz_sim,
        clock_bridge,
        robot_state_publisher,
        spawn_robot,

        RegisterEventHandler(OnProcessExit(target_action=spawn_robot,
                                           on_exit=[jsb, *counterweight_forces(world_name)])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm, crate, fwd])),
    ])
