"""MoveIt on mock hardware, with the scripted demo path.

    ros2 launch canopyag_moveit_config demo.launch.py              # runs the path once
    ros2 launch canopyag_moveit_config demo.launch.py loop:=true
    ros2 launch canopyag_moveit_config demo.launch.py demo:=false  # just MoveIt + RViz

No Gazebo and no real axes: ros2_control runs mock_components/GenericSystem,
which reflects every command straight back as state. The controllers, MoveIt
config and demo script are the same ones that will drive the real robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    desc_share = get_package_share_directory("canopyag_description")

    moveit_config = (
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

    controllers = os.path.join(desc_share, "config", "controllers.yaml")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    # Mock hardware runs on wall-clock time, not Gazebo's /clock.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, controllers, {"use_sim_time": False}],
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

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[moveit_config.to_dict()],
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
        output="log",
    )

    demo = Node(
        package="canopyag_moveit_config",
        executable="demo_path.py",
        parameters=[{"loop": LaunchConfiguration("loop")}],
        condition=IfCondition(LaunchConfiguration("demo")),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("demo", default_value="true",
                              description="Run the scripted demo path."),
        DeclareLaunchArgument("loop", default_value="false",
                              description="Repeat the demo path until stopped."),
        robot_state_publisher,
        control_node,
        jsb,
        RegisterEventHandler(OnProcessExit(
            target_action=jsb,
            on_exit=[spawner("arm_controller"), spawner("crate_controller")],
        )),
        move_group,
        rviz,
        # The script waits for move_group and both controllers itself.
        demo,
    ])
