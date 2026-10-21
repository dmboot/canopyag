"""The demo on mock hardware: mock.launch.py + motion_server.py + demo.py.

    ros2 launch canopyag_motion demo.launch.py
    ros2 launch canopyag_motion demo.launch.py side:=left loop:=true speed:=0.5
    ros2 launch canopyag_motion demo.launch.py demo:=false panel:=true   # pick goals in a window
    ros2 launch canopyag_motion demo.launch.py demo:=false     # just the server: send goals
    ros2 launch canopyag_motion demo.launch.py dexterity:=false   # no pitch / grip step
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    def include(pkg, name, args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare(pkg), "launch", name])),
            launch_arguments={k: LaunchConfiguration(k) for k in args}.items())

    return LaunchDescription([
        DeclareLaunchArgument("demo", default_value="true", description="Run the demo path."),
        DeclareLaunchArgument("loop", default_value="false"),
        DeclareLaunchArgument("speed", default_value="1.0"),
        DeclareLaunchArgument("side", default_value="right"),
        DeclareLaunchArgument("dexterity", default_value="true",
                              description="Pitch wiggle and grip at every fruit (skipped, with "
                                          "a warning, while there is no gripper_controller)."),
        DeclareLaunchArgument("panel", default_value="false",
                              description="Open goal_panel.py: x/y/z sliders to send goals."),
        DeclareLaunchArgument("rviz", default_value="true"),
        include("canopyag_bringup", "mock.launch.py", ["rviz"]),
        include("canopyag_motion", "motion.launch.py",
                ["demo", "loop", "speed", "side", "dexterity", "panel"]),
    ])
