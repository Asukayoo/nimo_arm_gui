"""HMI + RViz — run on the local PC (Jazzy). Subscribes to remote backend over DDS."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _setup(context, *args, **kwargs):
    share = get_package_share_directory("nimo_arm_description")
    urdf_path = os.path.join(share, "urdf", "nimo_arms.urdf")
    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("nimo_arm_description"), "rviz", "arms.rviz"]
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_cfg],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
        Node(
            package="nimo_arm_gui",
            executable="nimo_arm_hmi",
            name="nimo_arm_hmi",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_gui")),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_gui", default_value="true"),
            OpaqueFunction(function=_setup),
        ]
    )
