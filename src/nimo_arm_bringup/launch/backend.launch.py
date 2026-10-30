"""Backend only — run on nimo-12 Humble container (no GUI)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim", default_value="true"),
            DeclareLaunchArgument("auto_init", default_value="true"),
            DeclareLaunchArgument("config_path", default_value=""),
            DeclareLaunchArgument("publish_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("rt_enable", default_value="true"),
            DeclareLaunchArgument("rt_fifo", default_value="true"),
            DeclareLaunchArgument("rt_priority", default_value="80"),
            DeclareLaunchArgument("rt_cpu", default_value="-2"),
            DeclareLaunchArgument("rt_lock_memory", default_value="true"),
            DeclareLaunchArgument("rt_abort_on_fail", default_value="false"),
            # Structured diagnostics (see nimo_arm_ros/run_diagnostics.hpp)
            DeclareLaunchArgument("debug", default_value="false"),
            DeclareLaunchArgument(
                "debug_dir", default_value="/tmp/nimo_arm_diag"
            ),
            DeclareLaunchArgument("debug_csv_stride", default_value="0"),
            DeclareLaunchArgument(
                "servoj_interp_time_s", default_value="0.02"
            ),
            DeclareLaunchArgument(
                "servoj_align_tol_rad", default_value="0.25"
            ),
            DeclareLaunchArgument(
                "servoj_joint_timeout_s", default_value="0.25"
            ),
            Node(
                package="nimo_arm_ros",
                executable="dual_arm_node",
                name="nimo_arm_ros",
                output="screen",
                parameters=[
                    {
                        "use_sim": ParameterValue(
                            LaunchConfiguration("use_sim"), value_type=bool
                        ),
                        "auto_init": ParameterValue(
                            LaunchConfiguration("auto_init"), value_type=bool
                        ),
                        "config_path": LaunchConfiguration("config_path"),
                        "publish_rate_hz": ParameterValue(
                            LaunchConfiguration("publish_rate_hz"), value_type=float
                        ),
                        "rt_enable": ParameterValue(
                            LaunchConfiguration("rt_enable"), value_type=bool
                        ),
                        "rt_fifo": ParameterValue(
                            LaunchConfiguration("rt_fifo"), value_type=bool
                        ),
                        "rt_priority": ParameterValue(
                            LaunchConfiguration("rt_priority"), value_type=int
                        ),
                        "rt_cpu": ParameterValue(
                            LaunchConfiguration("rt_cpu"), value_type=int
                        ),
                        "rt_lock_memory": ParameterValue(
                            LaunchConfiguration("rt_lock_memory"), value_type=bool
                        ),
                        "rt_abort_on_fail": ParameterValue(
                            LaunchConfiguration("rt_abort_on_fail"), value_type=bool
                        ),
                        "debug": ParameterValue(
                            LaunchConfiguration("debug"), value_type=bool
                        ),
                        "debug_dir": LaunchConfiguration("debug_dir"),
                        "debug_csv_stride": ParameterValue(
                            LaunchConfiguration("debug_csv_stride"), value_type=int
                        ),
                        "servoj_interp_time_s": ParameterValue(
                            LaunchConfiguration("servoj_interp_time_s"),
                            value_type=float,
                        ),
                        "servoj_align_tol_rad": ParameterValue(
                            LaunchConfiguration("servoj_align_tol_rad"),
                            value_type=float,
                        ),
                        "servoj_joint_timeout_s": ParameterValue(
                            LaunchConfiguration("servoj_joint_timeout_s"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
