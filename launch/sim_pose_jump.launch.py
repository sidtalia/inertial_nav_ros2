"""Launch pose-jump scenario: MuJoCo sim + EKF (ext_nav profile) + trace recorder + optional viser."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_prefix = get_package_prefix("inertial_nav_ros2")
    shim_root = Path(pkg_prefix).parent.parent / "src" / "Inertial_nav_shim"
    if not (shim_root / "scripts" / "viser_log.py").is_file():
        shim_root = Path("/root/colcon_ws/src/Inertial_nav_shim")

    sim_pub = os.path.join(pkg_prefix, "lib", "inertial_nav_ros2", "mujoco_sim_pub.py")
    recorder = os.path.join(pkg_prefix, "lib", "inertial_nav_ros2", "ros_trace_recorder.py")
    viser_log = str(shim_root / "scripts" / "viser_log.py")

    default_params = PathJoinSubstitution(
        [FindPackageShare("inertial_nav_ros2"), "config", "ekf_sim_ext_nav.yaml"]
    )

    duration = LaunchConfiguration("duration")
    jump_at = LaunchConfiguration("jump_at")
    jump_dpos_n = LaunchConfiguration("jump_dpos_n")
    jump_dpos_e = LaunchConfiguration("jump_dpos_e")
    jump_dpos_d = LaunchConfiguration("jump_dpos_d")
    log_path = LaunchConfiguration("log_path")
    launch_viser = LaunchConfiguration("launch_viser")
    viser_port = LaunchConfiguration("viser_port")

    sim_process = ExecuteProcess(
        cmd=[
            "python3",
            sim_pub,
            "--duration",
            duration,
            "--profile",
            "ext_nav",
            "--jump-at",
            jump_at,
            "--jump-dpos-ned",
            jump_dpos_n,
            jump_dpos_e,
            jump_dpos_d,
        ],
        output="screen",
    )

    recorder_process = ExecuteProcess(
        cmd=[
            "python3",
            recorder,
            "--output",
            log_path,
            "--test-name",
            "ros_ext_nav_pose_jump",
            "--profile",
            "ext_nav_sim",
            "--jump-at",
            jump_at,
            "--jump-dpos-ned",
            jump_dpos_n,
            jump_dpos_e,
            jump_dpos_d,
        ],
        output="screen",
    )

    ekf_node = Node(
        package="inertial_nav_ros2",
        executable="ekf_ins_node",
        name="nav_filter_ekf",
        output="screen",
        parameters=[default_params],
    )

    delayed_recorder = TimerAction(period=1.0, actions=[recorder_process])

    viser_after_recorder = RegisterEventHandler(
        OnProcessExit(
            target_action=recorder_process,
            on_exit=[
                ExecuteProcess(
                    condition=IfCondition(launch_viser),
                    cmd=["python3", viser_log, log_path, "--port", viser_port],
                    output="screen",
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("duration", default_value="45.0"),
            DeclareLaunchArgument("jump_at", default_value="10.0"),
            DeclareLaunchArgument("jump_dpos_n", default_value="2.0"),
            DeclareLaunchArgument("jump_dpos_e", default_value="0.0"),
            DeclareLaunchArgument("jump_dpos_d", default_value="0.0"),
            DeclareLaunchArgument(
                "log_path",
                default_value=str(shim_root / "build" / "logs" / "ros_pose_jump.pkl"),
            ),
            DeclareLaunchArgument("launch_viser", default_value="true"),
            DeclareLaunchArgument("viser_port", default_value="8080"),
            sim_process,
            ekf_node,
            delayed_recorder,
            viser_after_recorder,
        ]
    )
