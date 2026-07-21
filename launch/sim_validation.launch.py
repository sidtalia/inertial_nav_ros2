"""Launch MuJoCo sim publisher + ekf_ins + validator for ROS integration tests."""

import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_prefix = get_package_prefix("inertial_nav_ros2")
    sim_pub = os.path.join(pkg_prefix, "lib", "inertial_nav_ros2", "mujoco_sim_pub.py")
    validator = os.path.join(pkg_prefix, "lib", "inertial_nav_ros2", "ros_sim_validator.py")
    default_params = PathJoinSubstitution(
        [FindPackageShare("inertial_nav_ros2"), "config", "ekf_sim_ext_nav.yaml"]
    )

    duration = LaunchConfiguration("duration")
    profile = LaunchConfiguration("profile")
    pos_rmse_limit = LaunchConfiguration("pos_rmse_limit")

    sim_process = ExecuteProcess(
        cmd=["python3", sim_pub, "--duration", duration, "--profile", profile],
        output="screen",
    )

    validator_process = ExecuteProcess(
        cmd=["python3", validator, "--pos-rmse-limit", pos_rmse_limit],
        output="screen",
    )

    ekf_node = Node(
        package="inertial_nav_ros2",
        executable="ekf_ins_node",
        name="nav_filter_ekf",
        output="screen",
        parameters=[default_params],
    )

    delayed_validator = TimerAction(period=2.0, actions=[validator_process])

    return LaunchDescription(
        [
            DeclareLaunchArgument("duration", default_value="60.0"),
            DeclareLaunchArgument("profile", default_value="ext_nav"),
            DeclareLaunchArgument("pos_rmse_limit", default_value="2.0"),
            sim_process,
            ekf_node,
            delayed_validator,
        ]
    )
