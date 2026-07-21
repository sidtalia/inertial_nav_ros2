/** @file ekf_ins.launch.py
 *
 * Launches the nav_filter EKF INS node. Remap topics to your robot drivers.
 * For hound_core + MAVROS + Isaac Visual SLAM, see the commented remaps below.
 */

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("inertial_nav_ros2"), "config", "ekf_ins.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=config,
                description="EKF INS parameter file",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock",
            ),
            Node(
                package="inertial_nav_ros2",
                executable="ekf_ins_node",
                name="nav_filter_ekf",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
