#!/usr/bin/env python3
"""Publish MuJoCo sensor data as ROS topics for ekf_ins validation."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Bool, Float64

def _find_shim_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        for candidate in (parent / "Inertial_nav_shim", parent / "src" / "Inertial_nav_shim"):
            if (candidate / "sim" / "mujoco_sim.py").is_file():
                return candidate
    raise RuntimeError("Could not locate Inertial_nav_shim (expected under colcon_ws/src)")


SHIM_ROOT = _find_shim_root()
sys.path.insert(0, str(SHIM_ROOT))

from sim.mujoco_sim import MujocoSensorSimulator  # noqa: E402
from sim.ros_frames import (  # noqa: E402
    GAUSS_TO_TESLA,
    frd_to_mujoco_body,
    ned_frd_quat_to_ros_enu,
    ned_to_enu,
)
from sim.sim_config import FusionProfile, SimConfig  # noqa: E402
from sim.sensors import JumpSchedule, NoiseConfig  # noqa: E402


def sec_to_time_msg(t_sec: float) -> Time:
    sec = int(t_sec)
    nanosec = int(round((t_sec - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    msg = Time()
    msg.sec = sec
    msg.nanosec = nanosec
    return msg


class MujocoSimPublisher(Node):
    def __init__(self, cfg: SimConfig):
        super().__init__("mujoco_sim_publisher")
        self.cfg = cfg
        self.sim = MujocoSensorSimulator(cfg)
        self.filter_initialized = False
        self.create_subscription(Odometry, "/ekf/odometry", self._on_ekf_odom, 10)

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.imu_pub = self.create_publisher(Imu, "/sim/imu", 10)
        self.ext_nav_pub = self.create_publisher(Odometry, "/sim/ext_nav/odom", 10)
        self.baro_pub = self.create_publisher(Float64, "/sim/baro", 10)
        self.mag_pub = self.create_publisher(MagneticField, "/sim/mag", 10)
        self.truth_pub = self.create_publisher(Odometry, "/sim/ground_truth", 10)
        self.done_pub = self.create_publisher(Bool, "/sim/done", 10)

        self.n_steps = int(cfg.duration_s / cfg.imu_dt)
        self.step = 0
        self.timer = self.create_timer(cfg.imu_dt, self._on_timer)
        self.get_logger().info(
            f"MuJoCo sim publisher: profile={cfg.profile.value} duration={cfg.duration_s}s "
            f"imu_dt={cfg.imu_dt}s steps={self.n_steps} jumps={len(cfg.jumps)}"
        )

    def _on_ekf_odom(self, _msg: Odometry):
        self.filter_initialized = True

    def _publish_clock(self, t_sec: float):
        msg = Clock()
        msg.clock = sec_to_time_msg(t_sec)
        self.clock_pub.publish(msg)

    def _publish_imu(self, tick):
        accel_mj = frd_to_mujoco_body(tick.imu_accel_frd)
        gyro_mj = frd_to_mujoco_body(tick.imu_gyro_frd)
        msg = Imu()
        msg.header.stamp = sec_to_time_msg(tick.t)
        msg.header.frame_id = "base_link"
        msg.linear_acceleration.x = float(accel_mj[0])
        msg.linear_acceleration.y = float(accel_mj[1])
        msg.linear_acceleration.z = float(accel_mj[2])
        msg.angular_velocity.x = float(gyro_mj[0])
        msg.angular_velocity.y = float(gyro_mj[1])
        msg.angular_velocity.z = float(gyro_mj[2])
        self.imu_pub.publish(msg)

    def _publish_ext_nav(self, tick):
        if tick.ext_nav is None:
            return
        ext = tick.ext_nav
        pos_enu = ned_to_enu(ext.pos_ned_local)
        q = ned_frd_quat_to_ros_enu(ext.quat_ned_wxyz)
        msg = Odometry()
        msg.header.stamp = sec_to_time_msg(tick.t)
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = float(pos_enu[0])
        msg.pose.pose.position.y = float(pos_enu[1])
        msg.pose.pose.position.z = float(pos_enu[2])
        msg.pose.pose.orientation = Quaternion(
            x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3])
        )
        msg.pose.covariance[0] = float(ext.pos_err * ext.pos_err)
        msg.pose.covariance[35] = float(ext.yaw_err * ext.yaw_err)

        body_frd = tick.odom.get(0)
        if body_frd is not None:
            body_mj = frd_to_mujoco_body(body_frd)
            msg.twist.twist.linear = Vector3(
                x=float(body_mj[0]), y=float(body_mj[1]), z=float(body_mj[2])
            )
            msg.twist.covariance[0] = float(ext.vel_err * ext.vel_err)
        self.ext_nav_pub.publish(msg)

    def _publish_baro(self, tick):
        if tick.baro_height_m is None:
            return
        msg = Float64()
        msg.data = float(tick.baro_height_m)
        self.baro_pub.publish(msg)

    def _publish_mag(self, tick):
        if tick.mag_body_frd is None:
            return
        field_mj = frd_to_mujoco_body(tick.mag_body_frd)
        msg = MagneticField()
        msg.header.stamp = sec_to_time_msg(tick.t)
        msg.header.frame_id = "base_link"
        msg.magnetic_field.x = float(field_mj[0] * GAUSS_TO_TESLA)
        msg.magnetic_field.y = float(field_mj[1] * GAUSS_TO_TESLA)
        msg.magnetic_field.z = float(field_mj[2] * GAUSS_TO_TESLA)
        self.mag_pub.publish(msg)

    def _publish_ground_truth(self, tick):
        truth = tick.truth
        pos_enu_local = truth["pos_enu"] - self.sim._spawn_pos_enu
        q_ros = ned_frd_quat_to_ros_enu(truth["quat_ned"])
        msg = Odometry()
        msg.header.stamp = sec_to_time_msg(tick.t)
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = float(pos_enu_local[0])
        msg.pose.pose.position.y = float(pos_enu_local[1])
        msg.pose.pose.position.z = float(pos_enu_local[2])
        msg.pose.pose.orientation = Quaternion(
            x=float(q_ros[0]), y=float(q_ros[1]), z=float(q_ros[2]), w=float(q_ros[3])
        )
        body_mj = frd_to_mujoco_body(truth["body_lin_frd"])
        msg.twist.twist.linear = Vector3(x=float(body_mj[0]), y=float(body_mj[1]), z=float(body_mj[2]))
        self.truth_pub.publish(msg)

    def _on_timer(self):
        if self.step >= self.n_steps:
            done = Bool()
            done.data = True
            self.done_pub.publish(done)
            self.get_logger().info("Simulation complete")
            self.timer.cancel()
            return

        tick = self.sim.tick(self.step, self.filter_initialized)
        self._publish_clock(tick.t)
        self._publish_imu(tick)
        self._publish_ext_nav(tick)
        self._publish_baro(tick)
        self._publish_mag(tick)
        self._publish_ground_truth(tick)
        self.step += 1


def parse_args():
    parser = argparse.ArgumentParser(description="MuJoCo sensor publisher for ROS EKF validation")
    parser.add_argument("--duration", type=float, default=60.0, help="Simulation duration (seconds)")
    parser.add_argument("--profile", choices=[p.value for p in FusionProfile], default="ext_nav")
    parser.add_argument("--trajectory", default="hover", choices=["hover", "circle", "mobile_base_lift"])
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--jump-at", type=float, default=None, help="Inject ext-nav pose jump at time (s)")
    parser.add_argument(
        "--jump-dpos-ned",
        type=float,
        nargs=3,
        metavar=("N", "E", "D"),
        default=None,
        help="Position jump offset in local NED meters (e.g. 2 0 0 = 2m north)",
    )
    parser.add_argument(
        "--jump-dyaw-deg",
        type=float,
        default=None,
        help="Yaw jump offset in degrees",
    )
    parser.add_argument(
        "--jump-duration",
        type=float,
        default=None,
        help="Jump duration in seconds (omit for permanent offset until sim end)",
    )
    return parser.parse_args()


def build_jumps(args) -> list[JumpSchedule]:
    if args.jump_at is None:
        return []
    dpos = np.zeros(3, dtype=np.float64)
    if args.jump_dpos_ned is not None:
        dpos = np.array(args.jump_dpos_ned, dtype=np.float64)
    dyaw = 0.0
    if args.jump_dyaw_deg is not None:
        dyaw = float(np.deg2rad(args.jump_dyaw_deg))
    if np.linalg.norm(dpos) < 1e-9 and abs(dyaw) < 1e-9:
        raise SystemExit("jump-at requires --jump-dpos-ned and/or --jump-dyaw-deg")
    return [
        JumpSchedule(
            t_jump=float(args.jump_at),
            dpos=dpos,
            dyaw=dyaw,
            duration_s=args.jump_duration,
        )
    ]


def main():
    args = parse_args()
    jumps = build_jumps(args)
    cfg = SimConfig(
        duration_s=args.duration,
        profile=FusionProfile(args.profile),
        trajectory=args.trajectory,
        seed=args.seed,
        jumps=jumps,
        noise=NoiseConfig(pos_std=0.05, yaw_std=0.03),
    )
    rclpy.init()
    node = MujocoSimPublisher(cfg)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
