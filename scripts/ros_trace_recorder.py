#!/usr/bin/env python3
"""Record ROS sim topics into a viser-compatible SimTrace .pkl log."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool

SHIM_ROOT = None
for parent in Path(__file__).resolve().parents:
    for candidate in (parent / "Inertial_nav_shim", parent / "src" / "Inertial_nav_shim"):
        if (candidate / "sim" / "trace.py").is_file():
            SHIM_ROOT = candidate
            break
    if SHIM_ROOT is not None:
        break
if SHIM_ROOT is None:
    raise RuntimeError("Could not locate Inertial_nav_shim")
sys.path.insert(0, str(SHIM_ROOT))

from sim.frames import euler_from_quat_ned  # noqa: E402
from sim.ros_frames import enu_to_ned, ros_enu_quat_to_ned_frd  # noqa: E402
from sim.trace import SimTrace, save_trace  # noqa: E402


def stamp_sec(msg: Odometry) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def odom_to_ned(msg: Odometry) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    pos = enu_to_ned(
        np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
            dtype=np.float64,
        )
    )
    q = msg.pose.pose.orientation
    quat = ros_enu_quat_to_ned_frd(np.array([q.x, q.y, q.z, q.w], dtype=np.float64))
    euler = euler_from_quat_ned(quat)
    vel = np.zeros(3, dtype=np.float64)
    return pos, quat, euler


class RosTraceRecorder(Node):
    def __init__(self, output: Path, meta: dict):
        super().__init__("ros_trace_recorder")
        self.output = output
        self.meta = dict(meta)
        self.trace = SimTrace()
        self.latest_est: Odometry | None = None
        self._last_meas_pos: np.ndarray | None = None
        self._last_meas_quat: np.ndarray | None = None
        self._meas_updated_since_truth = False
        self.have_est = False

        self.create_subscription(Odometry, "/sim/ground_truth", self._on_truth, 10)
        self.create_subscription(Odometry, "/ekf/odometry", self._on_est, 10)
        self.create_subscription(Odometry, "/sim/ext_nav/odom", self._on_meas, 10)
        self.create_subscription(Bool, "/sim/done", self._on_done, 10)
        self.get_logger().info(f"Recording trace to {output}")

    def _on_est(self, msg: Odometry):
        self.latest_est = msg
        self.have_est = True

    def _on_meas(self, msg: Odometry):
        self._last_meas_pos, self._last_meas_quat, _ = odom_to_ned(msg)
        self._meas_updated_since_truth = True

    def _on_truth(self, msg: Odometry):
        if not self.have_est or self.latest_est is None:
            return
        if self._last_meas_pos is None or self._last_meas_quat is None:
            return
        fed_ext_nav = self._meas_updated_since_truth
        self._meas_updated_since_truth = False
        t = stamp_sec(msg)
        truth_pos, truth_quat, truth_euler = odom_to_ned(msg)
        est_pos, est_quat, est_euler = odom_to_ned(self.latest_est)
        self.trace.append(
            t,
            truth_pos,
            np.zeros(3),
            truth_euler,
            truth_quat.astype(np.float32),
            est_pos,
            np.zeros(3),
            est_euler,
            est_quat.astype(np.float32),
            source_flags={"fed_ext_nav": fed_ext_nav},
            meas_pos=self._last_meas_pos,
            meas_quat=self._last_meas_quat.astype(np.float32),
        )

    def _on_done(self, msg: Bool):
        if not msg.data:
            return
        if self.trace.t.size == 0:
            self.get_logger().error("No trace samples recorded")
            raise SystemExit(1)
        save_trace(self.output, self.trace, self.meta)
        self.get_logger().info(f"Wrote {self.trace.t.size} samples to {self.output}")
        raise SystemExit(0)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("build/logs/ros_pose_jump.pkl"),
        help="output .pkl path (viser_log.py compatible)",
    )
    parser.add_argument("--test-name", default="ros_pose_jump")
    parser.add_argument("--profile", default="zed_vslam")
    parser.add_argument("--jump-at", type=float, default=None)
    parser.add_argument("--jump-dpos-ned", type=float, nargs=3, default=None)
    return parser.parse_args()


def main():
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    meta = {"test": args.test_name, "profile": args.profile, "trajectory": "hover", "source": "ros"}
    if args.jump_at is not None and args.jump_dpos_ned is not None:
        meta["jumps"] = [
            {
                "t_jump": args.jump_at,
                "dpos_ned": list(args.jump_dpos_ned),
                "duration_s": None,
            }
        ]
    node = RosTraceRecorder(args.output, meta)
    exit_code = 1
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        exit_code = int(exc.code) if exc.code is not None else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
