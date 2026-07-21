#!/usr/bin/env python3
"""Compare /ekf/odometry against /sim/ground_truth and report RMSE."""

from __future__ import annotations

import argparse
import math
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class RosSimValidator(Node):
    def __init__(self, pos_rmse_limit: float, skip_s: float):
        super().__init__("ros_sim_validator")
        self.pos_rmse_limit = pos_rmse_limit
        self.skip_s = skip_s
        self.truth: Odometry | None = None
        self.pos_err: list[float] = []
        self.yaw_err: list[float] = []
        self.err_times: list[float] = []
        self.initialized = False
        self.done = False

        self.create_subscription(Odometry, "/sim/ground_truth", self._on_truth, 10)
        self.create_subscription(Odometry, "/ekf/odometry", self._on_ekf, 10)
        self.create_subscription(Bool, "/sim/done", self._on_done, 10)
        self.get_logger().info(
            f"ROS sim validator listening (pos_rmse_limit={pos_rmse_limit} m, skip_s={skip_s})"
        )

    def _stamp_sec(self, msg: Odometry) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def _on_truth(self, msg: Odometry):
        self.truth = msg

    def _on_ekf(self, msg: Odometry):
        if self.truth is None:
            return
        t = self._stamp_sec(msg)
        if t < self.skip_s:
            return
        self.initialized = True
        gt = self.truth.pose.pose
        est = msg.pose.pose
        dpos = np.array(
            [
                est.position.x - gt.position.x,
                est.position.y - gt.position.y,
                est.position.z - gt.position.z,
            ],
            dtype=np.float64,
        )
        self.pos_err.append(float(np.linalg.norm(dpos)))
        dyaw = wrap_pi(quat_to_yaw(est.orientation) - quat_to_yaw(gt.orientation))
        self.yaw_err.append(float(dyaw))
        self.err_times.append(t)

    def _on_done(self, msg: Bool):
        if not msg.data or self.done:
            return
        self.done = True
        ok = self._report()
        if ok:
            self.get_logger().info("ROS validation PASSED")
        else:
            self.get_logger().error("ROS validation FAILED")
        raise SystemExit(0 if ok else 1)

    def _report(self) -> bool:
        if not self.initialized or not self.pos_err:
            self.get_logger().error("No EKF odometry samples received")
            return False

        pos_rmse = float(np.sqrt(np.mean(np.square(self.pos_err))))
        yaw_rmse_deg = float(np.rad2deg(np.sqrt(np.mean(np.square(self.yaw_err)))))
        self.get_logger().info(
            f"ROS validation: pos_rmse={pos_rmse:.3f} m yaw_rmse={yaw_rmse_deg:.2f} deg "
            f"samples={len(self.pos_err)}"
        )
        if pos_rmse > self.pos_rmse_limit:
            self.get_logger().error(
                f"pos_rmse {pos_rmse:.3f} exceeds limit {self.pos_rmse_limit:.3f}"
            )
            return False
        return True


def parse_args():
    parser = argparse.ArgumentParser(description="Validate ROS EKF against MuJoCo ground truth")
    parser.add_argument("--pos-rmse-limit", type=float, default=2.0)
    parser.add_argument("--skip-s", type=float, default=5.0, help="Ignore first N seconds after init")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = RosSimValidator(args.pos_rmse_limit, args.skip_s)
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
