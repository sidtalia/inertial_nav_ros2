#!/usr/bin/env python3
"""Publish static earth -> local TF from first VSLAM pose vs IMU AHRS.

cuVSLAM odometry still labels the VO parent frame as "odom" in messages; we publish
earth -> local so our alignment TF does not collide with cuVSLAM map/odom frame names.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def _ros_flu_to_frd(vec):
    return (vec[0], -vec[1], -vec[2])


def _attitude_quat_frd_wxyz(ax, ay, az, mx, my, mz, declination_rad):
    """Match nav_filter AttitudeInit (FRD body, NED world quaternion w,x,y,z)."""
    roll = math.atan2(-ay, -az)
    pitch = math.atan2(ax, -az)
    cos_roll = math.cos(roll)
    sin_roll = math.sin(roll)
    cos_pitch = math.cos(pitch)
    sin_pitch = math.sin(pitch)
    mag_x = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch
    mag_y = my * cos_roll - mz * sin_roll
    heading = math.atan2(-mag_y, mag_x) + declination_rad

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    ch = math.cos(heading * 0.5)
    sh = math.sin(heading * 0.5)
    qw = cr * cp * ch + sr * sp * sh
    qx = sr * cp * ch - cr * sp * sh
    qy = cr * sp * ch + sr * cp * sh
    qz = cr * cp * sh - sr * sp * ch
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    return (qw / norm, qx / norm, qy / norm, qz / norm)


def _quat_to_matrix(q_wxyz):
    w, x, y, z = q_wxyz
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def _matrix_to_quat_wxyz(rot):
    m00, m01, m02 = rot[0]
    m10, m11, m12 = rot[1]
    m20, m21, m22 = rot[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    return (w / norm, x / norm, y / norm, z / norm)


def _mat_mul(a, b):
    out = [[0.0, 0.0, 0.0] for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = sum(a[i][k] * b[k][j] for k in range(3))
    return out


def _mat_transpose(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


_R_NED_ENU = (
    (0.0, 1.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 0.0, -1.0),
)
_R_ROS_FLU_TO_FRD = (
    (1.0, 0.0, 0.0),
    (0.0, -1.0, 0.0),
    (0.0, 0.0, -1.0),
)


def _ned_frd_quat_to_enu(q_ned_frd_wxyz):
    r_ned_frd = _quat_to_matrix(q_ned_frd_wxyz)
    r_ned_enu = _R_NED_ENU
    r_body_frd = _R_ROS_FLU_TO_FRD
    r_enu_body = _mat_mul(_mat_transpose(r_ned_enu), _mat_mul(r_ned_frd, _mat_transpose(r_body_frd)))
    return _matrix_to_quat_wxyz(r_enu_body)


def _quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def _quat_inverse(q):
    w, x, y, z = q
    norm2 = w * w + x * x + y * y + z * z
    return (w / norm2, -x / norm2, -y / norm2, -z / norm2)


class ExtNavEarthAlignNode(Node):
    def __init__(self):
        super().__init__("extnav_earth_align")
        self.declare_parameter("earth_frame", "earth")
        self.declare_parameter("vslam_local_frame", "local")
        self.declare_parameter("vslam_odom_topic", "/visual_slam/tracking/odometry")
        self.declare_parameter("imu_topic", "/zed/zed_node/imu/data")
        self.declare_parameter("mag_topic", "/zed/zed_node/imu/mag")
        self.declare_parameter("mag_declination_deg", 15.5)
        self.declare_parameter("min_accel_norm", 8.0)
        self.declare_parameter("max_accel_norm", 11.5)

        self._earth = self.get_parameter("earth_frame").value
        self._local = self.get_parameter("vslam_local_frame").value
        self._decl = math.radians(float(self.get_parameter("mag_declination_deg").value))
        self._min_acc = float(self.get_parameter("min_accel_norm").value)
        self._max_acc = float(self.get_parameter("max_accel_norm").value)

        self._last_imu: Imu | None = None
        self._last_mag: MagneticField | None = None
        self._aligned = False
        self._broadcaster = StaticTransformBroadcaster(self)

        qos = qos_profile_sensor_data
        self.create_subscription(
            Imu, self.get_parameter("imu_topic").value, self._on_imu, qos)
        self.create_subscription(
            MagneticField, self.get_parameter("mag_topic").value, self._on_mag, qos)
        self.create_subscription(
            Odometry, self.get_parameter("vslam_odom_topic").value, self._on_odom, 10)

        self.get_logger().info(
            f"waiting for first VSLAM odom to align {self._earth} -> {self._local}"
        )

    def _on_imu(self, msg: Imu) -> None:
        self._last_imu = msg

    def _on_mag(self, msg: MagneticField) -> None:
        self._last_mag = msg

    def _on_odom(self, msg: Odometry) -> None:
        if self._aligned:
            return
        if self._last_imu is None or self._last_mag is None:
            return

        accel_ros = (
            self._last_imu.linear_acceleration.x,
            self._last_imu.linear_acceleration.y,
            self._last_imu.linear_acceleration.z,
        )
        acc_norm = math.sqrt(sum(v * v for v in accel_ros))
        if acc_norm < self._min_acc or acc_norm > self._max_acc:
            self.get_logger().warn(
                f"skipping align: |accel|={acc_norm:.2f} m/s^2 outside "
                f"[{self._min_acc}, {self._max_acc}]"
            )
            return

        mag_ros = (
            self._last_mag.magnetic_field.x * 1.0e6,
            self._last_mag.magnetic_field.y * 1.0e6,
            self._last_mag.magnetic_field.z * 1.0e6,
        )
        ax, ay, az = _ros_flu_to_frd(accel_ros)
        mx, my, mz = _ros_flu_to_frd(mag_ros)
        q_ned_frd = _attitude_quat_frd_wxyz(ax, ay, az, mx, my, mz, self._decl)
        q_map_body = _ned_frd_quat_to_enu(q_ned_frd)

        q_odom_body = (
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        )
        q_earth_local = _quat_multiply(q_map_body, _quat_inverse(q_odom_body))

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self._earth
        t.child_frame_id = self._local
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = q_earth_local[0]
        t.transform.rotation.x = q_earth_local[1]
        t.transform.rotation.y = q_earth_local[2]
        t.transform.rotation.z = q_earth_local[3]
        self._broadcaster.sendTransform(t)
        self._aligned = True
        self.get_logger().info(
            f"published static TF {self._earth} -> {self._local} from first VSLAM odom"
        )


def main() -> None:
    rclpy.init()
    node = ExtNavEarthAlignNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
