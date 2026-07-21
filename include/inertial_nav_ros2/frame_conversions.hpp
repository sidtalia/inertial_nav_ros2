#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"

namespace inertial_nav_ros2::frames {

/** Body-axis convention for ROS sensor messages before nav_filter FRD body frame. */
enum class BodyAxes : std::uint8_t {
  /** REP-103 base_link: X forward, Y left, Z up. */
  RosFlu = 0,
  /** Inertial_nav_shim MuJoCo body at identity (X East, Y North, Z up). */
  MujocoEnu = 1,
  /** Sensor/body frame already matches nav_filter FRD; no axis remap. */
  Identity = 2,
};

BodyAxes body_axes_from_string(const std::string & value);

/** ENU world position/velocity (m, m/s) -> NED. Matches sim.frames.mj_pos_to_ned. */
void enu_position_to_ned(const float enu[3], float ned[3]);
void enu_velocity_to_ned(const float enu[3], float ned[3]);

/** NED -> ENU world position/velocity. */
void ned_position_to_enu(const float ned[3], float enu[3]);
void ned_velocity_to_enu(const float ned[3], float enu[3]);

/**
 * ROS ENU world attitude (body in parent frame) -> nav_filter NED/FRD quaternion (w,x,y,z).
 * Matches sim.frames.mj_quat_to_ned when body_axes is MujocoEnu.
 */
void enu_quat_to_ned_frd(
  const geometry_msgs::msg::Quaternion & q_enu, BodyAxes body_axes, float q_ned_frd_wxyz[4]);

/** nav_filter NED/FRD quaternion (w,x,y,z) -> ROS ENU world attitude. */
void ned_frd_quat_to_enu(
  const float q_ned_frd_wxyz[4], BodyAxes body_axes, geometry_msgs::msg::Quaternion & q_enu);

/** Body-fixed vector in ROS sensor frame -> nav_filter FRD body frame. */
void ros_body_vector_to_frd(const float in[3], BodyAxes body_axes, float out[3]);

/** nav_filter FRD body vector -> ROS body/sensor frame. */
void frd_body_vector_to_ros(const float in[3], BodyAxes body_axes, float out[3]);

/** nav_filter FRD body velocity -> NED world velocity using attitude quaternion (w,x,y,z). */
void frd_body_velocity_to_ned(
  const float v_frd[3], const float q_ned_frd_wxyz[4], float v_ned[3]);

/** NED world velocity -> nav_filter FRD body velocity using attitude quaternion (w,x,y,z). */
void ned_velocity_to_frd_body(
  const float v_ned[3], const float q_ned_frd_wxyz[4], float v_frd[3]);

}  // namespace inertial_nav_ros2::frames
