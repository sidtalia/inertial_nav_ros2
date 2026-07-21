#include "inertial_nav_ros2/frame_conversions.hpp"

#include <cmath>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace inertial_nav_ros2::frames {
namespace {

constexpr double kR_NED_ENU[3][3] = {
  {0.0, 1.0, 0.0},
  {1.0, 0.0, 0.0},
  {0.0, 0.0, -1.0},
};

constexpr double kR_ROS_FLU_TO_FRD[3][3] = {
  {1.0, 0.0, 0.0},
  {0.0, -1.0, 0.0},
  {0.0, 0.0, -1.0},
};

constexpr double kR_IDENTITY[3][3] = {
  {1.0, 0.0, 0.0},
  {0.0, 1.0, 0.0},
  {0.0, 0.0, 1.0},
};

void mat3_vec(const double m[3][3], const float in[3], float out[3])
{
  for (int r = 0; r < 3; ++r) {
    out[r] = static_cast<float>(
      m[r][0] * in[0] + m[r][1] * in[1] + m[r][2] * in[2]);
  }
}

void mat3_transpose_vec(const double m[3][3], const float in[3], float out[3])
{
  for (int r = 0; r < 3; ++r) {
    out[r] = static_cast<float>(
      m[0][r] * in[0] + m[1][r] * in[1] + m[2][r] * in[2]);
  }
}

const double (* body_axes_to_mount(BodyAxes body_axes))[3]
{
  switch (body_axes) {
    case BodyAxes::RosFlu:
      return kR_ROS_FLU_TO_FRD;
    case BodyAxes::MujocoEnu:
      return kR_NED_ENU;
    case BodyAxes::Identity:
      return kR_IDENTITY;
  }
  return kR_ROS_FLU_TO_FRD;
}

tf2::Matrix3x3 array_to_tf2(const double m[3][3])
{
  return tf2::Matrix3x3(
    m[0][0], m[0][1], m[0][2],
    m[1][0], m[1][1], m[1][2],
    m[2][0], m[2][1], m[2][2]);
}

void tf2_to_wxyz(const tf2::Matrix3x3 & rot, float out_wxyz[4])
{
  tf2::Quaternion q;
  rot.getRotation(q);
  out_wxyz[0] = static_cast<float>(q.w());
  out_wxyz[1] = static_cast<float>(q.x());
  out_wxyz[2] = static_cast<float>(q.y());
  out_wxyz[3] = static_cast<float>(q.z());
}

tf2::Matrix3x3 wxyz_to_tf2(const float q_wxyz[4])
{
  return tf2::Matrix3x3(
    tf2::Quaternion(q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]));
}

tf2::Matrix3x3 ros_quat_to_tf2(const geometry_msgs::msg::Quaternion & q)
{
  return tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w));
}

}  // namespace

BodyAxes body_axes_from_string(const std::string & value)
{
  if (value == "mujoco_enu" || value == "mujoco") {
    return BodyAxes::MujocoEnu;
  }
  if (value == "identity" || value == "frd") {
    return BodyAxes::Identity;
  }
  return BodyAxes::RosFlu;
}

void enu_position_to_ned(const float enu[3], float ned[3])
{
  mat3_vec(kR_NED_ENU, enu, ned);
}

void enu_velocity_to_ned(const float enu[3], float ned[3])
{
  mat3_vec(kR_NED_ENU, enu, ned);
}

void ned_position_to_enu(const float ned[3], float enu[3])
{
  mat3_transpose_vec(kR_NED_ENU, ned, enu);
}

void ned_velocity_to_enu(const float ned[3], float enu[3])
{
  mat3_transpose_vec(kR_NED_ENU, ned, enu);
}

void enu_quat_to_ned_frd(
  const geometry_msgs::msg::Quaternion & q_enu, BodyAxes body_axes, float q_ned_frd_wxyz[4])
{
  const tf2::Matrix3x3 r_enu_body = ros_quat_to_tf2(q_enu);
  const tf2::Matrix3x3 r_ned_enu = array_to_tf2(kR_NED_ENU);
  const tf2::Matrix3x3 r_body_to_frd = array_to_tf2(body_axes_to_mount(body_axes));
  const tf2::Matrix3x3 r_ned_frd = r_ned_enu * r_enu_body * r_body_to_frd;
  tf2_to_wxyz(r_ned_frd, q_ned_frd_wxyz);
}

void ned_frd_quat_to_enu(
  const float q_ned_frd_wxyz[4], BodyAxes body_axes, geometry_msgs::msg::Quaternion & q_enu)
{
  const tf2::Matrix3x3 r_ned_frd = wxyz_to_tf2(q_ned_frd_wxyz);
  const tf2::Matrix3x3 r_ned_enu = array_to_tf2(kR_NED_ENU);
  const tf2::Matrix3x3 r_body_to_frd = array_to_tf2(body_axes_to_mount(body_axes));
  const tf2::Matrix3x3 r_enu_body = r_ned_enu.transpose() * r_ned_frd * r_body_to_frd.transpose();
  tf2::Quaternion q;
  r_enu_body.getRotation(q);
  q_enu.x = q.x();
  q_enu.y = q.y();
  q_enu.z = q.z();
  q_enu.w = q.w();
}

void ros_body_vector_to_frd(const float in[3], BodyAxes body_axes, float out[3])
{
  mat3_vec(body_axes_to_mount(body_axes), in, out);
}

void frd_body_vector_to_ros(const float in[3], BodyAxes body_axes, float out[3])
{
  mat3_transpose_vec(body_axes_to_mount(body_axes), in, out);
}

void frd_body_velocity_to_ned(
  const float v_frd[3], const float q_ned_frd_wxyz[4], float v_ned[3])
{
  const tf2::Matrix3x3 r_ned_frd = wxyz_to_tf2(q_ned_frd_wxyz);
  const tf2::Vector3 v_body(v_frd[0], v_frd[1], v_frd[2]);
  const tf2::Vector3 v_world = r_ned_frd * v_body;
  v_ned[0] = static_cast<float>(v_world.x());
  v_ned[1] = static_cast<float>(v_world.y());
  v_ned[2] = static_cast<float>(v_world.z());
}

void ned_velocity_to_frd_body(
  const float v_ned[3], const float q_ned_frd_wxyz[4], float v_frd[3])
{
  const tf2::Matrix3x3 r_ned_frd = wxyz_to_tf2(q_ned_frd_wxyz);
  const tf2::Vector3 v_world(v_ned[0], v_ned[1], v_ned[2]);
  const tf2::Vector3 v_body = r_ned_frd.transpose() * v_world;
  v_frd[0] = static_cast<float>(v_body.x());
  v_frd[1] = static_cast<float>(v_body.y());
  v_frd[2] = static_cast<float>(v_body.z());
}

}  // namespace inertial_nav_ros2::frames
