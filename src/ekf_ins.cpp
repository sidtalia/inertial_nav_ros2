#include "inertial_nav_ros2/ekf_ins.hpp"

#include <algorithm>
#include <cmath>

#include "ekf_time.h"
#include "inertial_nav_ros2/frame_conversions.hpp"

// nav_filter macros collide with tf2 / angles headers.
#ifdef deg2rad
#undef deg2rad
#endif
#ifdef rad2deg
#undef rad2deg
#endif

#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace inertial_nav_ros2 {
namespace {

constexpr float kDefaultPosErr = 0.5f;
constexpr float kDefaultYawErr = 0.1f;
constexpr float kDefaultVelErr = 0.5f;

void set_symmetric_cov3(
  std::array<double, 36> & cov, std::size_t d0, std::size_t d1, std::size_t d2,
  double c00, double c01, double c02, double c11, double c12, double c22)
{
  cov[d0] = c00;
  cov[d0 + 1] = c01;
  cov[d0 + 2] = c02;
  cov[d1] = c01;
  cov[d1 + 1] = c11;
  cov[d1 + 2] = c12;
  cov[d2] = c02;
  cov[d2 + 1] = c12;
  cov[d2 + 2] = c22;
}

void cov3_to_tf2(
  double c00, double c01, double c02, double c11, double c12, double c22, tf2::Matrix3x3 & out)
{
  out.setValue(c00, c01, c02, c01, c11, c12, c02, c12, c22);
}

void tf2_to_cov3(
  const tf2::Matrix3x3 & in,
  double & c00, double & c01, double & c02, double & c11, double & c12, double & c22)
{
  c00 = in[0][0];
  c01 = in[0][1];
  c02 = in[0][2];
  c11 = in[1][1];
  c12 = in[1][2];
  c22 = in[2][2];
}

tf2::Matrix3x3 ros_body_from_frd_rotation(frames::BodyAxes body_axes)
{
  switch (body_axes) {
    case frames::BodyAxes::RosFlu:
      return tf2::Matrix3x3(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0);
    case frames::BodyAxes::MujocoEnu:
      return tf2::Matrix3x3(0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0);
    case frames::BodyAxes::Identity:
      return tf2::Matrix3x3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
  }
  return tf2::Matrix3x3(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0);
}

float covariance_sigma(const std::array<double, 36> & cov, std::size_t idx, float fallback)
{
  const double v = cov[idx];
  if (v > 0.0) {
    return static_cast<float>(std::sqrt(v));
  }
  return fallback;
}

/** ISA barometric altitude (m AMSL) from static pressure (Pa). */
float pressure_to_amsl_altitude_m(float pressure_pa, float sea_level_pressure_pa)
{
  if (pressure_pa <= 0.0f || sea_level_pressure_pa <= 0.0f) {
    return 0.0f;
  }
  const float ratio = pressure_pa / sea_level_pressure_pa;
  return 44330.0f * (1.0f - std::pow(ratio, 0.190294957f));
}

}  // namespace

EkfIns::EkfIns(const rclcpp::NodeOptions & options)
: Node("nav_filter_ekf", options)
{
  declare_parameters();
  load_static_config();

  imu_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sensor_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions imu_options;
  imu_options.callback_group = imu_callback_group_;
  rclcpp::SubscriptionOptions sensor_options;
  sensor_options.callback_group = sensor_callback_group_;

  const auto imu_qos = rclcpp::SensorDataQoS();
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, imu_qos,
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(*msg); },
    imu_options);

  for (std::size_t i = 0; i < kNumPose; ++i) {
    const auto idx = i;
    pose_subs_[i] = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_params_[i].topic, rclcpp::QoS(10),
      [this, idx](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        on_pose(idx, *msg);
      },
      sensor_options);
    if (!pose_params_[i].odom_topic.empty()) {
      pose_odom_subs_[i] = create_subscription<nav_msgs::msg::Odometry>(
        pose_params_[i].odom_topic, rclcpp::QoS(10),
        [this, idx](const nav_msgs::msg::Odometry::SharedPtr msg) { on_pose_odom(idx, *msg); },
        sensor_options);
    }
  }

  for (std::size_t i = 0; i < kNumOdom; ++i) {
    const auto idx = i;
    if (!odom_params_[i].topic.empty()) {
      odom_subs_[i] = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        odom_params_[i].topic, rclcpp::QoS(10),
        [this, idx](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
          on_odom(idx, *msg);
        },
        sensor_options);
    }
    if (!odom_params_[i].odom_topic.empty()) {
      odom_odom_subs_[i] = create_subscription<nav_msgs::msg::Odometry>(
        odom_params_[i].odom_topic, rclcpp::QoS(10),
        [this, idx](const nav_msgs::msg::Odometry::SharedPtr msg) { on_odom_odom(idx, *msg); },
        sensor_options);
    }
  }

  for (std::size_t i = 0; i < kNumMag; ++i) {
    const auto idx = i;
    mag_subs_[i] = create_subscription<sensor_msgs::msg::MagneticField>(
      mag_params_[i].topic, rclcpp::QoS(10),
      [this, idx](const sensor_msgs::msg::MagneticField::SharedPtr msg) { on_mag(idx, *msg); },
      sensor_options);
  }

  if (!baro_params_.pressure_topic.empty()) {
    baro_pressure_sub_ = create_subscription<sensor_msgs::msg::FluidPressure>(
      baro_params_.pressure_topic, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::FluidPressure::SharedPtr msg) { on_baro_pressure(*msg); },
      sensor_options);
  } else if (!baro_params_.topic.empty()) {
    baro_sub_ = create_subscription<std_msgs::msg::Float64>(
      baro_params_.topic, rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64::SharedPtr msg) { on_baro(*msg); },
      sensor_options);
  }

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("ekf/odometry", rclcpp::QoS(10));

  bool need_tf = false;
  for (const auto & p : pose_params_) {
    if (p.extnav_not_earth_aligned) {
      need_tf = true;
      break;
    }
  }
  if (need_tf) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  ekf_.set_ext_nav_origin(ext_nav_origin_lat_, ext_nav_origin_lon_, ext_nav_origin_hgt_);

  RCLCPP_INFO(
    get_logger(),
    "EkfIns ready (nav_filter from Inertial_nav_shim). input_in_enu=%s body_axes=%s "
    "origin=(%.6f, %.6f, %.1f) init_reference=%s",
    pose_z_is_down_ ? "true" : "false",
    pose_z_is_down_ ?
    (body_axes_ == frames::BodyAxes::MujocoEnu ? "mujoco_enu" :
    body_axes_ == frames::BodyAxes::Identity ? "identity" : "ros_flu") :
    "passthrough",
    ext_nav_origin_lat_, ext_nav_origin_lon_, ext_nav_origin_hgt_,
    init_reference_ == InitReferenceSource::IMU_AHRS ? "imu_ahrs" :
    init_reference_ == InitReferenceSource::GPS ? "gps" : "ext_nav");
}

EkfIns::~EkfIns() = default;

bool EkfIns::transform_pose_odom_to_earth(std::size_t id, nav_msgs::msg::Odometry & msg) const
{
  if (id >= kNumPose || !pose_params_[id].extnav_not_earth_aligned) {
    return true;
  }
  if (!tf_buffer_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "extnav_not_earth_aligned set but TF buffer is not initialized");
    return false;
  }
  geometry_msgs::msg::PoseStamped in_pose;
  in_pose.header.stamp = msg.header.stamp;
  in_pose.header.frame_id = pose_params_[id].vslam_local_frame;
  in_pose.pose = msg.pose.pose;
  geometry_msgs::msg::PoseStamped out_pose;
  try {
    const tf2::Duration timeout = tf2::durationFromSec(pose_params_[id].extnav_tf_timeout_s);
    const auto tf = tf_buffer_->lookupTransform(
      pose_params_[id].earth_frame, pose_params_[id].vslam_local_frame,
      tf2::TimePointZero, timeout);
    tf2::doTransform(in_pose, out_pose, tf);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "ext-nav earth alignment TF lookup failed: %s", ex.what());
    return false;
  }
  msg.header.stamp = out_pose.header.stamp;
  msg.header.frame_id = pose_params_[id].earth_frame;
  msg.pose.pose = out_pose.pose;
  return true;
}

void EkfIns::declare_parameters()
{
  pose_z_is_down_ = declare_parameter<bool>("pose_z_is_down", true);
  body_axes_ = frames::body_axes_from_string(
    declare_parameter<std::string>("body_axes", "ros_flu"));
  max_transport_delay_ms_ = declare_parameter<int>("max_transport_delay_ms", 500);
  output_frame_ = declare_parameter<std::string>("output_frame", "map");
  child_frame_ = declare_parameter<std::string>("child_frame", "base_link");
  const std::string init_ref_str = declare_parameter<std::string>("init_reference", "ext_nav");
  if (init_ref_str == "imu_ahrs" || init_ref_str == "ahrs") {
    init_reference_ = InitReferenceSource::IMU_AHRS;
  } else if (init_ref_str == "gps") {
    init_reference_ = InitReferenceSource::GPS;
  } else {
    init_reference_ = InitReferenceSource::EXT_NAV;
  }
  imu_topic_ = declare_parameter<std::string>("topics.imu", "imu");
  declare_parameter<int>("executor_threads", 2);

  ext_nav_origin_lat_ = declare_parameter<double>("ext_nav_origin.lat", 0.0);
  ext_nav_origin_lon_ = declare_parameter<double>("ext_nav_origin.lon", 0.0);
  ext_nav_origin_hgt_ = static_cast<float>(declare_parameter<double>("ext_nav_origin.hgt", 0.0));

  const std::array<const char *, kNumPose> pose_names = {"pose_src1", "pose_src2"};
  for (std::size_t i = 0; i < kNumPose; ++i) {
    const std::string prefix = pose_names[i];
    auto & p = pose_params_[i];
    p.enable = declare_parameter<bool>("enable." + prefix, i == 0);
    p.delay_pos_ms = static_cast<uint32_t>(declare_parameter<int>(prefix + ".delay_ms.pos", 100));
    p.delay_yaw_ms = static_cast<uint32_t>(declare_parameter<int>(prefix + ".delay_ms.yaw", 100));
    p.delay_vel_ms = static_cast<uint32_t>(declare_parameter<int>(prefix + ".delay_ms.vel", 100));
    p.fuse_height = declare_parameter<bool>(prefix + ".fuse_height", false);
    p.fuse_yaw = declare_parameter<bool>(prefix + ".fuse_yaw", true);
    p.fuse_vel = declare_parameter<bool>(prefix + ".fuse_vel", false);
    p.fuse_pos = declare_parameter<bool>(prefix + ".fuse_pos", true);
    p.extnav_not_earth_aligned =
      declare_parameter<bool>(prefix + ".extnav_not_earth_aligned", false);
    p.extnav_tf_timeout_s = static_cast<float>(
      declare_parameter<double>(prefix + ".extnav_tf_timeout_s", 6.0));
    p.earth_frame = declare_parameter<std::string>(prefix + ".earth_frame", "earth");
    p.vslam_local_frame =
      declare_parameter<std::string>(prefix + ".vslam_local_frame", "local");
    p.max_rate_hz = static_cast<float>(declare_parameter<double>(prefix + ".max_rate_hz", 0.0));
    p.topic = declare_parameter<std::string>("topics." + prefix, prefix);
    p.odom_topic = declare_parameter<std::string>("topics." + prefix + "_odom", "");
  }

  const std::array<const char *, kNumOdom> odom_names = {"odom_src1", "odom_src2"};
  for (std::size_t i = 0; i < kNumOdom; ++i) {
    const std::string prefix = odom_names[i];
    auto & p = odom_params_[i];
    p.enable = declare_parameter<bool>("enable." + prefix, i == 0);
    p.delay_ms = static_cast<uint32_t>(declare_parameter<int>(prefix + ".delay_ms", 25));
    p.max_rate_hz = static_cast<float>(declare_parameter<double>(prefix + ".max_rate_hz", 0.0));
    p.max_body_vel_mps =
      static_cast<float>(declare_parameter<double>(prefix + ".max_body_vel_mps", 0.0));
    p.topic = declare_parameter<std::string>("topics." + prefix, prefix);
    p.odom_topic = declare_parameter<std::string>("topics." + prefix + "_odom", "");
  }

  const std::array<const char *, kNumMag> mag_names = {"mag_src1", "mag_src2"};
  for (std::size_t i = 0; i < kNumMag; ++i) {
    const std::string prefix = mag_names[i];
    auto & p = mag_params_[i];
    p.enable = declare_parameter<bool>("enable." + prefix, false);
    p.delay_ms = static_cast<uint32_t>(declare_parameter<int>(prefix + ".delay_ms", 25));
    p.max_rate_hz = static_cast<float>(declare_parameter<double>(prefix + ".max_rate_hz", 0.0));
    p.topic = declare_parameter<std::string>("topics." + prefix, prefix);
  }

  baro_params_.enable = declare_parameter<bool>("enable.baro", true);
  baro_params_.delay_ms = static_cast<uint32_t>(declare_parameter<int>("baro.delay_ms", 50));
  baro_params_.default_sigma =
    static_cast<float>(declare_parameter<double>("baro.default_sigma", 1.0));
  baro_params_.sea_level_pressure_pa = static_cast<float>(
    declare_parameter<double>("baro.sea_level_pressure_pa", 101325.0));
  baro_params_.max_rate_hz =
    static_cast<float>(declare_parameter<double>("baro.max_rate_hz", 0.0));
  baro_params_.topic = declare_parameter<std::string>("topics.baro", "baro");
  baro_params_.pressure_topic =
    declare_parameter<std::string>("topics.baro_pressure", "");
}

void EkfIns::load_static_config()
{
  EkfSourceConfig cfg;
  cfg.init_ref = init_reference_;

  for (std::size_t i = 0; i < kNumPose; ++i) {
    cfg.ext_nav[i].enabled = pose_params_[i].enable;
    cfg.ext_nav[i].fuse_height = pose_params_[i].fuse_height;
    cfg.ext_nav[i].fuse_yaw = pose_params_[i].fuse_yaw;
    cfg.ext_nav[i].fuse_vel = pose_params_[i].fuse_vel;
    cfg.ext_nav[i].fuse_pos = pose_params_[i].fuse_pos;
    cfg.ext_nav[i].pos_delay_ms = pose_params_[i].delay_pos_ms;
    cfg.ext_nav[i].yaw_delay_ms = pose_params_[i].delay_yaw_ms;
    cfg.ext_nav[i].vel_delay_ms = pose_params_[i].delay_vel_ms;
  }

  for (std::size_t i = 0; i < kNumOdom; ++i) {
    cfg.odom[i].enabled = odom_params_[i].enable;
    cfg.odom[i].delay_ms = odom_params_[i].delay_ms;
  }

  for (std::size_t i = 0; i < kNumMag; ++i) {
    cfg.mag[i].enabled = mag_params_[i].enable;
    cfg.mag[i].delay_ms = mag_params_[i].delay_ms;
  }

  cfg.baro_delay_ms = baro_params_.delay_ms;
  ekf_.set_ekf_config(cfg);
  ekf_.set_baro_enabled(baro_params_.enable);
}

void EkfIns::refresh_enable_flags()
{
  for (std::size_t i = 0; i < kNumPose; ++i) {
    const std::string prefix = (i == 0) ? "pose_src1" : "pose_src2";
    pose_params_[i].enable = get_parameter("enable." + prefix).as_bool();
    ekf_.set_ext_nav_enabled(static_cast<uint8_t>(i), pose_params_[i].enable);
  }
  for (std::size_t i = 0; i < kNumOdom; ++i) {
    const std::string prefix = (i == 0) ? "odom_src1" : "odom_src2";
    odom_params_[i].enable = get_parameter("enable." + prefix).as_bool();
    ekf_.set_odom_enabled(static_cast<uint8_t>(i), odom_params_[i].enable);
  }
  for (std::size_t i = 0; i < kNumMag; ++i) {
    const std::string prefix = (i == 0) ? "mag_src1" : "mag_src2";
    mag_params_[i].enable = get_parameter("enable." + prefix).as_bool();
    ekf_.set_mag_enabled(static_cast<uint8_t>(i), mag_params_[i].enable);
  }
  baro_params_.enable = get_parameter("enable.baro").as_bool();
  ekf_.set_baro_enabled(baro_params_.enable);
}

void EkfIns::apply_transport_delays(uint32_t fusion_ms, const SensorSnapshot & snapshot)
{
  for (std::size_t i = 0; i < kNumPose; ++i) {
    if (!pose_params_[i].enable) {
      continue;
    }
    const uint32_t msg_ms =
      snapshot.pose_valid[i] ? snapshot.pose[i].stamp.stamp_ms : fusion_ms;
    const uint32_t pos_eff = effective_delay_ms(
      pose_params_[i].delay_pos_ms, fusion_ms, msg_ms, max_transport_delay_ms_);
    const uint32_t yaw_eff = effective_delay_ms(
      pose_params_[i].delay_yaw_ms, fusion_ms, msg_ms, max_transport_delay_ms_);
    const uint32_t vel_eff = effective_delay_ms(
      pose_params_[i].delay_vel_ms, fusion_ms, msg_ms, max_transport_delay_ms_);
    ekf_.setExtNavFusionDelaysMs(static_cast<uint8_t>(i), pos_eff, vel_eff, yaw_eff);
  }

  for (std::size_t i = 0; i < kNumOdom; ++i) {
    if (!odom_params_[i].enable) {
      continue;
    }
    const uint32_t msg_ms =
      snapshot.odom_valid[i] ? snapshot.odom[i].stamp.stamp_ms : fusion_ms;
    const uint32_t eff =
      effective_delay_ms(odom_params_[i].delay_ms, fusion_ms, msg_ms, max_transport_delay_ms_);
    ekf_.setOdomFusionDelayMs(static_cast<uint8_t>(i), eff);
  }

  for (std::size_t i = 0; i < kNumMag; ++i) {
    if (!mag_params_[i].enable) {
      continue;
    }
    const uint32_t msg_ms = snapshot.mag_valid[i] ? snapshot.mag[i].stamp.stamp_ms : fusion_ms;
    const uint32_t eff =
      effective_delay_ms(mag_params_[i].delay_ms, fusion_ms, msg_ms, max_transport_delay_ms_);
    ekf_.setMagFusionDelayMs(static_cast<uint8_t>(i), eff);
  }

  if (baro_params_.enable) {
    const uint32_t msg_ms = snapshot.baro_valid ? snapshot.baro.stamp.stamp_ms : fusion_ms;
    const uint32_t eff =
      effective_delay_ms(baro_params_.delay_ms, fusion_ms, msg_ms, max_transport_delay_ms_);
    ekf_.setBaroFusionDelayMs(eff);
  }
}

SensorSnapshot EkfIns::take_sensor_snapshot()
{
  SensorSnapshot snapshot;
  std::lock_guard<std::mutex> lock(cache_mutex_);

  for (std::size_t i = 0; i < kNumPose; ++i) {
    if (!pose_cache_[i].stamp.valid) {
      continue;
    }
    snapshot.pose[i] = pose_cache_[i];
    snapshot.pose_valid[i] = true;
    pose_cache_[i].stamp.valid = false;
  }

  for (std::size_t i = 0; i < kNumOdom; ++i) {
    if (!odom_cache_[i].stamp.valid) {
      continue;
    }
    snapshot.odom[i] = odom_cache_[i];
    snapshot.odom_valid[i] = true;
    odom_cache_[i].stamp.valid = false;
  }

  for (std::size_t i = 0; i < kNumMag; ++i) {
    if (!mag_cache_[i].stamp.valid) {
      continue;
    }
    snapshot.mag[i] = mag_cache_[i];
    snapshot.mag_valid[i] = true;
    mag_cache_[i].stamp.valid = false;
  }

  if (baro_cache_.stamp.valid) {
    snapshot.baro = baro_cache_;
    snapshot.baro_valid = true;
    baro_cache_.stamp.valid = false;
  }

  return snapshot;
}

uint32_t EkfIns::stamp_to_ms(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<uint32_t>(stamp.sec) * 1000U +
         static_cast<uint32_t>(stamp.nanosec / 1000000U);
}

uint32_t EkfIns::effective_delay_ms(
  uint32_t baseline_ms, uint32_t fusion_ms, uint32_t msg_ms, uint32_t max_transport_ms)
{
  const uint32_t transport_ms = (fusion_ms >= msg_ms) ? (fusion_ms - msg_ms) : 0U;
  const uint32_t total = baseline_ms + transport_ms;
  return std::min(total, max_transport_ms);
}

void EkfIns::pose_msg_to_meas(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg, PoseMeas & out) const
{
  const auto & p = msg.pose.pose.position;
  const auto & q = msg.pose.pose.orientation;

  if (pose_z_is_down_) {
    const float pos_enu[3] = {
      static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)};
    float pos_ned[3];
    frames::enu_position_to_ned(pos_enu, pos_ned);
    out.pos = {pos_ned[0], pos_ned[1], pos_ned[2]};
    frames::enu_quat_to_ned_frd(q, body_axes_, out.quat.data());
  } else {
    out.pos = {
      static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)};
    out.quat = {
      static_cast<float>(q.w), static_cast<float>(q.x), static_cast<float>(q.y),
      static_cast<float>(q.z)};
  }

  out.vel = {0.0f, 0.0f, 0.0f};
  out.pos_err = covariance_sigma(msg.pose.covariance, 0, kDefaultPosErr);
  const float yaw_var = msg.pose.covariance[35] > 0.0 ? msg.pose.covariance[35] :
                                                        msg.pose.covariance[21];
  out.yaw_err = yaw_var > 0.0 ? static_cast<float>(std::sqrt(yaw_var)) : kDefaultYawErr;
  out.vel_err = kDefaultVelErr;
  out.stamp.stamp_ms = stamp_to_ms(msg.header.stamp);
  out.stamp.valid = true;
}

void EkfIns::pose_odom_to_meas(const nav_msgs::msg::Odometry & msg, PoseMeas & out) const
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header = msg.header;
  pose_msg.pose = msg.pose;
  pose_msg_to_meas(pose_msg, out);

  // nav_msgs/Odometry twist.linear is in child_frame (body). cuVSLAM derives it from
  // inv(pose0)*pose1/dt, so it is body-frame — not ENU/NED world velocity.
  const float body_vel_ros[3] = {
    static_cast<float>(msg.twist.twist.linear.x),
    static_cast<float>(msg.twist.twist.linear.y),
    static_cast<float>(msg.twist.twist.linear.z)};
  float body_vel_frd[3];
  if (pose_z_is_down_) {
    frames::ros_body_vector_to_frd(body_vel_ros, body_axes_, body_vel_frd);
    float vel_ned[3];
    frames::frd_body_velocity_to_ned(body_vel_frd, out.quat.data(), vel_ned);
    out.vel = {vel_ned[0], vel_ned[1], vel_ned[2]};
  } else {
    frames::frd_body_velocity_to_ned(body_vel_ros, out.quat.data(), out.vel.data());
  }
  out.vel_err = covariance_sigma(msg.twist.covariance, 0, kDefaultVelErr);
  out.stamp.stamp_ms = stamp_to_ms(msg.header.stamp);
  out.stamp.valid = true;
}

bool EkfIns::accept_pose_update(std::size_t id, uint32_t stamp_ms)
{
  if (id >= kNumPose) {
    return false;
  }
  const float max_hz = pose_params_[id].max_rate_hz;
  if (max_hz <= 0.0f) {
    return true;
  }
  const uint32_t min_interval_ms = static_cast<uint32_t>(1000.0f / max_hz);
  if (min_interval_ms == 0U) {
    return true;
  }
  const uint32_t last_ms = last_pose_accept_ms_[id];
  if (last_ms != 0U && stamp_ms >= last_ms && (stamp_ms - last_ms) < min_interval_ms) {
    return false;
  }
  last_pose_accept_ms_[id] = stamp_ms;
  return true;
}

bool EkfIns::accept_odom_update(std::size_t id, uint32_t stamp_ms)
{
  if (id >= kNumOdom) {
    return false;
  }
  const float max_hz = odom_params_[id].max_rate_hz;
  if (max_hz <= 0.0f) {
    return true;
  }
  const uint32_t min_interval_ms = static_cast<uint32_t>(1000.0f / max_hz);
  if (min_interval_ms == 0U) {
    return true;
  }
  const uint32_t last_ms = last_odom_accept_ms_[id];
  if (last_ms != 0U && stamp_ms >= last_ms && (stamp_ms - last_ms) < min_interval_ms) {
    return false;
  }
  last_odom_accept_ms_[id] = stamp_ms;
  return true;
}

bool EkfIns::odom_body_vel_sane(std::size_t id, const OdomMeas & meas) const
{
  if (id >= kNumOdom) {
    return false;
  }
  const float max_v = odom_params_[id].max_body_vel_mps;
  if (max_v <= 0.0f) {
    return true;
  }
  const float mag = std::sqrt(
    meas.body_vel[0] * meas.body_vel[0] + meas.body_vel[1] * meas.body_vel[1] +
    meas.body_vel[2] * meas.body_vel[2]);
  return mag <= max_v;
}

bool EkfIns::accept_mag_update(std::size_t id, uint32_t stamp_ms)
{
  if (id >= kNumMag) {
    return false;
  }
  const float max_hz = mag_params_[id].max_rate_hz;
  if (max_hz <= 0.0f) {
    return true;
  }
  const uint32_t min_interval_ms = static_cast<uint32_t>(1000.0f / max_hz);
  if (min_interval_ms == 0U) {
    return true;
  }
  const uint32_t last_ms = last_mag_accept_ms_[id];
  if (last_ms != 0U && stamp_ms >= last_ms && (stamp_ms - last_ms) < min_interval_ms) {
    return false;
  }
  last_mag_accept_ms_[id] = stamp_ms;
  return true;
}

bool EkfIns::pose_source_fuses(const EkfIns::PoseSourceParams & params)
{
  return params.fuse_height || params.fuse_yaw || params.fuse_vel || params.fuse_pos;
}

bool EkfIns::accept_baro_update(uint32_t stamp_ms)
{
  const float max_hz = baro_params_.max_rate_hz;
  if (max_hz <= 0.0f) {
    return true;
  }
  const uint32_t min_interval_ms = static_cast<uint32_t>(1000.0f / max_hz);
  if (min_interval_ms == 0U) {
    return true;
  }
  if (last_baro_accept_ms_ != 0U && stamp_ms >= last_baro_accept_ms_ &&
      (stamp_ms - last_baro_accept_ms_) < min_interval_ms)
  {
    return false;
  }
  last_baro_accept_ms_ = stamp_ms;
  return true;
}

void EkfIns::on_imu(const sensor_msgs::msg::Imu & msg)
{
  const SensorSnapshot snapshot = take_sensor_snapshot();
  std::lock_guard<std::mutex> ekf_lock(ekf_mutex_);
  ekf_ins_run(msg, snapshot);
}

void EkfIns::on_pose(std::size_t id, const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  if (id >= kNumPose) {
    return;
  }
  PoseMeas meas;
  pose_msg_to_meas(msg, meas);
  if (!pose_params_[id].fuse_vel) {
    meas.vel = {0.0f, 0.0f, 0.0f};
  }
  std::lock_guard<std::mutex> lock(cache_mutex_);
  if (!accept_pose_update(id, meas.stamp.stamp_ms)) {
    return;
  }
  pose_cache_[id] = meas;
}

void EkfIns::on_pose_odom(std::size_t id, const nav_msgs::msg::Odometry & msg)
{
  if (id >= kNumPose) {
    return;
  }
  nav_msgs::msg::Odometry aligned = msg;
  if (pose_params_[id].extnav_not_earth_aligned && !transform_pose_odom_to_earth(id, aligned)) {
    return;
  }
  PoseMeas meas;
  pose_odom_to_meas(aligned, meas);
  if (!pose_params_[id].fuse_vel) {
    meas.vel = {0.0f, 0.0f, 0.0f};
  }
  std::lock_guard<std::mutex> lock(cache_mutex_);
  if (!accept_pose_update(id, meas.stamp.stamp_ms)) {
    return;
  }
  pose_cache_[id] = meas;
}

void EkfIns::on_odom(std::size_t id, const geometry_msgs::msg::TwistWithCovarianceStamped & msg)
{
  if (id >= kNumOdom) {
    return;
  }
  OdomMeas meas;
  const float twist_ros[3] = {
    static_cast<float>(msg.twist.twist.linear.x),
    static_cast<float>(msg.twist.twist.linear.y),
    static_cast<float>(msg.twist.twist.linear.z)};
  if (pose_z_is_down_) {
    frames::ros_body_vector_to_frd(twist_ros, body_axes_, meas.body_vel.data());
  } else {
    meas.body_vel = {twist_ros[0], twist_ros[1], twist_ros[2]};
  }
  meas.vel_err = covariance_sigma(msg.twist.covariance, 0, kDefaultVelErr);
  meas.stamp.stamp_ms = stamp_to_ms(msg.header.stamp);
  meas.stamp.valid = true;
  std::lock_guard<std::mutex> lock(cache_mutex_);
  if (!odom_body_vel_sane(id, meas)) {
    return;
  }
  if (!accept_odom_update(id, meas.stamp.stamp_ms)) {
    return;
  }
  odom_cache_[id] = meas;
}

void EkfIns::on_odom_odom(std::size_t id, const nav_msgs::msg::Odometry & msg)
{
  if (id >= kNumOdom) {
    return;
  }
  OdomMeas meas;
  const float twist_ros[3] = {
    static_cast<float>(msg.twist.twist.linear.x),
    static_cast<float>(msg.twist.twist.linear.y),
    static_cast<float>(msg.twist.twist.linear.z)};
  if (pose_z_is_down_) {
    frames::ros_body_vector_to_frd(twist_ros, body_axes_, meas.body_vel.data());
  } else {
    meas.body_vel = {twist_ros[0], twist_ros[1], twist_ros[2]};
  }
  meas.vel_err = covariance_sigma(msg.twist.covariance, 0, kDefaultVelErr);
  meas.stamp.stamp_ms = stamp_to_ms(msg.header.stamp);
  meas.stamp.valid = true;
  std::lock_guard<std::mutex> lock(cache_mutex_);
  if (!odom_body_vel_sane(id, meas)) {
    return;
  }
  if (!accept_odom_update(id, meas.stamp.stamp_ms)) {
    return;
  }
  odom_cache_[id] = meas;
}

void EkfIns::on_mag(std::size_t id, const sensor_msgs::msg::MagneticField & msg)
{
  if (id >= kNumMag) {
    return;
  }
  MagMeas meas;
  const float field_t[3] = {
    static_cast<float>(msg.magnetic_field.x * 1.0e6),
    static_cast<float>(msg.magnetic_field.y * 1.0e6),
    static_cast<float>(msg.magnetic_field.z * 1.0e6)};
  if (pose_z_is_down_) {
    frames::ros_body_vector_to_frd(field_t, body_axes_, meas.field_ut.data());
  } else {
    meas.field_ut = {field_t[0], field_t[1], field_t[2]};
  }
  meas.stamp.stamp_ms = stamp_to_ms(msg.header.stamp);
  meas.stamp.valid = true;
  std::lock_guard<std::mutex> lock(cache_mutex_);
  if (!accept_mag_update(id, meas.stamp.stamp_ms)) {
    return;
  }
  mag_cache_[id] = meas;
}

void EkfIns::on_baro(const std_msgs::msg::Float64 & msg)
{
  BaroMeas meas;
  meas.height_m = static_cast<float>(msg.data);
  meas.sigma = baro_params_.default_sigma;
  meas.stamp.stamp_ms = stamp_to_ms(now());
  meas.stamp.valid = true;
  std::lock_guard<std::mutex> lock(cache_mutex_);
  if (!accept_baro_update(meas.stamp.stamp_ms)) {
    return;
  }
  baro_cache_ = meas;
}

void EkfIns::on_baro_pressure(const sensor_msgs::msg::FluidPressure & msg)
{
  BaroMeas meas;
  meas.height_m = pressure_to_amsl_altitude_m(
    static_cast<float>(msg.fluid_pressure), baro_params_.sea_level_pressure_pa);
  if (msg.variance > 0.0) {
    meas.sigma = static_cast<float>(std::sqrt(msg.variance));
  } else {
    meas.sigma = baro_params_.default_sigma;
  }
  meas.stamp.stamp_ms = stamp_to_ms(msg.header.stamp);
  meas.stamp.valid = true;
  std::lock_guard<std::mutex> lock(cache_mutex_);
  if (!accept_baro_update(meas.stamp.stamp_ms)) {
    return;
  }
  baro_cache_ = meas;
}

void EkfIns::ekf_ins_run(const sensor_msgs::msg::Imu & imu, const SensorSnapshot & snapshot)
{
  refresh_enable_flags();

  const uint32_t t_ms = stamp_to_ms(imu.header.stamp);
  apply_transport_delays(t_ms, snapshot);

  float imu_dt = 0.0025f;
  if (have_prev_imu_stamp_) {
    const uint32_t delta = (t_ms >= prev_imu_stamp_ms_) ? (t_ms - prev_imu_stamp_ms_) : 0U;
    if (delta > 0U && delta < 100U) {
      imu_dt = static_cast<float>(delta) * 0.001f;
    }
  }
  prev_imu_stamp_ms_ = t_ms;
  have_prev_imu_stamp_ = true;

  float accel[3];
  float gyro[3];
  const float accel_ros[3] = {
    static_cast<float>(imu.linear_acceleration.x),
    static_cast<float>(imu.linear_acceleration.y),
    static_cast<float>(imu.linear_acceleration.z)};
  const float gyro_ros[3] = {
    static_cast<float>(imu.angular_velocity.x),
    static_cast<float>(imu.angular_velocity.y),
    static_cast<float>(imu.angular_velocity.z)};
  if (pose_z_is_down_) {
    frames::ros_body_vector_to_frd(accel_ros, body_axes_, accel);
    frames::ros_body_vector_to_frd(gyro_ros, body_axes_, gyro);
  } else {
    accel[0] = accel_ros[0];
    accel[1] = accel_ros[1];
    accel[2] = accel_ros[2];
    gyro[0] = gyro_ros[0];
    gyro[1] = gyro_ros[1];
    gyro[2] = gyro_ros[2];
  }

  const uint64_t t_us = static_cast<uint64_t>(t_ms) * 1000ULL;
  ekf_.setIMUData(accel, gyro, imu_dt, static_cast<float>(t_ms), static_cast<float>(t_us));

  for (std::size_t i = 0; i < kNumPose; ++i) {
    if (!pose_params_[i].enable || !snapshot.pose_valid[i]) {
      continue;
    }
    if (!ekf_.is_initialized() && init_reference_ == InitReferenceSource::IMU_AHRS) {
      continue;
    }
    if (ekf_.is_initialized() && !pose_source_fuses(pose_params_[i])) {
      continue;
    }
    const auto & m = snapshot.pose[i];
    ekf_.set_ext_nav_pose(
      m.pos.data(), m.vel.data(), m.quat.data(), m.pos_err, m.yaw_err, m.vel_err, true,
      static_cast<uint8_t>(i));
  }

  for (std::size_t i = 0; i < kNumOdom; ++i) {
    if (!odom_params_[i].enable || !snapshot.odom_valid[i]) {
      continue;
    }
    const auto & m = snapshot.odom[i];
    float body_vel[4] = {m.body_vel[0], m.body_vel[1], m.body_vel[2], m.vel_err};
    ekf_.setOdomData(body_vel, true, static_cast<uint8_t>(i));
  }

  for (std::size_t i = 0; i < kNumMag; ++i) {
    if (!mag_params_[i].enable || !snapshot.mag_valid[i]) {
      continue;
    }
    const auto & m = snapshot.mag[i];
    float field[3] = {m.field_ut[0], m.field_ut[1], m.field_ut[2]};
    float zero_bias[3] = {0.0f, 0.0f, 0.0f};
    ekf_.setMagData(field, zero_bias, true, static_cast<uint8_t>(i));
  }

  if (baro_params_.enable && snapshot.baro_valid) {
    ekf_.setAirData(0.0f, snapshot.baro.height_m, snapshot.baro.sigma, imu_dt, true, false);
  }

  ekf_.run_filter(filter_reset_);
  filter_reset_ = false;

  if (ekf_.is_initialized()) {
    publish_odometry(t_ms);
  }
}

void EkfIns::ned_state_to_odom(
  const float * states, uint32_t stamp_ms, nav_msgs::msg::Odometry & odom) const
{
  odom.header.stamp.sec = static_cast<int32_t>(stamp_ms / 1000U);
  odom.header.stamp.nanosec = static_cast<uint32_t>((stamp_ms % 1000U) * 1000000U);
  odom.header.frame_id = output_frame_;
  odom.child_frame_id = child_frame_;

  const float quat_wxyz[4] = {states[0], states[1], states[2], states[3]};
  const float vel_ned[3] = {states[4], states[5], states[6]};
  const float pos_ned[3] = {states[7], states[8], states[9]};

  if (pose_z_is_down_) {
    float pos_enu[3];
    frames::ned_position_to_enu(pos_ned, pos_enu);
    odom.pose.pose.position.x = pos_enu[0];
    odom.pose.pose.position.y = pos_enu[1];
    odom.pose.pose.position.z = pos_enu[2];
    frames::ned_frd_quat_to_enu(quat_wxyz, body_axes_, odom.pose.pose.orientation);

    float vel_frd[3];
    float vel_body_ros[3];
    frames::ned_velocity_to_frd_body(vel_ned, quat_wxyz, vel_frd);
    frames::frd_body_vector_to_ros(vel_frd, body_axes_, vel_body_ros);
    odom.twist.twist.linear.x = vel_body_ros[0];
    odom.twist.twist.linear.y = vel_body_ros[1];
    odom.twist.twist.linear.z = vel_body_ros[2];
  } else {
    odom.pose.pose.position.x = pos_ned[0];
    odom.pose.pose.position.y = pos_ned[1];
    odom.pose.pose.position.z = pos_ned[2];
    odom.pose.pose.orientation.w = quat_wxyz[0];
    odom.pose.pose.orientation.x = quat_wxyz[1];
    odom.pose.pose.orientation.y = quat_wxyz[2];
    odom.pose.pose.orientation.z = quat_wxyz[3];

    float vel_frd[3];
    frames::ned_velocity_to_frd_body(vel_ned, quat_wxyz, vel_frd);
    odom.twist.twist.linear.x = vel_frd[0];
    odom.twist.twist.linear.y = vel_frd[1];
    odom.twist.twist.linear.z = vel_frd[2];
  }
}

void EkfIns::fill_output_covariance(
  const float P[22][22], const float q_ned_frd_wxyz[4],
  nav_msgs::msg::Odometry & odom) const
{
  odom.pose.covariance.fill(0.0);
  odom.twist.covariance.fill(0.0);

  const double pn = std::max(static_cast<double>(P[7][7]), 1e-9);
  const double pe = std::max(static_cast<double>(P[8][8]), 1e-9);
  const double pd = std::max(static_cast<double>(P[9][9]), 1e-9);
  const double pne = static_cast<double>(P[7][8]);
  const double pnd = static_cast<double>(P[7][9]);
  const double ped = static_cast<double>(P[8][9]);

  const double vn = std::max(static_cast<double>(P[4][4]), 1e-9);
  const double ve = std::max(static_cast<double>(P[5][5]), 1e-9);
  const double vd = std::max(static_cast<double>(P[6][6]), 1e-9);
  const double vne = static_cast<double>(P[4][5]);
  const double vnd = static_cast<double>(P[4][6]);
  const double ved = static_cast<double>(P[5][6]);

  if (pose_z_is_down_) {
    set_symmetric_cov3(odom.pose.covariance, 0, 6, 12, pe, pne, -ped, pn, -pnd, pd);
  } else {
    set_symmetric_cov3(odom.pose.covariance, 0, 6, 12, pn, pne, pnd, pe, ped, pd);
  }

  tf2::Matrix3x3 cov_vel_ned;
  cov3_to_tf2(vn, vne, vnd, ve, ved, vd, cov_vel_ned);
  const tf2::Matrix3x3 r_ned_frd(
    tf2::Quaternion(q_ned_frd_wxyz[1], q_ned_frd_wxyz[2], q_ned_frd_wxyz[3], q_ned_frd_wxyz[0]));
  tf2::Matrix3x3 cov_vel_frd = r_ned_frd.transpose() * cov_vel_ned * r_ned_frd;

  tf2::Matrix3x3 cov_vel_twist = cov_vel_frd;
  if (pose_z_is_down_) {
    const tf2::Matrix3x3 r_ros_from_frd = ros_body_from_frd_rotation(body_axes_);
    cov_vel_twist = r_ros_from_frd * cov_vel_frd * r_ros_from_frd.transpose();
  }

  double tc00, tc01, tc02, tc11, tc12, tc22;
  tf2_to_cov3(cov_vel_twist, tc00, tc01, tc02, tc11, tc12, tc22);
  set_symmetric_cov3(odom.twist.covariance, 0, 6, 12, tc00, tc01, tc02, tc11, tc12, tc22);

  // Small-angle proxy: quaternion state variances -> roll/pitch/yaw diagonal.
  const double att_var = std::max(
    0.5 * (static_cast<double>(P[1][1]) + static_cast<double>(P[2][2]) +
           static_cast<double>(P[3][3])),
    1e-9);
  odom.pose.covariance[21] = att_var;
  odom.pose.covariance[28] = att_var;
  odom.pose.covariance[35] = att_var;
}

void EkfIns::publish_odometry(uint32_t stamp_ms)
{
  const AttPosEKF * core = ekf_.ekf_core();
  if (core == nullptr) {
    return;
  }
  nav_msgs::msg::Odometry odom;
  ned_state_to_odom(core->states, stamp_ms, odom);
  fill_output_covariance(core->P, core->states, odom);
  odom_pub_->publish(odom);
}

}  // namespace inertial_nav_ros2
