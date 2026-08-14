#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "estimator_ekf.h"
#include "inertial_nav_ros2/frame_conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"

namespace tf2_ros {
class Buffer;
class TransformListener;
class TransformBroadcaster;
}

namespace inertial_nav_ros2 {

struct MeasStamp {
  uint32_t stamp_ms{0};
  bool valid{false};
};

struct PoseMeas {
  std::array<float, 3> pos{};
  std::array<float, 3> vel{};
  std::array<float, 4> quat{};  // w, x, y, z
  float pos_err{0.5f};
  float yaw_err{0.1f};
  float vel_err{0.5f};
  MeasStamp stamp;
};

struct OdomMeas {
  std::array<float, 3> body_vel{};
  float vel_err{0.5f};
  MeasStamp stamp;
};

struct MagMeas {
  std::array<float, 3> field_ut{};  // microtesla (setMagData input)
  MeasStamp stamp;
};

struct BaroMeas {
  float height_m{0.0f};
  float sigma{1.0f};
  MeasStamp stamp;
};

/** Latest sensor samples taken under cache_mutex_ at the start of each IMU step. */
struct SensorSnapshot {
  std::array<PoseMeas, MAX_EXT_NAV> pose{};
  std::array<bool, MAX_EXT_NAV> pose_valid{};
  std::array<OdomMeas, MAX_ODOM> odom{};
  std::array<bool, MAX_ODOM> odom_valid{};
  std::array<MagMeas, MAX_MAG> mag{};
  std::array<bool, MAX_MAG> mag_valid{};
  BaroMeas baro{};
  bool baro_valid{false};
};

class EkfIns : public rclcpp::Node {
public:
  explicit EkfIns(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~EkfIns() override;

private:
  static constexpr std::size_t kNumPose = MAX_EXT_NAV;
  static constexpr std::size_t kNumOdom = MAX_ODOM;
  static constexpr std::size_t kNumMag = MAX_MAG;

  void declare_parameters();
  void load_static_config();
  void refresh_enable_flags();
  void apply_transport_delays(uint32_t fusion_ms, const SensorSnapshot & snapshot);

  void on_imu(const sensor_msgs::msg::Imu & msg);
  void on_pose(std::size_t id, const geometry_msgs::msg::PoseWithCovarianceStamped & msg);
  void on_pose_odom(std::size_t id, const nav_msgs::msg::Odometry & msg);
  void on_odom(std::size_t id, const geometry_msgs::msg::TwistWithCovarianceStamped & msg);
  void on_odom_odom(std::size_t id, const nav_msgs::msg::Odometry & msg);
  void on_mag(std::size_t id, const sensor_msgs::msg::MagneticField & msg);
  void on_baro(const std_msgs::msg::Float64 & msg);
  void on_baro_pressure(const sensor_msgs::msg::FluidPressure & msg);
  void on_reset(const std_msgs::msg::Empty & msg);

  SensorSnapshot take_sensor_snapshot();
  void ekf_ins_run(const sensor_msgs::msg::Imu & imu, const SensorSnapshot & snapshot);
  void publish_odometry(uint32_t stamp_ms);

  static uint32_t stamp_to_ms(const builtin_interfaces::msg::Time & stamp);
  static uint32_t effective_delay_ms(
    uint32_t baseline_ms, uint32_t fusion_ms, uint32_t msg_ms, uint32_t max_transport_ms);

  void pose_msg_to_meas(
    const geometry_msgs::msg::PoseWithCovarianceStamped & msg, PoseMeas & out) const;
  void pose_odom_to_meas(const nav_msgs::msg::Odometry & msg, PoseMeas & out) const;
  bool transform_pose_odom_to_earth(std::size_t id, nav_msgs::msg::Odometry & msg) const;
  void ned_state_to_odom(
    const float * states, uint32_t stamp_ms, nav_msgs::msg::Odometry & odom) const;

  bool accept_pose_update(std::size_t id, uint32_t stamp_ms);
  bool accept_odom_update(std::size_t id, uint32_t stamp_ms);
  bool odom_body_vel_sane(std::size_t id, const OdomMeas & meas) const;
  bool accept_mag_update(std::size_t id, uint32_t stamp_ms);
  bool accept_baro_update(uint32_t stamp_ms);
  void fill_output_covariance(
    const float P[22][22], const float q_ned_frd_wxyz[4],
    nav_msgs::msg::Odometry & odom) const;

  estimator_ekf ekf_;
  std::mutex cache_mutex_;
  std::mutex ekf_mutex_;

  std::atomic<bool> filter_reset_{true};
  bool have_prev_imu_stamp_{false};
  uint32_t prev_imu_stamp_ms_{0};

  bool pose_z_is_down_{true};
  frames::BodyAxes body_axes_{frames::BodyAxes::RosFlu};
  uint32_t max_transport_delay_ms_{500};
  std::string imu_topic_;
  std::string output_frame_{"odom"};
  std::string child_frame_{"base_link"};
  bool publish_tf_{true};
  InitReferenceSource init_reference_{InitReferenceSource::EXT_NAV};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double ext_nav_origin_lat_{0.0};
  double ext_nav_origin_lon_{0.0};
  float ext_nav_origin_hgt_{0.0f};

  struct PoseSourceParams {
    bool enable{true};
    uint32_t delay_pos_ms{100};
    uint32_t delay_yaw_ms{100};
    uint32_t delay_vel_ms{100};
    bool fuse_height{false};
    bool fuse_yaw{true};
    bool fuse_vel{false};
    bool fuse_pos{true};
    bool extnav_not_earth_aligned{false};
    float extnav_tf_timeout_s{6.0f};
    std::string earth_frame{"earth"};
    std::string vslam_local_frame{"local"};
    float max_rate_hz{0.0f};  // 0 = no limit; pose/odom callbacks dropped above this rate
    std::string topic;
    std::string odom_topic;
  };
  struct OdomSourceParams {
    bool enable{true};
    uint32_t delay_ms{25};
    float max_rate_hz{0.0f};
    float max_body_vel_mps{0.0f};  // 0 = no limit; drop absurd cuVSLAM startup spikes
    std::string topic;
    std::string odom_topic;
  };
  struct MagSourceParams {
    bool enable{false};
    uint32_t delay_ms{25};
    float max_rate_hz{0.0f};
    std::string topic;
  };
  struct BaroParams {
    bool enable{true};
    uint32_t delay_ms{50};
    float default_sigma{1.0f};
    float sea_level_pressure_pa{101325.0f};
    float max_rate_hz{0.0f};
    std::string topic;
    std::string pressure_topic;
  };

  static bool pose_source_fuses(const PoseSourceParams & params);

  std::array<PoseSourceParams, kNumPose> pose_params_{};
  std::array<OdomSourceParams, kNumOdom> odom_params_{};
  std::array<MagSourceParams, kNumMag> mag_params_{};
  BaroParams baro_params_{};

  std::array<PoseMeas, kNumPose> pose_cache_{};
  std::array<uint32_t, kNumPose> last_pose_accept_ms_{};
  std::array<OdomMeas, kNumOdom> odom_cache_{};
  std::array<uint32_t, kNumOdom> last_odom_accept_ms_{};
  std::array<MagMeas, kNumMag> mag_cache_{};
  std::array<uint32_t, kNumMag> last_mag_accept_ms_{};
  BaroMeas baro_cache_{};
  uint32_t last_baro_accept_ms_{0};

  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::array<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr, kNumPose>
    pose_subs_{};
  std::array<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr, kNumPose> pose_odom_subs_{};
  std::array<rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr, kNumOdom>
    odom_subs_{};
  std::array<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr, kNumOdom> odom_odom_subs_{};
  std::array<rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr, kNumMag> mag_subs_{};
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr baro_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr baro_pressure_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  std::string reset_topic_{"~/reset"};
};

}  // namespace inertial_nav_ros2
