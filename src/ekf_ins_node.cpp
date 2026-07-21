#include <algorithm>
#include <cstdint>

#include "inertial_nav_ros2/ekf_ins.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<inertial_nav_ros2::EkfIns>();

  const size_t threads = static_cast<size_t>(std::max<int64_t>(
    2, node->get_parameter("executor_threads").as_int()));
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), threads);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
