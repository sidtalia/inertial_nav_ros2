cmake_minimum_required(VERSION 3.8)
project(inertial_nav_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Sibling checkout of Inertial_nav_shim (override with -DINERTIAL_NAV_SHIM=...)
if(NOT DEFINED INERTIAL_NAV_SHIM)
  set(INERTIAL_NAV_SHIM "${CMAKE_CURRENT_SOURCE_DIR}/../Inertial_nav_shim")
endif()

if(NOT EXISTS "${INERTIAL_NAV_SHIM}/nav_filter/estimator_ekf.h")
  message(FATAL_ERROR
    "nav_filter not found at ${INERTIAL_NAV_SHIM}/nav_filter. "
    "Clone Inertial_nav_shim with submodules or pass -DINERTIAL_NAV_SHIM=/path/to/Inertial_nav_shim")
endif()

add_library(nav_filter_core STATIC
  ${INERTIAL_NAV_SHIM}/nav_filter/estimator_22states.cpp
  ${INERTIAL_NAV_SHIM}/nav_filter/estimator_utilities.cpp
  ${INERTIAL_NAV_SHIM}/nav_filter/ekf_time.cpp
  ${INERTIAL_NAV_SHIM}/nav_filter/yaw_fusion.cpp
  ${INERTIAL_NAV_SHIM}/nav_filter/wmt_shim.cpp
)
target_include_directories(nav_filter_core PUBLIC
  ${INERTIAL_NAV_SHIM}/nav_filter
)
target_compile_definitions(nav_filter_core PRIVATE NOMINMAX)
target_compile_options(nav_filter_core PRIVATE
  -Wno-unused-parameter
  -Wno-reorder
  -Wno-class-memaccess
)
target_compile_features(nav_filter_core PUBLIC cxx_std_17)

include_directories(include)

add_executable(ekf_ins_node
  src/ekf_ins_node.cpp
  src/ekf_ins.cpp
  src/frame_conversions.cpp
)
target_link_libraries(ekf_ins_node nav_filter_core)
ament_target_dependencies(ekf_ins_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_compile_features(ekf_ins_node PUBLIC cxx_std_17)

install(TARGETS ekf_ins_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/mujoco_sim_pub.py
  scripts/ros_sim_validator.py
  scripts/ros_trace_recorder.py
  scripts/extnav_earth_align_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
