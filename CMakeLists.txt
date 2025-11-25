cmake_minimum_required(VERSION 3.8)
project(imu_yzf143_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(serial REQUIRED) # 必须有这一行

add_executable(imu_node src/imu_node.cpp)

target_include_directories(imu_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 链接库
ament_target_dependencies(imu_node
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  serial # 必须链接 serial
)

install(TARGETS imu_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()