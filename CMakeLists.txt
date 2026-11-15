cmake_minimum_required(VERSION 3.8)
project(imu_yzf143_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(serial REQUIRED)

add_executable(imu_node src/imu_node.cpp)
target_compile_features(imu_node PRIVATE cxx_std_17)
target_include_directories(imu_node PRIVATE include)
if(COMMAND ament_target_dependencies)
  ament_target_dependencies(imu_node rcl_interfaces rclcpp sensor_msgs tf2 serial)
else()
  target_link_libraries(imu_node PRIVATE
    rclcpp::rclcpp
    tf2::tf2
    ${rcl_interfaces_TARGETS}
    ${sensor_msgs_TARGETS}
    ${serial_TARGETS})
endif()

install(TARGETS imu_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame_parser test/test_frame_parser.cpp)
  target_compile_features(test_frame_parser PRIVATE cxx_std_17)
  target_include_directories(test_frame_parser PRIVATE include)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_imu_node test/test_imu_node.py TIMEOUT 120)
endif()

ament_package()
