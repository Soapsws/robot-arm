cmake_minimum_required(VERSION 3.20)
project(arm_control)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(joint_state_demo
  src/joint_state_demo.cpp
)

target_include_directories(joint_state_demo PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(joint_state_demo PRIVATE
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  geometry_msgs::geometry_msgs
  trajectory_msgs::trajectory_msgs
  tf2_ros::tf2_ros
)

install(TARGETS joint_state_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
