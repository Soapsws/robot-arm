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
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)

add_executable(flail_node
  src/flail_node.cpp
  src/flail_motion.cpp
)

target_include_directories(flail_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(flail_node PRIVATE
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  urdf::urdf
)

install(TARGETS flail_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(flail_motion_test test/flail_motion_test.cpp src/flail_motion.cpp)
  target_include_directories(flail_motion_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME flail_motion_test COMMAND flail_motion_test)
endif()

ament_package()
