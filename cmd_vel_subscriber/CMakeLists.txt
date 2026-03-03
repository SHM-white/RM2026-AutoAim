cmake_minimum_required(VERSION 3.8)
project(cmd_vel_subscriber)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(cmd_vel_subscriber src/cmd_vel_subscriber.cpp)

ament_target_dependencies(cmd_vel_subscriber
  rclcpp
  geometry_msgs
)

install(TARGETS
  cmd_vel_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
