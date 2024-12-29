cmake_minimum_required(VERSION 3.8)
project(cpp_pub_sub_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add both executables
add_executable(cpp_twist_publisher src/twist_publisher.cpp)
add_executable(cpp_twist_subscriber src/twist_subscriber.cpp)

# Link ROS2 dependencies
ament_target_dependencies(cpp_twist_publisher
  rclcpp
  geometry_msgs
)

ament_target_dependencies(cpp_twist_subscriber
  rclcpp
  geometry_msgs
)

# Install both executables
install(TARGETS
  cpp_twist_publisher
  cpp_twist_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()