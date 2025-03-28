cmake_minimum_required(VERSION 3.8)
project(mixed_cpp_py_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add C++ executable
add_executable(twist_publisher src/twist_publisher.cpp)

ament_target_dependencies(twist_publisher
  rclcpp
  geometry_msgs
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  mixed_cpp_py_pkg/twist_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install C++ executables
install(TARGETS
  twist_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()