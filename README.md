# ROS2 Tutorial - Communication Introduction

[![ROS2 Version](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Developer](https://img.shields.io/badge/Developer-shashank3199-green)](https://github.com/shashank3199)

This repository contains example packages demonstrating various communication patterns in ROS2, including both Python and C++ implementations. The examples showcase different combinations of publishers and subscribers, highlighting ROS2's language-agnostic nature and interoperability features. Read the [Mastering ROS2: Your Ultimate Beginner’s Guide to Robotics](https://shashank-goyal-blogs.medium.com/mastering-ros2-your-ultimate-beginners-guide-to-robotics-a72179db4c84) for a detailed explanation of the code.

## Table of Contents

- [Overview](#overview)
- [Repository Structure](#repository-structure)
- [Package Descriptions](#package-descriptions)
  - [Pure C++ Package (cpp_pub_sub_pkg)](#pure-c-package-cpp_pub_sub_pkg)
  - [Mixed C++/Python Package 1 (mixed_cpp_py_pkg)](#mixed-cpython-package-1-mixed_cpp_py_pkg)
  - [Mixed C++/Python Package 2 (mixed_pub_sub)](#mixed-cpython-package-2-mixed_pub_sub)
  - [Pure Python Package (py_pub_sub_pkg)](#pure-python-package-py_pub_sub_pkg)
- [Building the Packages](#building-the-packages)
- [Running the Examples](#running-the-examples)
- [Package Dependencies](#package-dependencies)

## Overview

This tutorial demonstrates four different approaches to implementing publisher-subscriber patterns in ROS2:

1. Pure C++ implementation
2. C++ Publisher with Python Subscriber
3. Python Publisher with C++ Subscriber
4. Pure Python implementation

Each package uses the `geometry_msgs/msg/TwistStamped` message type to publish and subscribe to velocity commands on the `cmd_vel` topic.

## Repository Structure

```plaintext
📦 ROS2-Tutorials
 ┣ 📂 cpp_pub_sub_pkg      # Pure C++ implementation
 ┣ 📂 mixed_cpp_py_pkg     # C++ Publisher, Python Subscriber
 ┣ 📂 mixed_pub_sub        # Python Publisher, C++ Subscriber
 ┣ 📂 py_pub_sub_pkg       # Pure Python implementation
 ┗ 📜 README.md
```

## Package Descriptions

### Pure C++ Package (cpp_pub_sub_pkg)

A complete C++ implementation of the publisher-subscriber pattern.

**Key Files:**

- `src/twist_publisher.cpp`: Publishes TwistStamped messages at 10Hz
- `src/twist_subscriber.cpp`: Subscribes to and logs TwistStamped messages
- `CMakeLists.txt`: Build configuration for C++ nodes
- `package.xml`: Package dependencies and metadata

### Mixed C++/Python Package 1 (mixed_cpp_py_pkg)

Demonstrates C++ Publisher with Python Subscriber implementation.

**Key Files:**

- `src/twist_publisher.cpp`: C++ publisher node
- `mixed_cpp_py_pkg/twist_subscriber.py`: Python subscriber node
- `CMakeLists.txt`: Mixed language build configuration
- `setup.py`: Python package configuration
- `package.xml`: Package dependencies for both languages

### Mixed C++/Python Package 2 (mixed_pub_sub)

Demonstrates Python Publisher with C++ Subscriber implementation.

**Key Files:**

- `mixed_pub_sub/twist_publisher.py`: Python publisher node
- `src/twist_subscriber.cpp`: C++ subscriber node
- `CMakeLists.txt`: Mixed language build configuration
- `setup.py`: Python package configuration
- `package.xml`: Package dependencies for both languages

### Pure Python Package (py_pub_sub_pkg)

A complete Python implementation of the publisher-subscriber pattern.

**Key Files:**

- `py_pub_sub_pkg/twist_publisher.py`: Publishes TwistStamped messages at 10Hz
- `py_pub_sub_pkg/twist_subscriber.py`: Subscribes to and logs TwistStamped messages
- `setup.py`: Python package configuration
- `package.xml`: Package dependencies
- `setup.cfg`: Python package installation configuration

## Building the Packages

1. Create a new ROS2 workspace:

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2. Clone this repository:

    ```bash
    git clone <repository-url>
    ```

3. Install dependencies:

    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

4. Build the packages:

    ```bash
    colcon build
    ```

5. Source the workspace:

    ```bash
    source install/setup.bash
    ```

## Running the Examples

Each package can be run using the following commands in separate terminals:

### Pure C++ Package

```bash
# Terminal 1
ros2 run cpp_pub_sub_pkg cpp_twist_publisher
# Terminal 2
ros2 run cpp_pub_sub_pkg cpp_twist_subscriber
```

### Mixed C++/Python Package 1

```bash
# Terminal 1
ros2 run mixed_cpp_py_pkg twist_publisher
# Terminal 2
ros2 run mixed_cpp_py_pkg twist_subscriber
```

### Mixed C++/Python Package 2

```bash
# Terminal 1
ros2 run mixed_pub_sub twist_publisher
# Terminal 2
ros2 run mixed_pub_sub twist_subscriber
```

### Pure Python Package

```bash
# Terminal 1
ros2 run py_pub_sub_pkg py_twist_publisher
# Terminal 2
ros2 run py_pub_sub_pkg py_twist_subscriber
```

## Package Dependencies

Common dependencies for all packages:

- ROS2 Humble
- geometry_msgs
- rclcpp (C++ packages)
- rclpy (Python packages)
- ament_cmake (C++ packages)
- ament_cmake_python (mixed packages)
- setuptools (Python packages)

Each package's specific dependencies can be found in their respective `package.xml` files.
