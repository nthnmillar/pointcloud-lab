cmake_minimum_required(VERSION 3.8)
project(first_robot)

# Find required packages
find_package(ament_cmake REQUIRED)

# Add subdirectories for nested packages
add_subdirectory(robot_bringup)
add_subdirectory(robot_description)

# Install the meta-package
ament_package()
