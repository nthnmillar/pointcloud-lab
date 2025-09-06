cmake_minimum_required(VERSION 3.8)

# Set CMake policies to suppress warnings BEFORE project() call
cmake_policy(SET CMP0074 NEW)
cmake_policy(SET CMP0144 NEW)

project(lidar_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io visualization)
find_package(rosbag2_cpp REQUIRED)

# Try to find LASzip for LAZ support
find_library(LASZIP_LIBRARY NAMES laszip_api)
find_path(LASZIP_INCLUDE_DIR NAMES laszip/laszip_api.h)

# Include directories
include_directories(include)

# Create executables
add_executable(lidar_recorder_node src/lidar_recorder_node.cpp)
ament_target_dependencies(lidar_recorder_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  pcl_ros
  pcl_conversions
  rosbag2_cpp
)
target_link_libraries(lidar_recorder_node ${PCL_LIBRARIES})
target_include_directories(lidar_recorder_node PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(lidar_recorder_node PRIVATE ${PCL_COMPILE_DEFINITIONS})

# Add LASzip support if found
if(LASZIP_LIBRARY AND LASZIP_INCLUDE_DIR)
  target_link_libraries(lidar_recorder_node ${LASZIP_LIBRARY})
  target_include_directories(lidar_recorder_node PRIVATE ${LASZIP_INCLUDE_DIR})
  target_compile_definitions(lidar_recorder_node PRIVATE HAVE_LASZIP)
  message(STATUS "Found LASzip: ${LASZIP_LIBRARY}")
else()
  message(WARNING "LASzip not found - LAZ conversion will be disabled")
endif()

add_executable(bag_recorder_node src/bag_recorder_node.cpp)
ament_target_dependencies(bag_recorder_node
  rclcpp
  sensor_msgs
  std_msgs
  rosbag2_cpp
)

add_executable(bag_filter_node src/bag_filter_node.cpp)
ament_target_dependencies(bag_filter_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  rosbag2_cpp
)
target_link_libraries(bag_filter_node ${PCL_LIBRARIES})
target_include_directories(bag_filter_node PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(bag_filter_node PRIVATE ${PCL_COMPILE_DEFINITIONS})

add_executable(pcd_to_laz_converter src/pcd_to_laz_converter.cpp)
ament_target_dependencies(pcd_to_laz_converter
  rclcpp
  std_msgs
)
target_link_libraries(pcd_to_laz_converter ${PCL_LIBRARIES})
target_include_directories(pcd_to_laz_converter PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(pcd_to_laz_converter PRIVATE ${PCL_COMPILE_DEFINITIONS})

# Add LASzip support to pcd_to_laz_converter if found
if(LASZIP_LIBRARY AND LASZIP_INCLUDE_DIR)
  target_link_libraries(pcd_to_laz_converter ${LASZIP_LIBRARY})
  target_include_directories(pcd_to_laz_converter PRIVATE ${LASZIP_INCLUDE_DIR})
  target_compile_definitions(pcd_to_laz_converter PRIVATE HAVE_LASZIP)
endif()

# Install executables
install(TARGETS
  lidar_recorder_node
  bag_recorder_node
  bag_filter_node
  pcd_to_laz_converter
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts if they exist
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  install(DIRECTORY scripts/
    DESTINATION lib/${PROJECT_NAME}
    USE_SOURCE_PERMISSIONS
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
