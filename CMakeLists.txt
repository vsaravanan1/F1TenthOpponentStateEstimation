cmake_minimum_required(VERSION 3.5)
project(lidar_processing)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================
# Find dependencies FIRST
# =====================
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(laser_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree segmentation filters)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Python COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)

# =====================
# Python package install
# =====================
ament_python_install_package(${PROJECT_NAME})
ament_python_install_package(racetrack_utilities)
ament_python_install_package(rrt_planner)
ament_python_install_package(f1tenth_mppi)

# =====================
# Includes
# =====================
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

include_directories(include)
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# =====================
# C++ executables
# =====================
add_executable(scan_to_cloud src/scan_to_cloud.cpp)
ament_target_dependencies(scan_to_cloud
  rclcpp
  sensor_msgs
  laser_geometry
  tf2
  tf2_ros
  tf2_sensor_msgs
)

add_executable(bounding_box src/bounding_box.cpp)
ament_target_dependencies(bounding_box
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

add_executable(mppi_node src/mppi_node.cpp src/utils.cpp src/dynamics_models.cpp)

ament_target_dependencies(mppi_node 
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2_ros message_filters
)

# add_executable(rrt_mppi src/ego_rrt_mppi.cpp)
# ament_target_dependencies(rrt_mppi
#   rclcpp
#   std_msgs
#   nav_msgs
# )

# target_link_libraries(rrt_mppi
#   pybind11::embed
#   Eigen3::Eigen
# )

# =====================
# Install targets
# =====================
install(TARGETS
  scan_to_cloud
  bounding_box
  # rrt_mppi
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/imm.py
  scripts/rmse.py
  scripts/clustering_rf.py
  scripts/spielberg_centerline_publisher.py
  scripts/frenet_opp_state.py
  # scripts/lstm_path_predictor.py
  scripts/imm_filterpy.py
  scripts/interceptor.py
  scripts/Overtake.py
  # scripts/lstm.py
  scripts/generate_candidate_trajectories.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  scripts/mppi_node_new.py
  DESTINATION lib/${PROJECT_NAME}
)

# =====================
# Testing
# =====================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
