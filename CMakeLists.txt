cmake_minimum_required(VERSION 3.8)
project(f1tenth_raceline_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ===== find packages =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(f110_msgs REQUIRED)

include_directories(
  include
)

# ===== 글로벌 플래너 라이브러리 (레이싱 라인) =====
add_library(raceline_global_planner SHARED
  src/raceline_global_planner.cpp
)

ament_target_dependencies(raceline_global_planner
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav_msgs
  geometry_msgs
  tf2_ros
)

# pluginlib에 글로벌 플래너 등록
pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

# ===== tracking_cpp 노드 (ObstacleArray -> PointCloud2) =====
add_executable(tracking_cpp
  src/tracking_cpp.cpp
)

ament_target_dependencies(tracking_cpp
  rclcpp
  sensor_msgs
  f110_msgs
)

# ===== 설치 설정 =====

# C++ 타겟
install(
  TARGETS
    raceline_global_planner
    tracking_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 헤더
install(
  DIRECTORY include/
  DESTINATION include
)

# Python 노드 설치 (/cmd_vel -> /drive 브리지)
install(
  PROGRAMS src/twist_to_drive.py
  DESTINATION lib/${PROJECT_NAME}
)

# launch / config 설치
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()