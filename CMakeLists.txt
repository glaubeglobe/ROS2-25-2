# ==========================================
# CMake 최소 버전 요구사항
# ==========================================
cmake_minimum_required(VERSION 3.0.2)

# ==========================================
# 프로젝트 이름 정의
# ==========================================
project(hybrid_controller)

# ==========================================
# C++ 표준 설정
# C++14: auto, lambda, std::make_unique 등 사용
# ==========================================
add_compile_options(-std=c++14)

# ==========================================
# ROS 패키지 찾기
# 
# [필수 패키지]
# - roscpp: C++ ROS 클라이언트 라이브러리
# - std_msgs: 표준 메시지 타입
# - nav_msgs: Odometry, Path 메시지
# - geometry_msgs: Pose, Twist 메시지
# - ackermann_msgs: Ackermann 드라이브 메시지
# - tf2, tf2_ros: 좌표 변환
# - visualization_msgs: RViz 마커
# ==========================================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  ackermann_msgs
  tf2
  tf2_ros
  visualization_msgs
)

# ==========================================
# Eigen3 라이브러리 찾기
# 
# [용도]
# - MPC 제어기에서 행렬 연산 사용
# - 벡터, 행렬 계산 최적화
# ==========================================
find_package(Eigen3 REQUIRED)

# ==========================================
# Catkin 패키지 설정
# 
# [설명]
# 이 패키지가 제공하는 것들을 정의
# - INCLUDE_DIRS: 헤더 파일 디렉토리
# - LIBRARIES: 제공하는 라이브러리
# - CATKIN_DEPENDS: 의존하는 다른 패키지
# - DEPENDS: 외부 라이브러리 의존성
# ==========================================
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mpc_steering_controller pid_speed_controller
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    nav_msgs 
    geometry_msgs 
    ackermann_msgs
    tf2
    tf2_ros
    visualization_msgs
  DEPENDS EIGEN3
)

# ==========================================
# Include 디렉토리 설정
# 
# [설명]
# 컴파일 시 헤더 파일을 찾을 경로 지정
# - include: 우리 패키지의 헤더
# - ${catkin_INCLUDE_DIRS}: ROS 패키지 헤더
# - ${EIGEN3_INCLUDE_DIRS}: Eigen3 헤더
# ==========================================
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ==========================================
# MPC Steering Controller 라이브러리 빌드
# 
# [과정]
# 1. mpc_steering_controller.cpp 컴파일
# 2. 라이브러리 파일 생성 (.so)
# 3. 다른 노드에서 링크하여 사용 가능
# ==========================================
add_library(mpc_steering_controller
  src/mpc_steering_controller.cpp
)

# 의존성 설정 (메시지 생성 완료 후 빌드)
add_dependencies(mpc_steering_controller 
  ${catkin_EXPORTED_TARGETS}
)

# 라이브러리 링크
target_link_libraries(mpc_steering_controller
  ${catkin_LIBRARIES}
)

# ==========================================
# PID Speed Controller 라이브러리 빌드
# ==========================================
add_library(pid_speed_controller
  src/pid_speed_controller.cpp
)

add_dependencies(pid_speed_controller 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(pid_speed_controller
  ${catkin_LIBRARIES}
)

# ==========================================
# Main Hybrid Controller Node 빌드
# 
# [과정]
# 1. hybrid_controller_node.cpp 컴파일
# 2. MPC, PID 라이브러리 링크
# 3. 실행 파일 생성
# ==========================================
add_executable(hybrid_controller_node
  src/hybrid_controller_node.cpp
)

add_dependencies(hybrid_controller_node 
  ${catkin_EXPORTED_TARGETS}
)

# 라이브러리 링크
# - 우리가 만든 MPC, PID 라이브러리
# - ROS 라이브러리
target_link_libraries(hybrid_controller_node
  mpc_steering_controller
  pid_speed_controller
  ${catkin_LIBRARIES}
)

# ==========================================
# 설치 규칙
# 
# [설명]
# catkin_make install 실행 시 파일 복사 위치
# ==========================================

# 실행 파일 및 라이브러리 설치
install(TARGETS 
    mpc_steering_controller 
    pid_speed_controller 
    hybrid_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 헤더 파일 설치
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# 런치 파일 설치
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# 설정 파일 설치
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# RViz 설정 파일 설치 (있는 경우)
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  OPTIONAL
)
