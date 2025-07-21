cmake_minimum_required(VERSION 3.8)
project(parking_tf)

# 컴파일러 옵션 설정
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 실행 파일 추가
add_executable(cone_pose_transform src/cone_pose_transform.cpp)

# 의존성 연결
ament_target_dependencies(cone_pose_transform
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 설치 경로 지정
install(TARGETS
  cone_pose_transform
  DESTINATION lib/${PROJECT_NAME}
)

# ament 패키지 선언
ament_package()
