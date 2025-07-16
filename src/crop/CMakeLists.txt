cmake_minimum_required(VERSION 3.5)
project(crop)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ament_cmake 및 필요한 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

# cropbox.cpp 소스 파일 추가 및 타겟 정의
add_executable(cropbox src/cropbox_bs.cpp)
add_executable(n_crop src/n.crop.cpp)
add_executable(concat src/image_concat_bs.cpp)
add_executable(z_crop src/z_crop.cpp)
add_executable(draw_line src/draw_line.cpp)
add_executable(line_detection src/line_detection.cpp)
add_executable(draw_path src/draw_path.cpp)

# 타겟 링크 (필요한 라이브러리 추가)
ament_target_dependencies(cropbox rclcpp sensor_msgs)
ament_target_dependencies(n_crop rclcpp sensor_msgs)
ament_target_dependencies(concat rclcpp sensor_msgs)
ament_target_dependencies(concat rclcpp cv_bridge OpenCV)
ament_target_dependencies(z_crop rclcpp sensor_msgs)
ament_target_dependencies(draw_path
  rclcpp
  sensor_msgs
  visualization_msgs  
)

ament_target_dependencies(line_detection rclcpp sensor_msgs)
ament_target_dependencies(draw_line
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
)

# 설치 설정
install(TARGETS
  cropbox
  n_crop
  concat
  z_crop
  draw_line
  draw_path
  line_detection
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ament 패키지 설정
ament_package()

