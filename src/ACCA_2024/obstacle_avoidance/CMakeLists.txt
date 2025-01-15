cmake_minimum_required(VERSION 3.5)
project(obstacle_avoidance)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Set compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED)

# Find PROJ library
find_path(PROJ_INCLUDE_DIR proj.h)
find_library(PROJ_LIBRARY proj)

if (PROJ_INCLUDE_DIR AND PROJ_LIBRARY)
  set(PROJ_FOUND TRUE)
  message(STATUS "Found PROJ: ${PROJ_LIBRARY}")
else()
  set(PROJ_FOUND FALSE)
  message(FATAL_ERROR "Could not find PROJ library")
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Protobuf_INCLUDE_DIRS}
  ${PROJ_INCLUDE_DIR}
)

add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)

ament_target_dependencies(obstacle_avoidance
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(obstacle_avoidance
  ${PROJ_LIBRARY}
)

install(TARGETS
  obstacle_avoidance
  DESTINATION lib/${PROJECT_NAME})

ament_package()
