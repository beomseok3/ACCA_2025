cmake_minimum_required(VERSION 3.5)
project(tf)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED proj)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(map_odom_tf_publisher_static src/map_odom_tf_publisher_static.cpp)
add_executable(odometry_map_frame src/odometry_map_frame.cpp)
add_executable(cone_pose_transform src/cone_pose_transform.cpp)
add_executable(localization_path_tf src/localization_path_tf.cpp)
add_executable(odom2map src/odom2map.cpp)



ament_target_dependencies(map_odom_tf_publisher_static
  rclcpp
  tf2_ros
  nav_msgs
  sensor_msgs
  Eigen3
)


ament_target_dependencies(odometry_map_frame
  rclcpp
  tf2 
  tf2_ros
  nav_msgs
  tf2_geometry_msgs
)

ament_target_dependencies(cone_pose_transform
  rclcpp 
  geometry_msgs 
  tf2 
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(localization_path_tf
  rclcpp
  tf2
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
)

ament_target_dependencies(odom2map
  rclcpp
  tf2
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
)

target_include_directories(map_odom_tf_publisher_static PRIVATE ${PROJ_INCLUDE_DIRS})
target_link_libraries(map_odom_tf_publisher_static ${PROJ_LIBRARIES})

install(TARGETS
  map_odom_tf_publisher_static
  odometry_map_frame
  cone_pose_transform
  localization_path_tf
  odom2map
  DESTINATION lib/${PROJECT_NAME})

ament_package()
