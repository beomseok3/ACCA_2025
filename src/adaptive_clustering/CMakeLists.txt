cmake_minimum_required(VERSION 3.5)
project(adaptive_clustering)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(adaptive_clustering_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Executable
add_executable(${PROJECT_NAME} src/adaptive_clustering.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
  pcl_conversions
  pcl_msgs
  PCL
  adaptive_clustering_msgs
)
target_link_libraries(adaptive_clustering ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_SEARCH_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES})


install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
