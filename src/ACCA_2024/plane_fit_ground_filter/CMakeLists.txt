cmake_minimum_required(VERSION 3.8)
project(plane_ground_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)


include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${ament_cmake_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
)


# Link directories
link_directories(
    ${PCL_LIBRARY_DIRS}
)


add_executable(plane_ground_filter_node src/plane_ground_filter_node.cpp src/plane_ground_filter_core.cpp)
ament_target_dependencies(plane_ground_filter_node rclcpp sensor_msgs pcl_conversions)



# Link libraries
target_link_libraries(plane_ground_filter_node
    ${PCL_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
)

# Install executables
install(TARGETS plane_ground_filter_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
