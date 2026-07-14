cmake_minimum_required(VERSION 3.8)
project(livox_laser_simulation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-rendering6 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES std_msgs
)

add_library(livox_laser_simulation_plugin SHARED
  src/livox_laser_simulation/livox_lidar_system.cpp
  src/livox_laser_simulation/livox_raycaster.cpp
)
set_target_properties(livox_laser_simulation_plugin PROPERTIES
  OUTPUT_NAME livox_laser_simulation)
target_include_directories(livox_laser_simulation_plugin PRIVATE
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
ament_target_dependencies(livox_laser_simulation_plugin
  ament_index_cpp
  geometry_msgs
  ignition-gazebo6
  ignition-rendering6
  rclcpp
  sensor_msgs
  tf2_ros
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(livox_laser_simulation_plugin "${cpp_typesupport_target}")

install(TARGETS livox_laser_simulation_plugin LIBRARY DESTINATION lib)
install(DIRECTORY assets launch models rviz scan_patterns worlds
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
