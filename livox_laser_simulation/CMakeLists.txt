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
find_package(gz-sim8 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
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
  src/livox_laser_simulation/livox_depth_sampler.cpp
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
  rclcpp
  sensor_msgs
  tf2_ros
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(livox_laser_simulation_plugin
  "${cpp_typesupport_target}"
  gz-sim8::gz-sim8
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_depth_sampler test/test_depth_sampler.cpp)
  target_include_directories(test_depth_sampler PRIVATE include)
  target_link_libraries(test_depth_sampler livox_laser_simulation_plugin)
  target_compile_definitions(test_depth_sampler PRIVATE
    TEST_SCAN_PATTERN="${CMAKE_CURRENT_SOURCE_DIR}/test/data/cardinal.csv")
endif()

install(TARGETS livox_laser_simulation_plugin LIBRARY DESTINATION lib)
install(DIRECTORY assets launch models rviz scan_patterns worlds
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
