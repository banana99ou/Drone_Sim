cmake_minimum_required(VERSION 3.8)
project(dsim_sensors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dsim_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Header-only sensor models, so the maths is testable without a node.
add_library(dsim_sensors_core INTERFACE)
target_include_directories(dsim_sensors_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(dsim_sensors_core INTERFACE Eigen3::Eigen)

add_executable(sensors_node src/sensors_node.cpp)
target_link_libraries(sensors_node dsim_sensors_core)
ament_target_dependencies(sensors_node rclcpp nav_msgs sensor_msgs dsim_msgs)

install(TARGETS sensors_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sensors test/test_sensors.cpp)
  target_link_libraries(test_sensors dsim_sensors_core)
endif()

ament_export_include_directories(include)
ament_package()
