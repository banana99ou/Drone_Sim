cmake_minimum_required(VERSION 3.8)
project(dsim_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(dsim_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# --- core library: no ROS, so it can be unit tested in isolation ------------
add_library(dsim_control_core SHARED
  src/mixer.cpp
  src/se3_controller.cpp
  src/trajectory_buffer.cpp
)
target_include_directories(dsim_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(dsim_control_core PUBLIC Eigen3::Eigen)
# trajectory_buffer.cpp touches dsim_msgs, so the core needs that dependency.
ament_target_dependencies(dsim_control_core PUBLIC dsim_msgs geometry_msgs)

# --- nodes ------------------------------------------------------------------
add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node dsim_control_core)
ament_target_dependencies(controller_node
  rclcpp nav_msgs geometry_msgs std_srvs actuator_msgs dsim_msgs)

add_executable(reference_generator_node src/reference_generator_node.cpp)
ament_target_dependencies(reference_generator_node rclcpp dsim_msgs)

install(TARGETS dsim_control_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS controller_node reference_generator_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control test/test_control.cpp)
  target_link_libraries(test_control dsim_control_core)
  ament_target_dependencies(test_control dsim_msgs geometry_msgs)
endif()

ament_export_include_directories(include)
ament_export_libraries(dsim_control_core)
ament_export_dependencies(rclcpp dsim_msgs geometry_msgs Eigen3)
ament_package()
