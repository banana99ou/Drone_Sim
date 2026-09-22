cmake_minimum_required(VERSION 3.8)
project(dsim_eval)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(dsim_msgs REQUIRED)
find_package(dsim_time REQUIRED)

add_executable(eval_node src/eval_node.cpp)
target_include_directories(eval_node PRIVATE include)
ament_target_dependencies(eval_node
  rclcpp nav_msgs actuator_msgs ros_gz_interfaces dsim_msgs dsim_time)
add_executable(viz_relay_node src/viz_relay_node.cpp)
ament_target_dependencies(viz_relay_node
  rclcpp nav_msgs sensor_msgs dsim_msgs)

install(TARGETS eval_node viz_relay_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# The referee's geometry has no ROS in it (include/dsim_eval/obstacle.hpp), so
# it is tested here AND mutation-checked on the host by scripts/mutation_check.sh.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_obstacle test/test_obstacle.cpp)
  target_include_directories(test_obstacle PRIVATE include)
endif()

ament_package()
