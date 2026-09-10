cmake_minimum_required(VERSION 3.8)
project(dsim_simctl)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dsim_msgs REQUIRED)
find_package(dsim_time REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)

add_executable(sim_control_node src/sim_control_node.cpp)
target_include_directories(sim_control_node PRIVATE include)
ament_target_dependencies(sim_control_node rclcpp dsim_msgs dsim_time)
target_link_libraries(sim_control_node gz-transport::core gz-msgs::core)

install(TARGETS sim_control_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # The pacing arithmetic is deliberately free of both ROS and Gazebo, so the
  # thing that decides how fast the world runs can be tested without either.
  ament_add_gtest(test_pacer test/test_pacer.cpp)
  target_include_directories(test_pacer PRIVATE include)
endif()

ament_package()
