cmake_minimum_required(VERSION 3.8)
project(dsim_estimation)

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
find_package(geometry_msgs REQUIRED)
find_package(dsim_msgs REQUIRED)
find_package(dsim_time REQUIRED)
find_package(Eigen3 REQUIRED)

# --- core library: no ROS, so it can be unit tested in isolation -----------
# scripts/mutation_check.sh compiles exactly these two sources with g++ on the
# host; a source added here must be added there too, or the harness aborts.
add_library(dsim_estimation_core SHARED
  src/attitude_filter.cpp
  src/velocity_filter.cpp
)
target_include_directories(dsim_estimation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(dsim_estimation_core PUBLIC Eigen3::Eigen)

# --- node ------------------------------------------------------------------
add_executable(estimator_node src/estimator_node.cpp)
target_link_libraries(estimator_node dsim_estimation_core)
ament_target_dependencies(estimator_node
  rclcpp nav_msgs sensor_msgs geometry_msgs dsim_msgs dsim_time)

install(TARGETS dsim_estimation_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS estimator_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_estimation test/test_estimation.cpp)
  target_link_libraries(test_estimation dsim_estimation_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(dsim_estimation_core)
ament_export_dependencies(Eigen3)
ament_package()
