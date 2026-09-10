cmake_minimum_required(VERSION 3.8)
project(dsim_time)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)

# Header-only on purpose: this is a rule, not a service. A node that has to
# link something to obey it will eventually not obey it.
add_library(dsim_time INTERFACE)
target_include_directories(dsim_time INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS dsim_time EXPORT export_dsim_time)
ament_export_targets(export_dsim_time HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sim_epoch test/test_sim_epoch.cpp)
  target_link_libraries(test_sim_epoch dsim_time)
endif()

ament_package()
