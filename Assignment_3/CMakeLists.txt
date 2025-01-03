cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(avoid_wall)

# Find the Ignition Transport library
find_package(ignition-transport11 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

include_directories(${CMAKE_BINARY_DIR})

# Add lidar_node executable
if (EXISTS "${CMAKE_SOURCE_DIR}/lidar_node.cc")
  add_executable(lidar_node lidar_node.cc)
  target_link_libraries(lidar_node ignition-transport${IGN_TRANSPORT_VER}::core)
endif()

# Add twist_publisher executable
if (EXISTS "${CMAKE_SOURCE_DIR}/twist_publisher.cpp")
  add_executable(twist_publisher twist_publisher.cpp)
  target_link_libraries(twist_publisher ignition-transport${IGN_TRANSPORT_VER}::core)
endif()
