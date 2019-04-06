cmake_minimum_required(VERSION 3.10)
project(rxros CXX)

set(CMAKE_BUILD_TYPE Release)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-psabi) # -Wconversion -Wpedantic
endif()

enable_testing()
include(CTest)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(catch2 REQUIRED) 

add_executable(rxros src/main.cpp)
ament_target_dependencies(rxros rclcpp  std_msgs )

ament_package()