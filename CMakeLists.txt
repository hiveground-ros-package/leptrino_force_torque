cmake_minimum_required(VERSION 3.5)
project(leptrino_force_torque)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

#add_library(${PROJECT_NAME} ${SRC_FILES})
include_directories(include)

## Declare a cpp executable
add_executable(${PROJECT_NAME}
  src/leptrino_force_torque.cpp
  src/rs_comm.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp geometry_msgs)

ament_package()