
cmake_minimum_required(VERSION 3.5)
project(romea_rtls_transceiver_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    msg/Payload.msg
    msg/Range.msg
    msg/RangingRequest.msg
    msg/RangingResult.msg
    DEPENDENCIES std_msgs geometry_msgs)


ament_export_dependencies(rosidl_default_runtime)
ament_package()
