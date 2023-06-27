cmake_minimum_required(VERSION 3.5)
project(romea_rtls_transceiver_gazebo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-inconsistent-missing-override")
endif()

find_package(ament_cmake REQUIRED)
find_package(romea_core_common REQUIRED)
find_package(romea_core_rtls_transceiver REQUIRED)
find_package(romea_rtls_transceiver_msgs REQUIRED)
find_package(romea_rtls_transceiver_utils REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

add_library(romea_gazebo_ros_rtls_transceiver SHARED
  src/gazebo_ros_rtls_network.cpp
  src/gazebo_ros_rtls_transceiver.cpp
  src/gazebo_ros_rtls_transceiver_interface.cpp
)

target_include_directories(romea_gazebo_ros_rtls_transceiver PUBLIC include)
ament_target_dependencies(romea_gazebo_ros_rtls_transceiver
  "gazebo_ros"
  "gazebo_msgs"
  "gazebo_dev"
  "romea_core_common"
  "romea_core_rtls"
  "romea_rtls_transceiver_msgs"
  "romea_rtls_transceiver_utils"

)
ament_export_libraries(romea_gazebo_ros_rtls_transceiver)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(romea_core_common)
ament_export_dependencies(romea_core_rtls_transceiver)
ament_export_dependencies(romea_rtls_transceiver_msgs)
ament_export_dependencies(romea_rtls_transceiver_utils)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(gazebo_ros)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

# ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
  romea_gazebo_ros_rtls_transceiver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
