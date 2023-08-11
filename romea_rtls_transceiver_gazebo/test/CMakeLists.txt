find_package(ament_cmake_gtest REQUIRED)

get_filename_component(TEST_DIR_SRC ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
configure_file(test_helper.h.in test_helper.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

ament_add_gtest(
${PROJECT_NAME}_test_gazebo_ros_rtls_transceiver
test_gazebo_ros_rtls_transceiver.cpp
WORKING_DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}
# Long timeout because has to run gazebo several times
TIMEOUT
  100
)

target_link_libraries(${PROJECT_NAME}_test_gazebo_ros_rtls_transceiver gazebo_test_fixture)

ament_target_dependencies(${PROJECT_NAME}_test_gazebo_ros_rtls_transceiver
  "gazebo_dev"
  "gazebo_msgs"
  "gazebo_ros"
  "romea_core_common"
  "romea_core_rtls_transceiver"
  "romea_rtls_transceiver_utils"
  "romea_rtls_transceiver_msgs"
)
