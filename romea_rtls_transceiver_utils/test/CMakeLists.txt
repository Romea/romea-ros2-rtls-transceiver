find_package(ament_cmake_gtest REQUIRED)

get_filename_component(TEST_DIR_SRC ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
configure_file(test_helper.h.in test_helper.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# ament_add_gtest(${PROJECT_NAME}_test_data_conversions test_data_conversions.cpp)
# target_link_libraries(${PROJECT_NAME}_test_data_conversions ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_transceiver_parameters test_transceiver_parameters.cpp)
target_link_libraries(${PROJECT_NAME}_test_transceiver_parameters ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_transceiver_interface test_transceiver_interface.cpp)
target_link_libraries(${PROJECT_NAME}_test_transceiver_interface ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_transceiver_server_client_interfaces test_transceiver_server_client_interfaces.cpp)
target_link_libraries(${PROJECT_NAME}_test_transceiver_server_client_interfaces ${PROJECT_NAME})
