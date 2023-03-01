find_package(ament_cmake_gtest REQUIRED)

# ament_add_gtest(${PROJECT_NAME}_test_data_conversions test_data_conversions.cpp)
# target_link_libraries(${PROJECT_NAME}_test_data_conversions ${PROJECT_NAME})

# ament_add_gtest(${PROJECT_NAME}_test_transceiver_parameters test_transceiver_parameters.cpp)
# target_link_libraries(${PROJECT_NAME}_test_transceiver_parameters ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_transceiver_interfaces test_transceiver_interfaces.cpp)
target_link_libraries(${PROJECT_NAME}_test_transceiver_interfaces ${PROJECT_NAME})
