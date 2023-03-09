// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license


#ifndef ROMEA_RTLS_TRANSCEIVER_UTILS__TRANSCEIVER_PARAMETERS_HPP_
#define ROMEA_RTLS_TRANSCEIVER_UTILS__TRANSCEIVER_PARAMETERS_HPP_

// eigen
// #include <Eigen/Geometry>

// std
#include <memory>
#include <string>

// ros
#include "rclcpp/node.hpp"


namespace romea
{

void declare_transceiver_id(std::shared_ptr<rclcpp::Node> node);
void declare_transceiver_pan_id(std::shared_ptr<rclcpp::Node> node);
void declare_transceiver_communication_configuration(std::shared_ptr<rclcpp::Node> node);

double get_transceiver_id(std::shared_ptr<rclcpp::Node> node);
double get_transceiver_pan_id(std::shared_ptr<rclcpp::Node> node);
double get_transceiver_communication_configuration(std::shared_ptr<rclcpp::Node> node);


}  // namespace romea

#endif  // ROMEA_RTLS_TRANSCEIVER_UTILS__TRANSCEIVER_PARAMETERS_HPP_
