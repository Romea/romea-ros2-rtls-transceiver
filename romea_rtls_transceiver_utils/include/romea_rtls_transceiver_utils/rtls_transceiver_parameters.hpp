// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROMEA_RTLS_TRANSCEIVER_UTILS__RTLS_TRANSCEIVER_PARAMETERS_HPP_
#define ROMEA_RTLS_TRANSCEIVER_UTILS__RTLS_TRANSCEIVER_PARAMETERS_HPP_

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

uint16_t get_transceiver_id(std::shared_ptr<rclcpp::Node> node);
uint16_t  get_transceiver_pan_id(std::shared_ptr<rclcpp::Node> node);
std::string get_transceiver_communication_configuration(std::shared_ptr<rclcpp::Node> node);


}  // namespace romea

#endif  // ROMEA_RTLS_TRANSCEIVER_UTILS__RTLS_TRANSCEIVER_PARAMETERS_HPP_
