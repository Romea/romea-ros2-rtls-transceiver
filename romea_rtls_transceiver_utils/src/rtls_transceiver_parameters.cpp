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

// std
#include <string>
#include <memory>
#include <limits>

// romea ros
#include "romea_common_utils/params/node_parameters.hpp"
#include "romea_common_utils/params/eigen_parameters.hpp"

// local
#include "romea_rtls_transceiver_utils/rtls_transceiver_parameters.hpp"

namespace
{
const char id_param_name[] = "transceiver.id";
const char pan_id_param_name[] = "transceiver.pan_id";
const char communication_param_name[] = "transceiver.communication";
}

namespace romea
{

//-----------------------------------------------------------------------------
void declare_transceiver_id(rclcpp::Node::SharedPtr node)
{
  declare_parameter<int64_t>(node, id_param_name);
}

//-----------------------------------------------------------------------------
void declare_transceiver_pan_id(rclcpp::Node::SharedPtr node)
{
  declare_parameter<int64_t>(node, pan_id_param_name);
}

//-----------------------------------------------------------------------------
void declare_transceiver_communication_configuration(rclcpp::Node::SharedPtr node)
{
  declare_parameter<std::string>(node, communication_param_name);
}

// //-----------------------------------------------------------------------------
// void declare_gps_antenna_body_position(rclcpp::Node::SharedPtr node)
// {
//   return declare_eigen_xyz_vector_parameter<Eigen::Vector3d>(node, "gps");
// }

//-----------------------------------------------------------------------------
uint16_t get_transceiver_id(rclcpp::Node::SharedPtr node)
{
  auto int64_id = get_parameter<int64_t>(node, id_param_name);
  assert(int64_id >= 0 && int64_id <= std::numeric_limits<uint16_t>::max());
  return int64_id;
}


//-----------------------------------------------------------------------------
uint16_t get_transceiver_pan_id(rclcpp::Node::SharedPtr node)
{
  auto int64_pan_id = get_parameter<int64_t>(node, pan_id_param_name);
  assert(int64_pan_id >= 0 && int64_pan_id <= std::numeric_limits<uint16_t>::max());
  return int64_pan_id;
}

//-----------------------------------------------------------------------------
std::string get_transceiver_communication_configuration(rclcpp::Node::SharedPtr node)
{
  return get_parameter<std::string>(node, communication_param_name);
}

// //-----------------------------------------------------------------------------
// Eigen::Vector3d get_gps_antenna_body_position(rclcpp::Node::SharedPtr node)
// {
//   return get_eigen_xyz_vector_parameter<Eigen::Vector3d>(node, "gps");
// }


}  // namespace romea
