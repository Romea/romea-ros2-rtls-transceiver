// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license


// std
#include <string>

// romea ros
#include "romea_common_utils/params/node_parameters.hpp"
#include "romea_common_utils/params/eigen_parameters.hpp"

// local
#include "romea_rtls_transceiver_utils/transceiver_parameters.hpp"

namespace
{
const char id_param_name[] = "transceiver.id";
const char pan_id_param_name[] = "transceiver.pan_id";
const char communication_param_name[] = "communication";
}

namespace romea
{

//-----------------------------------------------------------------------------
void declare_transceiver_id(rclcpp::Node::SharedPtr node)
{
  declare_parameter<int>(node, id_param_name);
}

//-----------------------------------------------------------------------------
void declare_transceiver_pan_id(rclcpp::Node::SharedPtr node)
{
  declare_parameter<int>(node, pan_id_param_name);
}

//-----------------------------------------------------------------------------
void declare_transceiver_communication(rclcpp::Node::SharedPtr node)
{
  declare_parameter<std::string>(node, communication_param_name);
}

// //-----------------------------------------------------------------------------
// void declare_gps_antenna_body_position(rclcpp::Node::SharedPtr node)
// {
//   return declare_eigen_xyz_vector_parameter<Eigen::Vector3d>(node, "gps");
// }

//-----------------------------------------------------------------------------
double get_transceiver_id(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, id_param_name);
}


//-----------------------------------------------------------------------------
double get_transceiver_pan_id(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, pan_id_param_name);
}

//-----------------------------------------------------------------------------
double get_transceiver_communication(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, communication_param_name);
}

// //-----------------------------------------------------------------------------
// Eigen::Vector3d get_gps_antenna_body_position(rclcpp::Node::SharedPtr node)
// {
//   return get_eigen_xyz_vector_parameter<Eigen::Vector3d>(node, "gps");
// }


}  // namespace romea
