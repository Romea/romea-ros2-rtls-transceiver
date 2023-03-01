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

#include "romea_common_utils/params/node_parameters.hpp"
#include "romea_common_utils/params/eigen_parameters.hpp"


namespace romea
{


//-----------------------------------------------------------------------------
template<typename Node>
void declare_transceiver_id(std::shared_ptr<Node> node)
{
  declare_parameter<int>(node, "transceiver.id");
}

//-----------------------------------------------------------------------------
template<typename Node>
void declare_transceiver_pan_id(std::shared_ptr<Node> node)
{
  declare_parameter<int>(node, "transceiver.pan_id");
}

//-----------------------------------------------------------------------------
template<typename Node>
void declare_transceiver_communication_configuration(std::shared_ptr<Node> node)
{
  declare_parameter<std::string>(node, "transceiver.communication_conf");
}

// //-----------------------------------------------------------------------------
// template<typename Node>
// void declare_gps_antenna_body_position(rclcpp::Node::SharedPtr node)
// {
//   return declare_eigen_xyz_vector_parameter<Eigen::Vector3d>(node, "transceiver.xyz");
// }

//-----------------------------------------------------------------------------
template<typename Node>
double get_transceiver_id(std::shared_ptr<Node> node)
{
  return get_parameter<double>(node, "transceiver.id");
}


//-----------------------------------------------------------------------------
template<typename Node>
double get_transceiver_pan_id(std::shared_ptr<Node> node)
{
  return get_parameter<double>(node, "transceiver.pan_id");
}

//-----------------------------------------------------------------------------
template<typename Node>
double get_transceiver_communication_configuration(std::shared_ptr<Node> node)
{
  return get_parameter<double>(node, "transceiver.communication_conf");
}

// //-----------------------------------------------------------------------------
// template<typename Node>
// void get_gps_antenna_body_position(rclcpp::Node::SharedPtr node)
// {
//   return declare_eigen_xyz_vector_parameter<Eigen::Vector3d>(node, "transceiver.xyz");
// }


}  // namespace romea

#endif  // ROMEA_RTLS_TRANSCEIVER_UTILS__TRANSCEIVER_PARAMETERS_HPP_
