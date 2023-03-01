// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_RTLS_TRANSCEIVER_GAZEBO__GAZEBO_ROS_RTLS_NETWORK_HPP_
#define ROMEA_RTLS_TRANSCEIVER_GAZEBO__GAZEBO_ROS_RTLS_NETWORK_HPP_

// std
#include <map>

// romea
#include "romea_core_rtls/RTLSTransceiverEUID.hpp"
#include "romea_rtls_transceiver_msgs/msg/ranging_result.hpp"

// local
#include "romea_rtls_transceiver_gazebo/gazebo_ros_rtls_transceiver.hpp"
// #include "romea_rtls_gazebo/gazebo_ros_rtls_transceiver_euid.hpp"

namespace gazebo
{

class GazeboRosRTLSNetwork
{
public:
  // delete copy and move constructors and assign operators
  GazeboRosRTLSNetwork(GazeboRosRTLSNetwork const &) = delete;             // Copy construct
  GazeboRosRTLSNetwork(GazeboRosRTLSNetwork &&) = delete;                  // Move construct
  GazeboRosRTLSNetwork & operator=(GazeboRosRTLSNetwork const &) = delete;  // Copy assign
  GazeboRosRTLSNetwork & operator=(GazeboRosRTLSNetwork &&) = delete;      // Move assign

protected:
  GazeboRosRTLSNetwork();

public:
  static GazeboRosRTLSNetwork & Instance();

  void add_transceiver(
    const romea::RTLSTransceiverEUID & euid,
    GazeboRosRTLSTransceiver * transceiver);

  bool range(
    romea::RTLSTransceiverEUID initiator_euid,
    romea::RTLSTransceiverEUID responder_euid,
    romea_rtls_transceiver_msgs::msg::RangingResult & range);

private:
  std::map<romea::RTLSTransceiverEUID, GazeboRosRTLSTransceiver *> transceivers_;
};


}  // namespace gazebo

#endif  // ROMEA_RTLS_TRANSCEIVER_GAZEBO__GAZEBO_ROS_RTLS_NETWORK_HPP_
