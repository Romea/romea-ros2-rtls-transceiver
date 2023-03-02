// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <limits>

// romea
#include "romea_rtls_transceiver_gazebo/gazebo_ros_rtls_network.hpp"


namespace gazebo
{

//-----------------------------------------------------------------------------
GazeboRosRTLSNetwork & GazeboRosRTLSNetwork::Instance()
{
  static GazeboRosRTLSNetwork myInstance;
  return myInstance;
}

//-----------------------------------------------------------------------------
GazeboRosRTLSNetwork::GazeboRosRTLSNetwork()
{
}

//-----------------------------------------------------------------------------
void GazeboRosRTLSNetwork::add_transceiver(
  const romea::RTLSTransceiverEUID & euid,
  GazeboRosRTLSTransceiver * transceiver)
{
  transceivers_[euid] = transceiver;
}

//-----------------------------------------------------------------------------
bool GazeboRosRTLSNetwork::range(
  romea::RTLSTransceiverEUID initiator_euid,
  romea::RTLSTransceiverEUID responder_euid,
  romea_rtls_transceiver_msgs::msg::RangingResult & range_result)
{
  GazeboRosRTLSTransceiver * initiator = transceivers_[initiator_euid];
  GazeboRosRTLSTransceiver * responder = transceivers_[responder_euid];

  if (responder != nullptr && initiator != nullptr) {
    auto range = initiator->ComputeRange(responder);
    if (!range.has_value()) {
      range_result.range = std::numeric_limits<double>::quiet_NaN();
      return false;
    } else {
      range_result.range = range.value();
      return true;
    }

    // TODO(Jean) simulate channel transmission
    // TODO(Jean) simulate power loss
  } else {
    // ROS_ERROR_STREAM("no responder" << responder_euid.pan_id << " " << responder_euid.id);
    return false;
  }
}

}  // namespace gazebo
