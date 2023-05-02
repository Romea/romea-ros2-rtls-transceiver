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
