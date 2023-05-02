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
#include <memory>
#include <string>
#include <vector>

// local
#include "romea_rtls_transceiver_gazebo/gazebo_ros_rtls_network.hpp"
#include "romea_rtls_transceiver_gazebo/gazebo_ros_rtls_transceiver_interface.hpp"

namespace gazebo
{

//-----------------------------------------------------------------------------
GazeboRosRTLSTransceiverInterface::GazeboRosRTLSTransceiverInterface(
  std::shared_ptr<rclcpp::Node> node,
  const romea::RTLSTransceiverEUID & transceiver_euid)
: TransceiverInterfaceServer(node, transceiver_euid),
  input_payload_(),
  output_payload_()
{
  init_get_payload_service_server_();
  init_set_payload_service_server_();
  init_range_action_server_();
}

//-----------------------------------------------------------------------------
void GazeboRosRTLSTransceiverInterface::exchange_payloads(
  GazeboRosRTLSTransceiverInterface & other)
{
  other.output_payload_ = input_payload_;
  output_payload_ = other.input_payload_;
}

//-----------------------------------------------------------------------------
bool GazeboRosRTLSTransceiverInterface::ranging_(
  const uint16_t & responder_id,
  RangingResult & range)
{
  romea::RTLSTransceiverEUID responder_euid;
  responder_euid.pan_id = transceiver_euid_.pan_id;
  responder_euid.id = responder_id;

  return GazeboRosRTLSNetwork::Instance().range(transceiver_euid_, responder_euid, range);
}

//-----------------------------------------------------------------------------
bool GazeboRosRTLSTransceiverInterface::set_payload_(const Payload & payload)
{
  input_payload_ = payload.data;
  return true;
}

//-----------------------------------------------------------------------------
bool GazeboRosRTLSTransceiverInterface::get_payload_(Payload & payload)
{
  payload.data = output_payload_;
  return true;
}


}  // namespace gazebo
