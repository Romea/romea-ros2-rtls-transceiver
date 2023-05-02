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


#ifndef ROMEA_RTLS_TRANSCEIVER_GAZEBO__GAZEBO_ROS_RTLS_TRANSCEIVER_INTERFACE_HPP_
#define ROMEA_RTLS_TRANSCEIVER_GAZEBO__GAZEBO_ROS_RTLS_TRANSCEIVER_INTERFACE_HPP_

// std
#include <memory>
#include <string>
#include <vector>

// romea
#include "romea_rtls_transceiver_utils/transceiver_interface_server.hpp"

namespace gazebo
{

class GazeboRosRTLSTransceiverInterface : public romea::TransceiverInterfaceServer
{
public:
  GazeboRosRTLSTransceiverInterface(
    std::shared_ptr<rclcpp::Node> node,
    const romea::RTLSTransceiverEUID & euid);

  virtual ~GazeboRosRTLSTransceiverInterface() = default;

  void exchange_payloads(GazeboRosRTLSTransceiverInterface & other);

private:
  bool ranging_(const uint16_t & responder_id, RangingResult & range) override;

  bool set_payload_(const Payload & payload);

  bool get_payload_(Payload & payload);

private:
  std::vector<unsigned char> input_payload_;
  std::vector<unsigned char> output_payload_;
};


}  // namespace gazebo

#endif  // ROMEA_RTLS_TRANSCEIVER_GAZEBO__GAZEBO_ROS_RTLS_TRANSCEIVER_INTERFACE_HPP_
