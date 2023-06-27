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


#ifndef ROMEA_RTLS_TRANSCEIVER_GAZEBO__GAZEBO_ROS_RTLS_NETWORK_HPP_
#define ROMEA_RTLS_TRANSCEIVER_GAZEBO__GAZEBO_ROS_RTLS_NETWORK_HPP_

// std
#include <map>

// romea
#include "romea_core_rtls_transceiver/RTLSTransceiverEUID.hpp"
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
