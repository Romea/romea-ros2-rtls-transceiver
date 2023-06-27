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

#ifndef ROMEA_RTLS_TRANSCEIVER_UTILS__TRANSCEIVER_INTERFACE_CLIENT_HPP_
#define ROMEA_RTLS_TRANSCEIVER_UTILS__TRANSCEIVER_INTERFACE_CLIENT_HPP_

// std
#include <optional>
#include <memory>
#include <string>
#include <vector>

// ros
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// romea core
#include "romea_core_rtls_transceiver/RTLSTransceiverEUID.hpp"

// romea ros
#include "romea_common_utils/services/service_client_async.hpp"
#include "romea_common_utils/actions/action_client_async.hpp"

#include "romea_rtls_transceiver_msgs/action/ranging.hpp"
#include "romea_rtls_transceiver_msgs/msg/ranging_result.hpp"
#include "romea_rtls_transceiver_msgs/msg/payload.hpp"
#include "romea_rtls_transceiver_msgs/srv/set_payload.hpp"
#include "romea_rtls_transceiver_msgs/srv/get_payload.hpp"

// local
#include "romea_rtls_transceiver_utils/transceiver_parameters.hpp"

namespace romea
{

class TransceiverInterfaceClient
{
public:
  using Payload = romea_rtls_transceiver_msgs::msg::Payload;
  using RangingResult = romea_rtls_transceiver_msgs::msg::RangingResult;
  using RangingAction = romea_rtls_transceiver_msgs::action::Ranging;
  using RangingActionClient = ActionClientAsync<RangingAction, rclcpp::Node>;
  using GetPayloadService = romea_rtls_transceiver_msgs::srv::GetPayload;
  using GetPayloadServiceClient = ServiceClientAsync<GetPayloadService, rclcpp::Node>;
  using SetPayloadService = romea_rtls_transceiver_msgs::srv::SetPayload;
  using SetPayloadServiceClient = ServiceClientAsync<SetPayloadService, rclcpp::Node>;

public:
  TransceiverInterfaceClient(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & transceiver_name);

  bool ranging(
    const uint16_t & responder_id,
    RangingResult & range,
    const std::chrono::milliseconds & timeout);

  bool set_payload(const Payload & payload);

  bool get_payload(Payload & payload);

protected:
  std::shared_ptr<RangingActionClient> ranging_action_client_;
  std::shared_ptr<SetPayloadServiceClient> set_payload_service_client_;
  std::shared_ptr<GetPayloadServiceClient> get_payload_service_client_;
};

}  // namespace romea

#endif  // ROMEA_RTLS_TRANSCEIVER_UTILS__TRANSCEIVER_INTERFACE_CLIENT_HPP_
