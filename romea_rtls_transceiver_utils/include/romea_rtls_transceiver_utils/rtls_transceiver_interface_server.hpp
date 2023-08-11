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

#ifndef ROMEA_RTLS_TRANSCEIVER_UTILS__RTLS_TRANSCEIVER_INTERFACE_SERVER_HPP_
#define ROMEA_RTLS_TRANSCEIVER_UTILS__RTLS_TRANSCEIVER_INTERFACE_SERVER_HPP_

// std
#include <optional>
#include <memory>
#include <string>

// ros
#include "rclcpp/rclcpp.hpp"

// romea ros
#include "romea_rtls_transceiver_utils/rtls_transceiver_interface.hpp"
#include "romea_rtls_transceiver_msgs/msg/ranging_request.hpp"
#include "romea_rtls_transceiver_msgs/msg/ranging_result.hpp"


namespace romea
{

class RTLSTransceiverInterfaceServer
{
public:
  using Range = romea_rtls_transceiver_msgs::msg::Range;
  using Payload = romea_rtls_transceiver_msgs::msg::Payload;
  using RangingRequest = romea_rtls_transceiver_msgs::msg::RangingRequest;
  using RangingResult = romea_rtls_transceiver_msgs::msg::RangingResult;
  using RangingRequestCallback = std::function<void (RangingRequest::ConstSharedPtr)>;

public:
  RTLSTransceiverInterfaceServer(
    std::shared_ptr<rclcpp::Node> node,
    RangingRequestCallback ranging_request_callback
  );

  void send_ranging_result(const RangingResult & result);

  void send_payload(const Payload & payload);

private:
  std::shared_ptr<rclcpp::Subscription<RangingRequest>> request_sub_;
  std::shared_ptr<rclcpp::Publisher<RangingResult>> result_pub_;
  std::shared_ptr<rclcpp::Publisher<Payload>> payload_pub_;
};

}  // namespace romea

#endif  // ROMEA_RTLS_TRANSCEIVER_UTILS__RTLS_TRANSCEIVER_INTERFACE_SERVER_HPP_
