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

#ifndef ROMEA_RTLS_TRANSCEIVER_UTILS__RTLS_TRANSCEIVER_INTERFACE_CLIENT_HPP_
#define ROMEA_RTLS_TRANSCEIVER_UTILS__RTLS_TRANSCEIVER_INTERFACE_CLIENT_HPP_

// std
#include <optional>
#include <memory>
#include <string>
#include <functional>

// ros
#include "rclcpp/rclcpp.hpp"


// romea ros
#include "romea_rtls_transceiver_msgs/msg/payload.hpp"
#include "romea_rtls_transceiver_msgs/msg/ranging_result.hpp"
#include "romea_rtls_transceiver_msgs/msg/ranging_request.hpp"

namespace romea
{

class RTLSTransceiverInterfaceClient
{
public:
  using Payload = romea_rtls_transceiver_msgs::msg::Payload;
  using RangingRequest = romea_rtls_transceiver_msgs::msg::RangingRequest;
  using RangingResult = romea_rtls_transceiver_msgs::msg::RangingResult;
  using RangingResultCallback = std::function<void (RangingResult::ConstSharedPtr)>;
  using PayloadCallback = std::function<void (Payload::ConstSharedPtr)>;

public:
  RTLSTransceiverInterfaceClient(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & transceiver_name,
    RangingResultCallback ranging_result_callback = {},
    PayloadCallback payload_callback = {});

  // RTLSTransceiverInterfaceClient(
  //   std::shared_ptr<rclcpp::Node> node,
  //   const std::string & transceiver_name,
  //   RangingResultCallback ranging_result_callback);

  bool is_server_available() const;

  void send_ranging_request(const RangingRequest & resquest);

  void send_payload(const Payload & payload);

private:
  std::string transceiver_name_;
  std::shared_ptr<rclcpp::Publisher<RangingRequest>> request_pub_;
  std::shared_ptr<rclcpp::Subscription<RangingResult>> result_sub_;
  std::shared_ptr<rclcpp::Subscription<Payload>> payload_sub_;
  rclcpp::Logger logger_;
};

}  // namespace romea

#endif  // ROMEA_RTLS_TRANSCEIVER_UTILS__RTLS_TRANSCEIVER_INTERFACE_CLIENT_HPP_
