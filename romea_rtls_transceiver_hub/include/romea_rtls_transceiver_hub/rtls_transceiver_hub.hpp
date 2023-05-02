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


#ifndef ROMEA_RTLS_TRANSCEIVER_HUB__RTLS_TRANSCEIVER_HUB_HPP_
#define ROMEA_RTLS_TRANSCEIVER_HUB__RTLS_TRANSCEIVER_HUB_HPP_

// std
#include <map>
#include <memory>
#include <string>

// ros
#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_rtls_transceiver_utils/transceiver_interface_client.hpp"
#include "romea_rtls_transceiver_msgs/msg/poll.hpp"
#include "romea_rtls_transceiver_msgs/msg/range.hpp"
#include "romea_rtls_transceiver_msgs/srv/set_transceivers_configuration.hpp"

// local
#include "romea_rtls_transceiver_hub/visibility_control.h"

namespace romea
{


class RTLSTransceiverHub
{
public:
  using Poll = romea_rtls_transceiver_msgs::msg::Poll;
  using Payload = romea_rtls_transceiver_msgs::msg::Payload;
  using Range = romea_rtls_transceiver_msgs::msg::Range;

  using SetTranceiversConfiguration =
    romea_rtls_transceiver_msgs::srv::SetTransceiversConfiguration;

public:
  ROMEA_RTLS_TRANSCEIVER_HUB_PUBLIC
  explicit RTLSTransceiverHub(const rclcpp::NodeOptions & options);

  ROMEA_RTLS_TRANSCEIVER_HUB_PUBLIC
  virtual ~RTLSTransceiverHub() = default;

  ROMEA_RTLS_TRANSCEIVER_HUB_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

protected:
  void init_range_pub_();

  void init_poll_sub_();

  void init_payload_sub_();

  void init_embedded_transceivers_inferfaces_();

  void init_external_transceivers_configuration_service_client_();

  void poll_callback_(std::shared_ptr<Poll> msg);

  void payload_callback_(std::shared_ptr<Payload> msg);

  void set_external_transceivers_configuration_callback_(
    const std::shared_ptr<SetTranceiversConfiguration::Request> request,
    std::shared_ptr<SetTranceiversConfiguration::Response> response);

protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Publisher<Range>> range_pub_;
  std::shared_ptr<rclcpp::Subscription<Poll>> poll_sub_;
  std::shared_ptr<rclcpp::Subscription<Payload>> payload_sub_;
  std::shared_ptr<rclcpp::Service<SetTranceiversConfiguration>>
  set_external_transceivers_configuration_service_client_;

  std::map<std::string, std::unique_ptr<TransceiverInterfaceClient>> embedded_transceivers_;
  std::map<std::string, uint16_t> external_transceivers_ids_;
};

}  // namespace romea

#endif   // ROMEA_RTLS_TRANSCEIVER_HUB__RTLS_TRANSCEIVER_HUB_HPP_
