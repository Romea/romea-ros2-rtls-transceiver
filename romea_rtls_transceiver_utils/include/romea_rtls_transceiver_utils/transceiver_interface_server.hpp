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

#ifndef ROMEA_RTLS_TRANSCEIVER_UTILS__TRANSCEIVER_INTERFACE_SERVER_HPP_
#define ROMEA_RTLS_TRANSCEIVER_UTILS__TRANSCEIVER_INTERFACE_SERVER_HPP_

// std
#include <optional>
#include <memory>
#include <string>
#include <vector>

// ros
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// romea core
#include "romea_core_rtls/RTLSRange.hpp"
#include "romea_core_rtls/RTLSTransceiverEUID.hpp"
#include "romea_core_rtls/RTLSTransceiverFunction.hpp"

// romea ros
#include "romea_rtls_transceiver_msgs/action/ranging.hpp"
#include "romea_rtls_transceiver_msgs/msg/ranging_result.hpp"
#include "romea_rtls_transceiver_msgs/msg/payload.hpp"
#include "romea_rtls_transceiver_msgs/srv/get_payload.hpp"
#include "romea_rtls_transceiver_msgs/srv/set_payload.hpp"

namespace romea
{

class TransceiverInterfaceServer
{
public:
  using Payload = romea_rtls_transceiver_msgs::msg::Payload;
  using RangingResult = romea_rtls_transceiver_msgs::msg::RangingResult;
  using RangingAction = romea_rtls_transceiver_msgs::action::Ranging;
  using RangingActionServer = rclcpp_action::Server<RangingAction>;
  using RangingActionGoalHandle = rclcpp_action::ServerGoalHandle<RangingAction>;
  using GetPayloadService = romea_rtls_transceiver_msgs::srv::GetPayload;
  using GetPayloadServiceServer = rclcpp::Service<GetPayloadService>;
  using SetPayloadService = romea_rtls_transceiver_msgs::srv::SetPayload;
  using SetPayloadServiceServer = rclcpp::Service<SetPayloadService>;

public:
  TransceiverInterfaceServer(
    std::shared_ptr<rclcpp::Node> node,
    const RTLSTransceiverEUID & transceiver_euid,
    const RTLSTransceiverFunction & transceiver_function = RTLSTransceiverFunction::NONE);

  virtual ~TransceiverInterfaceServer() = default;

protected:
  virtual bool ranging_(const uint16_t & responder_id, RangingResult & range) = 0;

  virtual bool set_payload_(const Payload & data) = 0;

  virtual bool get_payload_(Payload & data) = 0;

protected:
  void init_get_payload_service_server_();

  void init_set_payload_service_server_();

  void init_range_action_server_();

protected:
  rclcpp_action::GoalResponse ranging_handle_goal_(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const RangingAction::Goal> goal);

  virtual rclcpp_action::CancelResponse ranging_handle_cancel_(
    const std::shared_ptr<RangingActionGoalHandle> goal_handle);

  void ranging_handle_accepted_(const std::shared_ptr<RangingActionGoalHandle> goal_handle);

  void execute_ranging_(const std::shared_ptr<RangingActionGoalHandle> range_goal_handle);

  void get_payload_callback_(
    const std::shared_ptr<romea_rtls_transceiver_msgs::srv::GetPayload::Request> request,
    std::shared_ptr<romea_rtls_transceiver_msgs::srv::GetPayload::Response> response);

  void set_payload_callback_(
    const std::shared_ptr<romea_rtls_transceiver_msgs::srv::SetPayload::Request> request,
    std::shared_ptr<romea_rtls_transceiver_msgs::srv::SetPayload::Response> response);

protected:
  std::shared_ptr<rclcpp::Node> node_;

  RTLSTransceiverEUID transceiver_euid_;
  RTLSTransceiverFunction transceiver_function_;

  std::atomic<bool> ranging_is_pending_;
  std::shared_ptr<RangingActionServer> ranging_action_server_;
  std::shared_ptr<GetPayloadServiceServer> get_payload_service_server_;
  std::shared_ptr<SetPayloadServiceServer> set_payload_service_server_;
};

}  // namespace romea

#endif  // ROMEA_RTLS_TRANSCEIVER_UTILS__TRANSCEIVER_INTERFACE_SERVER_HPP_
