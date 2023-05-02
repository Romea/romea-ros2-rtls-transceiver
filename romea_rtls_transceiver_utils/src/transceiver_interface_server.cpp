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
#include <functional>
#include <memory>
#include <string>

// local
#include "romea_rtls_transceiver_utils/transceiver_interface_server.hpp"
#include "romea_rtls_transceiver_utils/transceiver_parameters.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
TransceiverInterfaceServer::TransceiverInterfaceServer(
  std::shared_ptr<rclcpp::Node> node,
  const RTLSTransceiverEUID & transceiver_euid,
  const RTLSTransceiverFunction & transceiver_function)
: node_(node),
  transceiver_euid_(transceiver_euid),
  transceiver_function_(transceiver_function),
  ranging_is_pending_(false),
  ranging_action_server_(nullptr),
  get_payload_service_server_(nullptr),
  set_payload_service_server_(nullptr)
{
}

// //-----------------------------------------------------------------------------
// void TransceiverInterfaceServer::get_transceiver_parameters_(std::shared_ptr<rclcpp::Node> node)
// {
//   declare_transceiver_id(node);
//   declare_transceiver_pan_id(node);
//   declare_transceiver_communication_configuration(node);

//   transceiver_euid_.id = get_transceiver_id(node);
//   transceiver_euid_.id = get_transceiver_pan_id(node);
//   transceiver_communication_configuration_ = get_transceiver_communication_configuration(node);
// }

//-----------------------------------------------------------------------------
void TransceiverInterfaceServer::init_range_action_server_()
{
  using namespace std::placeholders;
  ranging_action_server_ = rclcpp_action::create_server<RangingAction>(
    node_, "range",
    std::bind(&TransceiverInterfaceServer::ranging_handle_goal_, this, _1, _2),
    std::bind(&TransceiverInterfaceServer::ranging_handle_cancel_, this, _1),
    std::bind(&TransceiverInterfaceServer::ranging_handle_accepted_, this, _1));
}

//-----------------------------------------------------------------------------
void TransceiverInterfaceServer::init_get_payload_service_server_()
{
  using namespace std::placeholders;

  get_payload_service_server_ = node_->create_service<GetPayloadService>(
    "get_payload",
    std::bind(&TransceiverInterfaceServer::get_payload_callback_, this, _1, _2));
}

//-----------------------------------------------------------------------------
void TransceiverInterfaceServer::init_set_payload_service_server_()
{
  using namespace std::placeholders;

  set_payload_service_server_ = node_->create_service<SetPayloadService>(
    "set_payload",
    std::bind(&TransceiverInterfaceServer::set_payload_callback_, this, _1, _2));
}

//-----------------------------------------------------------------------------
rclcpp_action::GoalResponse TransceiverInterfaceServer::ranging_handle_goal_(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const RangingAction::Goal> goal)
{
  if (transceiver_function_ == RTLSTransceiverFunction::RESPONDER ||
    transceiver_function_ == RTLSTransceiverFunction::LISTENER)
  {
    std::stringstream msg;
    msg << "Ranging action has been rejected from transceiver ";
    msg << node_->get_namespace();
    msg << " because it is a ";
    msg << functionToString(transceiver_function_);
    RCLCPP_ERROR_STREAM(node_->get_logger(), msg.str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->responder_id == transceiver_euid_.id) {
    std::stringstream msg;
    msg << "Ranging action has been rejected from transceiver ";
    msg << node_->get_namespace();
    msg << " because it cannot achieve ranging with itself ";
    RCLCPP_ERROR_STREAM(node_->get_logger(), msg.str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (ranging_is_pending_) {
    std::stringstream msg;
    msg << "Ranging action has been rejected from transceiver ";
    msg << node_->get_namespace();
    msg << " beacause a ranging is already pending ";
    RCLCPP_ERROR_STREAM(node_->get_logger(), msg.str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

//-----------------------------------------------------------------------------
rclcpp_action::CancelResponse TransceiverInterfaceServer::ranging_handle_cancel_(
  const std::shared_ptr<RangingActionGoalHandle> goal_handle)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

//-----------------------------------------------------------------------------
void TransceiverInterfaceServer::ranging_handle_accepted_(
  const std::shared_ptr<RangingActionGoalHandle> ranging_goal_handle)
{
  using namespace std::placeholders;

  std::thread{
    std::bind(&TransceiverInterfaceServer::execute_ranging_, this, _1),
    ranging_goal_handle}.detach();
}

//-----------------------------------------------------------------------------
void TransceiverInterfaceServer::execute_ranging_(
  const std::shared_ptr<RangingActionGoalHandle> range_goal_handle)
{
  ranging_is_pending_ = true;
  auto result = std::make_shared<RangingAction::Result>();
  bool success = ranging_(range_goal_handle->get_goal()->responder_id, result->range);

  if (range_goal_handle->is_canceling()) {
    range_goal_handle->canceled(result);
  } else {
    if (success) {
      range_goal_handle->succeed(result);
    } else {
      range_goal_handle->abort(result);
    }
  }

  ranging_is_pending_ = false;
}

//-----------------------------------------------------------------------------
void TransceiverInterfaceServer::get_payload_callback_(
  const std::shared_ptr<romea_rtls_transceiver_msgs::srv::GetPayload::Request> request,
  std::shared_ptr<romea_rtls_transceiver_msgs::srv::GetPayload::Response> response)
{
  if (transceiver_function_ == RTLSTransceiverFunction::RESPONDER ||
    transceiver_function_ == RTLSTransceiverFunction::LISTENER)
  {
    std::stringstream msg;
    msg << "Cannot get payload from transceiver ";
    msg << node_->get_namespace();
    msg << " because it is a ";
    msg << functionToString(transceiver_function_);
    RCLCPP_ERROR_STREAM(node_->get_logger(), msg.str());
    response->success = false;
  } else {
    response->success = get_payload_(response->payload);
  }
}

//-----------------------------------------------------------------------------
void TransceiverInterfaceServer::set_payload_callback_(
  const std::shared_ptr<romea_rtls_transceiver_msgs::srv::SetPayload::Request> request,
  std::shared_ptr<romea_rtls_transceiver_msgs::srv::SetPayload::Response> response)
{
  if (transceiver_function_ == RTLSTransceiverFunction::LISTENER) {
    std::stringstream msg;
    msg << "Cannot set payload from transceiver ";
    msg << node_->get_namespace();
    msg << " because it is a ";
    msg << functionToString(transceiver_function_);
    RCLCPP_ERROR_STREAM(node_->get_logger(), msg.str());
    response->success = false;
  } else {
    response->success = set_payload_(request->payload);
  }
}


}  // namespace romea
