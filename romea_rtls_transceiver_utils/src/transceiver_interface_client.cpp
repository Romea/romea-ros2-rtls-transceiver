// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <memory>
#include <string>
#include <vector>


// local
#include "romea_rtls_transceiver_utils/transceiver_interface_client.hpp"

namespace
{
const std::chrono::seconds WAIT_TIMEOUT(5);
}

namespace romea
{

//-----------------------------------------------------------------------------
TransceiverInterfaceClient::TransceiverInterfaceClient(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & transceiver_name)
{
  get_payload_service_client_ = std::make_shared<GetPayloadServiceClient>(
    node, transceiver_name + "/get_payload", WAIT_TIMEOUT);

  set_payload_service_client_ = std::make_shared<SetPayloadServiceClient>(
    node, transceiver_name + "/set_payload", WAIT_TIMEOUT);

  ranging_action_client_ = std::make_shared<RangingActionClient>(
    node, transceiver_name + "/range", WAIT_TIMEOUT);
}

//-----------------------------------------------------------------------------
bool TransceiverInterfaceClient::ranging(
  const uint16_t & responder_id,
  RangingResult & range,
  const std::chrono::milliseconds & timeout)
{
  auto goal = RangingAction::Goal();
  goal.responder_id = responder_id;

  auto ranging_action_result = ranging_action_client_->send_goal(goal, timeout);

  if (ranging_action_result != nullptr) {
    range = ranging_action_result->range;
    return true;
  } else {
    return false;
  }
}

//-----------------------------------------------------------------------------
bool TransceiverInterfaceClient::set_payload(const Payload & payload)
{
  auto request = std::make_shared<SetPayloadService::Request>();
  request->payload = payload;
  auto response = set_payload_service_client_->send_request(request);

  if (response != nullptr) {
    return response->success;
  } else {
    return false;
  }
}

//-----------------------------------------------------------------------------
bool TransceiverInterfaceClient::get_payload(Payload & payload)
{
  auto request = std::make_shared<GetPayloadService::Request>();
  auto response = get_payload_service_client_->send_request(request);

  if (response != nullptr) {
    payload = response->payload;
    return response->success;
  } else {
    return false;
  }
}


}  // namespace romea
