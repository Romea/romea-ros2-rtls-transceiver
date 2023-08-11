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

// local
#include "romea_rtls_transceiver_utils/rtls_transceiver_interface.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
RTLSTransceiverInterface::RTLSTransceiverInterface(
  std::shared_ptr<rclcpp::Node> node,
  const RTLSTransceiverEUID & transceiver_euid,
  const RTLSTransceiverFunction & transceiver_function)
: transceiver_name_(node->get_namespace()),
  transceiver_euid_(transceiver_euid),
  transceiver_function_(transceiver_function),
  ranging_is_pending_(false),
  logger_(node->get_logger())
{
}


//-----------------------------------------------------------------------------
bool RTLSTransceiverInterface::ranging(
  const uint16_t & responder_id,
  const double & timeout,
  Range & range)
{
  if (transceiver_function_ == RTLSTransceiverFunction::RESPONDER ||
    transceiver_function_ == RTLSTransceiverFunction::LISTENER)
  {
    std::stringstream msg;
    msg << "Ranging request has been rejected from transceiver ";
    msg << transceiver_name_;
    msg << " because it is a ";
    msg << functionToString(transceiver_function_);
    RCLCPP_ERROR_STREAM(logger_, msg.str());
    return false;
  }

  if (responder_id == transceiver_euid_.id) {
    std::stringstream msg;
    msg << "Ranging request has been rejected from transceiver ";
    msg << transceiver_name_;
    msg << " because it cannot achieve ranging with itself ";
    RCLCPP_ERROR_STREAM(logger_, msg.str());
    return false;
  }

  if (ranging_is_pending_) {
    std::stringstream msg;
    msg << "Ranging request has been rejected from transceiver ";
    msg << transceiver_name_;
    msg << " beacause a ranging is already pending ";
    RCLCPP_ERROR_STREAM(logger_, msg.str());
    return false;
  }


  if (!transceiver_ranging_(responder_id, timeout, range)) {
    std::stringstream msg;
    msg << "Ranging from transceiver ";
    msg << transceiver_name_;
    msg << " has failed ";
    RCLCPP_ERROR_STREAM(logger_, msg.str());
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
bool RTLSTransceiverInterface::set_payload(const Payload & payload)
{
  if (transceiver_function_ == RTLSTransceiverFunction::LISTENER) {
    std::stringstream msg;
    msg << "Cannot set payload from transceiver ";
    msg << transceiver_name_;
    msg << " because it is a ";
    msg << functionToString(transceiver_function_);
    RCLCPP_ERROR_STREAM(logger_, msg.str());
    return false;
  }

  if (!transceiver_set_payload_(payload)) {
    std::stringstream msg;
    msg << "Set payload to transceiver ";
    msg << transceiver_name_;
    msg << " has failed ";
    RCLCPP_ERROR_STREAM(logger_, msg.str());
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
bool RTLSTransceiverInterface::get_payload(Payload & payload)
{
  if (transceiver_function_ == RTLSTransceiverFunction::LISTENER) {
    std::stringstream msg;
    msg << "Cannot get payload from transceiver ";
    msg << transceiver_name_;
    msg << " because it is a ";
    msg << functionToString(transceiver_function_);
    RCLCPP_ERROR_STREAM(logger_, msg.str());
    return false;
  }

  if (!transceiver_get_payload_(payload)) {
    std::stringstream msg;
    msg << "Get payload to transceiver ";
    msg << transceiver_name_;
    msg << " has failed ";
    RCLCPP_ERROR_STREAM(logger_, msg.str());
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
const std::string & RTLSTransceiverInterface::get_name()
{
  return transceiver_name_;
}

const RTLSTransceiverEUID & RTLSTransceiverInterface::get_euid()
{
  return transceiver_euid_;
}

// //-----------------------------------------------------------------------------
// std::optional<romea_rtls_transceiver_msgs::msg::RangingResult> process_request(
//   std::shared_ptr<RTLSTransceiverInterface> transceiver,
//   const romea_rtls_transceiver_msgs::msg::RangingRequest & request)
// {
//   std::cout << " receive request" << request.responder_id << std::endl;

//   if (!request.payload.data.empty()) {
//     transceiver->set_payload(request.payload);
//   }

//   if (request.responder_id != std::numeric_limits<uint16_t>::max()) {
//     romea_rtls_transceiver_msgs::msg::RangingResult result;
//     result.initiator_id = transceiver->get_euid().id;
//     result.responder_id = request.responder_id;
//     if (transceiver->ranging(request.responder_id, request.timeout, result.range)) {
//       transceiver->get_payload(result.payload);
//     }
//     return result;
//   }

//   return {};
// }


}  // namespace romea
