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
#include <functional>
#include <limits>


// romea
#include "romea_common_utils/qos.hpp"
#include "romea_rtls_transceiver_utils/rtls_transceiver_interface_client.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
RTLSTransceiverInterfaceClient::RTLSTransceiverInterfaceClient(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & transceiver_name,
  RangingResultCallback ranging_result_callback,
  PayloadCallback payload_callback)
: transceiver_name_(transceiver_name),
  logger_(node->get_logger())
{
  request_pub_ = node->create_publisher<RangingRequest>(
    transceiver_name + "/request", sensor_data_qos());
  // std::cout << "client " << request_pub_->get_topic_name();

  if (ranging_result_callback) {
    result_sub_ = node->create_subscription<RangingResult>(
      transceiver_name + "/range", best_effort(1), ranging_result_callback);
    // std::cout << " " << result_sub_->get_topic_name();
  }
  if (payload_callback) {
    payload_sub_ = node->create_subscription<Payload>(
      transceiver_name + "/payload", best_effort(1), payload_callback);
    // std::cout << " " << payload_sub_->get_topic_name();
  }
  // std::cout << std::endl;
}

//-----------------------------------------------------------------------------
bool RTLSTransceiverInterfaceClient::is_server_available() const
{
  return request_pub_->get_subscription_count() == 1;
}

//-----------------------------------------------------------------------------
void RTLSTransceiverInterfaceClient::send_ranging_request(const RangingRequest & resquest)
{
  if (is_server_available()) {
    request_pub_->publish(resquest);
  } else {
    std::stringstream msg;
    msg << "Ranging request has not been send to tranceiver ";
    msg << transceiver_name_;
    msg << " because server interface is not available";
    RCLCPP_ERROR_STREAM(logger_, msg.str());
  }
}

//-----------------------------------------------------------------------------
void RTLSTransceiverInterfaceClient::send_payload(const Payload & payload)
{
  if (is_server_available()) {
    auto request = std::make_unique<RangingRequest>();
    request->responder_id = std::numeric_limits<uint16_t>::max();
    request->payload = payload;
    request_pub_->publish(std::move(request));
  } else {
    std::stringstream msg;
    msg << "Payload has not been send to tranceiver ";
    msg << transceiver_name_;
    msg << " because server interface is not available";
    RCLCPP_ERROR_STREAM(logger_, msg.str());
  }
}


}  // namespace romea
