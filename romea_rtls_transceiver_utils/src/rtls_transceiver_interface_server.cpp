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
#include <limits>

// romea
#include "romea_common_utils/qos.hpp"
#include "romea_rtls_transceiver_utils/rtls_transceiver_interface_server.hpp"
#include "romea_rtls_transceiver_utils/rtls_transceiver_parameters.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
RTLSTransceiverInterfaceServer::RTLSTransceiverInterfaceServer(
  std::shared_ptr<rclcpp::Node> node,
  RangingRequestCallback ranging_request_callback)
: request_sub_(nullptr),
  result_pub_(nullptr),
  payload_pub_(nullptr)
{
  // std::cout << "init server ";

  if (ranging_request_callback) {
    request_sub_ = node->create_subscription<RangingRequest>(
      "request", best_effort(1), ranging_request_callback);
    // std::cout << request_sub_->get_topic_name() << " ";
  }

  result_pub_ = node->create_publisher<RangingResult>("range", sensor_data_qos());
  payload_pub_ = node->create_publisher<Payload>("payload", sensor_data_qos());

  // std::cout << result_pub_->get_topic_name() << " "
  //           << payload_pub_->get_topic_name() << std::endl;
}

//-----------------------------------------------------------------------------
void RTLSTransceiverInterfaceServer::send_ranging_result(const RangingResult & result)
{
  result_pub_->publish(result);
}

//-----------------------------------------------------------------------------
void RTLSTransceiverInterfaceServer::send_payload(const Payload & payload)
{
  payload_pub_->publish(payload);
}

}  // namespace romea
