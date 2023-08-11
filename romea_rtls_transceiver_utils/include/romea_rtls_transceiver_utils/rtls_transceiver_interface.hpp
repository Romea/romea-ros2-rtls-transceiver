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

#ifndef ROMEA_RTLS_TRANSCEIVER_UTILS__RTLS_TRANSCEIVER_INTERFACE_HPP_
#define ROMEA_RTLS_TRANSCEIVER_UTILS__RTLS_TRANSCEIVER_INTERFACE_HPP_

// std
#include <optional>
#include <memory>
#include <string>
#include <vector>

// ros
#include "rclcpp/rclcpp.hpp"

// romea core
#include "romea_core_rtls_transceiver/RTLSTransceiverEUID.hpp"
#include "romea_core_rtls_transceiver/RTLSTransceiverFunction.hpp"

// romea ros
#include "romea_rtls_transceiver_msgs/msg/ranging_request.hpp"
#include "romea_rtls_transceiver_msgs/msg/ranging_result.hpp"
#include "romea_rtls_transceiver_msgs/msg/payload.hpp"
#include "romea_rtls_transceiver_msgs/msg/range.hpp"


namespace romea
{

class RTLSTransceiverInterface
{
public:
  using Range = romea_rtls_transceiver_msgs::msg::Range;
  using Payload = romea_rtls_transceiver_msgs::msg::Payload;

public:
  RTLSTransceiverInterface(
    std::shared_ptr<rclcpp::Node> node,
    const RTLSTransceiverEUID & transceiver_euid,
    const RTLSTransceiverFunction & transceiver_function = RTLSTransceiverFunction::NONE);

  virtual ~RTLSTransceiverInterface() = default;

  bool ranging(
    const uint16_t & responder_id,
    const double & timeout,
    Range & range);

  bool set_payload(const Payload & data);

  bool get_payload(Payload & data);

  const std::string & get_name();

  const RTLSTransceiverEUID & get_euid();

protected:
  virtual bool transceiver_ranging_(
    const uint16_t & responder_id,
    const double & timeout,
    Range & range) = 0;

  virtual bool transceiver_set_payload_(const Payload & data) = 0;

  virtual bool transceiver_get_payload_(Payload & data) = 0;

protected:
  std::string transceiver_name_;
  RTLSTransceiverEUID transceiver_euid_;
  RTLSTransceiverFunction transceiver_function_;
  std::atomic<bool> ranging_is_pending_;
  rclcpp::Logger logger_;
};

// romea_rtls_transceiver_msgs::msg::RangingResult process_request(
//   std::shared_ptr<RTLSTransceiverInterface> transceiver,
//   const romea_rtls_transceiver_msgs::msg::RangingRequest & request);

}  // namespace romea

#endif  // ROMEA_RTLS_TRANSCEIVER_UTILS__RTLS_TRANSCEIVER_INTERFACE_HPP_
