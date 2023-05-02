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



#ifndef ROMEA_RTLS_TRANSCEIVER_UTILS__RTLS_CONVERSIONS_HPP_
#define ROMEA_RTLS_TRANSCEIVER_UTILS__RTLS_CONVERSIONS_HPP_

// std
#include <memory>
#include <string>

// romea core
#include "romea_core_rtls/RTLSRange.hpp"
#include "romea_core_rtls/RTLSTransceiverEUID.hpp"

// romea ros
#include "romea_rtls_transceiver_msgs/msg/ranging_result.hpp"
#include "romea_rtls_transceiver_msgs/msg/transceiver_euid.hpp"


namespace romea
{

void to_romea(
  const romea_rtls_transceiver_msgs::msg::RangingResult ranging_result_msg,
  RTLSRange & romea_range);

void to_romea(
  const romea_rtls_transceiver_msgs::msg::TransceiverEUID transceiver_euid_msg,
  RTLSTransceiverEUID & romea_transceiver_euid);


}  // namespace romea

#endif  // ROMEA_RTLS_TRANSCEIVER_UTILS__RTLS_CONVERSIONS_HPP_
