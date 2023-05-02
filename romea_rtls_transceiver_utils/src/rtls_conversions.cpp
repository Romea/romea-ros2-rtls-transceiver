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
#include <string>

// local
#include "romea_rtls_transceiver_utils/rtls_conversions.hpp"
#include "romea_common_utils/conversions/time_conversions.hpp"

namespace romea
{

void to_romea(
  const romea_rtls_transceiver_msgs::msg::RangingResult ranging_result,
  RTLSRange & romea_range)
{
  auto ros_time = rclcpp::Time(ranging_result.stamp.sec, ranging_result.stamp.nanosec);
  romea_range.duration = to_romea_duration(ros_time);
  romea_range.range = ranging_result.range;
  romea_range.firstPathRxPowerLevel = ranging_result.first_path_rx_power_level;
  romea_range.totalRxPowerLevel = ranging_result.total_rx_power_level;
}

void to_romea(
  const romea_rtls_transceiver_msgs::msg::TransceiverEUID transceiver_euid_msg,
  RTLSTransceiverEUID & romea_transceiver_euid)
{
  romea_transceiver_euid.pan_id = transceiver_euid_msg.pan_id;
  romea_transceiver_euid.id = transceiver_euid_msg.id;
}


}  // namespace romea
