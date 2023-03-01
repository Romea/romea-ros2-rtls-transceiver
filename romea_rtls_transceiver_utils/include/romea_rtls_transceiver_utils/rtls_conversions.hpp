// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license


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
