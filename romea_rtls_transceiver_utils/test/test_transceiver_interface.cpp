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
#include <limits>
#include <string>
#include <memory>
#include <vector>
#include <thread>

// gtest
#include "gtest/gtest.h"

// local
#include "romea_rtls_transceiver_utils/rtls_transceiver_interface_client.hpp"
#include "romea_rtls_transceiver_utils/rtls_transceiver_interface_server.hpp"

#define TESTING

class MocTransceiverInterface : public romea::RTLSTransceiverInterface
{
public:
  MocTransceiverInterface(
    std::shared_ptr<rclcpp::Node> node,
    const romea::RTLSTransceiverFunction & transceiver_function,
    const bool & ranging_failed)
  : RTLSTransceiverInterface(node, {0, 0}, transceiver_function),
    ranging_failed_(ranging_failed)
  {
  }

  virtual ~MocTransceiverInterface() = default;

private:
  bool transceiver_ranging_(
    const uint16_t & /*responder_id*/,
    const double & /*timeout*/,
    Range & /*range*/) override
  {
    return ranging_failed_;
  }

  bool transceiver_set_payload_(const Payload & /*payload*/) override
  {
    return true;
  }

  bool transceiver_get_payload_(Payload & /*payload*/) override
  {
    return true;
  }

  bool ranging_failed_;
};

class TestRTLSTransceiverInterface : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("test_rtls_transceiver_interfaces");
  }

  void TearDown() override
  {
  }


  void make_transceiver_interface(
    const romea::RTLSTransceiverFunction & transceiver_function,
    const bool & ranging_failed = true)
  {
    transceiver = std::make_shared<MocTransceiverInterface>(
      node, transceiver_function, ranging_failed);
  }

  bool ranging(const size_t & responder_id)
  {
    romea_rtls_transceiver_msgs::msg::Range range;
    return transceiver->ranging(responder_id, 0., range);
  }

  bool set_payload()
  {
    romea_rtls_transceiver_msgs::msg::Payload payload;
    return transceiver->set_payload(payload);
  }

  bool get_payload()
  {
    romea_rtls_transceiver_msgs::msg::Payload payload;
    return transceiver->get_payload(payload);
  }

  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<MocTransceiverInterface> transceiver;
};

TEST_F(TestRTLSTransceiverInterface, set_payload_ok_when_transceiver_is_a_initiator) {
  make_transceiver_interface(romea::RTLSTransceiverFunction::INITIATOR);
  EXPECT_TRUE(set_payload());
}

TEST_F(TestRTLSTransceiverInterface, set_payload_ok_when_transceiver_is_a_responder) {
  make_transceiver_interface(romea::RTLSTransceiverFunction::RESPONDER);
  EXPECT_TRUE(set_payload());
}

TEST_F(TestRTLSTransceiverInterface, set_payload_failed_when_transceiver_is_a_listener) {
  make_transceiver_interface(romea::RTLSTransceiverFunction::LISTENER);
  EXPECT_FALSE(set_payload());
}

TEST_F(TestRTLSTransceiverInterface, get_payload_ok_when_transceiver_is_a_initiator) {
  make_transceiver_interface(romea::RTLSTransceiverFunction::INITIATOR);
  EXPECT_TRUE(get_payload());
}

TEST_F(TestRTLSTransceiverInterface, get_payload_ok_when_transceiver_is_a_responder) {
  make_transceiver_interface(romea::RTLSTransceiverFunction::RESPONDER);
  EXPECT_TRUE(get_payload());
}

TEST_F(TestRTLSTransceiverInterface, get_payload_failed_when_transceiver_is_a_listener) {
  make_transceiver_interface(romea::RTLSTransceiverFunction::LISTENER);
  EXPECT_FALSE(get_payload());
}

TEST_F(
  TestRTLSTransceiverInterface,
  ranging_ok_when_transceiver_is_a_initiator_and_responder_id_is_1) {
  make_transceiver_interface(romea::RTLSTransceiverFunction::INITIATOR);
  EXPECT_TRUE(ranging(1));
}

TEST_F(
  TestRTLSTransceiverInterface,
  ranging_faile_when_transceiver_is_a_initiator_and_responder_id_is_0) {
  make_transceiver_interface(romea::RTLSTransceiverFunction::INITIATOR);
  EXPECT_FALSE(ranging(0));
}

TEST_F(
  TestRTLSTransceiverInterface,
  ranging_failed_when_transceiver_is_a_initiator_and_ranging_process_fail) {
  make_transceiver_interface(romea::RTLSTransceiverFunction::INITIATOR);
  EXPECT_TRUE(ranging(1));
}


TEST_F(TestRTLSTransceiverInterface, ranging_failed_when_transceiver_is_a_responder) {
  make_transceiver_interface(romea::RTLSTransceiverFunction::RESPONDER);
  EXPECT_FALSE(ranging(1));
}

TEST_F(TestRTLSTransceiverInterface, ranging_failed_when_transceiver_is_a_listener) {
  make_transceiver_interface(romea::RTLSTransceiverFunction::LISTENER);
  EXPECT_FALSE(ranging(1));
}


// TEST_F(TestRTLSTransceiverInterface, processPayloadRequest)
// {
//   make_transceiver_interface(romea::RTLSTransceiverFunction::INITIATOR);
//   romea_rtls_transceiver_msgs::msg::RangingRequest request;
//   request.responder_id = std::numeric_limits<uint16_t>::max();
//   EXPECT_FALSE(romea::process_request(transceiver, request).has_value());
// }

// TEST_F(TestRTLSTransceiverInterface, processRangingRequest)
// {
//   make_transceiver_interface(romea::RTLSTransceiverFunction::INITIATOR);
//   romea_rtls_transceiver_msgs::msg::RangingRequest request;
//   request.responder_id = 1;
//   EXPECT_TRUE(romea::process_request(transceiver, request).has_value());
// }


//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
