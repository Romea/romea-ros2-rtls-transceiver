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

// gazebo
#include "gazebo/test/ServerFixture.hh"
#include "gazebo_ros/node.hpp"
#include "gazebo_ros/testing_utils.hpp"

// ros
#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_rtls_transceiver_utils/rtls_transceiver_interface_client.hpp"

// local
#include "../test/test_helper.h"


/// Tests the gazebo_ros_gps_sensor plugin
class GazeboRosRTLSTransceiverTest : public gazebo::ServerFixture
{
public:
  void SetUp() override
  {
    auto world_name = std::string(TEST_DIR) +
      std::string("/test_gazebo_ros_rtls_transceiver.world");

    this->Load(world_name, true);

    node_transceiver0_client = std::make_shared<rclcpp::Node>("node_transceiver0_client");

    auto node_transceiver0_range_callback = std::bind(
      &GazeboRosRTLSTransceiverTest::transceiver0_range_callback, this, std::placeholders::_1);
    auto node_transceiver0_payload_callback = std::bind(
      &GazeboRosRTLSTransceiverTest::transceiver0_payload_callback, this, std::placeholders::_1);

    transceiver0_client = std::make_shared<romea::RTLSTransceiverInterfaceClient>(
      node_transceiver0_client, "transceiver0",
      node_transceiver0_range_callback,
      node_transceiver0_payload_callback);

    node_transceiver2_client = std::make_shared<rclcpp::Node>("node_transceiver2_client");

    auto node_transceiver2_range_callback = std::bind(
      &GazeboRosRTLSTransceiverTest::transceiver2_range_callback, this, std::placeholders::_1);
    auto node_transceiver2_payload_callback = std::bind(
      &GazeboRosRTLSTransceiverTest::transceiver2_payload_callback, this, std::placeholders::_1);

    transceiver2_client = std::make_shared<romea::RTLSTransceiverInterfaceClient>(
      node_transceiver2_client, "transceiver2",
      node_transceiver2_range_callback,
      node_transceiver2_payload_callback);

    range = 0;
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node_transceiver0_client);
    executor->add_node(node_transceiver2_client);
    executor_thread = std::thread([this]() {this->executor->spin();});

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  void TearDown() override
  {
    executor->remove_node(node_transceiver0_client);
    executor->remove_node(node_transceiver2_client);
    executor->cancel();
    if (executor_thread.joinable()) {
      executor_thread.join();
    }

    this->Unload();
  }

  void transceiver0_range_callback(
    romea::RTLSTransceiverInterfaceClient::RangingResult::ConstSharedPtr msg)
  {
    range = msg->range.range;
  }

  void transceiver2_range_callback(
    romea::RTLSTransceiverInterfaceClient::RangingResult::ConstSharedPtr msg)
  {
    range = msg->range.range;
  }

  void transceiver0_payload_callback(
    romea::RTLSTransceiverInterfaceClient::Payload::ConstSharedPtr msg)
  {
    sended_transceiver0_payload = msg->data;
  }

  void transceiver2_payload_callback(
    romea::RTLSTransceiverInterfaceClient::Payload::ConstSharedPtr msg)
  {
    sended_transceiver2_payload = msg->data;
  }

  std::shared_ptr<rclcpp::Node> node_transceiver0_client;
  std::shared_ptr<romea::RTLSTransceiverInterfaceClient> transceiver0_client;
  std::vector<uint8_t> sended_transceiver0_payload;

  std::shared_ptr<rclcpp::Node> node_transceiver2_client;
  std::shared_ptr<romea::RTLSTransceiverInterfaceClient> transceiver2_client;
  std::vector<uint8_t> sended_transceiver2_payload;

  double range;

  std::shared_ptr<rclcpp::Executor> executor;
  std::thread executor_thread;
};

TEST_F(GazeboRosRTLSTransceiverTest, checkRangeWithNoPayloadExchange)
{
  romea::RTLSTransceiverInterfaceClient::RangingRequest request;
  request.responder_id = 1;
  request.timeout = 0.1;
  transceiver0_client->send_ranging_request(request);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  EXPECT_NEAR(range, 10.0, 0.01);
  EXPECT_EQ(sended_transceiver0_payload.size(), 0u);
  EXPECT_EQ(sended_transceiver2_payload.size(), 0u);
}

TEST_F(GazeboRosRTLSTransceiverTest, checkRangeWithPayloadExchange)
{
  romea::RTLSTransceiverInterfaceClient::RangingRequest transceiver0_request;
  transceiver0_request.responder_id = 2;
  transceiver0_request.timeout = 1.;
  transceiver0_request.payload.data = {1, 2, 3};

  romea::RTLSTransceiverInterfaceClient::Payload transceiver2_payload;
  transceiver2_payload.data = {4, 5, 6};

  transceiver2_client->send_payload(transceiver2_payload);
  transceiver0_client->send_ranging_request(transceiver0_request);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  EXPECT_NEAR(range, 23.0, 0.01);
  EXPECT_EQ(sended_transceiver0_payload[0], 4);
  EXPECT_EQ(sended_transceiver0_payload[1], 5);
  EXPECT_EQ(sended_transceiver0_payload[2], 6);
  EXPECT_EQ(sended_transceiver2_payload[0], 1);
  EXPECT_EQ(sended_transceiver2_payload[1], 2);
  EXPECT_EQ(sended_transceiver2_payload[2], 3);
}

TEST_F(GazeboRosRTLSTransceiverTest, checkRangeFaileWhenTransceiverAreToFar)
{
  romea::RTLSTransceiverInterfaceClient::RangingRequest request;
  request.responder_id = 1;
  request.timeout = 1.;
  request.payload.data = {1, 2, 3};

  transceiver2_client->send_ranging_request(request);
  EXPECT_NEAR(range, 0.0, 0.01);
}

TEST_F(GazeboRosRTLSTransceiverTest, checkNoRangeWhenSendPayload)
{
  romea::RTLSTransceiverInterfaceClient::Payload payload;
  payload.data = {1, 2, 3};

  transceiver2_client->send_payload(payload);
  EXPECT_NEAR(range, 0.0, 0.01);
  EXPECT_EQ(sended_transceiver0_payload.size(), 0u);
  EXPECT_EQ(sended_transceiver2_payload.size(), 0u);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
