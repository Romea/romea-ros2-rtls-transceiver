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


using Payload = romea_rtls_transceiver_msgs::msg::Payload;
using RangingRequest = romea_rtls_transceiver_msgs::msg::RangingRequest;
using RangingResult = romea_rtls_transceiver_msgs::msg::RangingResult;

class TestRTLSTransceiverServerClientInterfaces : public ::testing::Test
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
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    node_client = std::make_shared<rclcpp::Node>("client");
    node_server = std::make_shared<rclcpp::Node>("server", "tag");
    executor->add_node(node_client);
    executor->add_node(node_server);
    executor_thread = std::thread([this]() {this->executor->spin();});
  }

  void TearDown() override
  {
    executor->cancel();
    if (executor_thread.joinable()) {
      executor_thread.join();
    }
  }

  void make_client(
    std::function<void(RangingResult::ConstSharedPtr)> result_callback = {},
    std::function<void(Payload::ConstSharedPtr)> payload_callback = {})
  {
    client = std::make_shared<romea::RTLSTransceiverInterfaceClient>(
      node_client, "tag", result_callback, payload_callback);
  }

  void make_server(std::function<void(RangingRequest::ConstSharedPtr)> request_callback = {})
  {
    server = std::make_shared<romea::RTLSTransceiverInterfaceServer>(node_server, request_callback);
  }


  std::shared_ptr<rclcpp::Node> node_client;
  std::shared_ptr<rclcpp::Node> node_server;

  rclcpp::Executor::SharedPtr executor;
  std::thread executor_thread;

  std::shared_ptr<romea::RTLSTransceiverInterfaceClient> client;
  std::shared_ptr<romea::RTLSTransceiverInterfaceServer> server;
};


TEST_F(TestRTLSTransceiverServerClientInterfaces, test_request_topic_not_connected) {
  make_server();
  make_client();

  EXPECT_EQ(node_client->count_publishers("/tag/request"), 1u);
  EXPECT_EQ(node_client->count_subscribers("/tag/range"), 0u);
  EXPECT_EQ(node_client->count_subscribers("/tag/payload"), 0u);

  EXPECT_EQ(node_server->count_subscribers("/tag/request"), 0u);
  EXPECT_EQ(node_server->count_publishers("/tag/range"), 1u);
  EXPECT_EQ(node_server->count_publishers("/tag/payload"), 1u);
}

TEST_F(TestRTLSTransceiverServerClientInterfaces, test_request_topic_connected) {
  make_server([](RangingRequest::ConstSharedPtr) {});
  make_client();

  EXPECT_EQ(node_client->count_publishers("/tag/request"), 1u);
  EXPECT_EQ(node_client->count_subscribers("/tag/range"), 0u);
  EXPECT_EQ(node_client->count_subscribers("/tag/payload"), 0u);

  EXPECT_EQ(node_server->count_subscribers("/tag/request"), 1u);
  EXPECT_EQ(node_server->count_publishers("/tag/range"), 1u);
  EXPECT_EQ(node_server->count_publishers("/tag/payload"), 1u);
}

TEST_F(TestRTLSTransceiverServerClientInterfaces, test_range_topics_connected) {
  make_server();
  make_client([](RangingResult::ConstSharedPtr) {}, {});

  EXPECT_EQ(node_client->count_publishers("/tag/request"), 1u);
  EXPECT_EQ(node_client->count_subscribers("/tag/range"), 1u);
  EXPECT_EQ(node_client->count_subscribers("/tag/payload"), 0u);

  EXPECT_EQ(node_server->count_subscribers("/tag/request"), 0u);
  EXPECT_EQ(node_server->count_publishers("/tag/range"), 1u);
  EXPECT_EQ(node_server->count_publishers("/tag/payload"), 1u);
}

TEST_F(TestRTLSTransceiverServerClientInterfaces, test_payload_topics_connected) {
  make_server();
  make_client({}, [](Payload::ConstSharedPtr) {});

  EXPECT_EQ(node_client->count_publishers("/tag/request"), 1u);
  EXPECT_EQ(node_client->count_subscribers("/tag/range"), 0u);
  EXPECT_EQ(node_client->count_subscribers("/tag/payload"), 1u);

  EXPECT_EQ(node_server->count_subscribers("/tag/request"), 0u);
  EXPECT_EQ(node_server->count_publishers("/tag/range"), 1u);
  EXPECT_EQ(node_server->count_publishers("/tag/payload"), 1u);
}

TEST_F(TestRTLSTransceiverServerClientInterfaces, test_all_topics_connected) {
  make_server([](RangingRequest::ConstSharedPtr) {});
  make_client([](RangingResult::ConstSharedPtr) {}, [](Payload::ConstSharedPtr) {});

  EXPECT_EQ(node_client->count_publishers("/tag/request"), 1u);
  EXPECT_EQ(node_client->count_subscribers("/tag/range"), 1u);
  EXPECT_EQ(node_client->count_subscribers("/tag/payload"), 1u);

  EXPECT_EQ(node_server->count_subscribers("/tag/request"), 1u);
  EXPECT_EQ(node_server->count_publishers("/tag/range"), 1u);
  EXPECT_EQ(node_server->count_publishers("/tag/payload"), 1u);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
