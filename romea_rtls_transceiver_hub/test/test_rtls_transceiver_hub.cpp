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

// romea
#include "romea_common_utils/qos.hpp"
#include "romea_common_utils/publishers/data_publisher.hpp"
#include "romea_common_utils/services/service_client_async.hpp"
#include "romea_rtls_transceiver_utils/transceiver_interface_server.hpp"

// local
#include "../test/test_helper.h"
#include "romea_rtls_transceiver_hub/rtls_transceiver_hub.hpp"
#include "romea_rtls_transceiver_msgs/srv/set_payload.hpp"

class Tag : public romea::TransceiverInterfaceServer
{
public:
  Tag(
    std::shared_ptr<rclcpp::Node> node,
    const romea::RTLSTransceiverEUID & transceiver_euid_,
    const romea::RTLSTransceiverFunction & transceiver_function)
  : TransceiverInterfaceServer(node, transceiver_euid_, transceiver_function)
  {
    init_get_payload_service_server_();
    init_set_payload_service_server_();
    init_range_action_server_();
  }

  virtual ~Tag() = default;

private:
  bool ranging_(const uint16_t & responder_id, RangingResult & range) override
  {
    // std::cout << " tag ranging" << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    range.range = static_cast<double>(transceiver_euid_.id * 65556 + responder_id);
    return true;
  }

  bool set_payload_(const Payload & payload) override
  {
    // std::cout << " tag set payload" << std::endl;
    payload_ = payload.data;
    return true;
  }

  bool get_payload_(Payload & payload) override
  {
    // std::cout << " tag get payload" << std::endl;
    payload.data = payload_;
    return true;
  }

private:
  std::vector<unsigned char> payload_;
};

class Master
{
public:
  using  SetTransceiversConfigurationService =
    romea_rtls_transceiver_msgs::srv::SetTransceiversConfiguration;
  using  SetTransceiversConfigurationServiceClient =
    romea::ServiceClientAsync<SetTransceiversConfigurationService>;

  using Poll = romea_rtls_transceiver_msgs::msg::Poll;
  using PollPublisher = rclcpp::Publisher<Poll>;

  using Range = romea_rtls_transceiver_msgs::msg::Range;
  using RangeSubscriber = rclcpp::Subscription<Range>;

public:
  explicit Master(std::shared_ptr<rclcpp::Node> node)
  : hub_config_srv_client_(nullptr),
    poll_pub_(nullptr),
    range_sub_(nullptr)
  {
    hub_config_srv_client_ = std::make_shared<SetTransceiversConfigurationServiceClient>(
      node, "set_external_transceivers_configuration", std::chrono::seconds(1));

    auto configuration = std::make_shared<SetTransceiversConfigurationService::Request>();
    configuration->transceivers_names = {"anchor0", "anchor1"};
    configuration->transceivers_ids = {255, 256};
    hub_config_srv_client_->send_request(configuration);

    poll_pub_ = node->create_publisher<Poll>("/poll", romea::sensor_data_qos());

    using namespace std::placeholders;
    auto callback = std::bind(&Master::range_callback_, this, _1);

    rclcpp::SubscriptionOptions options;
    options.callback_group = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    range_sub_ = node->create_subscription<Range>(
      "range", romea::best_effort(1), callback, options);
  }

  void poll(Poll poll)
  {
    poll_pub_->publish(poll);
  }

private:
  void range_callback_(Range::SharedPtr range_msg)
  {
    range = range_msg;
  }

public:
  std::shared_ptr<Range> range;

private:
  std::shared_ptr<SetTransceiversConfigurationServiceClient> hub_config_srv_client_;
  std::shared_ptr<PollPublisher> poll_pub_;
  std::shared_ptr<RangeSubscriber> range_sub_;
};

class TestRTLSTransceiverHub : public ::testing::Test
{
public:
  using Hub = romea::RTLSTransceiverHub;
  using Poll = romea_rtls_transceiver_msgs::msg::Poll;

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
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_thread_ = std::thread([this]() {this->executor_->spin();});

    node_master = std::make_shared<rclcpp::Node>("test_rtls_transceiver_master");
    node_tag0 = std::make_shared<rclcpp::Node>("test_rtls_transceiver_tag0", "tag0");
    node_tag1 = std::make_shared<rclcpp::Node>("test_rtls_transceiver_tag1", "tag1");

    executor_->add_node(node_tag0);
    tag0_ = std::make_shared<Tag>(
      node_tag0,
      romea::RTLSTransceiverEUID{0, 0},
      romea::RTLSTransceiverFunction::INITIATOR);

    executor_->add_node(node_tag1);
    tag1_ = std::make_shared<Tag>(
      node_tag1,
      romea::RTLSTransceiverEUID{0, 1},
      romea::RTLSTransceiverFunction::INITIATOR);

    rclcpp::NodeOptions no;
    no.arguments(
      {"--ros-args",
        "--params-file",
        std::string(TEST_DIR) + "/test_rtls_transceiver_hub.yaml"});

    hub_ = std::make_shared<Hub>(no);
    executor_->add_node(hub_->get_node_base_interface());

    executor_->add_node(node_master);
    master_ = std::make_shared<Master>(node_master);
  }


  void TearDown() override
  {
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
  }

  std::shared_ptr<rclcpp::Node> node_tag0;
  std::shared_ptr<rclcpp::Node> node_tag1;
  std::shared_ptr<rclcpp::Node> node_master;
  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;

  std::shared_ptr<Tag> tag0_;
  std::shared_ptr<Tag> tag1_;
  std::shared_ptr<Hub> hub_;
  std::shared_ptr<Master> master_;
};

TEST_F(TestRTLSTransceiverHub, check_poll_tag2_anchor_0) {
  Poll poll;
  poll.transceivers.initiator_name = "tag2";
  poll.transceivers.responder_name = "anchor0";
  master_->poll(poll);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(master_->range, nullptr);
}

TEST_F(TestRTLSTransceiverHub, check_poll_tag0_anchor_2) {
  Poll poll;
  poll.transceivers.initiator_name = "tag0";
  poll.transceivers.responder_name = "anchor2";
  master_->poll(poll);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(master_->range, nullptr);
}

TEST_F(TestRTLSTransceiverHub, check_poll_tag0_anchor_0) {
  Poll poll;
  poll.transceivers.initiator_name = "tag0";
  poll.transceivers.responder_name = "anchor0";
  master_->poll(poll);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_DOUBLE_EQ(master_->range->range.range, 255);
  EXPECT_EQ(master_->range->payload.data.size(), 0u);
}

TEST_F(TestRTLSTransceiverHub, check_poll_tag1_anchor_0) {
  Poll poll;
  poll.payload.data = {1, 2, 3, 4};
  poll.transceivers.initiator_name = "tag1";
  poll.transceivers.responder_name = "anchor0";
  master_->poll(poll);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_DOUBLE_EQ(master_->range->range.range, 65811);
  EXPECT_EQ(master_->range->payload.data.size(), 4u);
  EXPECT_EQ(master_->range->payload.data[0], 1);
  EXPECT_EQ(master_->range->payload.data[1], 2);
  EXPECT_EQ(master_->range->payload.data[2], 3);
  EXPECT_EQ(master_->range->payload.data[3], 4);
}

TEST_F(TestRTLSTransceiverHub, check_poll_tag0_anchor_1) {
  Poll poll;
  poll.transceivers.initiator_name = "tag0";
  poll.transceivers.responder_name = "anchor1";
  master_->poll(poll);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_DOUBLE_EQ(master_->range->range.range, 256);
  EXPECT_EQ(master_->range->payload.data.size(), 0u);
}

TEST_F(TestRTLSTransceiverHub, check_poll_tag1_anchor_1) {
  Poll poll;
  poll.payload.data = {1, 2, 3, 4};
  poll.transceivers.initiator_name = "tag1";
  poll.transceivers.responder_name = "anchor1";
  master_->poll(poll);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_DOUBLE_EQ(master_->range->range.range, 65812);
  EXPECT_EQ(master_->range->payload.data.size(), 4u);
  EXPECT_EQ(master_->range->payload.data[0], 1);
  EXPECT_EQ(master_->range->payload.data[1], 2);
  EXPECT_EQ(master_->range->payload.data[2], 3);
  EXPECT_EQ(master_->range->payload.data[3], 4);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
