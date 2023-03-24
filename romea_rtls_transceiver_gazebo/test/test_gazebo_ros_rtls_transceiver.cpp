// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

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
#include "romea_rtls_transceiver_utils/transceiver_interface_client.hpp"

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
    transceiver0_client = std::make_shared<romea::TransceiverInterfaceClient>(
      node_transceiver0_client, "transceiver0");

    node_transceiver2_client = std::make_shared<rclcpp::Node>("node_transceiver2_client");
    transceiver2_client = std::make_shared<romea::TransceiverInterfaceClient>(
      node_transceiver2_client, "transceiver2");

    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node_transceiver0_client);
    executor->add_node(node_transceiver2_client);
    executor_thread = std::thread([this]() {this->executor->spin();});
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


  std::shared_ptr<rclcpp::Node> node_transceiver0_client;
  std::shared_ptr<romea::TransceiverInterfaceClient> transceiver0_client;
  std::shared_ptr<rclcpp::Node> node_transceiver2_client;
  std::shared_ptr<romea::TransceiverInterfaceClient> transceiver2_client;

  std::shared_ptr<rclcpp::Executor> executor;
  std::thread executor_thread;
};

TEST_F(GazeboRosRTLSTransceiverTest, checkRangeWithNoPayloadExchange)
{
  romea::TransceiverInterfaceClient::RangingResult result;
  romea::TransceiverInterfaceClient::Payload transceiver0_payload;
  EXPECT_TRUE(transceiver0_client->ranging(1, result, std::chrono::seconds(1)));
  EXPECT_TRUE(transceiver0_client->get_payload(transceiver0_payload));
  EXPECT_EQ(transceiver0_payload.data.size(), 0u);
}

TEST_F(GazeboRosRTLSTransceiverTest, checkRangeWithPayloadExchange)
{
  romea::TransceiverInterfaceClient::Payload transceiver0_payload;
  romea::TransceiverInterfaceClient::Payload transceiver2_payload;

  transceiver0_payload.data = {1, 2, 3};
  transceiver2_payload.data = {4, 5, 6};

  EXPECT_TRUE(transceiver0_client->set_payload(transceiver0_payload));
  EXPECT_TRUE(transceiver2_client->set_payload(transceiver2_payload));

  romea::TransceiverInterfaceClient::RangingResult result;
  EXPECT_TRUE(transceiver0_client->ranging(2, result, std::chrono::seconds(1)));

  EXPECT_TRUE(transceiver0_client->get_payload(transceiver0_payload));
  EXPECT_TRUE(transceiver2_client->get_payload(transceiver2_payload));
  EXPECT_EQ(transceiver0_payload.data[0], 4);
  EXPECT_EQ(transceiver0_payload.data[1], 5);
  EXPECT_EQ(transceiver0_payload.data[2], 6);
  EXPECT_EQ(transceiver2_payload.data[0], 1);
  EXPECT_EQ(transceiver2_payload.data[1], 2);
  EXPECT_EQ(transceiver2_payload.data[2], 3);
}

TEST_F(GazeboRosRTLSTransceiverTest, checkRangeFailed)
{
  romea::TransceiverInterfaceClient::RangingResult result;
  EXPECT_FALSE(transceiver2_client->ranging(1, result, std::chrono::seconds(1)));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
