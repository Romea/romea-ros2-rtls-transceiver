// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license


// std
#include <limits>
#include <string>
#include <memory>
#include <vector>
#include <thread>

// gtest
#include "gtest/gtest.h"

// local
#include "romea_rtls_transceiver_utils/transceiver_interface_client.hpp"
#include "romea_rtls_transceiver_utils/transceiver_interface_server.hpp"

#define TESTING

class MocTransceiverInterfaceServer : public romea::TransceiverInterfaceServer
{
public:
  MocTransceiverInterfaceServer(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & transceiver_name,
    const romea::RTLSTransceiverFunction & transceiver_function)
  : TransceiverInterfaceServer(node, transceiver_name, {0, 0}, transceiver_function),
    ranging_success_(true),
    get_payload_success_(true),
    set_payload_success_(true)
  {
    init_get_payload_service_server_(node, transceiver_name);
    init_set_payload_service_server_(node, transceiver_name);
    init_range_action_server_(node, transceiver_name);
  }

  void ranging_will_failed() {ranging_success_ = false;}
  void set_payload_will_failed() {set_payload_success_ = false;}
  void get_payload_will_failed() {get_payload_success_ = false;}

  virtual ~MocTransceiverInterfaceServer() = default;

private:
  bool ranging_(const uint16_t & responder_id, RangingResult & range) override
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (ranging_success_) {
      range.range = 1.0;
    } else {
      range.range = std::numeric_limits<double>::quiet_NaN();
    }

    return ranging_success_;
  }

  bool set_payload_(const Payload & payload) override
  {
    if (set_payload_success_) {
      payload_ = payload.data;
    }
    return set_payload_success_;
  }

  bool get_payload_(Payload & payload) override
  {
    if (get_payload_success_) {
      payload.data = payload_;
    }
    return get_payload_success_;
  }

private:
  bool ranging_success_;
  bool get_payload_success_;
  bool set_payload_success_;
  std::vector<unsigned char> payload_;
};

class TestRTLSTransceiverInterfaces : public ::testing::Test
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
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    node_client_ = std::make_shared<rclcpp::Node>("test_rtls_transceiver_interfaces_client");
    node_server_ = std::make_shared<rclcpp::Node>("test_rtls_transceiver_interfaces_server");
    executor_->add_node(node_client_);
    executor_->add_node(node_server_);
    executor_thread_ = std::thread([this]() {this->executor_->spin();});
  }

  void TearDown() override
  {
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
  }

  void make_client()
  {
    client_input_payload_.data = {1, 2, 3};
    client_ = std::make_unique<romea::TransceiverInterfaceClient>(node_client_, "tag");
  }

  void make_server(
    const romea::RTLSTransceiverFunction & transceiver_function =
    romea::RTLSTransceiverFunction::INITIATOR)
  {
    server_ = std::make_unique<MocTransceiverInterfaceServer>(
      node_server_, "tag", transceiver_function);
  }

  std::shared_ptr<rclcpp::Node> node_client_;
  std::shared_ptr<rclcpp::Node> node_server_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;

  std::unique_ptr<romea::TransceiverInterfaceClient> client_;
  std::unique_ptr<MocTransceiverInterfaceServer> server_;
  romea_rtls_transceiver_msgs::msg::Payload client_input_payload_;
  romea_rtls_transceiver_msgs::msg::Payload client_output_payload_;
};

TEST_F(TestRTLSTransceiverInterfaces, set_get_payload_ok) {
  make_server();
  make_client();

  EXPECT_TRUE(client_->set_payload(client_input_payload_));
  EXPECT_TRUE(client_->get_payload(client_output_payload_));
  EXPECT_EQ(client_input_payload_.data.size(), client_output_payload_.data.size());
}


TEST_F(TestRTLSTransceiverInterfaces, check_get_payload_failed_because_server_is_down) {
  make_server();
  make_client();
  server_.reset();
  executor_->remove_node(node_server_);
  node_server_.reset();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  EXPECT_FALSE(client_->get_payload(client_output_payload_));
}

TEST_F(TestRTLSTransceiverInterfaces, check_set_payload_failed_because_server_is_down) {
  make_server();
  make_client();
  server_.reset();
  executor_->remove_node(node_server_);
  node_server_.reset();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  EXPECT_FALSE(client_->set_payload(client_input_payload_));
}

TEST_F(TestRTLSTransceiverInterfaces, check_get_payload_not_succeded) {
  make_server();
  make_client();
  server_->get_payload_will_failed();
  EXPECT_FALSE(client_->get_payload(client_output_payload_));
}

TEST_F(TestRTLSTransceiverInterfaces, check_set_payload_not_succeded) {
  make_server();
  make_client();
  server_->set_payload_will_failed();
  EXPECT_FALSE(client_->set_payload(client_input_payload_));
}

TEST_F(TestRTLSTransceiverInterfaces, check_get_payload_not_succeded_because_tag_is_a_listener) {
  make_server(romea::RTLSTransceiverFunction::LISTENER);
  make_client();
  EXPECT_FALSE(client_->get_payload(client_output_payload_));
}

TEST_F(TestRTLSTransceiverInterfaces, check_get_payload_not_succeded_because_tag_is_a_responder) {
  make_server(romea::RTLSTransceiverFunction::RESPONDER);
  make_client();
  EXPECT_FALSE(client_->get_payload(client_output_payload_));
}

TEST_F(TestRTLSTransceiverInterfaces, check_set_payload_not_succeded_because_tag_is_a_listener) {
  make_server(romea::RTLSTransceiverFunction::LISTENER);
  make_client();
  EXPECT_FALSE(client_->set_payload(client_input_payload_));
}

TEST_F(TestRTLSTransceiverInterfaces, check_ranging_succeded) {
  make_server();
  make_client();
  romea_rtls_transceiver_msgs::msg::RangingResult result;
  EXPECT_TRUE(client_->ranging(1, result, std::chrono::seconds(1)));
  EXPECT_DOUBLE_EQ(result.range, 1.0);
}

TEST_F(TestRTLSTransceiverInterfaces, check_ranging_not_succeded) {
  make_server();
  make_client();
  server_->ranging_will_failed();
  romea_rtls_transceiver_msgs::msg::RangingResult result;
  EXPECT_FALSE(client_->ranging(1, result, std::chrono::seconds(1)));
}

TEST_F(TestRTLSTransceiverInterfaces, check_ranging_not_succeded_because_tag_is_a_responder) {
  make_server(romea::RTLSTransceiverFunction::RESPONDER);
  make_client();
  romea_rtls_transceiver_msgs::msg::RangingResult result;
  EXPECT_FALSE(client_->ranging(1, result, std::chrono::seconds(1)));
}

TEST_F(TestRTLSTransceiverInterfaces, check_ranging_not_succeded_because_tag_is_a_listener) {
  make_server(romea::RTLSTransceiverFunction::LISTENER);
  make_client();
  romea_rtls_transceiver_msgs::msg::RangingResult result;
  EXPECT_FALSE(client_->ranging(1, result, std::chrono::seconds(1)));
}

TEST_F(
  TestRTLSTransceiverInterfaces,
  check_ranging_not_succeded_because_tag_ranging_with_itself) {
  make_server();
  make_client();
  romea_rtls_transceiver_msgs::msg::RangingResult result;
  EXPECT_FALSE(client_->ranging(0, result, std::chrono::seconds(1)));
}

TEST_F(TestRTLSTransceiverInterfaces, check_ranging_not_succeded_because_of_timeout) {
  make_server();
  make_client();
  romea_rtls_transceiver_msgs::msg::RangingResult result;
  EXPECT_FALSE(client_->ranging(1, result, std::chrono::milliseconds(20)));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  // size_t n = 0;
  // while (n != 20) {
  //   rclcpp::spin_some(node->get_node_base_interface());
  //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
  //   n++;
  // }
}

TEST_F(TestRTLSTransceiverInterfaces, check_ranging_not_succeded_because_ranging_is_pending) {
  make_server();
  make_client();
  romea_rtls_transceiver_msgs::msg::RangingResult result;
  EXPECT_FALSE(client_->ranging(1, result, std::chrono::milliseconds(20)));
  EXPECT_FALSE(client_->ranging(1, result, std::chrono::milliseconds(20)));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // size_t n = 0;
  // while (n != 20) {
  //   rclcpp::spin_some(node->get_node_base_interface());
  //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
  //   n++;
  // }
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
