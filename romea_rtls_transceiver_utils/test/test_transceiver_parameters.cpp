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
#include <memory>

// gtest
#include "gtest/gtest.h"

// local
#include "romea_rtls_transceiver_utils/rtls_transceiver_parameters.hpp"

#include "../test/test_helper.h"

class TestTransceiverParameters : public ::testing::Test
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
    rclcpp::NodeOptions no;

    no.arguments(
      {"--ros-args",
        "--params-file",
        std::string(TEST_DIR) + "/test_transceiver_parameters.yaml"});

    node = std::make_shared<rclcpp::Node>("test_transceiver_parameters", "ns", no);
  }

  std::shared_ptr<rclcpp::Node> node;
};

TEST_F(TestTransceiverParameters, getId) {
  romea::declare_transceiver_id(node);
  EXPECT_EQ(romea::get_transceiver_id(node), 0);
}

TEST_F(TestTransceiverParameters, getPanId) {
  romea::declare_transceiver_pan_id(node);
  EXPECT_EQ(romea::get_transceiver_pan_id(node), 1);
}

TEST_F(TestTransceiverParameters, getCommunicationConfiguration) {
  romea::declare_transceiver_communication_configuration(node);
  EXPECT_STREQ(romea::get_transceiver_communication_configuration(node).c_str(), "foo");
}


//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
