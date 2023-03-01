// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license


// std
#include <string>

// gtest
#include "gtest/gtest.h"

// local
#include "romea_gps_utils/gps_data_conversions.hpp"


//-----------------------------------------------------------------------------
TEST(TestGPSDataConversions, to_nmea_msg)
{
  nmea_msgs::msg::Sentence msg;
  std::string frame_id = "foo";
  std::string sentence = "bar";
  rclcpp::Time stamp(1, 2);

  romea::to_ros_msg(stamp, frame_id, sentence, msg);

  EXPECT_STREQ(msg.sentence.c_str(), sentence.c_str());
  EXPECT_STREQ(msg.header.frame_id.c_str(), frame_id.c_str());
  EXPECT_EQ(msg.header.stamp.sec, 1);
  EXPECT_EQ(msg.header.stamp.nanosec, 2u);
}

//-----------------------------------------------------------------------------
TEST(TestGPSDataConversions, to_navsat_fix_msg)
{
  sensor_msgs::msg::NavSatFix msg;
  rclcpp::Time stamp(1, 2);
  std::string frame_id = "foo";
  romea::GGAFrame frame;
  frame.latitude = romea::Latitude(45 / 180. * M_PI);
  frame.longitude = romea::Longitude(3 / 180. * M_PI);
  frame.altitudeAboveGeoid = 50.0;
  frame.geoidHeight = 350.0;
  frame.horizontalDilutionOfPrecision = 2.0;
  frame.talkerId = romea::TalkerId::GP;
  frame.fixQuality = romea::FixQuality::RTK_FIX;


  romea::to_ros_msg(stamp, frame_id, frame, msg);

  EXPECT_DOUBLE_EQ(msg.latitude, 45.);
  EXPECT_DOUBLE_EQ(msg.longitude, 3.);
  EXPECT_DOUBLE_EQ(msg.altitude, 400.0);

  EXPECT_EQ(
    msg.position_covariance_type,
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED);
  EXPECT_DOUBLE_EQ(msg.position_covariance[0], 0.0036);
  EXPECT_DOUBLE_EQ(msg.position_covariance[0], 0.0036);

  EXPECT_EQ(msg.status.service, sensor_msgs::msg::NavSatStatus::SERVICE_GPS);
  EXPECT_EQ(msg.status.status, sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX);

  EXPECT_STREQ(msg.header.frame_id.c_str(), frame_id.c_str());
  EXPECT_EQ(msg.header.stamp.sec, 1);
  EXPECT_EQ(msg.header.stamp.nanosec, 2u);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
