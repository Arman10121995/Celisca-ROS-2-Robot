#include "robot_lab_firmware/serial_protocol.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace robot_lab_firmware
{
namespace
{

TEST(SerialProtocol, ParsesSignedWheelSamples)
{
  const auto right = parse_wheel_velocity_token("rp1.25");
  ASSERT_TRUE(right);
  EXPECT_EQ(right->wheel, WheelSide::RIGHT);
  EXPECT_DOUBLE_EQ(right->velocity, 1.25);

  const auto left = parse_wheel_velocity_token(" \tln0.50\r\n");
  ASSERT_TRUE(left);
  EXPECT_EQ(left->wheel, WheelSide::LEFT);
  EXPECT_DOUBLE_EQ(left->velocity, -0.5);
}

TEST(SerialProtocol, RejectsMalformedTokensWithoutThrowing)
{
  const std::vector<std::string> malformed{
    "", "r", "rp", "xp1.0", "rx1.0", "rp-1.0", "rp+1.0", "rp1e2", "rp1.0junk",
    "rpnan", "rpinf", "rp..1"};

  for (const auto & token : malformed)
  {
    EXPECT_NO_THROW({EXPECT_FALSE(parse_wheel_velocity_token(token));}) << token;
  }
}

TEST(SerialProtocol, KeepsValidSamplesWhenAFrameIsPartiallyMalformed)
{
  const auto frame = parse_wheel_velocity_frame("junk,rp1.50,ln2.25,bad,\r\n");

  ASSERT_TRUE(frame.right_velocity);
  ASSERT_TRUE(frame.left_velocity);
  EXPECT_DOUBLE_EQ(*frame.right_velocity, 1.5);
  EXPECT_DOUBLE_EQ(*frame.left_velocity, -2.25);
  EXPECT_EQ(frame.malformed_tokens, 2U);
}

TEST(SerialProtocol, LastValidSampleWinsForEachWheel)
{
  const auto frame = parse_wheel_velocity_frame("rp1.0,lp2.0,rn3.0,");

  ASSERT_TRUE(frame.right_velocity);
  ASSERT_TRUE(frame.left_velocity);
  EXPECT_DOUBLE_EQ(*frame.right_velocity, -3.0);
  EXPECT_DOUBLE_EQ(*frame.left_velocity, 2.0);
  EXPECT_EQ(frame.malformed_tokens, 0U);
}

TEST(SerialProtocol, FormatsRightThenLeftWithFixedWidthMagnitudes)
{
  const auto frame = format_wheel_command_frame(1.5, -2.25);
  ASSERT_TRUE(frame);
  EXPECT_EQ(*frame, "rp01.50,ln02.25,");

  const auto signed_zero = format_wheel_command_frame(-0.0, 0.0);
  ASSERT_TRUE(signed_zero);
  EXPECT_EQ(*signed_zero, "rn00.00,lp00.00,");
}

TEST(SerialProtocol, RejectsValuesTheFirmwareCannotRepresent)
{
  EXPECT_FALSE(format_wheel_command_frame(std::numeric_limits<double>::quiet_NaN(), 0.0));
  EXPECT_FALSE(format_wheel_command_frame(0.0, std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(format_wheel_command_frame(100.0, 0.0));
}

}  // namespace
}  // namespace robot_lab_firmware
