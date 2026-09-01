#ifndef BUMPERBOT_FIRMWARE__SERIAL_PROTOCOL_HPP_
#define BUMPERBOT_FIRMWARE__SERIAL_PROTOCOL_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>

namespace robot_lab_firmware
{

enum class WheelSide
{
  LEFT,
  RIGHT,
};

struct WheelVelocitySample
{
  WheelSide wheel;
  double velocity;
};

struct WheelVelocityFrame
{
  std::optional<double> left_velocity;
  std::optional<double> right_velocity;
  std::size_t malformed_tokens{0};
};

/// Parse one Arduino wheel token such as ``rp1.25`` or ``ln0.50``.
std::optional<WheelVelocitySample> parse_wheel_velocity_token(std::string_view token);

/// Parse a comma-separated encoder frame. Invalid tokens are counted and skipped.
WheelVelocityFrame parse_wheel_velocity_frame(std::string_view frame);

/// Encode right/left velocity commands using the Arduino's fixed-width wire format.
std::optional<std::string> format_wheel_command_frame(
  double right_velocity, double left_velocity);

}  // namespace robot_lab_firmware

#endif  // BUMPERBOT_FIRMWARE__SERIAL_PROTOCOL_HPP_
