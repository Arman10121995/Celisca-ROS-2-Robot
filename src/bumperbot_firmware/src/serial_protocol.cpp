#include "bumperbot_firmware/serial_protocol.hpp"

#include <charconv>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <system_error>

namespace bumperbot_firmware
{
namespace
{

std::string_view trim_ascii_whitespace(std::string_view value)
{
  constexpr std::string_view whitespace{" \t\r\n"};
  const auto first = value.find_first_not_of(whitespace);
  if (first == std::string_view::npos)
  {
    return {};
  }

  const auto last = value.find_last_not_of(whitespace);
  return value.substr(first, last - first + 1);
}

bool has_decimal_magnitude_grammar(std::string_view value)
{
  if (value.empty())
  {
    return false;
  }

  bool saw_digit = false;
  bool saw_decimal_point = false;
  for (const char character : value)
  {
    if (character >= '0' && character <= '9')
    {
      saw_digit = true;
      continue;
    }

    if (character == '.' && !saw_decimal_point)
    {
      saw_decimal_point = true;
      continue;
    }

    return false;
  }

  return saw_digit;
}

std::string format_velocity_token(char wheel, double velocity)
{
  std::ostringstream token;
  token << wheel << (std::signbit(velocity) ? 'n' : 'p') << std::fixed << std::setfill('0') <<
    std::setw(5) << std::setprecision(2) << std::abs(velocity);
  return token.str();
}

}  // namespace

std::optional<WheelVelocitySample> parse_wheel_velocity_token(std::string_view token)
{
  token = trim_ascii_whitespace(token);
  if (token.size() < 3)
  {
    return std::nullopt;
  }

  WheelSide wheel;
  if (token[0] == 'l')
  {
    wheel = WheelSide::LEFT;
  }
  else if (token[0] == 'r')
  {
    wheel = WheelSide::RIGHT;
  }
  else
  {
    return std::nullopt;
  }

  double multiplier = 0.0;
  if (token[1] == 'p')
  {
    multiplier = 1.0;
  }
  else if (token[1] == 'n')
  {
    multiplier = -1.0;
  }
  else
  {
    return std::nullopt;
  }

  const std::string_view magnitude_text = token.substr(2);
  if (!has_decimal_magnitude_grammar(magnitude_text))
  {
    return std::nullopt;
  }

  double magnitude = 0.0;
  const auto result = std::from_chars(
    magnitude_text.data(), magnitude_text.data() + magnitude_text.size(), magnitude,
    std::chars_format::fixed);
  if (
    result.ec != std::errc{} || result.ptr != magnitude_text.data() + magnitude_text.size() ||
    !std::isfinite(magnitude))
  {
    return std::nullopt;
  }

  return WheelVelocitySample{wheel, multiplier * magnitude};
}

WheelVelocityFrame parse_wheel_velocity_frame(std::string_view frame)
{
  WheelVelocityFrame parsed;
  std::size_t token_start = 0;

  while (token_start <= frame.size())
  {
    const std::size_t separator = frame.find(',', token_start);
    const std::size_t token_end = separator == std::string_view::npos ? frame.size() : separator;
    const std::string_view token = frame.substr(token_start, token_end - token_start);

    if (!trim_ascii_whitespace(token).empty())
    {
      const auto sample = parse_wheel_velocity_token(token);
      if (!sample)
      {
        ++parsed.malformed_tokens;
      }
      else if (sample->wheel == WheelSide::LEFT)
      {
        parsed.left_velocity = sample->velocity;
      }
      else
      {
        parsed.right_velocity = sample->velocity;
      }
    }

    if (separator == std::string_view::npos)
    {
      break;
    }
    token_start = separator + 1;
  }

  return parsed;
}

std::optional<std::string> format_wheel_command_frame(
  double right_velocity, double left_velocity)
{
  // The firmware reserves five characters for each unsigned magnitude ("00.00").
  constexpr double max_wire_magnitude = 99.99;
  if (
    !std::isfinite(right_velocity) || !std::isfinite(left_velocity) ||
    std::abs(right_velocity) > max_wire_magnitude ||
    std::abs(left_velocity) > max_wire_magnitude)
  {
    return std::nullopt;
  }

  return format_velocity_token('r', right_velocity) + "," +
         format_velocity_token('l', left_velocity) + ",";
}

}  // namespace bumperbot_firmware
