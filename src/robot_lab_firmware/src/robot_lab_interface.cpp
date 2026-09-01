#include "robot_lab_firmware/robot_lab_interface.hpp"
#include "robot_lab_firmware/serial_protocol.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>
#include <unordered_set>

namespace robot_lab_firmware
{
BumperbotInterface::BumperbotInterface()
{
}


BumperbotInterface::~BumperbotInterface()
{
  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}


CallbackReturn BumperbotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  const CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  const auto port_parameter = info_.hardware_parameters.find("port");
  if (port_parameter == info_.hardware_parameters.end() || port_parameter->second.empty())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("BumperbotInterface"),
      "A non-empty 'port' hardware parameter is required");
    return CallbackReturn::FAILURE;
  }
  port_ = port_parameter->second;

  constexpr std::size_t expected_joint_count = 2;
  if (info_.joints.size() != expected_joint_count)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("BumperbotInterface"),
      "Expected exactly %zu wheel joints, received %zu", expected_joint_count,
      info_.joints.size());
    return CallbackReturn::FAILURE;
  }

  bool found_left_wheel = false;
  bool found_right_wheel = false;
  for (std::size_t index = 0; index < info_.joints.size(); ++index)
  {
    const auto & joint = info_.joints[index];
    if (joint.name == "wheel_left_joint" && !found_left_wheel)
    {
      left_wheel_index_ = index;
      found_left_wheel = true;
    }
    else if (joint.name == "wheel_right_joint" && !found_right_wheel)
    {
      right_wheel_index_ = index;
      found_right_wheel = true;
    }
    else
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("BumperbotInterface"),
        "Unexpected or duplicate joint '%s'; expected wheel_left_joint and wheel_right_joint",
        joint.name.c_str());
      return CallbackReturn::FAILURE;
    }

    if (
      joint.command_interfaces.size() != 1 ||
      joint.command_interfaces.front().name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("BumperbotInterface"),
        "Joint '%s' must expose exactly one velocity command interface", joint.name.c_str());
      return CallbackReturn::FAILURE;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("BumperbotInterface"),
        "Joint '%s' must expose exactly position and velocity state interfaces",
        joint.name.c_str());
      return CallbackReturn::FAILURE;
    }

    std::unordered_set<std::string> state_interface_names;
    for (const auto & state_interface : joint.state_interfaces)
    {
      state_interface_names.insert(state_interface.name);
    }
    if (
      state_interface_names.size() != 2 ||
      state_interface_names.count(hardware_interface::HW_IF_POSITION) != 1 ||
      state_interface_names.count(hardware_interface::HW_IF_VELOCITY) != 1)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("BumperbotInterface"),
        "Joint '%s' must expose exactly position and velocity state interfaces",
        joint.name.c_str());
      return CallbackReturn::FAILURE;
    }
  }

  if (!found_left_wheel || !found_right_wheel)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("BumperbotInterface"),
      "Both wheel_left_joint and wheel_right_joint are required");
    return CallbackReturn::FAILURE;
  }

  // The exported interfaces store pointers into these vectors. Size them once here and
  // subsequently mutate their elements without replacing the backing storage.
  velocity_commands_.assign(info_.joints.size(), 0.0);
  position_states_.assign(info_.joints.size(), 0.0);
  velocity_states_.assign(info_.joints.size(), 0.0);

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> BumperbotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() * 2);

  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> BumperbotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}


CallbackReturn BumperbotInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Starting robot hardware ...");

  // Reset commands and states
  std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
  std::fill(position_states_.begin(), position_states_.end(), 0.0);
  std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);

  try
  {
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn BumperbotInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Stopping robot hardware ...");

  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type BumperbotInterface::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double period_seconds = period.seconds();
  if (!std::isfinite(period_seconds) || period_seconds < 0.0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("BumperbotInterface"), "Invalid read period: %.9f seconds",
      period_seconds);
    return hardware_interface::return_type::ERROR;
  }

  try
  {
    if (arduino_.IsDataAvailable())
    {
      std::string message;
      arduino_.ReadLine(message);
      const WheelVelocityFrame frame = parse_wheel_velocity_frame(message);
      if (frame.right_velocity)
      {
        velocity_states_[right_wheel_index_] = *frame.right_velocity;
      }
      if (frame.left_velocity)
      {
        velocity_states_[left_wheel_index_] = *frame.left_velocity;
      }
      if (frame.malformed_tokens > 0)
      {
        RCLCPP_WARN(
          rclcpp::get_logger("BumperbotInterface"),
          "Ignored %zu malformed token(s) in an encoder frame", frame.malformed_tokens);
      }
    }
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("BumperbotInterface"), "Failed to read from serial port '%s': %s",
      port_.c_str(), exception.what());
    return hardware_interface::return_type::ERROR;
  }
  catch (...)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("BumperbotInterface"),
      "Failed to read from serial port '%s': unknown error", port_.c_str());
    return hardware_interface::return_type::ERROR;
  }

  // ros2_control supplies the measured loop duration. Using it keeps integration deterministic
  // under simulated time and avoids a second, unrelated wall-clock measurement.
  for (std::size_t index = 0; index < position_states_.size(); ++index)
  {
    position_states_[index] += velocity_states_[index] * period_seconds;
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type BumperbotInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto message = format_wheel_command_frame(
    velocity_commands_[right_wheel_index_], velocity_commands_[left_wheel_index_]);
  if (!message)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("BumperbotInterface"),
      "Wheel command is non-finite or outside the serial protocol range");
    return hardware_interface::return_type::ERROR;
  }

  try
  {
    arduino_.Write(*message);
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("BumperbotInterface"),
      "Failed to send wheel command '%s' to serial port '%s': %s", message->c_str(),
      port_.c_str(), exception.what());
    return hardware_interface::return_type::ERROR;
  }
  catch (...)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("BumperbotInterface"),
      "Failed to send wheel command '%s' to serial port '%s': unknown error",
      message->c_str(), port_.c_str());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}
}  // namespace robot_lab_firmware

PLUGINLIB_EXPORT_CLASS(robot_lab_firmware::BumperbotInterface, hardware_interface::SystemInterface)
