#include "bumperbot_firmware/bumperbot_interface.hpp"

#include <gtest/gtest.h>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace bumperbot_firmware
{
namespace
{

hardware_interface::InterfaceInfo interface_named(const std::string & name)
{
  hardware_interface::InterfaceInfo interface;
  interface.name = name;
  return interface;
}

hardware_interface::ComponentInfo wheel_joint(const std::string & name)
{
  hardware_interface::ComponentInfo joint;
  joint.name = name;
  joint.type = "joint";
  joint.command_interfaces = {interface_named(hardware_interface::HW_IF_VELOCITY)};
  joint.state_interfaces = {
    interface_named(hardware_interface::HW_IF_POSITION),
    interface_named(hardware_interface::HW_IF_VELOCITY)};
  return joint;
}

hardware_interface::HardwareInfo valid_hardware_info(bool reverse_joint_order = false)
{
  hardware_interface::HardwareInfo info;
  info.name = "RobotSystem";
  info.type = "system";
  info.hardware_class_type = "bumperbot_firmware/BumperbotInterface";
  info.hardware_parameters["port"] = "/dev/null";
  info.joints = {
    wheel_joint("wheel_left_joint"), wheel_joint("wheel_right_joint")};
  if (reverse_joint_order)
  {
    std::reverse(info.joints.begin(), info.joints.end());
  }
  return info;
}

TEST(BumperbotInterface, SizesStorageBeforeExportingInterfaces)
{
  BumperbotInterface hardware;
  ASSERT_EQ(hardware.on_init(valid_hardware_info()), CallbackReturn::SUCCESS);

  auto states = hardware.export_state_interfaces();
  auto commands = hardware.export_command_interfaces();

  ASSERT_EQ(states.size(), 4U);
  ASSERT_EQ(commands.size(), 2U);
  EXPECT_EQ(states[0].get_name(), "wheel_left_joint/position");
  EXPECT_EQ(states[1].get_name(), "wheel_left_joint/velocity");
  EXPECT_EQ(states[2].get_name(), "wheel_right_joint/position");
  EXPECT_EQ(states[3].get_name(), "wheel_right_joint/velocity");
  EXPECT_EQ(commands[0].get_name(), "wheel_left_joint/velocity");
  EXPECT_EQ(commands[1].get_name(), "wheel_right_joint/velocity");
  for (const auto & state : states)
  {
    EXPECT_DOUBLE_EQ(state.get_value(), 0.0);
  }
  for (auto & command : commands)
  {
    EXPECT_NO_THROW(command.set_value(0.25));
    EXPECT_DOUBLE_EQ(command.get_value(), 0.25);
  }
}

TEST(BumperbotInterface, AcceptsEitherJointDeclarationOrder)
{
  BumperbotInterface hardware;
  ASSERT_EQ(hardware.on_init(valid_hardware_info(true)), CallbackReturn::SUCCESS);

  const auto states = hardware.export_state_interfaces();
  ASSERT_EQ(states.size(), 4U);
  EXPECT_EQ(states[0].get_name(), "wheel_right_joint/position");
  EXPECT_EQ(states[2].get_name(), "wheel_left_joint/position");
}

TEST(BumperbotInterface, RequiresANonEmptySerialPort)
{
  auto missing = valid_hardware_info();
  missing.hardware_parameters.clear();
  BumperbotInterface missing_port;
  EXPECT_EQ(missing_port.on_init(missing), CallbackReturn::FAILURE);

  auto empty = valid_hardware_info();
  empty.hardware_parameters["port"] = "";
  BumperbotInterface empty_port;
  EXPECT_EQ(empty_port.on_init(empty), CallbackReturn::FAILURE);
}

TEST(BumperbotInterface, RequiresExactlyTheTwoNamedWheelJoints)
{
  auto one_joint = valid_hardware_info();
  one_joint.joints.pop_back();
  BumperbotInterface missing_joint;
  EXPECT_EQ(missing_joint.on_init(one_joint), CallbackReturn::FAILURE);

  auto unknown_name = valid_hardware_info();
  unknown_name.joints[1].name = "another_joint";
  BumperbotInterface unknown_joint;
  EXPECT_EQ(unknown_joint.on_init(unknown_name), CallbackReturn::FAILURE);

  auto duplicate = valid_hardware_info();
  duplicate.joints[1].name = "wheel_left_joint";
  BumperbotInterface duplicate_joint;
  EXPECT_EQ(duplicate_joint.on_init(duplicate), CallbackReturn::FAILURE);
}

TEST(BumperbotInterface, RequiresExactlyOneVelocityCommandInterfacePerJoint)
{
  auto missing = valid_hardware_info();
  missing.joints[0].command_interfaces.clear();
  BumperbotInterface missing_command;
  EXPECT_EQ(missing_command.on_init(missing), CallbackReturn::FAILURE);

  auto wrong = valid_hardware_info();
  wrong.joints[0].command_interfaces[0].name = hardware_interface::HW_IF_POSITION;
  BumperbotInterface wrong_command;
  EXPECT_EQ(wrong_command.on_init(wrong), CallbackReturn::FAILURE);

  auto extra = valid_hardware_info();
  extra.joints[0].command_interfaces.push_back(
    interface_named(hardware_interface::HW_IF_POSITION));
  BumperbotInterface extra_command;
  EXPECT_EQ(extra_command.on_init(extra), CallbackReturn::FAILURE);
}

TEST(BumperbotInterface, RequiresExactlyPositionAndVelocityStateInterfacesPerJoint)
{
  auto missing = valid_hardware_info();
  missing.joints[0].state_interfaces.pop_back();
  BumperbotInterface missing_state;
  EXPECT_EQ(missing_state.on_init(missing), CallbackReturn::FAILURE);

  auto duplicate = valid_hardware_info();
  duplicate.joints[0].state_interfaces[1].name = hardware_interface::HW_IF_POSITION;
  BumperbotInterface duplicate_state;
  EXPECT_EQ(duplicate_state.on_init(duplicate), CallbackReturn::FAILURE);

  auto extra = valid_hardware_info();
  extra.joints[0].state_interfaces.push_back(interface_named("effort"));
  BumperbotInterface extra_state;
  EXPECT_EQ(extra_state.on_init(extra), CallbackReturn::FAILURE);
}

}  // namespace
}  // namespace bumperbot_firmware
