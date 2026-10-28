// Copyright 2026 Robot Lab contributors. SPDX-License-Identifier: Apache-2.0
// A native wall-clock lease on Twist commands, independent of ROS processes.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <stdexcept>
#include <ignition/gazebo/System.hh>
#include <ignition/msgs/twist.pb.h>
#include <ignition/msgs/double.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace robot_lab {
class CommandWatchdog final : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate {
  using Clock = std::chrono::steady_clock;
  ignition::transport::Node node;
  ignition::transport::Node::Publisher publisher;
  std::mutex mutex;
  ignition::msgs::Twist command;
  Clock::time_point received{};
  double timeout{.4}, linear{.3}, angular{.5};
 public:
  void Configure(const ignition::gazebo::Entity &,
      const std::shared_ptr<const sdf::Element> &sdf,
      ignition::gazebo::EntityComponentManager &,
      ignition::gazebo::EventManager &) override {
    const auto input = sdf->Get<std::string>("input", "/tracked/cmd_vel").first;
    const auto output = sdf->Get<std::string>("output", "/tracked/actuator_cmd").first;
    timeout = sdf->Get<double>("timeout_s", .4).first;
    linear = sdf->Get<double>("max_linear", .3).first;
    angular = sdf->Get<double>("max_angular", .5).first;
    if (!std::isfinite(timeout) || timeout<=0 || timeout>1 ||
        !std::isfinite(linear) || linear<=0 || !std::isfinite(angular) || angular<=0)
      throw std::runtime_error("Finite positive native command limits required");
    publisher = node.Advertise<ignition::msgs::Twist>(output);
    node.Subscribe(input, &CommandWatchdog::Receive, this);
  }
  void Receive(const ignition::msgs::Twist &message) {
    std::lock_guard<std::mutex> guard(mutex);
    command.Clear();
    if (!std::isfinite(message.linear().x()) || !std::isfinite(message.angular().z()) ||
        !std::isfinite(message.linear().y()) || !std::isfinite(message.linear().z()) ||
        std::abs(message.linear().y())>.001 || std::abs(message.linear().z())>.001)
      return;
    command.mutable_linear()->set_x(std::clamp(message.linear().x(),-linear,linear));
    command.mutable_angular()->set_z(std::clamp(message.angular().z(),-angular,angular));
    received = Clock::now();
  }
  void PreUpdate(const ignition::gazebo::UpdateInfo &,
      ignition::gazebo::EntityComponentManager &) override {
    ignition::msgs::Twist output;
    {
      std::lock_guard<std::mutex> guard(mutex);
      if (std::chrono::duration<double>(Clock::now()-received).count()<=timeout)
        output=command;
    }
    publisher.Publish(output);
  }
};

// Native force timeout: a suspended ROS thrust owner cannot keep a propeller
// energized. Buoyancy, hydrodynamics and passive coasting remain physical.
class ThrustWatchdog final : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate {
  using Clock = std::chrono::steady_clock;
  ignition::transport::Node node;
  ignition::transport::Node::Publisher publisher;
  std::mutex mutex;
  Clock::time_point received{};
  double command{}, timeout{.4}, maximum{35};
 public:
  void Configure(const ignition::gazebo::Entity &,
      const std::shared_ptr<const sdf::Element> &sdf,
      ignition::gazebo::EntityComponentManager &,
      ignition::gazebo::EventManager &) override {
    const auto input=sdf->Get<std::string>("input");
    const auto output=sdf->Get<std::string>("output");
    timeout=sdf->Get<double>("timeout_s",.4).first;
    maximum=sdf->Get<double>("max_force",35.).first;
    if (!std::isfinite(timeout) || timeout<=0 || timeout>1 ||
        !std::isfinite(maximum) || maximum<=0)
      throw std::runtime_error("Finite positive thrust watchdog limits required");
    publisher=node.Advertise<ignition::msgs::Double>(output);
    node.Subscribe(input,&ThrustWatchdog::Receive,this);
  }
  void Receive(const ignition::msgs::Double &message) {
    std::lock_guard<std::mutex> guard(mutex);
    command=std::isfinite(message.data()) ? std::clamp(message.data(),-maximum,maximum) : 0.;
    received=Clock::now();
  }
  void PreUpdate(const ignition::gazebo::UpdateInfo &,
      ignition::gazebo::EntityComponentManager &) override {
    ignition::msgs::Double output;
    {
      std::lock_guard<std::mutex> guard(mutex);
      output.set_data(std::chrono::duration<double>(Clock::now()-received).count()<=timeout ? command : 0.);
    }
    publisher.Publish(output);
  }
};
}
IGNITION_ADD_PLUGIN(robot_lab::CommandWatchdog, ignition::gazebo::System,
                   robot_lab::CommandWatchdog::ISystemConfigure,
                   robot_lab::CommandWatchdog::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(robot_lab::CommandWatchdog,"robot_lab::CommandWatchdog")
IGNITION_ADD_PLUGIN(robot_lab::ThrustWatchdog, ignition::gazebo::System,
                   robot_lab::ThrustWatchdog::ISystemConfigure,
                   robot_lab::ThrustWatchdog::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(robot_lab::ThrustWatchdog,"robot_lab::ThrustWatchdog")
