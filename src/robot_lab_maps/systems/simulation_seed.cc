// Copyright 2026 Robot Lab contributors. SPDX-License-Identifier: Apache-2.0
// Seed Gazebo's shared random generator (ignition::math::Rand) when the world
// loads, so sensor noise models (Gaussian noise on IMU, lidar, cameras) draw a
// sequence that repeats for the same seed (ROADMAP R7, simulator-applied seeds).
#include <cstdint>
#include <stdexcept>
#include <ignition/common/Console.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/math/Rand.hh>
#include <ignition/plugin/Register.hh>

namespace robot_lab {
class SimulationSeed final : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure {
 public:
  void Configure(const ignition::gazebo::Entity &,
      const std::shared_ptr<const sdf::Element> &sdf,
      ignition::gazebo::EntityComponentManager &,
      ignition::gazebo::EventManager &) override {
    // sdf parameters have no 64-bit integer type; a double holds every seed exactly.
    const double value = sdf->Get<double>("seed", -1.).first;
    if (!(value >= 0 && value <= 4294967295.0) || value != static_cast<double>(static_cast<uint64_t>(value)))
      throw std::runtime_error("robot_lab::SimulationSeed needs a seed in [0, 2^32)");
    ignition::math::Rand::Seed(static_cast<unsigned int>(value));
    ignmsg << "Robot Lab simulation seed " << value << std::endl;
  }
};
}  // namespace robot_lab

IGNITION_ADD_PLUGIN(robot_lab::SimulationSeed, ignition::gazebo::System,
                    robot_lab::SimulationSeed::ISystemConfigure)
