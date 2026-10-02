// SPDX-License-Identifier: Apache-2.0
// Four parallel wheel planes admit translation or pivot, never both at once.
#include <cmath>
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace robot_lab_controller
{
class ParallelSteeringCritic : public dwb_core::TrajectoryCritic
{
public:
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    const auto & v = trajectory.velocity;
    if (std::hypot(v.x, v.y) > 1e-4 && std::abs(v.theta) > 1e-4) {
      throw dwb_core::IllegalTrajectoryException(
        name_, "Parallel steering requires translation or pivot, not simultaneous yaw");
    }
    return 0.0;
  }
};
}  // namespace robot_lab_controller

PLUGINLIB_EXPORT_CLASS(robot_lab_controller::ParallelSteeringCritic, dwb_core::TrajectoryCritic)
