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
  void onInit() override
  {
    auto node = node_.lock();
    if (!node) {throw std::runtime_error("ParallelSteeringCritic lifecycle node expired");}
    const auto parameter = dwb_plugin_name_ + ".xy_goal_tolerance";
    if (!node->has_parameter(parameter)) {node->declare_parameter(parameter, 0.25);}
    node->get_parameter(parameter, goal_radius_);
  }

  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D &) override
  {
    // Full-range parallel steering can translate towards the path in any body
    // direction. Pivoting before arrival cannot improve translational progress.
    // Match RotateToGoal's latched arrival window. Localization can drift a
    // few millimetres outside it during pivoting; disabling pivot then leaves
    // RotateToGoal rejecting translation while this critic rejects rotation.
    near_goal_ = near_goal_ ||
      std::hypot(pose.x - goal.x, pose.y - goal.y) <= goal_radius_;
    terminal_approach_ = near_goal_ || std::hypot(pose.x - goal.x, pose.y - goal.y) <= 0.5;
    goal_x_ = goal.x;
    goal_y_ = goal.y;
    return true;
  }

  void reset() override {near_goal_ = false;}

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    const auto & v = trajectory.velocity;
    if (std::hypot(v.x, v.y) > 1e-4 && std::abs(v.theta) > 1e-4) {
      throw dwb_core::IllegalTrajectoryException(
        name_, "Parallel steering requires translation or pivot, not simultaneous yaw");
    }
    // Prefer progress through translation before the final pivot. Keep pure
    // rotation admissible: a hard arrival gate can deadlock with other DWB
    // critics after a changed plan or localization adjustment.
    if (trajectory.poses.empty()) {
      throw dwb_core::IllegalTrajectoryException(name_, "Empty trajectory");
    }
    // Grid distance critics have a cell-sized plateau near the goal. Resolve
    // it using continuous endpoint distance so tight pose goals remain
    // reachable instead of accepting a stationary trajectory on that plateau.
    const auto & end = trajectory.poses.back();
    const double distance = std::hypot(end.x - goal_x_, end.y - goal_y_);
    // Use that precision term only near the terminal pose. Farther away the
    // grid path critics must choose the obstacle detour: a global Euclidean
    // attraction can cut its corners and enter lethal space.
    return (terminal_approach_ ? 100.0 * distance : 0.0) +
           (near_goal_ ? 0.0 : 2.0 * std::abs(v.theta));
  }

private:
  double goal_radius_ = 0.25;
  double goal_x_ = 0.0, goal_y_ = 0.0;
  bool near_goal_ = false;
  bool terminal_approach_ = false;
};
}  // namespace robot_lab_controller

PLUGINLIB_EXPORT_CLASS(robot_lab_controller::ParallelSteeringCritic, dwb_core::TrajectoryCritic)
