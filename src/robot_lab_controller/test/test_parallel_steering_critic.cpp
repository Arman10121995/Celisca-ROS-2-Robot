// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_loader.hpp"

TEST(ParallelSteering, TranslationThenLatchedFinalPivot)
{
  pluginlib::ClassLoader<dwb_core::TrajectoryCritic> loader(
    "dwb_core", "dwb_core::TrajectoryCritic");
  auto critic = loader.createSharedInstance("robot_lab_controller::ParallelSteeringCritic");
  geometry_msgs::msg::Pose2D pose, goal;
  goal.x = 2.0;
  nav_2d_msgs::msg::Twist2D measured;
  nav_2d_msgs::msg::Path2D path;
  dwb_msgs::msg::Trajectory2D translation, pivot, mixed, stopped;
  translation.velocity.y = 0.2;
  pivot.velocity.theta = 0.3;
  mixed.velocity = translation.velocity;
  mixed.velocity.theta = 0.3;
  translation.poses.emplace_back();
  translation.poses.back().x = 0.4;
  pivot.poses.push_back(pose);
  stopped.poses.push_back(pose);

  critic->prepare(pose, measured, goal, path);
  // Far from the goal, upstream path critics select the obstacle detour.
  EXPECT_DOUBLE_EQ(critic->scoreTrajectory(translation), critic->scoreTrajectory(stopped));
  EXPECT_GT(critic->scoreTrajectory(pivot), critic->scoreTrajectory(stopped));
  EXPECT_THROW(critic->scoreTrajectory(mixed), dwb_core::IllegalTrajectoryException);

  pose.x = 1.7;
  translation.poses.back().x = 1.9;
  stopped.poses.back() = pivot.poses.back() = pose;
  critic->prepare(pose, measured, goal, path);
  EXPECT_LT(critic->scoreTrajectory(translation), critic->scoreTrajectory(stopped));

  pose.x = 1.99;
  critic->prepare(pose, measured, goal, path);
  pivot.poses.back() = stopped.poses.back() = pose;
  EXPECT_DOUBLE_EQ(critic->scoreTrajectory(pivot), critic->scoreTrajectory(stopped));
  // A later estimate outside the arrival window must not deadlock with
  // RotateToGoal, which has already rejected further translation.
  pose.x = 1.5;
  critic->prepare(pose, measured, goal, path);
  pivot.poses.back() = stopped.poses.back() = pose;
  EXPECT_DOUBLE_EQ(critic->scoreTrajectory(pivot), critic->scoreTrajectory(stopped));

  critic->reset();
  critic->prepare(pose, measured, goal, path);
  EXPECT_GT(critic->scoreTrajectory(pivot), critic->scoreTrajectory(stopped));
}
