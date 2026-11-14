// Copyright 2026 Robot Lab contributors. SPDX-License-Identifier: Apache-2.0
// Physical stand-ins for scripted Gazebo actors (ROADMAP R6.2).
//
// Fortress actors are visual only: robots drive through people and traffic.
// For every actor this system moves a companion collision model
// "<actor>__collider" (inserted by gazebo.launch.py: gravity off, a cylinder
// for people, a box for vehicles) to the actor's scripted pose each step.
// The pose is computed the way the renderer animates the actor
// (ign-gazebo6 SceneManager: common::TrajectoryInfo per SDF trajectory,
// sequenced from delay_start, looping or held at the end), so the collider
// stays under the drawn skin. Its velocity follows the trajectory, so a
// robot touching it gets a moving obstacle, not a teleported one.
#include <chrono>
#include <map>
#include <string>
#include <vector>

#include <ignition/common/Animation.hh>
#include <ignition/common/KeyFrame.hh>
#include <ignition/common/Console.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Actor.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/Actor.hh>

namespace robot_lab {
namespace gz = ignition::gazebo;
using TP = std::chrono::steady_clock::time_point;

struct Collider {
  gz::Entity model{gz::kNullEntity};
  gz::Entity link{gz::kNullEntity};
  ignition::math::Pose3d actorPose;
  std::vector<ignition::common::TrajectoryInfo> trajectories;
  bool loop{true};
  double z{0.0};
  ignition::math::Pose3d last;
  bool started{false};
};

// The actor's root pose at simulation time t (ign-gazebo6 ActorTrajectoryAt).
static bool TrajectoryAt(Collider &c, std::chrono::steady_clock::duration time,
                         ignition::math::Pose3d &pose) {
  auto &trajs = c.trajectories;
  if (trajs.empty()) return false;
  const TP zero{};
  if (time < trajs.front().StartTime() - zero) return false;   // delay_start not yet over
  const auto total = trajs.back().EndTime() - zero;
  if (c.loop && total.count() > 0) {
    while (time > total) time -= total - (trajs.front().StartTime() - zero);
  } else if (time > total) {
    time = total;
  }
  for (auto &traj : trajs) {
    if (traj.StartTime() - zero <= time && traj.EndTime() - zero >= time) {
      if (!traj.Translated() || !traj.Waypoints()) return false;
      traj.Waypoints()->Time(std::chrono::duration<double>(time - (traj.StartTime() - zero)).count());
      ignition::common::PoseKeyFrame frame(0.0);
      traj.Waypoints()->InterpolatedKeyFrame(frame);
      pose = c.actorPose * ignition::math::Pose3d(frame.Translation(), frame.Rotation());
      return true;
    }
  }
  return false;
}

class ActorColliders final : public gz::System, public gz::ISystemConfigure, public gz::ISystemPreUpdate {
 public:
  void Configure(const gz::Entity &, const std::shared_ptr<const sdf::Element> &,
                 gz::EntityComponentManager &, gz::EventManager &) override {}

  // World systems are configured before the world's actors exist as
  // entities, so the actors are collected on the first update.
  void Discover(gz::EntityComponentManager &ecm) {
    this->discovered = true;
    ecm.Each<gz::components::Actor, gz::components::Name>(
        [&](const gz::Entity &, const gz::components::Actor *actor, const gz::components::Name *name) -> bool {
          const sdf::Actor &sdfActor = actor->Data();
          Collider c;
          c.actorPose = sdfActor.RawPose();
          c.loop = sdfActor.ScriptLoop();
          for (uint64_t i = 0; i < sdfActor.TrajectoryCount(); ++i) {
            const sdf::Trajectory *traj = sdfActor.TrajectoryByIndex(i);
            std::map<TP, ignition::math::Pose3d> waypoints;
            for (uint64_t j = 0; j < traj->WaypointCount(); ++j) {
              const sdf::Waypoint *point = traj->WaypointByIndex(j);
              waypoints[TP(std::chrono::milliseconds(static_cast<int>(point->Time() * 1000)))] = point->Pose();
            }
            ignition::common::TrajectoryInfo info;
            info.SetId(traj->Id());
            if (waypoints.size() >= 2) {
              info.SetWaypoints(waypoints, traj->Tension());
            } else {
              info.SetTranslated(false);
            }
            c.trajectories.push_back(info);
          }
          // Sequence back to back after delay_start (ign-gazebo6 SequenceTrajectories).
          TP time{std::chrono::milliseconds(static_cast<int>(sdfActor.ScriptDelayStart() * 1000))};
          for (auto &traj : c.trajectories) {
            traj.SetStartTime(time);
            time += traj.Duration();
            traj.SetEndTime(time);
          }
          this->colliders[name->Data()] = c;
          return true;
        });
    ignmsg << "Robot Lab actor colliders for " << this->colliders.size() << " actors" << std::endl;
  }

  void PreUpdate(const gz::UpdateInfo &info, gz::EntityComponentManager &ecm) override {
    if (!this->discovered) this->Discover(ecm);
    if (info.paused) return;
    for (auto &[name, c] : this->colliders) {
      if (c.model == gz::kNullEntity) {
        c.model = gz::Model(ecm.EntityByComponents(gz::components::Name(name + "__collider"))).Entity();
        if (c.model == gz::kNullEntity) continue;
        auto links = ecm.ChildrenByComponents(c.model, gz::components::Link());
        if (links.empty()) continue;
        c.link = links.front();
        auto pose = ecm.Component<gz::components::Pose>(c.model);
        c.z = pose ? pose->Data().Pos().Z() : 0.0;
      }
      ignition::math::Pose3d pose;
      if (!TrajectoryAt(c, info.simTime, pose)) continue;
      // Upright at the authored height: a person's hip or a car's wheelbase
      // stays level, only x, y and heading follow the script.
      ignition::math::Pose3d target(pose.Pos().X(), pose.Pos().Y(), c.z, 0, 0, pose.Rot().Yaw());
      gz::Model(c.model).SetWorldPoseCmd(ecm, target);
      const double dt = std::chrono::duration<double>(info.dt).count();
      ignition::math::Vector3d velocity, spin;
      if (c.started && dt > 0) {
        velocity = (target.Pos() - c.last.Pos()) / dt;
        spin.Z((target.Rot().Yaw() - c.last.Rot().Yaw()) / dt);
        if (std::abs(spin.Z()) > 50) spin.Z(0);       // heading wrap
      }
      c.last = target;
      c.started = true;
      auto linear = ecm.Component<gz::components::LinearVelocityCmd>(c.link);
      if (!linear) {
        ecm.CreateComponent(c.link, gz::components::LinearVelocityCmd(velocity));
      } else {
        linear->Data() = velocity;
      }
      auto angular = ecm.Component<gz::components::AngularVelocityCmd>(c.link);
      if (!angular) {
        ecm.CreateComponent(c.link, gz::components::AngularVelocityCmd(spin));
      } else {
        angular->Data() = spin;
      }
    }
  }

 private:
  std::map<std::string, Collider> colliders;
  bool discovered{false};
};
}  // namespace robot_lab

IGNITION_ADD_PLUGIN(robot_lab::ActorColliders, ignition::gazebo::System,
                    robot_lab::ActorColliders::ISystemConfigure, robot_lab::ActorColliders::ISystemPreUpdate)
