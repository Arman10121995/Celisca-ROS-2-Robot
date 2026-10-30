// Copyright 2026 Robot Lab contributors. SPDX-License-Identifier: Apache-2.0
// Real wheel forces and immersion-gated native propellers. No pose/velocity writes.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Joint.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/msgs/twist.pb.h>
#include <ignition/msgs/double.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace robot_lab {
class AmphibiousDrive final : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate {
  using Wall = std::chrono::steady_clock;
  ignition::transport::Node node;
  ignition::gazebo::Link hull;
  std::array<ignition::gazebo::Joint,4> wheels;
  std::array<ignition::gazebo::Joint,4> suspension;
  std::array<ignition::transport::Node::Publisher,2> propellers;
  ignition::transport::Node::Publisher state;
  std::mutex mutex;
  std::array<double,2> requested{}, drive{};
  std::array<Wall::time_point,2> thrustAt{};
  Wall::time_point driveAt{};
  double surface{}, radius{.22}, halfTrack{.49}, torqueLimit{8.}, lastSim{-1.}, lastState{-1.};
  bool configured{false};
 public:
  void Configure(const ignition::gazebo::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      ignition::gazebo::EntityComponentManager &ecm,
      ignition::gazebo::EventManager &) override {
    ignition::gazebo::Model model(entity);
    hull=ignition::gazebo::Link(model.LinkByName(ecm,"base_link"));
    if (!hull.Valid(ecm)) throw std::runtime_error("Amphibious hull missing");
    surface=sdf->Get<double>("surface_z",0.).first;
    radius=sdf->Get<double>("wheel_radius",.22).first;
    halfTrack=sdf->Get<double>("half_track",.49).first;
    torqueLimit=sdf->Get<double>("wheel_torque",8.).first;
    if (!std::isfinite(surface) || !std::isfinite(radius) || radius<=0 ||
        !std::isfinite(halfTrack) || halfTrack<=0 || !std::isfinite(torqueLimit) || torqueLimit<=0 || torqueLimit>8.)
      throw std::runtime_error("Invalid amphibious source limits");
    const std::array<std::string,4> names={"front_left_wheel_joint","rear_left_wheel_joint","front_right_wheel_joint","rear_right_wheel_joint"};
    for (size_t i=0;i<4;++i) {
      wheels[i]=ignition::gazebo::Joint(model.JointByName(ecm,names[i]));
      if (!wheels[i].Valid(ecm)) throw std::runtime_error("Missing amphibious wheel: "+names[i]);
      wheels[i].EnableVelocityCheck(ecm);
      std::string spring=names[i];spring.replace(spring.find("wheel_joint"),11,"suspension_joint");
      suspension[i]=ignition::gazebo::Joint(model.JointByName(ecm,spring));
      if (!suspension[i].Valid(ecm)) throw std::runtime_error("Missing passive suspension: "+spring);
      suspension[i].EnableVelocityCheck(ecm);suspension[i].EnablePositionCheck(ecm);
    }
    hull.EnableVelocityChecks(ecm);
    const auto name=model.Name(ecm);
    for (size_t i=0;i<2;++i) {
      const std::string joint=i==0 ? "port_propeller_joint" : "starboard_propeller_joint";
      propellers[i]=node.Advertise<ignition::msgs::Double>("/model/"+name+"/joint/"+joint+"/cmd_thrust");
    }
    node.Subscribe("/surface/request/port_propeller_joint",&AmphibiousDrive::Port,this);
    node.Subscribe("/surface/request/starboard_propeller_joint",&AmphibiousDrive::Starboard,this);
    node.Subscribe("/amphibious/wheels",&AmphibiousDrive::Drive,this);
    state=node.Advertise<ignition::msgs::StringMsg>("/amphibious/plant");
    configured=true;
  }
  void Port(const ignition::msgs::Double &msg) { Thrust(0,msg); }
  void Starboard(const ignition::msgs::Double &msg) { Thrust(1,msg); }
  void Thrust(size_t i,const ignition::msgs::Double &msg) {
    std::lock_guard<std::mutex> guard(mutex);
    requested[i]=std::isfinite(msg.data()) ? std::clamp(msg.data(),-35.,35.) : 0.;
    thrustAt[i]=Wall::now();
  }
  void Drive(const ignition::msgs::Twist &msg) {
    std::lock_guard<std::mutex> guard(mutex);
    drive={0.,0.}; driveAt=Wall::time_point{};
    if (!std::isfinite(msg.linear().x()) || !std::isfinite(msg.angular().z()) ||
        !std::isfinite(msg.linear().y()) || !std::isfinite(msg.linear().z()) ||
        std::abs(msg.linear().y())>.001 || std::abs(msg.linear().z())>.001) return;
    drive={std::clamp(msg.linear().x(),-.4,.4),std::clamp(msg.angular().z(),-.3,.3)};
    driveAt=Wall::now();
  }
  void PreUpdate(const ignition::gazebo::UpdateInfo &info,
      ignition::gazebo::EntityComponentManager &ecm) override {
    if (!configured || info.paused) return;
    const double sim=std::chrono::duration<double>(info.simTime).count();
    std::array<double,2> force{}, command{}; bool fresh=false;
    {
      std::lock_guard<std::mutex> guard(mutex);
      if (lastSim>=0 && sim<lastSim) { driveAt={}; thrustAt={}; lastState=-1.; }
      fresh=std::chrono::duration<double>(Wall::now()-driveAt).count()<=.4;
      if (fresh) command=drive;
      for (size_t i=0;i<2;++i)
        if (std::chrono::duration<double>(Wall::now()-thrustAt[i]).count()<=.4) force[i]=requested[i];
    }
    lastSim=sim;
    const auto pose=hull.WorldPose(ecm); const auto velocity=hull.WorldLinearVelocity(ecm);
    const auto angular=hull.WorldAngularVelocity(ecm);
    if (!pose || !velocity || !angular) return;
    // Authored wetted fractions, evaluated from physical water plane and hull.
    const double wet=std::clamp((surface-(pose->Pos().Z()-.17))/.10,0.,1.);
    for (size_t i=0;i<2;++i) {
      const auto point=pose->Pos()+pose->Rot().RotateVector(ignition::math::Vector3d(-.64,i==0?.38:-.38,-.15));
      force[i]*=std::clamp((surface-point.Z()+.065)/.13,0.,1.);
      ignition::msgs::Double message; message.set_data(force[i]); propellers[i].Publish(message);
    }
    const auto local=pose->Rot().RotateVectorReverse(*velocity);
    const auto rates=pose->Rot().RotateVectorReverse(*angular);
    ignition::math::Vector3d drag, torque;
    const std::array<double,3> linear={65.,160.,160.}, quadratic={70.,160.,180.}, rotation={12.,18.,16.};
    for (size_t i=0;i<3;++i) {
      drag[i]=-wet*(linear[i]*local[i]+quadratic[i]*std::abs(local[i])*local[i]);
      torque[i]=-wet*rotation[i]*rates[i];
    }
    hull.AddWorldWrench(ecm,pose->Rot().RotateVector(drag),pose->Rot().RotateVector(torque));
    std::array<double,4> measured{}, efforts{};
    for (size_t i=0;i<4;++i) {
      // Passive elastic storage + dissipation, independent of command power.
      // Positive travel compresses the wheel upwards relative to its hull.
      const auto travel=suspension[i].Position(ecm), travelRate=suspension[i].Velocity(ecm);
      double springForce=0.;
      if (travel && travelRate && !travel->empty() && !travelRate->empty() &&
          std::isfinite((*travel)[0]) && std::isfinite((*travelRate)[0]))
        springForce=std::clamp(-10000.*(*travel)[0]-200.*(*travelRate)[0],-600.,600.);
      auto spring=ecm.Component<ignition::gazebo::components::JointForceCmd>(suspension[i].Entity());
      if (spring) spring->Data()={springForce};
      else ecm.CreateComponent(suspension[i].Entity(),ignition::gazebo::components::JointForceCmd({springForce}));
      const auto speed=wheels[i].Velocity(ecm);
      if (speed && !speed->empty() && std::isfinite((*speed)[0])) {
        measured[i]=(*speed)[0];
        const double target=std::clamp((command[0]+(i<2?-1.:1.)*halfTrack*command[1])/radius,-3.,3.);
        efforts[i]=fresh ? std::clamp(12.*(target-measured[i]),-torqueLimit,torqueLimit) : 0.;
      }
      auto component=ecm.Component<ignition::gazebo::components::JointForceCmd>(wheels[i].Entity());
      if (component) component->Data()={efforts[i]};
      else ecm.CreateComponent(wheels[i].Entity(),ignition::gazebo::components::JointForceCmd({efforts[i]}));
    }
    if (sim-lastState>=.1) {
      lastState=sim; std::ostringstream json;
      json << "{\"sim_time\":" << sim << ",\"wet_fraction\":" << wet << ",\"wheel_input_fresh\":" << (fresh?"true":"false")
           << ",\"applied_thrust_N\":[" << force[0] << "," << force[1] << "],\"wheel_rates_rad_s\":[";
      for (size_t i=0;i<4;++i) json << (i?",":"") << measured[i];
      json << "],\"wheel_efforts_Nm\":[";
      for (size_t i=0;i<4;++i) json << (i?",":"") << efforts[i];
      json << "]}"; ignition::msgs::StringMsg message;message.set_data(json.str());state.Publish(message);
    }
  }
};
}
IGNITION_ADD_PLUGIN(robot_lab::AmphibiousDrive,ignition::gazebo::System,
                   robot_lab::AmphibiousDrive::ISystemConfigure,robot_lab::AmphibiousDrive::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(robot_lab::AmphibiousDrive,"robot_lab::AmphibiousDrive")
