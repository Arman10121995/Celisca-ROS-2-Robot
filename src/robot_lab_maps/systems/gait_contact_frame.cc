// Copyright 2026 Robot Lab contributors. SPDX-License-Identifier: Apache-2.0
// Observe complete native contact frames, including actual empty components.
// Fortress's stock Contact topic omits empty frames, so silence cannot safely
// be interpreted as either zero force or a live sensor.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace robot_lab {
namespace sim=ignition::gazebo;
class GaitContactFrame final: public sim::System,public sim::ISystemConfigure,
    public sim::ISystemPreUpdate,public sim::ISystemPostUpdate {
  sim::Entity model{sim::kNullEntity};
  std::map<std::string,std::vector<std::string>> links;
  std::map<std::string,std::vector<sim::Entity>> groups;
  std::set<sim::Entity> all;
  ignition::transport::Node node;
  ignition::transport::Node::Publisher pub;
  bool initialized{false};double last{-1.};
 public:
  void Configure(const sim::Entity &entity,const std::shared_ptr<const sdf::Element> &sdf,
      sim::EntityComponentManager &,sim::EventManager &) override {
    model=entity;auto config=sdf->Clone();
    for(auto group=config->GetElement("group");group;group=group->GetNextElement("group")) {
      const auto name=group->Get<std::string>("name");
      for(auto link=group->GetElement("link");link;link=link->GetNextElement("link"))
        links[name].push_back(link->Get<std::string>());
    }
    pub=node.Advertise<ignition::msgs::StringMsg>("/gait/native_contact_frame");
  }
  void PreUpdate(const sim::UpdateInfo &,sim::EntityComponentManager &ecm) override {
    if(initialized)return;
    groups.clear();all.clear();
    for(const auto link:ecm.ChildrenByComponents(model,sim::components::Link()))
      for(const auto collision:ecm.ChildrenByComponents(link,sim::components::Collision()))all.insert(collision);
    for(const auto &entry:links) {
      for(const auto &name:entry.second) {
        auto found=ecm.ChildrenByComponents(model,sim::components::Link(),sim::components::Name(name));
        if(found.size()!=1)return;
        for(const auto collision:ecm.ChildrenByComponents(found.front(),sim::components::Collision())) {
          groups[entry.first].push_back(collision);
          if(!ecm.Component<sim::components::ContactSensorData>(collision))
            ecm.CreateComponent(collision,sim::components::ContactSensorData());
        }
      }
    }
    initialized=groups.size()==links.size();
  }
  void PostUpdate(const sim::UpdateInfo &info,const sim::EntityComponentManager &ecm) override {
    if(info.paused)return;
    const double now=std::chrono::duration<double>(info.simTime).count();
    if(now>=last && now-last<.02)return;
    last=now;bool valid=initialized;std::map<std::string,double> forces;
    for(const auto &group:groups) {
      double force=0.;if(group.second.empty())valid=false;
      for(const auto collision:group.second) {
        auto data=ecm.Component<sim::components::ContactSensorData>(collision);
        if(!data){valid=false;continue;}
        for(const auto &contact:data->Data().contact()) {
          if(all.count(contact.collision1().id()) && all.count(contact.collision2().id()))continue;
          if(contact.wrench_size()==0){valid=false;continue;}
          for(int i=0;i<contact.wrench_size();++i) {
            const auto &f=contact.wrench(i).body_1_wrench().force();
            double value=std::sqrt(f.x()*f.x()+f.y()*f.y()+f.z()*f.z());
            if(i<contact.normal_size()) {
              const auto &n=contact.normal(i);
              const auto length=std::sqrt(n.x()*n.x()+n.y()*n.y()+n.z()*n.z());
              if(length>1e-9)value=std::abs((f.x()*n.x()+f.y()*n.y()+f.z()*n.z())/length);
            }
            if(!std::isfinite(value)){valid=false;continue;}
            force+=value;
          }
        }
      }
      forces[group.first]=force;
    }
    std::ostringstream json;json<<std::setprecision(17)<<"{\"schema_version\":1,\"valid\":"<<(valid?"true":"false")
      <<",\"clock_time\":"<<now<<",\"sim_time\":"<<now<<",\"normal_forces_N\":{";
    bool first=true;for(const auto &force:forces){if(!first)json<<",";first=false;json<<"\""<<force.first<<"\":"<<force.second;}
    json<<"},\"source\":\"Actual Fortress native ContactSensorData and measured wrenches; empty observed components are zero, missing components are invalid\"}";
    ignition::msgs::StringMsg msg;msg.set_data(json.str());pub.Publish(msg);
  }
};
}
IGNITION_ADD_PLUGIN(robot_lab::GaitContactFrame,ignition::gazebo::System,
  ignition::gazebo::ISystemConfigure,ignition::gazebo::ISystemPreUpdate,ignition::gazebo::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(robot_lab::GaitContactFrame,"robot_lab::GaitContactFrame")
