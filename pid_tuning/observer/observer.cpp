// Independent read-only commanded torque observer, loaded after the motor plugin.
// JointForceCmd is an input command, NOT a force sensor or contact-force measurement.
#include <array>
#include <chrono>
#include <limits>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double_v.pb.h>
class PIDObserver: public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 std::array<gz::sim::Entity,8> joints_;gz::transport::Node n_;gz::transport::Node::Publisher pub_;double previous_{-1};
 public:
 void Configure(const gz::sim::Entity &e,const std::shared_ptr<const sdf::Element>&,gz::sim::EntityComponentManager &ecm,gz::sim::EventManager&)override{
  gz::sim::Model m(e);std::array<std::string,4> sides{"front_left","front_right","rear_left","rear_right"};
  for(size_t i=0;i<8;++i)joints_[i]=m.JointByName(ecm,sides[i/2]+(i%2?"_wheel_joint":"_steer_joint"));
  pub_=n_.Advertise<gz::msgs::Double_V>("/pid_tuning/motor_telemetry");
 }
 void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm)override{
  if(info.paused)return;double t=std::chrono::duration<double>(info.simTime).count();if(t-previous_<.01)return;previous_=t;gz::msgs::Double_V out;out.add_data(t);
  for(auto e:joints_){
   auto p=ecm.Component<gz::sim::components::JointPosition>(e);auto v=ecm.Component<gz::sim::components::JointVelocity>(e);auto f=ecm.Component<gz::sim::components::JointForceCmd>(e);
   out.add_data(p&&!p->Data().empty()?p->Data()[0]:std::numeric_limits<double>::quiet_NaN());
   out.add_data(v&&!v->Data().empty()?v->Data()[0]:std::numeric_limits<double>::quiet_NaN());
   out.add_data(f&&!f->Data().empty()?f->Data()[0]:std::numeric_limits<double>::quiet_NaN());
  }
  pub_.Publish(out);
 }
};
GZ_ADD_PLUGIN(PIDObserver,gz::sim::System,PIDObserver::ISystemConfigure,PIDObserver::ISystemPreUpdate)
