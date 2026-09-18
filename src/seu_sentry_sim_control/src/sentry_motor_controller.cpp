#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <gz/plugin/Register.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/double_v.pb.h>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/transport/Node.hh>

namespace seu_sentry_sim_control {
// sentry2026 motor loops run next to physics, without ROS/DDS feedback latency.
// Gazebo official JointForceCmd interface: revolute commands are Nm.
class SentryMotorController : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
  struct Motor {
    gz::sim::Entity entity{gz::sim::kNullEntity};
    std::string name;
    bool steer{}, updated{}, active{}, initialized{};
    double target{}, initial{}, integral{}, previous_error{}, received_at{};
  };
  std::array<Motor, 8> motors_;
  gz::transport::Node transport_;
  std::mutex mutex_;
  double wheel_kp_{0.1}, wheel_ki_{0.2}, wheel_kd_{0};
  double steer_kp_{8}, steer_ki_{0.2}, steer_kd_{0.08};
  double wheel_limit_{2}, steer_limit_{5}, integral_limit_{1}, timeout_{0.2};
public:
  void Configure(const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &) override {
    auto param = [&](const char * key, double & v) {
      if (sdf->HasElement(key)) v = sdf->Get<double>(key);
      if (!std::isfinite(v) || v < 0) throw std::runtime_error("Invalid motor loop parameter");
    };
    param("wheel_kp",wheel_kp_); param("wheel_ki",wheel_ki_); param("wheel_kd",wheel_kd_);
    param("steer_kp",steer_kp_); param("steer_ki",steer_ki_); param("steer_kd",steer_kd_);
    param("wheel_torque_limit",wheel_limit_); param("steer_torque_limit",steer_limit_);
    param("integral_limit",integral_limit_); param("command_timeout",timeout_);
    // Bounded live tuning: wheel P/I/D, steer P/I/D; no change to kinematics.
    transport_.Subscribe<gz::msgs::Double_V>("/simulation/motor_gains",
      [this](const gz::msgs::Double_V & msg) {
        if (msg.data_size()!=6) return;
        for (const double value:msg.data()) if (!std::isfinite(value) || value<0 || value>100) return;
        std::lock_guard<std::mutex> lock(mutex_);
        wheel_kp_=msg.data(0); wheel_ki_=msg.data(1); wheel_kd_=msg.data(2);
        steer_kp_=msg.data(3); steer_ki_=msg.data(4); steer_kd_=msg.data(5);
        for (auto & motor:motors_) {motor.integral=0; motor.previous_error=0;}
      });
    gz::sim::Model model(entity);
    const std::array<std::string,4> sides{"front_left","front_right","rear_left","rear_right"};
    for (size_t i=0;i<8;++i) {
      auto & m=motors_[i]; m.steer=i%2==0;
      m.name=sides[i/2]+(m.steer?"_steer_joint":"_wheel_joint");
      m.entity=model.JointByName(ecm,m.name);
      if (m.entity==gz::sim::kNullEntity) throw std::runtime_error("Missing motor joint "+m.name);
      // Request actual physics feedback (Gazebo creates/populates these components).
      if (!ecm.Component<gz::sim::components::JointPosition>(m.entity))
        ecm.CreateComponent(m.entity,gz::sim::components::JointPosition());
      if (!ecm.Component<gz::sim::components::JointVelocity>(m.entity))
        ecm.CreateComponent(m.entity,gz::sim::components::JointVelocity());
      const auto topic="/model/"+model.Name(ecm)+"/joint/"+m.name+(m.steer?"/target_angle":"/target_speed");
      transport_.Subscribe<gz::msgs::Double>(topic,[this,i](const gz::msgs::Double & msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (std::isfinite(msg.data())) {motors_[i].target=msg.data(); motors_[i].updated=true;}
      });
    }
  }
  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override {
    if (info.paused) return;
    const double dt=std::chrono::duration<double>(info.dt).count();
    const double now=std::chrono::duration<double>(info.simTime).count();
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & m:motors_) {
      auto pos=ecm.Component<gz::sim::components::JointPosition>(m.entity);
      auto vel=ecm.Component<gz::sim::components::JointVelocity>(m.entity);
      double torque=0;
      if (dt>0 && dt<=0.01 && pos && vel && !pos->Data().empty() && !vel->Data().empty()) {
        const double angle=pos->Data()[0], speed=vel->Data()[0];
        if (!m.initialized) {m.initial=angle; m.initialized=true;}
        if (m.updated) {m.received_at=now; m.active=true; m.updated=false;}
        const bool fresh=m.active && now>=m.received_at && now-m.received_at<=timeout_;
        const double target=fresh?m.target:(m.steer?m.initial:0.0);
        // MCU PIDUpdate / PIDUpdate_extw, adapted to SI and simulation dt.
        const double error=m.steer?std::remainder(target-angle,2*M_PI):target-speed;
        m.integral=std::clamp(m.integral+error*dt,-integral_limit_,integral_limit_);
        torque=m.steer?steer_kp_*error+steer_ki_*m.integral-steer_kd_*speed:
          wheel_kp_*error+wheel_ki_*m.integral+wheel_kd_*(error-m.previous_error)/dt;
        m.previous_error=error;
        const double limit=m.steer?steer_limit_:wheel_limit_;
        torque=std::isfinite(torque)?std::clamp(torque,-limit,limit):0.0;
      } else {m.integral=0; m.previous_error=0;}
      auto force=ecm.Component<gz::sim::components::JointForceCmd>(m.entity);
      if (!force) ecm.CreateComponent(m.entity,gz::sim::components::JointForceCmd({torque}));
      else {if (force->Data().empty()) force->Data().resize(1); force->Data()[0]+=torque;}
    }
  }
};
}
GZ_ADD_PLUGIN(seu_sentry_sim_control::SentryMotorController,gz::sim::System,
  seu_sentry_sim_control::SentryMotorController::ISystemConfigure,
  seu_sentry_sim_control::SentryMotorController::ISystemPreUpdate)
