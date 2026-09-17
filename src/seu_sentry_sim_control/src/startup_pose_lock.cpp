#include <atomic>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/sim/Model.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include "startup_motion_gate.hpp"

namespace seu_sentry_sim_control
{
// Hold only the chassis with a physical fixed constraint. Gimbal and steering
// controllers remain independent; no teleporting or joint velocity resets.
class StartupPoseLock final : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager & events) override
  {
    const gz::sim::Model robot(entity);
    const auto chassis = robot.LinkByName(ecm, "chassis");
    // Model plugins are configured before SdfEntityCreator assigns the model
    // its ParentEntity. The world itself already exists at this stage.
    const auto world = ecm.EntityByComponents(gz::sim::components::World());
    if (chassis == gz::sim::kNullEntity || world == gz::sim::kNullEntity) {
      throw std::runtime_error("Startup lock requires a chassis link and world parent");
    }
    if (sdf && sdf->HasElement("command_topic")) {
      command_topic_ = sdf->Get<std::string>("command_topic");
    }
    if (sdf && sdf->HasElement("command_deadband")) {
      command_deadband_ = sdf->Get<double>("command_deadband");
    }
    if (!std::isfinite(command_deadband_) || command_deadband_ < 0.0) {
      throw std::runtime_error("Invalid startup command deadband");
    }

    // An invisible static anchor is a genuine world-fixed physics link. A
    // detachable joint preserves the chassis pose when attached to this link.
    sdf::Model anchor;
    anchor.SetName("sentry_startup_anchor_" + std::to_string(entity));
    anchor.SetStatic(true);
    sdf::Link anchor_link;
    anchor_link.SetName("anchor");
    anchor.AddLink(anchor_link);
    gz::sim::SdfEntityCreator creator(ecm, events);
    anchor_entity_ = creator.CreateEntities(&anchor);
    creator.SetParent(anchor_entity_, world);
    const auto anchor_link_entity = gz::sim::Model(anchor_entity_).LinkByName(ecm, "anchor");
    joint_entity_ = ecm.CreateEntity();
    ecm.CreateComponent(joint_entity_, gz::sim::components::DetachableJoint(
      {anchor_link_entity, chassis, "fixed"}));

    transport_.Subscribe(command_topic_, &StartupPoseLock::OnCommand, this);
    status_publisher_ = transport_.Advertise<gz::msgs::Boolean>(
      "/simulation/robot_released");
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (release_requested_.load() && !released_) {
      if (!removal_requested_) {
        ecm.RequestRemoveEntity(joint_entity_);
        removal_requested_ = true;
      } else if (!ecm.HasEntity(joint_entity_)) {
        released_ = true;
        ecm.RequestRemoveEntity(anchor_entity_, true);
      }
    }
    const double now = std::chrono::duration<double>(info.simTime).count();
    if (!status_sent_ || released_ != last_status_ || now < last_status_time_ ||
        now - last_status_time_ >= 0.25) {
      gz::msgs::Boolean status;
      status.set_data(released_);
      status_publisher_.Publish(status);
      last_status_ = released_;
      last_status_time_ = now;
      status_sent_ = true;
    }
  }

private:
  void OnCommand(const gz::msgs::Twist & command)
  {
    const double vx = command.linear().x();
    const double vy = command.linear().y();
    const double wz = command.angular().z();
    if (is_startup_motion_command(vx, vy, wz, command_deadband_)) {
      release_requested_.store(true);
    }
  }

  gz::transport::Node transport_;
  gz::transport::Node::Publisher status_publisher_;
  gz::sim::Entity anchor_entity_{gz::sim::kNullEntity};
  gz::sim::Entity joint_entity_{gz::sim::kNullEntity};
  std::string command_topic_{"/simulation/startup_cmd_vel"};
  double command_deadband_{0.0001};
  std::atomic_bool release_requested_{false};
  bool removal_requested_{false};
  bool released_{false};
  bool status_sent_{false};
  bool last_status_{false};
  double last_status_time_{0.0};
};
}  // namespace seu_sentry_sim_control

GZ_ADD_PLUGIN(seu_sentry_sim_control::StartupPoseLock,
  gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
