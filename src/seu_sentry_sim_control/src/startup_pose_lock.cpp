#include <atomic>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace seu_sentry_sim_control
{
class StartupPoseLock final : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(entity);
    if (sdf && sdf->HasElement("release_topic")) {
      release_topic_ = sdf->Get<std::string>("release_topic");
    }
    transport_.Subscribe(release_topic_, &StartupPoseLock::OnRelease, this);
  }

  void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager & ecm) override
  {
    if (!model_.Valid(ecm)) return;
    if (!pose_initialized_) {
      initial_pose_ = gz::sim::worldPose(model_.Entity(), ecm);
      pose_initialized_ = true;
    }
    if (released_.load()) {
      if (!release_applied_) {
        for (const auto joint : model_.Joints(ecm)) {
          ecm.SetComponentData<gz::sim::components::JointVelocityReset>(joint, {0.0});
        }
        release_applied_ = true;
      }
      return;
    }
    model_.SetWorldPoseCmd(ecm, initial_pose_);
    for (const auto link_entity : model_.Links(ecm)) {
      gz::sim::Link link(link_entity);
      link.SetLinearVelocity(ecm, gz::math::Vector3d::Zero);
      link.SetAngularVelocity(ecm, gz::math::Vector3d::Zero);
    }
    for (const auto joint : model_.Joints(ecm)) {
      ecm.SetComponentData<gz::sim::components::JointVelocityReset>(joint, {0.0});
    }
  }

private:
  void OnRelease(const gz::msgs::Boolean & message)
  {
    if (message.data()) released_.store(true);
  }

  gz::sim::Model model_;
  gz::transport::Node transport_;
  gz::math::Pose3d initial_pose_;
  std::string release_topic_{"/simulation/robot_release"};
  std::atomic_bool released_{false};
  bool pose_initialized_{false};
  bool release_applied_{false};
};
}  // namespace seu_sentry_sim_control

GZ_ADD_PLUGIN(seu_sentry_sim_control::StartupPoseLock,
  gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
