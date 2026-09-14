#include <cmath>
#include <functional>
#include <mutex>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robots_msgs/msg/chassis_odom.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace velocity_control
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}

// Feedback-only adapter. Command selection and keyboard coordinate conversion
// deliberately live in cmd_vel_mux so navigation commands are never transformed
// a second time.
class WheelSimAdapter : public rclcpp::Node
{
public:
  WheelSimAdapter()
  : Node("wheel_sim_adapter")
  {
    gimbal_joint_name_ = declare_parameter<std::string>(
      "gimbal_joint_name", "gimbal_yaw_joint");
    gimbal_mount_yaw_ = declare_parameter<double>("gimbal_mount_yaw", 1.6032);
    odometry_topic_ = declare_parameter<std::string>("odometry_topic", "/wheel/odometry");
    chassis_odometry_topic_ =
      declare_parameter<std::string>("chassis_odometry_topic", "/ChassisOdom");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");

    chassis_odom_pub_ = create_publisher<robots_msgs::msg::ChassisOdom>(
      chassis_odometry_topic_, 10);
    wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, rclcpp::SensorDataQoS(),
      std::bind(&WheelSimAdapter::on_wheel_odom, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&WheelSimAdapter::on_joint_state, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Wheel feedback adapter publishing %s",
      chassis_odometry_topic_.c_str());
  }

private:
  double gimbal_angle_locked() const
  {
    return std::remainder(gimbal_mount_yaw_ + gimbal_joint_position_, 2.0 * kPi);
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (std::size_t index = 0; index < msg->name.size(); ++index) {
      if (msg->name[index] == gimbal_joint_name_ && index < msg->position.size()) {
        std::lock_guard<std::mutex> lock(mutex_);
        gimbal_joint_position_ = msg->position[index];
        return;
      }
    }
  }

  void on_wheel_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double yaw = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      yaw = gimbal_angle_locked();
    }
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    const auto & twist = msg->twist.twist;

    robots_msgs::msg::ChassisOdom feedback;
    feedback.speed_x = static_cast<float>(cosine * twist.linear.x + sine * twist.linear.y);
    feedback.speed_y = static_cast<float>(-sine * twist.linear.x + cosine * twist.linear.y);
    feedback.speed_w = static_cast<float>(twist.angular.z);
    feedback.gimbal_angle = static_cast<float>(yaw * 180.0 / kPi);
    chassis_odom_pub_->publish(feedback);
  }

  mutable std::mutex mutex_;
  std::string gimbal_joint_name_;
  std::string odometry_topic_;
  std::string chassis_odometry_topic_;
  std::string joint_state_topic_;
  double gimbal_mount_yaw_{1.6032};
  double gimbal_joint_position_{0.0};
  rclcpp::Publisher<robots_msgs::msg::ChassisOdom>::SharedPtr chassis_odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};
}  // namespace velocity_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<velocity_control::WheelSimAdapter>());
  rclcpp::shutdown();
  return 0;
}
