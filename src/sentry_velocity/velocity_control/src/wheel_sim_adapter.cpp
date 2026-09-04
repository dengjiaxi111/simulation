#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robots_msgs/msg/chassis_odom.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

namespace velocity_control
{

class WheelSimAdapter : public rclcpp::Node
{
public:
  WheelSimAdapter()
  : Node("wheel_sim_adapter")
  {
    gimbal_joint_name_ = declare_parameter<std::string>(
      "gimbal_joint_name", "gimbal_yaw_joint");
    gimbal_spin_speed_ = declare_parameter<double>("gimbal_spin_speed", 3.14);

    chassis_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel_chassis", 10);
    chassis_odom_pub_ = create_publisher<robots_msgs::msg::ChassisOdom>(
      "/ChassisOdom", 10);
    gimbal_cmd_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/model/sentry/joint/gimbal_yaw_joint/cmd_vel", 10);

    gimbal_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_gimbal", 10,
      std::bind(&WheelSimAdapter::on_gimbal_cmd, this, std::placeholders::_1));
    wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/wheel/odometry", rclcpp::SensorDataQoS(),
      std::bind(&WheelSimAdapter::on_wheel_odom, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&WheelSimAdapter::on_joint_state, this, std::placeholders::_1));

    gimbal_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&WheelSimAdapter::publish_gimbal_command, this));

    RCLCPP_INFO(
      get_logger(),
      "Wheel simulation adapter started: gimbal spin %.3f rad/s",
      gimbal_spin_speed_);
  }

private:
  double gimbal_angle() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::remainder(gimbal_angle_, 2.0 * 3.14159265358979323846);
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (std::size_t index = 0; index < msg->name.size(); ++index) {
      if (msg->name[index] == gimbal_joint_name_ && index < msg->position.size()) {
        std::lock_guard<std::mutex> lock(mutex_);
        gimbal_angle_ = msg->position[index];
        return;
      }
    }
  }

  void on_gimbal_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double yaw = gimbal_angle();
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);

    // The real controller accepts velocity in the rotating gimbal/base_link frame.
    // Gazebo's mecanum plugin accepts velocity in the chassis frame.
    geometry_msgs::msg::Twist chassis_cmd;
    chassis_cmd.linear.x = cosine * msg->linear.x - sine * msg->linear.y;
    chassis_cmd.linear.y = sine * msg->linear.x + cosine * msg->linear.y;
    chassis_cmd.linear.z = msg->linear.z;
    chassis_cmd.angular = msg->angular;
    chassis_cmd_pub_->publish(chassis_cmd);
  }

  void on_wheel_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double yaw = gimbal_angle();
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    const auto & chassis_twist = msg->twist.twist;

    // Convert the chassis-frame feedback back to the rotating base_link frame,
    // matching the feedback contract of the real embedded controller.
    robots_msgs::msg::ChassisOdom feedback;
    feedback.speed_x = static_cast<float>(
      cosine * chassis_twist.linear.x + sine * chassis_twist.linear.y);
    feedback.speed_y = static_cast<float>(
      -sine * chassis_twist.linear.x + cosine * chassis_twist.linear.y);
    feedback.speed_w = static_cast<float>(chassis_twist.angular.z);
    feedback.gimbal_angle = static_cast<float>(yaw * 180.0 / 3.14159265358979323846);
    chassis_odom_pub_->publish(feedback);
  }

  void publish_gimbal_command()
  {
    std_msgs::msg::Float64 command;
    command.data = gimbal_spin_speed_;
    gimbal_cmd_pub_->publish(command);
  }

  mutable std::mutex mutex_;
  std::string gimbal_joint_name_;
  double gimbal_spin_speed_{3.14};
  double gimbal_angle_{0.0};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr chassis_cmd_pub_;
  rclcpp::Publisher<robots_msgs::msg::ChassisOdom>::SharedPtr chassis_odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gimbal_cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr gimbal_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr gimbal_timer_;
};

}  // namespace velocity_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<velocity_control::WheelSimAdapter>());
  rclcpp::shutdown();
  return 0;
}
