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
#include "std_msgs/msg/string.hpp"

namespace velocity_control
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}

class WheelSimAdapter : public rclcpp::Node
{
public:
  WheelSimAdapter()
  : Node("wheel_sim_adapter")
  {
    gimbal_joint_name_ = declare_parameter<std::string>(
      "gimbal_joint_name", "gimbal_yaw_joint");
    gimbal_mount_yaw_ = declare_parameter<double>("gimbal_mount_yaw", 1.6032);
    gimbal_spin_speed_ = declare_parameter<double>("gimbal_spin_speed", 3.14);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.25);

    chassis_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_chassis", 10);
    chassis_odom_pub_ = create_publisher<robots_msgs::msg::ChassisOdom>("/ChassisOdom", 10);
    gimbal_cmd_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/model/sentry/joint/gimbal_yaw_joint/cmd_vel", 10);

    navigation_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_gimbal", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        navigation_command_ = *msg;
        navigation_command_stamp_ = now();
        have_navigation_command_ = true;
      });
    keyboard_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/keyboard/cmd_vel_gimbal", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        keyboard_command_ = *msg;
        keyboard_command_stamp_ = now();
        have_keyboard_command_ = true;
      });
    keyboard_gimbal_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/keyboard/gimbal_cmd_vel", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        keyboard_gimbal_command_ = msg->data;
        keyboard_gimbal_stamp_ = now();
        have_keyboard_gimbal_command_ = true;
      });
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/control_mode", 10,
      std::bind(&WheelSimAdapter::on_mode, this, std::placeholders::_1));
    wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/wheel/odometry", rclcpp::SensorDataQoS(),
      std::bind(&WheelSimAdapter::on_wheel_odom, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&WheelSimAdapter::on_joint_state, this, std::placeholders::_1));

    command_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&WheelSimAdapter::publish_selected_commands, this));

    publish_stop();
    RCLCPP_INFO(
      get_logger(),
      "Wheel adapter started in keyboard mode: mount yaw %.4f rad, navigation gimbal spin %.3f rad/s",
      gimbal_mount_yaw_, gimbal_spin_speed_);
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

  void on_mode(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data != "keyboard" && msg->data != "navigation") {
      RCLCPP_WARN(get_logger(), "Ignoring unknown control mode '%s'", msg->data.c_str());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (mode_ == msg->data) return;
      mode_ = msg->data;
      have_keyboard_command_ = false;
      have_keyboard_gimbal_command_ = false;
      have_navigation_command_ = false;
    }
    publish_stop();
    RCLCPP_INFO(get_logger(), "Control mode changed to %s", msg->data.c_str());
  }

  geometry_msgs::msg::Twist to_chassis_command(
    const geometry_msgs::msg::Twist & command, double yaw) const
  {
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    geometry_msgs::msg::Twist result;
    result.linear.x = cosine * command.linear.x - sine * command.linear.y;
    result.linear.y = sine * command.linear.x + cosine * command.linear.y;
    result.linear.z = command.linear.z;
    result.angular = command.angular;
    return result;
  }

  bool command_is_fresh(bool available, const rclcpp::Time & stamp) const
  {
    return available && (now() - stamp).seconds() <= command_timeout_;
  }

  void publish_selected_commands()
  {
    geometry_msgs::msg::Twist selected_command;
    double gimbal_command = 0.0;
    double yaw = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      yaw = gimbal_angle_locked();
      if (mode_ == "keyboard") {
        if (command_is_fresh(have_keyboard_command_, keyboard_command_stamp_)) {
          selected_command = keyboard_command_;
        }
        if (command_is_fresh(have_keyboard_gimbal_command_, keyboard_gimbal_stamp_)) {
          gimbal_command = keyboard_gimbal_command_;
        }
      } else {
        if (command_is_fresh(have_navigation_command_, navigation_command_stamp_)) {
          selected_command = navigation_command_;
        }
        gimbal_command = gimbal_spin_speed_;
      }
    }

    chassis_cmd_pub_->publish(to_chassis_command(selected_command, yaw));
    std_msgs::msg::Float64 message;
    message.data = gimbal_command;
    gimbal_cmd_pub_->publish(message);
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

  void publish_stop()
  {
    chassis_cmd_pub_->publish(geometry_msgs::msg::Twist{});
    std_msgs::msg::Float64 stop;
    stop.data = 0.0;
    gimbal_cmd_pub_->publish(stop);
  }

  mutable std::mutex mutex_;
  std::string gimbal_joint_name_;
  std::string mode_{"keyboard"};
  double gimbal_mount_yaw_{1.6032};
  double gimbal_spin_speed_{3.14};
  double command_timeout_{0.25};
  double gimbal_joint_position_{0.0};
  double keyboard_gimbal_command_{0.0};
  bool have_navigation_command_{false};
  bool have_keyboard_command_{false};
  bool have_keyboard_gimbal_command_{false};
  geometry_msgs::msg::Twist navigation_command_;
  geometry_msgs::msg::Twist keyboard_command_;
  rclcpp::Time navigation_command_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time keyboard_command_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time keyboard_gimbal_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr chassis_cmd_pub_;
  rclcpp::Publisher<robots_msgs::msg::ChassisOdom>::SharedPtr chassis_odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gimbal_cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr navigation_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr keyboard_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr keyboard_gimbal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr command_timer_;
};

}  // namespace velocity_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<velocity_control::WheelSimAdapter>());
  rclcpp::shutdown();
  return 0;
}
