#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

namespace velocity_control
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}

class KeyboardSimAdapter : public rclcpp::Node
{
public:
  KeyboardSimAdapter()
  : Node("keyboard_sim_adapter")
  {
    gimbal_joint_name_ = declare_parameter<std::string>(
      "gimbal_joint_name", "gimbal_yaw_joint");
    gimbal_mount_yaw_ = declare_parameter<double>("gimbal_mount_yaw", 1.6032);
    navigation_gimbal_speed_ =
      declare_parameter<double>("navigation_gimbal_speed", 100.0 * kPi / 180.0);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.25);
    raw_keyboard_topic_ = declare_parameter<std::string>(
      "raw_keyboard_topic", "/keyboard/cmd_vel_gimbal");
    keyboard_output_topic_ =
      declare_parameter<std::string>("keyboard_output_topic", "/keyboard/cmd_vel");

    keyboard_output_pub_ =
      create_publisher<geometry_msgs::msg::Twist>(keyboard_output_topic_, 10);
    gimbal_output_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/model/sentry/joint/gimbal_yaw_joint/cmd_vel", 10);
    keyboard_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      raw_keyboard_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        keyboard_command_ = *msg;
        keyboard_stamp_ = now();
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
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "keyboard" || msg->data == "navigation") {
          std::lock_guard<std::mutex> lock(mutex_);
          mode_ = msg->data;
        }
      });
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&KeyboardSimAdapter::on_joint_state, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&KeyboardSimAdapter::update, this));
  }

private:
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

  geometry_msgs::msg::Twist to_chassis(
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

  bool fresh(bool available, const rclcpp::Time & stamp) const
  {
    return available && (now() - stamp).seconds() <= command_timeout_;
  }

  void update()
  {
    geometry_msgs::msg::Twist keyboard_output;
    double gimbal_output = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (mode_ == "keyboard") {
        if (fresh(have_keyboard_command_, keyboard_stamp_)) {
          const double yaw = std::remainder(
            gimbal_mount_yaw_ + gimbal_joint_position_, 2.0 * kPi);
          keyboard_output = to_chassis(keyboard_command_, yaw);
        }
        if (fresh(have_keyboard_gimbal_command_, keyboard_gimbal_stamp_)) {
          gimbal_output = keyboard_gimbal_command_;
        }
      } else {
        // Physical gimbal motion is independent of cmd_vel arbitration.
        gimbal_output = navigation_gimbal_speed_;
      }
    }
    keyboard_output_pub_->publish(keyboard_output);
    std_msgs::msg::Float64 gimbal_message;
    gimbal_message.data = gimbal_output;
    gimbal_output_pub_->publish(gimbal_message);
  }

  mutable std::mutex mutex_;
  std::string gimbal_joint_name_;
  std::string raw_keyboard_topic_;
  std::string keyboard_output_topic_;
  std::string mode_{"keyboard"};
  double gimbal_mount_yaw_{1.6032};
  double navigation_gimbal_speed_{3.14};
  double command_timeout_{0.25};
  double gimbal_joint_position_{0.0};
  double keyboard_gimbal_command_{0.0};
  bool have_keyboard_command_{false};
  bool have_keyboard_gimbal_command_{false};
  geometry_msgs::msg::Twist keyboard_command_;
  rclcpp::Time keyboard_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time keyboard_gimbal_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr keyboard_output_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gimbal_output_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr keyboard_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr keyboard_gimbal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace velocity_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<velocity_control::KeyboardSimAdapter>());
  rclcpp::shutdown();
  return 0;
}
