#include <chrono>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace velocity_control
{

class CmdVelMux : public rclcpp::Node
{
public:
  CmdVelMux()
  : Node("cmd_vel_mux")
  {
    navigation_topic_ =
      declare_parameter<std::string>("navigation_topic", "/navigation/cmd_vel");
    keyboard_topic_ = declare_parameter<std::string>("keyboard_topic", "/keyboard/cmd_vel");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cmd_vel_chassis");
    mode_topic_ = declare_parameter<std::string>("mode_topic", "/control_mode");
    mode_ = declare_parameter<std::string>("default_mode", "keyboard");
    command_timeout_ = declare_parameter<double>("command_timeout", 0.25);
    publish_rate_ = declare_parameter<double>("publish_rate", 50.0);

    if (mode_ != "keyboard" && mode_ != "navigation") {
      throw std::runtime_error("default_mode must be 'keyboard' or 'navigation'");
    }
    if (command_timeout_ <= 0.0 || publish_rate_ <= 0.0) {
      throw std::runtime_error("command_timeout and publish_rate must be positive");
    }

    output_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
    navigation_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      navigation_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        navigation_command_ = *msg;
        navigation_stamp_ = now();
        have_navigation_command_ = true;
      });
    keyboard_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      keyboard_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        keyboard_command_ = *msg;
        keyboard_stamp_ = now();
        have_keyboard_command_ = true;
      });
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      mode_topic_, 10, std::bind(&CmdVelMux::on_mode, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CmdVelMux::publish_selected_command, this));
    publish_stop();
    RCLCPP_INFO(
      get_logger(), "cmd_vel_mux started in %s mode: %s + %s -> %s", mode_.c_str(),
      keyboard_topic_.c_str(), navigation_topic_.c_str(), output_topic_.c_str());
  }

private:
  void on_mode(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data != "keyboard" && msg->data != "navigation") {
      RCLCPP_WARN(get_logger(), "Ignoring unknown control mode '%s'", msg->data.c_str());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (mode_ == msg->data) {
        return;
      }
      mode_ = msg->data;
      // A mode switch must never revive a command received in the old mode.
      have_keyboard_command_ = false;
      have_navigation_command_ = false;
    }
    publish_stop();
    RCLCPP_INFO(get_logger(), "Control mode changed to %s", msg->data.c_str());
  }

  bool command_is_fresh(bool available, const rclcpp::Time & stamp) const
  {
    return available && (now() - stamp).seconds() <= command_timeout_;
  }

  void publish_selected_command()
  {
    geometry_msgs::msg::Twist selected;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (mode_ == "navigation") {
        if (command_is_fresh(have_navigation_command_, navigation_stamp_)) {
          selected = navigation_command_;
        }
      } else if (command_is_fresh(have_keyboard_command_, keyboard_stamp_)) {
        selected = keyboard_command_;
      }
    }
    output_pub_->publish(selected);
  }

  void publish_stop()
  {
    output_pub_->publish(geometry_msgs::msg::Twist{});
  }

  mutable std::mutex mutex_;
  std::string navigation_topic_;
  std::string keyboard_topic_;
  std::string output_topic_;
  std::string mode_topic_;
  std::string mode_{"keyboard"};
  double command_timeout_{0.25};
  double publish_rate_{50.0};
  bool have_navigation_command_{false};
  bool have_keyboard_command_{false};
  geometry_msgs::msg::Twist navigation_command_;
  geometry_msgs::msg::Twist keyboard_command_;
  rclcpp::Time navigation_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time keyboard_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr navigation_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr keyboard_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace velocity_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<velocity_control::CmdVelMux>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("cmd_vel_mux"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
