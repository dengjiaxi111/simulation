#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

class TerminalRawMode
{
public:
  TerminalRawMode()
  {
    if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &original_) != 0) {
      throw std::runtime_error("keyboard_teleop requires an interactive terminal");
    }
    termios raw = original_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      throw std::runtime_error("failed to enable raw terminal mode");
    }
  }

  ~TerminalRawMode() {tcsetattr(STDIN_FILENO, TCSANOW, &original_);}

private:
  termios original_{};
};

class KeyboardTeleop : public rclcpp::Node
{
public:
  KeyboardTeleop()
  : Node("keyboard_teleop")
  {
    linear_speed_ = declare_parameter<double>("linear_speed", 0.8);
    chassis_yaw_speed_ = declare_parameter<double>("chassis_yaw_speed", 1.2);
    gimbal_yaw_speed_ = declare_parameter<double>("gimbal_yaw_speed", 1.5);
    chassis_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_chassis", 10);
    gimbal_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/model/sentry/joint/gimbal_yaw_joint/cmd_vel", 10);
    active_pub_ = create_publisher<std_msgs::msg::Bool>("/keyboard_control_active", 10);
  }

  void run()
  {
    TerminalRawMode raw_mode;
    print_help();
    rclcpp::WallRate rate(20.0);
    while (rclcpp::ok() && !exit_requested_) {
      char key = 0;
      if (read_key(key)) handle_key(key);
      publish_state();
      rclcpp::spin_some(shared_from_this());
      rate.sleep();
    }
    stop_and_release();
  }

private:
  static bool read_key(char & key)
  {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    timeval timeout{0, 0};
    if (select(STDIN_FILENO + 1, &set, nullptr, nullptr, &timeout) <= 0) return false;
    return read(STDIN_FILENO, &key, 1) == 1;
  }

  void handle_key(char key)
  {
    if (key >= 'A' && key <= 'Z') key = static_cast<char>(key - 'A' + 'a');
    geometry_msgs::msg::Twist next_chassis;
    double next_gimbal = 0.0;
    bool recognized = true;

    switch (key) {
      case 'w': next_chassis.linear.x = linear_speed_; break;
      case 's': next_chassis.linear.x = -linear_speed_; break;
      case 'a': next_chassis.linear.y = linear_speed_; break;
      case 'd': next_chassis.linear.y = -linear_speed_; break;
      case 'j': next_chassis.angular.z = chassis_yaw_speed_; break;
      case 'l': next_chassis.angular.z = -chassis_yaw_speed_; break;
      case 'q': next_gimbal = gimbal_yaw_speed_; break;
      case 'e': next_gimbal = -gimbal_yaw_speed_; break;
      case ' ': break;
      case 'x': exit_requested_ = true; break;
      default: recognized = false; break;
    }
    if (!recognized) return;
    active_ = !exit_requested_;
    chassis_command_ = next_chassis;
    gimbal_command_ = next_gimbal;
  }

  void publish_state()
  {
    std_msgs::msg::Bool active;
    active.data = active_;
    active_pub_->publish(active);
    if (!active_) return;
    chassis_pub_->publish(chassis_command_);
    std_msgs::msg::Float64 gimbal;
    gimbal.data = gimbal_command_;
    gimbal_pub_->publish(gimbal);
  }

  void stop_and_release()
  {
    chassis_pub_->publish(geometry_msgs::msg::Twist{});
    std_msgs::msg::Float64 stop_gimbal;
    stop_gimbal.data = 0.0;
    gimbal_pub_->publish(stop_gimbal);
    std_msgs::msg::Bool active;
    active.data = false;
    for (int i = 0; i < 3; ++i) {
      active_pub_->publish(active);
      rclcpp::sleep_for(std::chrono::milliseconds(30));
    }
  }

  static void print_help()
  {
    std::cout << "\nSEU 仿真键盘控制（按键即接管）\n"
              << "W/S: 前进/后退  A/D: 左移/右移\n"
              << "J/L: 底盘逆/顺时针  Q/E: 云台逆/顺时针\n"
              << "空格: 急停并保持键盘接管\n"
              << "X: 停止、退出键盘控制并恢复导航/自动云台\n" << std::flush;
  }

  double linear_speed_{0.8};
  double chassis_yaw_speed_{1.2};
  double gimbal_yaw_speed_{1.5};
  bool active_{false};
  bool exit_requested_{false};
  geometry_msgs::msg::Twist chassis_command_;
  double gimbal_command_{0.0};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr chassis_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gimbal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardTeleop>();
  try {
    node->run();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(node->get_logger(), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
