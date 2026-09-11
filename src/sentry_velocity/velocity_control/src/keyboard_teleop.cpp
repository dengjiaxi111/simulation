#include <chrono>
#include <cctype>
#include <iostream>
#include <stdexcept>
#include <string>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

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
    linear_speed_ = declare_parameter<double>("linear_speed", 1.0);
    chassis_yaw_speed_ = declare_parameter<double>("chassis_yaw_speed", 1.2);
    gimbal_yaw_speed_ = declare_parameter<double>(
      "gimbal_yaw_speed", 1.7453292519943295);
    gimbal_key_timeout_ = declare_parameter<double>("gimbal_key_timeout", 0.20);

    keyboard_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/keyboard/cmd_vel_gimbal", 10);
    keyboard_gimbal_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/keyboard/gimbal_cmd_vel", 10);
    mode_pub_ = create_publisher<std_msgs::msg::String>("/control_mode", 10);
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/control_mode", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "keyboard" || msg->data == "navigation") {
          mode_ = msg->data;
        }
      });
  }

  void run()
  {
    TerminalRawMode raw_mode;
    print_help();
    rclcpp::WallRate rate(50.0);
    while (rclcpp::ok() && !exit_requested_) {
      char key = 0;
      while (read_key(key)) handle_key(key);
      publish_state();
      rclcpp::spin_some(shared_from_this());
      rate.sleep();
    }
    publish_stop();
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
    key = static_cast<char>(std::tolower(static_cast<unsigned char>(key)));
    if (key == 'k') {
      publish_stop();
      publish_mode("keyboard");
      return;
    }
    if (key == 'n') {
      publish_stop();
      publish_mode("navigation");
      return;
    }
    if (key == 'x') {
      exit_requested_ = true;
      return;
    }

    geometry_msgs::msg::Twist next_command;
    double next_gimbal = 0.0;
    bool recognized = true;
    switch (key) {
      case 'w': next_command.linear.x = linear_speed_; break;
      case 's': next_command.linear.x = -linear_speed_; break;
      case 'a': next_command.linear.y = linear_speed_; break;
      case 'd': next_command.linear.y = -linear_speed_; break;
      case 'j': next_command.angular.z = chassis_yaw_speed_; break;
      case 'l': next_command.angular.z = -chassis_yaw_speed_; break;
      case 'q': next_gimbal = gimbal_yaw_speed_; break;
      case 'e': next_gimbal = -gimbal_yaw_speed_; break;
      case ' ': break;
      default: recognized = false; break;
    }
    if (!recognized) return;

    if (key == 'q' || key == 'e') {
      // Gimbal rotation is independent of the latched chassis command.
      gimbal_command_ = next_gimbal;
      gimbal_key_active_ = true;
      last_gimbal_key_time_ = std::chrono::steady_clock::now();
    } else {
      keyboard_command_ = next_command;
      gimbal_command_ = 0.0;
      gimbal_key_active_ = false;
    }
  }

  void publish_state()
  {
    if (mode_ != "keyboard") {
      publish_stop();
      return;
    }
    keyboard_cmd_pub_->publish(keyboard_command_);
    std_msgs::msg::Float64 gimbal;
    const bool gimbal_fresh = gimbal_key_active_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_gimbal_key_time_).count() <=
      gimbal_key_timeout_;
    gimbal.data = gimbal_fresh ? gimbal_command_ : 0.0;
    keyboard_gimbal_pub_->publish(gimbal);
  }

  void publish_stop()
  {
    keyboard_command_ = geometry_msgs::msg::Twist{};
    gimbal_command_ = 0.0;
    keyboard_cmd_pub_->publish(keyboard_command_);
    std_msgs::msg::Float64 gimbal;
    gimbal.data = 0.0;
    keyboard_gimbal_pub_->publish(gimbal);
  }

  void publish_mode(const std::string & mode)
  {
    mode_ = mode;
    std_msgs::msg::String message;
    message.data = mode;
    mode_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "Control mode: %s", mode.c_str());
  }

  static void print_help()
  {
    std::cout << "\nSEU 仿真键盘控制（速度以云台/base_link坐标系表达）\n"
              << "K: 键盘模式  N: 导航模式（不恢复旧目标）\n"
              << "W/S: 云台前进/后退  A/D: 云台左移/右移（1.0 m/s）\n"
              << "J/L: 底盘逆/顺时针  Q/E: 云台逆/顺时针（100 deg/s）\n"
              << "控制键单击后持续生效，按其他控制键可替换当前命令\n"
              << "空格: 急停  X: 停止并退出键盘终端\n" << std::flush;
  }

  double linear_speed_{1.0};
  double chassis_yaw_speed_{1.2};
  double gimbal_yaw_speed_{1.7453292519943295};
  double gimbal_key_timeout_{0.20};
  bool exit_requested_{false};
  bool gimbal_key_active_{false};
  std::string mode_{"keyboard"};
  std::chrono::steady_clock::time_point last_gimbal_key_time_{};
  geometry_msgs::msg::Twist keyboard_command_;
  double gimbal_command_{0.0};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr keyboard_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr keyboard_gimbal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
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
