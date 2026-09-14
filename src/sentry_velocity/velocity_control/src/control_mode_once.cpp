#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ControlModeOnce : public rclcpp::Node
{
public:
  ControlModeOnce()
  : Node("control_mode_once")
  {
    mode_ = declare_parameter<std::string>("mode", "navigation");
    publisher_ = create_publisher<std_msgs::msg::String>("/control_mode", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
      std_msgs::msg::String message;
      message.data = mode_;
      publisher_->publish(message);
      if (++publish_count_ >= 4) {
        RCLCPP_INFO(get_logger(), "Selected control mode: %s", mode_.c_str());
        rclcpp::shutdown();
      }
    });
  }

private:
  std::string mode_;
  int publish_count_{0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlModeOnce>());
  return 0;
}
