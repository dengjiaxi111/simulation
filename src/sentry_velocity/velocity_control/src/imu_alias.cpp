#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

using std::placeholders::_1;

class ImuAliasNode : public rclcpp::Node {
public:
  ImuAliasNode() : Node("imu_alias") {
    declare_parameter<std::string>("input_topic", "/livox/imu");
    declare_parameter<std::vector<std::string>>(
      "output_topics", std::vector<std::string>{});
    declare_parameter<std::string>("frame_id", "livox_frame");

    const auto input_topic = get_parameter("input_topic").as_string();
    output_topics_ = get_parameter("output_topics").as_string_array();
    output_frame_id_ = get_parameter("frame_id").as_string();

    for (const auto &topic : output_topics_) {
      pubs_.push_back(create_publisher<sensor_msgs::msg::Imu>(topic, rclcpp::SensorDataQoS()));
    }

    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic, rclcpp::SensorDataQoS(), std::bind(&ImuAliasNode::callback, this, _1));

    RCLCPP_INFO(get_logger(), "imu_alias: subscribing to '%s' publishing %zu topic(s)",
                input_topic.c_str(), pubs_.size());
  }

private:
  void callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!msg) {
      return;
    }
    auto out = *msg;
    out.header.frame_id = output_frame_id_;
    for (auto &pub : pubs_) {
      pub->publish(out);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> pubs_;
  std::vector<std::string> output_topics_;
  std::string output_frame_id_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuAliasNode>());
  rclcpp::shutdown();
  return 0;
}
