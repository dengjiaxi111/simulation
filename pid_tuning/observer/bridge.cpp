#include <mutex>
#include <gz/transport/Node.hh>
#include <gz/msgs/double_v.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
int main(int argc,char**argv){
 rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("pid_tuning_telemetry_bridge");auto pub=node->create_publisher<std_msgs::msg::Float64MultiArray>("/pid_tuning/motor_telemetry",rclcpp::SensorDataQoS());gz::transport::Node gznode;
 gznode.Subscribe<gz::msgs::Double_V>("/pid_tuning/motor_telemetry",[pub](const gz::msgs::Double_V&m){std_msgs::msg::Float64MultiArray out;for(double v:m.data())out.data.push_back(v);pub->publish(out);});rclcpp::spin(node);rclcpp::shutdown();
}
