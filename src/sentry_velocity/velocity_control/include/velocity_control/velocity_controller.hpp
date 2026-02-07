#ifndef VELOCITY_CONTROL__VELOCITY_CONTROLLER_HPP_
#define VELOCITY_CONTROL__VELOCITY_CONTROLLER_HPP_
#include <memory>
#include <string>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "control_toolbox/pid.hpp"
namespace my_sim
{
    class VelocityController : public controller_interface::ControllerInterface
    {
    public:
        VelocityController();
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::return_type update(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;
    private:
        std::string left_wheel_name_;
        std::string right_wheel_name_;
        std::string caster_wheel_name_;
        double wheel_radius_;
        double wheel_separation_;
        // 命令接口引用
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> 
         left_wheel_velocity_command_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> 
         right_wheel_velocity_command_;
        // 状态接口引用
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> 
         left_wheel_position_state_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> 
         left_wheel_velocity_state_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> 
         right_wheel_position_state_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> 
         right_wheel_velocity_state_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> 
         caster_velocity_state_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> 
         caster_position_state_;
         // ROS通信
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> 
          received_cmd_vel_msg_;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub_;
        std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> 
          realtime_odom_pub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        // 里程计状态
        double x_;
        double y_;
        double theta_;
        rclcpp::Time last_time_;
        void reset_odometry();
        void publish_odometry(const rclcpp::Time & current_time,
           double linear_vel,
           double angular_vel);
        void publish_tf_transform(const rclcpp::Time&current_time);
        control_toolbox::Pid left_wheel_pid_;
        control_toolbox::Pid right_wheel_pid_;
        double left_target_vel_;
        double right_target_vel_;
    };
}
#endif
