#include "velocity_control/velocity_controller.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "control_toolbox/pid.hpp"

namespace my_sim
{
    VelocityController::VelocityController()
    : controller_interface::ControllerInterface(),
    x_(0.0),
    y_(0.0),
    theta_(0.0),
    left_target_vel_(0.0),//初始化目标速度
    right_target_vel_(0.0)

    {
    }

    controller_interface::CallbackReturn VelocityController::on_init()
    {
        try
        {
            auto_declare<std::string>("left_wheel_name","left_wheel_joint");
            auto_declare<std::string>("right_wheel_name","right_wheel_joint");
            auto_declare<std::string>("caster_wheel_name","caster_wheel_joint");
            auto_declare<double>("wheel_separation",0.34);
            auto_declare<double>("wheel_radius",0.07);
            auto_declare<double>("left_wheel_pid_p",1.0);
            auto_declare<double>("right_wheel_pid_p",1.0);
            auto_declare<double>("left_wheel_pid_i",0.05);
            auto_declare<double>("right_wheel_pid_i",0.05);
            auto_declare<double>("left_wheel_pid_d",0.01);
            auto_declare<double>("right_wheel_pid_d",0.01);
        }
        catch(const std::exception& e)
        {
           RCLCPP_ERROR(
            get_node()->get_logger(), "Exception during init: %s", e.what());
           return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }
    controller_interface::InterfaceConfiguration 
    VelocityController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.push_back(left_wheel_name_ + "/velocity");
        config.names.push_back(right_wheel_name_ + "/velocity");
        return config;
    }
    controller_interface::InterfaceConfiguration
    VelocityController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.push_back(left_wheel_name_ + "/position");
        config.names.push_back(left_wheel_name_ + "/velocity");
        config.names.push_back(right_wheel_name_ + "/position");
        config.names.push_back(right_wheel_name_ + "/velocity");
        config.names.push_back(caster_wheel_name_ + "/position");
        config.names.push_back(caster_wheel_name_ + "/velocity");
        return config;
    }
    controller_interface::CallbackReturn VelocityController::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        left_wheel_name_ = get_node()->get_parameter("left_wheel_name").as_string(); 
        right_wheel_name_ = get_node()->get_parameter("right_wheel_name").as_string();
        caster_wheel_name_ = get_node()->get_parameter("caster_wheel_name").as_string();
        wheel_separation_ = get_node()->get_parameter("wheel_separation").as_double();
        wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
        double left_wheel_p = get_node()->get_parameter("left_wheel_pid_p").as_double();
        double left_wheel_i = get_node()->get_parameter("left_wheel_pid_i").as_double();
        double left_wheel_d = get_node()->get_parameter("left_wheel_pid_d").as_double();
        double right_wheel_p = get_node()->get_parameter("right_wheel_pid_p").as_double();
        double right_wheel_i = get_node()->get_parameter("right_wheel_pid_i").as_double();
        double right_wheel_d = get_node()->get_parameter("right_wheel_pid_d").as_double();
        left_wheel_pid_.initPid(left_wheel_p, left_wheel_i, left_wheel_d, 0.0, 20.0);  //pid 积分上下限暂定为0.0
        right_wheel_pid_.initPid(right_wheel_p, right_wheel_i, right_wheel_d, 0.0, 20.0);
        RCLCPP_INFO(get_node()->get_logger(), 
         "Configured with left_wheel: %s, right_wheel: %s,  separation: %.3f, radius: %.3f",
         left_wheel_name_.c_str(), right_wheel_name_.c_str(),
         wheel_separation_, wheel_radius_);
         cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
          "/cmd_vel", 10,
          [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) {
            received_cmd_vel_msg_.writeFromNonRT(msg);
          });
        odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
          "~/odom", 10);
        realtime_odom_pub_ = 
          std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
            odom_pub_); 
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(get_node());
        //重置里程计
        reset_odometry();
        return controller_interface::CallbackReturn::SUCCESS;
    }
    controller_interface::CallbackReturn VelocityController::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
        {
            left_wheel_velocity_command_.clear();
            right_wheel_velocity_command_.clear();
            for (auto & interface : command_interfaces_)
            {
              if (interface.get_prefix_name() == left_wheel_name_ &&
                  interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
                  {
                    left_wheel_velocity_command_.emplace_back(std::ref(interface));
                  }
              else if (interface.get_prefix_name() == right_wheel_name_ &&
                       interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
                       {
                        right_wheel_velocity_command_.emplace_back(std::ref(interface));
                       }

            }
             if (left_wheel_velocity_command_.empty() || right_wheel_velocity_command_.empty() )
             {
               RCLCPP_ERROR(get_node()->get_logger(), 
                 "Failed to get command interfaces!");
               return controller_interface::CallbackReturn::ERROR;
             }
            left_wheel_position_state_.clear();
            left_wheel_velocity_state_.clear();
            right_wheel_position_state_.clear();
            right_wheel_velocity_state_.clear();
            caster_velocity_state_.clear();
            caster_position_state_.clear();
            for (auto & interface : state_interfaces_)
            {
                if (interface.get_prefix_name() == left_wheel_name_)
                {
                    if (interface.get_interface_name() == hardware_interface::HW_IF_POSITION) 
                    {
                        left_wheel_position_state_.emplace_back(std::ref(interface));
                    }
                    else if (interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
                    {
                        left_wheel_velocity_state_.emplace_back(std::ref(interface));
                    }
                }
                else if (interface.get_prefix_name() == right_wheel_name_)
                {
                    if (interface.get_interface_name() == hardware_interface::HW_IF_POSITION)
                    {
                        right_wheel_position_state_.emplace_back(std::ref(interface));
                    }
                    else if (interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
                    {
                        right_wheel_velocity_state_.emplace_back(std::ref(interface));
                    }
      
                }
                else if (interface.get_prefix_name()==caster_wheel_name_)
                {
                    if (interface.get_interface_name() == hardware_interface::HW_IF_POSITION)
                    {
                        caster_position_state_.emplace_back(std::ref(interface));
                    }
                    else if (interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
                    {
                        caster_velocity_state_.emplace_back(std::ref(interface));
                    }
                    
                }
     
            }
            if (left_wheel_position_state_.empty() || left_wheel_velocity_state_.empty() ||
                right_wheel_position_state_.empty() || right_wheel_velocity_state_.empty()||caster_position_state_.empty()
                ||caster_velocity_state_.empty())
                {
                    RCLCPP_ERROR(get_node()->get_logger(), 
                       "Failed to get state interfaces!");
                    return controller_interface::CallbackReturn::ERROR;
                }
            reset_odometry();
            last_time_ = get_node()->now() ;
            left_wheel_pid_.reset();
            right_wheel_pid_.reset();
            RCLCPP_INFO(get_node()->get_logger(), "VelocityController activated !");
            return controller_interface::CallbackReturn::SUCCESS;
        }
    controller_interface::CallbackReturn VelocityController::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
        {
            left_wheel_velocity_command_[0].get().set_value(0.0);
            right_wheel_velocity_command_[0].get().set_value(0.0);
            left_wheel_pid_.reset();
            right_wheel_pid_.reset();
            return controller_interface::CallbackReturn::SUCCESS;
        }
    controller_interface::return_type VelocityController::update( 
        const rclcpp::Time & time, const rclcpp::Duration & )   
        {
            auto cmd_vel = received_cmd_vel_msg_.readFromRT();
            double linear_vel = 0.0;
            double angular_vel = 0.0;
            if (cmd_vel && *cmd_vel) 
            {
                linear_vel = (*cmd_vel)->linear.x;
                angular_vel = (*cmd_vel)->angular.z;
  
            }
            left_target_vel_ = (linear_vel - angular_vel * wheel_separation_ / 2.0) / wheel_radius_;
            right_target_vel_ = (linear_vel + angular_vel * wheel_separation_ / 2.0) / wheel_radius_;
            double actual_left_vel = left_wheel_velocity_state_[0].get().get_value();
            double actual_right_vel = right_wheel_velocity_state_[0].get().get_value();
            rclcpp::Duration dt_duration = time - last_time_;
            double dt = dt_duration.seconds();
            if(dt<1e-6){
                last_time_ = time;
                return controller_interface::return_type::OK;
            }
            //关键pid输出
            //double left_velocity = left_wheel_pid_.computeCommand(left_target_vel_-actual_left_vel,dt);
            //double right_velocity =right_wheel_pid_.computeCommand(right_target_vel_-actual_right_vel,dt);
            //发送输出速度给硬件
            left_wheel_velocity_command_[0].get().set_value(left_target_vel_);
            right_wheel_velocity_command_[0].get().set_value(right_target_vel_);

            double robot_linear = (actual_left_vel+actual_right_vel)*wheel_radius_/2.0;
            double robot_angular = (actual_right_vel-actual_left_vel)* wheel_radius_/wheel_separation_;
            double delta_theta = robot_angular * dt;
            double avg_theta = theta_ + delta_theta /2.0;
            double delta_x = robot_linear * cos(avg_theta)*dt;
            double delta_y = robot_linear *sin(avg_theta)*dt;
            x_ += delta_x;
            y_ += delta_y;
            theta_ +=delta_theta;
            theta_ = std::fmod(theta_ + M_PI,2 * M_PI) - M_PI;
            //publish_odometry(time, robot_linear, robot_angular);
            //publish_tf_transform(time);
            last_time_ = time;
            return controller_interface::return_type::OK;

        }
    void VelocityController::reset_odometry()
    {
        x_=0.0;
        y_=0.0;
        theta_=0.0;
    }
    void VelocityController::publish_tf_transform(const rclcpp::Time &current_time)
    {
        geometry_msgs::msg::TransformStamped tf_msgs;
        tf_msgs.header.stamp =current_time; 
        tf_msgs.header.frame_id = "odom";
        tf_msgs.child_frame_id = "base_link";
        tf_msgs.transform.translation.x = x_;
        tf_msgs.transform.translation.y = y_;
        tf_msgs.transform.translation.z = 0.0;
        tf_msgs.transform.rotation.x = 0.0;
        tf_msgs.transform.rotation.y = 0.0;
        tf_msgs.transform.rotation.z = sin(theta_/2.0);
        tf_msgs.transform.rotation.w = cos(theta_/2.0);
        tf_broadcaster_->sendTransform(tf_msgs);



    }
    void VelocityController::publish_odometry(
        const rclcpp::Time & current_time,
        double linear_vel,
        double angular_vel)
        {
            if (!realtime_odom_pub_->trylock()) {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to lock realtime odom publisher!");
                return;}
            auto &odom_msg = realtime_odom_pub_->msg_;
            odom_msg.header.stamp = current_time;
            odom_msg.header.frame_id ="odom";
            odom_msg.child_frame_id = "base_link";
            odom_msg.pose.pose.position.x = x_;
            odom_msg.pose.pose.position.y = y_;
            odom_msg.pose.pose.position.z = 0.0;
            odom_msg.pose.pose.orientation.x = 0.0;
            odom_msg.pose.pose.orientation.y = 0.0;
            odom_msg.pose.pose.orientation.z = sin(theta_/2.0);
            odom_msg.pose.pose.orientation.w = cos(theta_/2.0);
            odom_msg.twist.twist.linear.x = linear_vel;
            odom_msg.twist.twist.angular.z = angular_vel;
            realtime_odom_pub_->unlockAndPublish();
        }


}
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  my_sim::VelocityController, controller_interface::ControllerInterface)


