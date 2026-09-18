#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

namespace seu_sentry_sim_control
{
namespace
{
constexpr std::size_t kModuleCount = 4;
constexpr double kPi = 3.14159265358979323846;

double normalize_angle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}

template<typename T>
std::array<T, kModuleCount> to_array(
  const std::vector<T> & values, const std::string & parameter_name)
{
  if (values.size() != kModuleCount) {
    throw std::runtime_error(
            "Parameter '" + parameter_name + "' must contain exactly four entries");
  }
  std::array<T, kModuleCount> result;
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}
}  // namespace

struct Module
{
  std::string steer_joint;
  std::string wheel_joint;
  double x{0.0};
  double y{0.0};
  double steer_direction_sign{1.0};
  double steer_zero_offset{0.0};
  double drive_sign{1.0};
  double steer_position{0.0};
  double steer_command{0.0};
  double wheel_command{0.0};
  double initial_angle{0.0};
  double steer_velocity{0.0};
  double wheel_velocity{0.0};
  bool has_initial_angle{false};
  bool has_wheel{false};
  bool has_position{false};
};

class SwerveSimController : public rclcpp::Node
{
public:
  SwerveSimController()
  : Node("swerve_sim_controller")
  {
    command_topic_ = declare_parameter<std::string>("command_topic", "/cmd_vel_chassis");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");
    control_rate_ = declare_parameter<double>("control_rate", 50.0);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.2);
    command_deadband_ = declare_parameter<double>("command_deadband", 1.0e-4);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.0535);
    max_wheel_speed_ = declare_parameter<double>("max_wheel_angular_speed", 50.0);

    const auto module_x = to_array(
      declare_parameter<std::vector<double>>(
        "module_x", {-0.159, 0.159, -0.159, 0.159}), "module_x");
    const auto module_y = to_array(
      declare_parameter<std::vector<double>>(
        "module_y", {-0.159, -0.159, 0.159, 0.159}), "module_y");
    const auto steer_names = to_array(
      declare_parameter<std::vector<std::string>>(
        "steer_joint_names",
        {"front_left_steer_joint", "front_right_steer_joint",
          "rear_left_steer_joint", "rear_right_steer_joint"}),
      "steer_joint_names");
    const auto wheel_names = to_array(
      declare_parameter<std::vector<std::string>>(
        "wheel_joint_names",
        {"front_left_wheel_joint", "front_right_wheel_joint",
          "rear_left_wheel_joint", "rear_right_wheel_joint"}),
      "wheel_joint_names");
    const auto direction_signs = to_array(
      declare_parameter<std::vector<double>>(
        "steer_direction_signs", {1.0, 1.0, 1.0, 1.0}),
      "steer_direction_signs");
    const auto zero_offsets = to_array(
      declare_parameter<std::vector<double>>(
        "steer_zero_offsets", {-0.785398, 0.785398, -2.356194, 2.356194}),
      "steer_zero_offsets");
    const auto drive_signs = to_array(
      declare_parameter<std::vector<double>>("drive_signs", {1.0, 1.0, 1.0, 1.0}),
      "drive_signs");

    validate_parameters();
    for (std::size_t i = 0; i < kModuleCount; ++i) {
      modules_[i].steer_joint = steer_names[i];
      modules_[i].wheel_joint = wheel_names[i];
      modules_[i].x = module_x[i];
      modules_[i].y = module_y[i];
      modules_[i].steer_direction_sign = direction_signs[i];
      modules_[i].steer_zero_offset = zero_offsets[i];
      modules_[i].drive_sign = drive_signs[i];
      steer_publishers_[i] = create_publisher<std_msgs::msg::Float64>(
        "/swerve/" + modules_[i].steer_joint + "/target_angle", 10);
      wheel_publishers_[i] = create_publisher<std_msgs::msg::Float64>(
        "/swerve/" + modules_[i].wheel_joint + "/target_speed", 10);
    }

    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      command_topic_, 10,
      std::bind(&SwerveSimController::on_command, this, std::placeholders::_1));
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SwerveSimController::on_joint_state, this, std::placeholders::_1));
    const auto period = std::chrono::duration<double>(1.0 / control_rate_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SwerveSimController::control_update, this));

    last_command_time_ = now();
    last_control_time_ = now();
    RCLCPP_INFO(
      get_logger(),
      "Swerve controller ready: %.1f Hz, wheel radius %.4f m, timeout %.3f s",
      control_rate_, wheel_radius_, command_timeout_);
  }

private:
  void validate_parameters() const
  {
    if (control_rate_ <= 0.0 || wheel_radius_ <= 0.0 || command_timeout_ <= 0.0) {
      throw std::runtime_error("control_rate, wheel_radius and command_timeout must be positive");
    }
    if (max_wheel_speed_ <= 0.0 || !std::isfinite(max_wheel_speed_)) {
      throw std::runtime_error("max_wheel_angular_speed must be finite and positive");
    }
    for (const auto & module : modules_) {
      if (std::abs(module.steer_direction_sign) != 1.0) {
        throw std::runtime_error("steer_direction_signs entries must be either -1 or 1");
      }
    }
  }

  void on_command(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    target_command_ = *message;
    last_command_time_ = now();
    has_command_ = true;
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    const auto stamp = now();
    for (auto & module : modules_) {
      // Each cycle requires valid position AND velocity feedback for both motors.
      module.has_position = false;
      module.has_wheel = false;
      for (std::size_t i = 0; i < message->name.size(); ++i) {
        if (i >= message->velocity.size() || !std::isfinite(message->velocity[i])) continue;
        if (message->name[i] == module.steer_joint && i < message->position.size() &&
          std::isfinite(message->position[i])) {
          module.steer_position = message->position[i];
          module.steer_velocity = message->velocity[i];
          if (!module.has_initial_angle) {
            module.initial_angle = module.steer_position;
            module.has_initial_angle = true;
            module.steer_command = module.initial_angle;
          }
          module.has_position = true;
        }
        if (message->name[i] == module.wheel_joint) {
          module.wheel_velocity = message->velocity[i];
          module.has_wheel = true;
        }
      }
    }
    initialized_ = std::all_of(modules_.begin(), modules_.end(),
      [](const Module & m) { return m.has_position && m.has_wheel; });
    last_feedback_time_ = stamp;
  }

  bool command_is_fresh() const
  {
    return has_command_ && (now() - last_command_time_).seconds() <= command_timeout_;
  }

  void control_update()
  {
    const double dt = (now() - last_control_time_).seconds();
    if (dt <= 0.0) return;  // Simulation pause: do not integrate against wall time.
    last_control_time_ = now();
    feedback_valid_ = initialized_ && (now() - last_feedback_time_).seconds() <= 0.2;
    geometry_msgs::msg::Twist command;
    if (command_is_fresh()) {
      command = target_command_;
    }
    const bool stopped =
      std::hypot(command.linear.x, command.linear.y) <= command_deadband_ &&
      std::abs(command.angular.z) <= command_deadband_;
    if (stopped) {
      for (auto & module : modules_) {
        module.wheel_command = 0.0;
        // sentry2026 low-speed parking, adapted to recorded spawn steer angles.
        if (estimated_speed() <= 0.5) module.steer_command = module.initial_angle;
      }
    } else {
      calculate_commands(command.linear.x, command.linear.y, command.angular.z);
    }
    publish_commands();
  }

  void calculate_commands(double vx, double vy, double wz)
  {
    double greatest_wheel_speed = 0.0;
    for (auto & module : modules_) {
      // /cmd_vel_chassis is already expressed in the SDF chassis frame by
      // wheel_sim_adapter. Per-module positions replace sentry2026's square
      // chassis wR shortcut without applying another frame transform here.
      const double module_vx = vx - wz * module.y;
      const double module_vy = vy + wz * module.x;
      double target_direction = std::atan2(module_vy, module_vx);
      double target_speed = std::hypot(module_vx, module_vy) / wheel_radius_;
      const double current_direction =
        module.steer_direction_sign * module.steer_position + module.steer_zero_offset;
      const double direction_error = normalize_angle(target_direction - current_direction);

      // sentry2026 Judge_Reverse: reverse the wheel instead of steering over 90 degrees.
      if (std::abs(direction_error) > kPi / 2.0) {
        target_direction = normalize_angle(target_direction + kPi);
        target_speed = -target_speed;
      }
      const double raw_target =
        (target_direction - module.steer_zero_offset) / module.steer_direction_sign;
      module.steer_command =
        module.steer_position + normalize_angle(raw_target - module.steer_position);
      module.wheel_command = module.drive_sign * target_speed;
      greatest_wheel_speed = std::max(greatest_wheel_speed, std::abs(module.wheel_command));
    }

    // Preserve motion direction when a wheel reaches the configured speed limit.
    if (greatest_wheel_speed > max_wheel_speed_) {
      const double scale = max_wheel_speed_ / greatest_wheel_speed;
      for (auto & module : modules_) {
        module.wheel_command *= scale;
      }
    }
  }

  double estimated_speed() const
  {
    double vx = 0.0, vy = 0.0;
    for (const auto & m : modules_) {
      const double a = m.steer_direction_sign * m.steer_position + m.steer_zero_offset;
      const double v = m.drive_sign * m.wheel_velocity * wheel_radius_;
      vx += v * std::cos(a); vy += v * std::sin(a);
    }
    return std::hypot(vx, vy) / kModuleCount;
  }

  void publish_commands()
  {
    for (std::size_t i = 0; i < kModuleCount; ++i) {
      auto & m = modules_[i];
      std_msgs::msg::Float64 wheel_message, steer_message;
      // Legacy direct velocity/position actuation is disabled. These are setpoints;
      // sentry_motor_controller executes feedback PID at every physics step.
      wheel_message.data = feedback_valid_ ? m.wheel_command : 0.0;
      steer_message.data = feedback_valid_ ? m.steer_command : m.initial_angle;
      steer_publishers_[i]->publish(steer_message);
      wheel_publishers_[i]->publish(wheel_message);
    }
  }

  std::string command_topic_;
  std::string joint_state_topic_;
  double control_rate_{50.0};
  bool initialized_{false}, feedback_valid_{false};
  rclcpp::Time last_control_time_, last_feedback_time_;
  double command_timeout_{0.2};
  double command_deadband_{1.0e-4};
  double wheel_radius_{0.0535};
  double max_wheel_speed_{50.0};
  bool has_command_{false};
  std::array<Module, kModuleCount> modules_;
  geometry_msgs::msg::Twist target_command_;
  rclcpp::Time last_command_time_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, kModuleCount>
  steer_publishers_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, kModuleCount>
  wheel_publishers_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};
}  // namespace seu_sentry_sim_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<seu_sentry_sim_control::SwerveSimController>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("swerve_sim_controller"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
