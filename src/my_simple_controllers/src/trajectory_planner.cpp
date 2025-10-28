#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class TrajectoryPlanner : public rclcpp::Node {
public:
  TrajectoryPlanner() : Node("trajectory_planner") {
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cartesian_velocity_command", 10);

    rclcpp::sleep_for(2s);

    RCLCPP_INFO(this->get_logger(), "Executing trajectory...");
    execute_trajectory();
    RCLCPP_INFO(this->get_logger(), "Trajectory finished.");
  }

private:
  void execute_trajectory() {
    const double total_distance = 0.95;    
    const double max_velocity = 0.05;     
    const double max_acceleration = 0.05; 
    const double publish_rate_hz = 100.0;
    const auto dt = std::chrono::duration<double>(1.0 / publish_rate_hz);

    const double time_to_accel = max_velocity / max_acceleration;
    const double distance_to_accel = 0.5 * max_acceleration * time_to_accel * time_to_accel;

    if (2 * distance_to_accel > total_distance) {
      RCLCPP_ERROR(this->get_logger(), "Cannot reach max velocity in this distance. Motion profile is triangular.");
      return;
    }

    const double const_velocity_distance = total_distance - 2 * distance_to_accel;
    const double time_at_const_velocity = const_velocity_distance / max_velocity;
    const double total_time = 2 * time_to_accel + time_at_const_velocity;

    const double dir_x = 0.4;
    const double dir_y = 0.4;
    const double dir_z = -0.6;

    const double magnitude = std::sqrt(dir_x*dir_x + dir_y*dir_y + dir_z*dir_z);

    const double norm_x = dir_x / magnitude;
    const double norm_y = dir_y / magnitude;
    const double norm_z = dir_z / magnitude;

    rclcpp::Rate rate(publish_rate_hz);
    double time_elapsed = 0.0;

    while (time_elapsed < total_time) {
      double current_velocity = 0.0;

      // Phase 1: Acceleration
      if (time_elapsed < time_to_accel) {
        current_velocity = max_acceleration * time_elapsed;
      }
      // Phase 2: Constant Velocity
      else if (time_elapsed < (time_to_accel + time_at_const_velocity)) {
        current_velocity = max_velocity;
      }
      // Phase 3: Deceleration
      else {
        double time_in_decel = time_elapsed - (time_to_accel + time_at_const_velocity);
        current_velocity = max_velocity - max_acceleration * time_in_decel;
      }

      geometry_msgs::msg::Twist command;
      command.linear.x = norm_x * current_velocity;
      command.linear.y = norm_y * current_velocity;
      command.linear.z = norm_z * current_velocity;

      RCLCPP_INFO(this->get_logger(), "Time: %.2f s, Speed: %.4f m/s, Vel: [x:%.3f, y:%.3f, z:%.3f]",
                  time_elapsed, current_velocity, command.linear.x, command.linear.y, command.linear.z);
      velocity_publisher_->publish(command);

      time_elapsed += dt.count();
      rate.sleep();
    }

    RCLCPP_INFO(this->get_logger(), "Trajectory finished. Sending zero velocity.");
    geometry_msgs::msg::Twist zero_command;
    for (int i = 0; i < 20; ++i) {
        velocity_publisher_->publish(zero_command);
        rate.sleep();
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
}; 

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
