#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visp3/vs/vpServo.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMath.h>
#include <chrono>
#include <fstream>
#include <iomanip>

class IBVSController : public rclcpp::Node {
public:
  IBVSController() : Node("ibvs_controller") {
    this->declare_parameter("log_file_path", "/tmp/ibvs_log.csv");
    this->declare_parameter("enable_logging", true);
    
    log_file_path_ = this->get_parameter("log_file_path").as_string();
    enable_logging_ = this->get_parameter("enable_logging").as_bool();
    
    centroid_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/ball_centroid", 10, std::bind(&IBVSController::centroidCallback, this, std::placeholders::_1));
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/panda/cartesian_velocity_controller/commands", 10);

    error_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ibvs/feature_error", 10);
    velocity_pub_monitor_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ibvs/control_velocity", 10);
    metrics_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ibvs/metrics", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&IBVSController::timerCallback, this));

    // ViSP setup for single point IBVS
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(lambda_);

    // Desired feature (ball at center, normalized coords)
    pd.buildFrom(0.0, 0.0, 1.0);
    task.addFeature(p, pd);
    
    // Initialize with zero velocity
    current_twist_.linear.x = 0.0;
    current_twist_.linear.y = 0.0;
    current_twist_.linear.z = 0.0;
    current_twist_.angular.x = 0.0;
    current_twist_.angular.y = 0.0;
    current_twist_.angular.z = 0.0;
    
    last_detection_time_ = this->now();
    start_time_ = this->now();
    
    // Initialize logging
    if (enable_logging_) {
      initializeLogFile();
    }
    
    RCLCPP_INFO(this->get_logger(), "IBVS Controller initialized - publishing at 50Hz");
    RCLCPP_INFO(this->get_logger(), "Logging enabled: %s", enable_logging_ ? "YES" : "NO");
    if (enable_logging_) {
      RCLCPP_INFO(this->get_logger(), "Log file: %s", log_file_path_.c_str());
    }
  }
  
  ~IBVSController() {
    if (log_file_.is_open()) {
      log_file_.close();
      RCLCPP_INFO(this->get_logger(), "Log file closed successfully");
    }
  }

private:
  void initializeLogFile() {
    log_file_.open(log_file_path_, std::ios::out | std::ios::trunc);
    if (log_file_.is_open()) {
      log_file_ << "timestamp,";
      log_file_ << "ball_detected,";
      log_file_ << "ball_u,ball_v,";
      log_file_ << "ball_u_norm,ball_v_norm,";
      log_file_ << "error_u,error_v,";
      log_file_ << "error_norm,";
      log_file_ << "vx,vy,vz,";
      log_file_ << "wx,wy,wz,";
      log_file_ << "v_linear_norm,v_angular_norm,";
      log_file_ << "lambda,";
      log_file_ << "time_since_detection,";
      log_file_ << "convergence_ratio" << std::endl;
      
      RCLCPP_INFO(this->get_logger(), "Log file initialized successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open log file: %s", log_file_path_.c_str());
      enable_logging_ = false;
    }
  }
  
  void logData(bool ball_detected, double ball_u, double ball_v, 
               double u_norm, double v_norm, double error_u, double error_v,
               const vpColVector& v) {
    if (!enable_logging_ || !log_file_.is_open()) {
      return;
    }
    
    auto now = this->now();
    double timestamp = (now - start_time_).seconds();
    double time_since_detection = (now - last_detection_time_).seconds();
    
    double error_norm = std::sqrt(error_u * error_u + error_v * error_v);
    double v_linear_norm = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    double v_angular_norm = std::sqrt(v[3]*v[3] + v[4]*v[4] + v[5]*v[5]);
    
    double convergence_ratio = error_norm;
    if (prev_error_norm_ > 0.0) {
      convergence_ratio = error_norm / prev_error_norm_;
    }
    prev_error_norm_ = error_norm;
    
    log_file_ << std::fixed << std::setprecision(6);
    log_file_ << timestamp << ",";
    log_file_ << (ball_detected ? 1 : 0) << ",";
    log_file_ << ball_u << "," << ball_v << ",";
    log_file_ << u_norm << "," << v_norm << ",";
    log_file_ << error_u << "," << error_v << ",";
    log_file_ << error_norm << ",";
    log_file_ << v[0] << "," << v[1] << "," << v[2] << ",";
    log_file_ << v[3] << "," << v[4] << "," << v[5] << ",";
    log_file_ << v_linear_norm << "," << v_angular_norm << ",";
    log_file_ << lambda_ << ",";
    log_file_ << time_since_detection << ",";
    log_file_ << convergence_ratio << std::endl;
    
    log_counter_++;
    if (log_counter_ % 50 == 0) {
      log_file_.flush();
    }
  }
  
  void publishMetrics(double error_u, double error_v, const vpColVector& v) {
    std_msgs::msg::Float64MultiArray error_msg;
    error_msg.data = {error_u, error_v, std::sqrt(error_u*error_u + error_v*error_v)};
    error_pub_->publish(error_msg);
    
    std_msgs::msg::Float64MultiArray vel_msg;
    vel_msg.data = {v[0], v[1], v[2], v[3], v[4], v[5]};
    velocity_pub_monitor_->publish(vel_msg);
    
    std_msgs::msg::Float64MultiArray metrics_msg;
    double v_linear_norm = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    double v_angular_norm = std::sqrt(v[3]*v[3] + v[4]*v[4] + v[5]*v[5]);
    double error_norm = std::sqrt(error_u*error_u + error_v*error_v);
    metrics_msg.data = {error_norm, v_linear_norm, v_angular_norm, lambda_};
    metrics_pub_->publish(metrics_msg);
  }

  void timerCallback() {
    velocity_pub_->publish(current_twist_);
    
    // If no ball detected for 1 second, slowly reduce velocities to zero
    auto time_since_detection = (this->now() - last_detection_time_).seconds();
    if (time_since_detection > 1.0) {
      // Gradually decay velocities
      current_twist_.linear.x *= 0.95;
      current_twist_.linear.y *= 0.95;
      current_twist_.linear.z *= 0.95;
      current_twist_.angular.x *= 0.95;
      current_twist_.angular.y *= 0.95;
      current_twist_.angular.z *= 0.95;
      
      // Stop completely if very small
      if (std::abs(current_twist_.linear.x) < 0.001) current_twist_.linear.x = 0.0;
      if (std::abs(current_twist_.linear.y) < 0.001) current_twist_.linear.y = 0.0;
      if (std::abs(current_twist_.linear.z) < 0.001) current_twist_.linear.z = 0.0;
      if (std::abs(current_twist_.angular.x) < 0.001) current_twist_.angular.x = 0.0;
      if (std::abs(current_twist_.angular.y) < 0.001) current_twist_.angular.y = 0.0;
      if (std::abs(current_twist_.angular.z) < 0.001) current_twist_.angular.z = 0.0;
      
      // Log the decay
      vpColVector v(6);
      v[0] = current_twist_.linear.x;
      v[1] = current_twist_.linear.y;
      v[2] = current_twist_.linear.z;
      v[3] = current_twist_.angular.x;
      v[4] = current_twist_.angular.y;
      v[5] = current_twist_.angular.z;
      
      logData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, v);
    }
  }

  void centroidCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
    last_detection_time_ = this->now();
    
    // Convert pixel (u,v) to normalized coords
    // Assume image size 640x480 with principal point at center
    double u_norm = (msg->x - 320.0) / fx_;
    double v_norm = (msg->y - 240.0) / fy_;
    double Z = 1.0;  // Fixed depth estimate

    p.buildFrom(u_norm, v_norm, Z);

    vpColVector v = task.computeControlLaw();
    
    vpColVector error = task.getError();
    double error_u = error[0];
    double error_v = error[1];

    // Update current twist with new values
    current_twist_.linear.x = v[0];
    current_twist_.linear.y = v[1];
    current_twist_.linear.z = v[2];
    current_twist_.angular.x = v[3];
    current_twist_.angular.y = v[4];
    current_twist_.angular.z = v[5];
    
    // Apply velocity limits for safety
    double max_linear = 0.5;
    double max_angular = 1.5;
    
    current_twist_.linear.x = std::clamp(current_twist_.linear.x, -max_linear, max_linear);
    current_twist_.linear.y = std::clamp(current_twist_.linear.y, -max_linear, max_linear);
    current_twist_.linear.z = std::clamp(current_twist_.linear.z, -max_linear, max_linear);
    current_twist_.angular.x = std::clamp(current_twist_.angular.x, -max_angular, max_angular);
    current_twist_.angular.y = std::clamp(current_twist_.angular.y, -max_angular, max_angular);
    current_twist_.angular.z = std::clamp(current_twist_.angular.z, -max_angular, max_angular);

    v[0] = current_twist_.linear.x;
    v[1] = current_twist_.linear.y;
    v[2] = current_twist_.linear.z;
    v[3] = current_twist_.angular.x;
    v[4] = current_twist_.angular.y;
    v[5] = current_twist_.angular.z;

    logData(true, msg->x, msg->y, u_norm, v_norm, error_u, error_v, v);
    
    publishMetrics(error_u, error_v, v);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Ball: (%.1f, %.1f) | Error: (%.4f, %.4f) | Vel: (%.3f, %.3f, %.3f)",
                         msg->x, msg->y, error_u, error_v, v[0], v[1], v[2]);
  }

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr centroid_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_monitor_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr metrics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  vpServo task;
  vpFeaturePoint p;
  vpFeaturePoint pd;
  
  double fx_ = 554.0;
  double fy_ = 554.0;
  double lambda_ = 0.5;
  
  geometry_msgs::msg::Twist current_twist_;
  rclcpp::Time last_detection_time_;
  rclcpp::Time start_time_;
  
  // Logging
  std::string log_file_path_;
  bool enable_logging_;
  std::ofstream log_file_;
  int log_counter_ = 0;
  double prev_error_norm_ = 0.0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IBVSController>());
  rclcpp::shutdown();
  return 0;
}
