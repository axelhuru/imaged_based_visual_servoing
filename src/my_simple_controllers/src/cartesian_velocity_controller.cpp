#include <my_simple_controllers/cartesian_velocity_controller.h>  
#include <pluginlib/class_list_macros.hpp>  
#include <controller_interface/controller_interface_base.hpp>
#include "controller_interface/helpers.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames_io.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/qos.hpp>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <chrono>
using namespace std::chrono_literals;

namespace my_simple_controllers {

controller_interface::CallbackReturn CartesianVelocityController::on_init() {
  RCLCPP_INFO(get_node()->get_logger(), "Cartesian velocity controller initializing");

  try {
    param_listener_ = std::make_shared<my_simple_controllers::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  if (!getRobotDescriptionFromServer()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not fetch robot_description from server");
    return CallbackReturn::ERROR;
  }

  if (params_.kp.size() != 6) {
    RCLCPP_ERROR(get_node()->get_logger(), "kp must be a vector of 6 doubles.");
    return CallbackReturn::ERROR;
  }
  kp_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(params_.kp.data());

  return CallbackReturn::SUCCESS;
}

bool CartesianVelocityController::getRobotDescriptionFromServer() {
  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(get_node(), "/panda/robot_state_publisher");
  while (!param_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(get_node()->get_logger(), "Service not available, waiting again...");
  }

  auto parameters = param_client->get_parameters({"robot_description"});
  for (auto& parameter : parameters) {
    if (parameter.get_name() == "robot_description") {
      urdf_ = parameter.value_to_string();
      break;
    }
  }
  return true;
}

controller_interface::InterfaceConfiguration CartesianVelocityController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(params_.joints.size() * params_.command_interfaces.size());
  for (const auto & joint : params_.joints) {
    for (const auto & intf : params_.command_interfaces) {
      conf.names.push_back(joint + "/" + intf);
    }
  }
  return conf;
}

controller_interface::InterfaceConfiguration CartesianVelocityController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(params_.joints.size() * params_.state_interfaces.size());
  for (const auto & joint : params_.joints) {
    for (const auto & intf : params_.state_interfaces) {
      conf.names.push_back(joint + "/" + intf);
    }
  }
  return conf;
}

controller_interface::CallbackReturn CartesianVelocityController::on_configure(
    const rclcpp_lifecycle::State & previous_state) {

  (void) previous_state;
  const auto logger = get_node()->get_logger();
  RCLCPP_INFO(logger, "Cartesian velocity controller configuring");

  if (!param_listener_) {
    RCLCPP_ERROR(logger, "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  if (params_.joints.empty()) {
    RCLCPP_ERROR(logger, "'joints' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  joint_names_ = params_.joints;
  n_joints_ = joint_names_.size();

  if (params_.state_interfaces.empty()) {
    RCLCPP_ERROR(logger, "'state_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  if (params_.command_interfaces.empty()) {
    RCLCPP_ERROR(logger, "'command_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  for (const auto & interface : params_.state_interfaces) {
    if (std::find(allowed_state_interface_types_.begin(), allowed_state_interface_types_.end(), interface) ==
        allowed_state_interface_types_.end()) {
      RCLCPP_ERROR(logger, "Invalid state interface %s", interface.c_str());
      return CallbackReturn::FAILURE;
    }
  }

  for (const auto & interface : params_.command_interfaces) {
    if (std::find(allowed_command_interface_types_.begin(), allowed_command_interface_types_.end(), interface) ==
        allowed_command_interface_types_.end()) {
      RCLCPP_ERROR(logger, "Invalid command interface %s", interface.c_str());
      return CallbackReturn::FAILURE;
    }
  }

  if (!kdl_parser::treeFromString(urdf_, kdl_tree_)) {
    RCLCPP_ERROR(logger, "Failed to construct KDL tree from URDF.");
    return CallbackReturn::ERROR;
  }
  if (!kdl_tree_.getChain("panda_link0", "panda_link7", kdl_chain_)) {
    RCLCPP_ERROR(logger, "Failed to get KDL chain from 'base_link' to 'tool_link'.");
    return CallbackReturn::ERROR;
  }

  if (kdl_chain_.getNrOfJoints() != n_joints_) {
    RCLCPP_ERROR(logger, "Joint count mismatch: URDF has %u, params have %zu", kdl_chain_.getNrOfJoints(), n_joints_);
    return CallbackReturn::ERROR;
  }

  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_chain_);
  joint_positions_ = KDL::JntArray(n_joints_);
  joint_velocities_ = KDL::JntArray(n_joints_);
  jac_ = KDL::Jacobian(n_joints_);

  twist_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    params_.command_topic, rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      desired_twist_.vel = KDL::Vector(msg->linear.x, msg->linear.y, msg->linear.z);
      desired_twist_.rot = KDL::Vector(msg->angular.x, msg->angular.y, msg->angular.z);
      last_command_time_ = this->get_node()->get_clock()->now();
    });
  joint_state_interface_.resize(allowed_state_interface_types_.size());
  joint_command_interface_.resize(allowed_command_interface_types_.size());

  RCLCPP_INFO(logger, "Cartesian velocity controller configured");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityController::on_activate(
    const rclcpp_lifecycle::State & previous_state) {

  (void)previous_state;
  const auto logger = get_node()->get_logger();
  RCLCPP_INFO(logger, "Cartesian velocity controller activating");

  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  for (const auto & interface : params_.state_interfaces) {
    auto it = std::find(allowed_state_interface_types_.begin(), allowed_state_interface_types_.end(), interface);
    if (it == allowed_state_interface_types_.end()) {
      RCLCPP_ERROR(logger, "Could not find state interface '%s'", interface.c_str());
      return CallbackReturn::ERROR;
    }
    auto index = std::distance(allowed_state_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, joint_names_, interface, joint_state_interface_[index])) {
      RCLCPP_ERROR(logger, "Expected %zu '%s' state interfaces, got %zu.", n_joints_,
                   interface.c_str(), joint_state_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }

  for (const auto & interface : params_.command_interfaces) {
    auto it = std::find(allowed_command_interface_types_.begin(), allowed_command_interface_types_.end(), interface);
    if (it == allowed_command_interface_types_.end()) {
      RCLCPP_ERROR(logger, "Could not find command interface '%s'", interface.c_str());
      return CallbackReturn::ERROR;
    }
    auto index = std::distance(allowed_command_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
          command_interfaces_, joint_names_, interface, joint_command_interface_[index])) {
      RCLCPP_ERROR(logger, "Expected %zu '%s' command interfaces, got %zu.", n_joints_,
                   interface.c_str(), joint_command_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }

  for (size_t i = 0; i < n_joints_; ++i) {
    joint_positions_(i) = joint_state_interface_[0][i].get().get_value();
  }

  KDL::Frame initial_frame;
  if (fk_solver_->JntToCart(joint_positions_, initial_frame) < 0) {
    RCLCPP_ERROR(logger, "Failed to compute initial FK.");
    return CallbackReturn::ERROR;
  }
  desired_frame_ = initial_frame;
  desired_twist_ = KDL::Twist::Zero();
  last_command_time_ = this->get_node()->get_clock()->now();

  RCLCPP_INFO(logger, "Cartesian velocity controller activated");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityController::on_deactivate(
    const rclcpp_lifecycle::State & previous_state) {

  (void)previous_state;
  RCLCPP_INFO(get_node()->get_logger(), "Cartesian velocity controller deactivating");
  for (auto & cmd : joint_command_interface_[0]) {
    cmd.get().set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianVelocityController::update(
    const rclcpp::Time & current_time, const rclcpp::Duration & period) {

  if ((current_time - last_command_time_).seconds() > params_.command_timeout) {
    desired_twist_ = KDL::Twist::Zero();
  }
  double dt = period.seconds();
  if (dt <= 0.0) {
    return controller_interface::return_type::OK;
  }

  for (size_t i = 0; i < n_joints_; ++i) {
    joint_positions_(i) = joint_state_interface_[0][i].get().get_value();
    joint_velocities_(i) = joint_state_interface_[1][i].get().get_value();
  }

  KDL::Frame current_frame;
  if (fk_solver_->JntToCart(joint_positions_, current_frame) < 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "FK computation failed.");
    return controller_interface::return_type::ERROR;
  }

  // Integrate desired pose
  desired_frame_.p.x(desired_frame_.p.x() + desired_twist_.vel.x() * dt);
  desired_frame_.p.y(desired_frame_.p.y() + desired_twist_.vel.y() * dt);
  desired_frame_.p.z(desired_frame_.p.z() + desired_twist_.vel.z() * dt);

  double rot_norm = desired_twist_.rot.Norm();
  if (rot_norm > 1e-10) {
    KDL::Vector axis = desired_twist_.rot / rot_norm;
    KDL::Rotation delta_rot = KDL::Rotation::Rot(axis, rot_norm * dt);
    desired_frame_.M = desired_frame_.M * delta_rot;
  }

  KDL::Twist error = KDL::diff(current_frame, desired_frame_, 1.0);

  KDL::Twist kp_error(
      KDL::Vector(kp_(0) * error.vel(0), kp_(1) * error.vel(1), kp_(2) * error.vel(2)),
      KDL::Vector(kp_(3) * error.rot(0), kp_(4) * error.rot(1), kp_(5) * error.rot(2)));

  KDL::Twist u = desired_twist_ + kp_error;

  // Calculate Jacobian
  if (jac_solver_->JntToJac(joint_positions_, jac_) < 0) {
    RCLCPP_WARN(get_node()->get_logger(), "Jacobian computation failed, setting zero velocities.");
    for (auto & cmd : joint_command_interface_[0]) {
      cmd.get().set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  Eigen::MatrixXd J = jac_.data;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::VectorXd s = svd.singularValues();
  Eigen::MatrixXd SigmaInv = Eigen::MatrixXd::Zero(J.rows(), J.rows());
  for (int i = 0; i < s.size(); ++i) {
    if (s(i) > 1e-6) {
	double lambda = 0.05;  
	SigmaInv(i, i) = s(i) / (s(i) * s(i) + lambda * lambda);
      //SigmaInv(i, i) = 1.0 / s(i);
    }
  }
  Eigen::MatrixXd pinv = svd.matrixV() * SigmaInv * svd.matrixU().transpose();

  Eigen::Matrix<double, 6, 1> u_eig;
  u_eig << u.vel.x(), u.vel.y(), u.vel.z(), u.rot.x(), u.rot.y(), u.rot.z();

  Eigen::VectorXd q_dot_eig = pinv * u_eig;

  KDL::JntArray q_dot(n_joints_);
  for (size_t i = 0; i < n_joints_; ++i) {
    q_dot(i) = q_dot_eig(i);
  }

  for (size_t i = 0; i < n_joints_; ++i) {
    joint_command_interface_[0][i].get().set_value(q_dot(i));
  }

  return controller_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(my_simple_controllers::CartesianVelocityController,
                       controller_interface::ControllerInterface)
}  

