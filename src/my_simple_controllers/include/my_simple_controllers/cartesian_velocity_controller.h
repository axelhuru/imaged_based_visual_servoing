#ifndef MY_SIMPLE_CONTROLLERS_CARTESIAN_VELOCITY_CONTROLLER_H
#define MY_SIMPLE_CONTROLLERS_CARTESIAN_VELOCITY_CONTROLLER_H

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp/subscription.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>  

#include <geometry_msgs/msg/twist.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/tree.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <my_simple_controllers/my_simple_controller_parameters.hpp>  
#include <Eigen/Dense>

namespace my_simple_controllers {

class CartesianVelocityController : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::return_type update(const rclcpp::Time &current_time, const rclcpp::Duration &period) override;

protected:
  bool getRobotDescriptionFromServer();

  std::shared_ptr<my_simple_controllers::ParamListener> param_listener_;
  my_simple_controllers::Params params_;

  std::string urdf_;
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;

  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  KDL::JntArray joint_velocities_;
  KDL::Jacobian jac_;

  KDL::JntArray joint_positions_;
  KDL::Frame desired_frame_;
  KDL::Twist desired_twist_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

  template<typename T>
  using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

  InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;
  InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;

  const std::vector<std::string> allowed_state_interface_types_ = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY
  };

  const std::vector<std::string> allowed_command_interface_types_ = {
    hardware_interface::HW_IF_VELOCITY
  };

  Eigen::Matrix<double, 6, 1> kp_;  

  size_t n_joints_;
  std::vector<std::string> joint_names_;
  rclcpp::Time last_command_time_;
};

}  // namespace my_simple_controllers

#endif  // MY_SIMPLE_CONTROLLERS_CARTESIAN_VELOCITY_CONTROLLER_H
