#include<my_simple_controllers/state_controller.h>
#include <pluginlib/class_list_macros.hpp>  // to allow the controller to be loaded as a plugin
#include <unistd.h>  // usleep()
#include <controller_interface/controller_interface_base.hpp>
#include "controller_interface/helpers.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include<chrono>
using namespace std::chrono_literals;

namespace my_simple_controllers {


////////////////////////////////////////////////////////////////////////////////
//  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
////////////////////////////////////////////////////////////////////////////////
//
//              R O S   C O N T R O L L E R   I N T E R F A C E
//
////////////////////////////////////////////////////////////////////////////////

//=====================================================================================
controller_interface::CallbackReturn StateController::on_init() {
  RCLCPP_INFO(get_node()->get_logger(), "State controller initializing");
  
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  //here read robot description from parameter server
  if(!getRobotDescriptionFromServer()) 
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not fetch robot_description from server");
    return CallbackReturn::ERROR;
  }

  //read other parameters as required from config
  return CallbackReturn::SUCCESS;
}
        
bool StateController::getRobotDescriptionFromServer() {
  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(get_node(), "/robot_state_publisher");
  while (!param_client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(get_node()->get_logger(), "Service not available, waiting again...");
  }

  auto parameters = param_client->get_parameters({ "robot_description" });
  for (auto& parameter : parameters)
  {
    if (parameter.get_name() == "robot_description")
    {
      urdf_ = parameter.value_to_string();
      break;
    }
  }
  return true;
}

//called during configuration of command interfaces
controller_interface::InterfaceConfiguration StateController::command_interface_configuration() const
{
  //RCLCPP_INFO(get_node()->get_logger(), "State controller does not claim any command interfaces");
  
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::NONE;
  return conf;
}

//called during configuration of state interfaces
controller_interface::InterfaceConfiguration StateController::state_interface_configuration() const 
{
  //RCLCPP_INFO(get_node()->get_logger(), "State controller claiming state interfaces");
  
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::ALL;
  return conf; 
}

//configure parameters
controller_interface::CallbackReturn StateController::on_configure(
    const rclcpp_lifecycle::State & previous_state) {

  (void) previous_state; //clears warning 
  const auto logger = get_node()->get_logger();
  RCLCPP_INFO(get_node()->get_logger(), "State controller configuring");

  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  if (params_.joints.empty())
  {
    RCLCPP_ERROR(logger, "'joints' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  joint_names_ =  params_.joints;
  // get degrees of freedom
  n_joints_ = joint_names_.size();

  if (params_.state_interfaces.empty())
  {
    RCLCPP_ERROR(logger, "'state_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  joint_state_interface_.resize(allowed_state_interface_types_.size());

  auto has_ifce_type = [](const std::vector<std::string> & interface_type_list, const std::string & interface_type) {
    return std::find(interface_type_list.begin(), interface_type_list.end(), interface_type) !=
         interface_type_list.end();
  };
  bool has_pos = has_ifce_type(params_.state_interfaces, hardware_interface::HW_IF_POSITION);
  bool has_vel = has_ifce_type(params_.state_interfaces, hardware_interface::HW_IF_VELOCITY);

  if(has_pos && has_vel) {
    RCLCPP_INFO(logger,"We have both position and velocity state info, all good.\n");
  } else {
    RCLCPP_ERROR(logger, "'state_interfaces' need to contain both position and velocity");
    return CallbackReturn::FAILURE;
  }
  
  auto get_interface_list = [](const std::vector<std::string> & interface_types)
  {
    std::stringstream ss_interfaces;
    for (size_t index = 0; index < interface_types.size(); ++index)
    {
      if (index != 0)
      {
        ss_interfaces << " ";
      }
      ss_interfaces << interface_types[index];
    }
    return ss_interfaces.str();
  };

  // Print output so users can be sure the interface setup is correct
  RCLCPP_INFO(
    logger, "State interfaces are [%s].",
    get_interface_list(params_.state_interfaces).c_str());

  //TODO initialize publishers and subscribers
  //TODO parse urdf string
    // Initialize KDL parser
  if (!kdl_parser::treeFromString(urdf_, kdl_tree_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to construct KDL tree from URDF.");
      return CallbackReturn::ERROR;
  }

  // Get the chain from base to tip
  // Make sure your URDF has "base_link" and "tool_link"
  if (!kdl_tree_.getChain("base_link", "tool_link", kdl_chain_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to get KDL chain from 'base_link' to 'tool_link'.");
      return CallbackReturn::ERROR;
  }

  // Initialize FK solver
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
  joint_positions_ = KDL::JntArray(kdl_chain_.getNrOfJoints());

  // Initialize TF broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this->get_node());


  RCLCPP_INFO(logger, "State controller configured");
  return CallbackReturn::SUCCESS;

}

//clears state and will start control after this
controller_interface::CallbackReturn StateController::on_activate(
    const rclcpp_lifecycle::State & previous_state) {

  (void)previous_state;
  const auto logger = get_node()->get_logger();
  RCLCPP_INFO(logger, "State controller activating");

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  //order all joints in storage
  for (const auto & interface : params_.state_interfaces)
  {
    auto it =
      std::find(allowed_state_interface_types_.begin(), allowed_state_interface_types_.end(), interface);
    auto index = std::distance(allowed_state_interface_types_.begin(), it);
    if (it==allowed_state_interface_types_.end()) {
      RCLCPP_ERROR(logger, "Could not find interface of type %s in allowed interfaces",interface.c_str());
      return CallbackReturn::ERROR;
    }
    if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, joint_names_, interface, joint_state_interface_[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %u '%s' state interfaces, got %lu.", n_joints_,
        interface.c_str(), joint_state_interface_[index].size());
      return CallbackReturn::ERROR;
    } 
  }

  RCLCPP_INFO(logger, "State controller activated");
  return CallbackReturn::SUCCESS;
}

//clean-up back to a state from which we can start the controller
controller_interface::CallbackReturn StateController::on_deactivate(
    const rclcpp_lifecycle::State & previous_state) {

  (void)previous_state;
  RCLCPP_INFO(get_node()->get_logger(), "State controller deactivating");
  return CallbackReturn::SUCCESS;
}

//called once every cycle to update --> use realtime tools within this
controller_interface::return_type StateController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period) {

  //TODO here read joint state interface, compute FKsolution and publish TF message
  for (size_t i = 0; i < n_joints_; ++i) {
    // The position interfaces are at index 0 of the joint_state_interface_ vector
    joint_positions_(i) = joint_state_interface_[0][i].get().get_value();
  }

  KDL::Frame end_effector_pose;
  if (fk_solver_->JntToCart(joint_positions_, end_effector_pose) < 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "JntToCart failed.");
    return controller_interface::return_type::ERROR;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = this->get_node()->get_clock()->now();
  transform_stamped.header.frame_id = "base_link";
  transform_stamped.child_frame_id = "my_fk_tip";

  transform_stamped.transform.translation.x = end_effector_pose.p.x();
  transform_stamped.transform.translation.y = end_effector_pose.p.y();
  transform_stamped.transform.translation.z = end_effector_pose.p.z();

  double qx, qy, qz, qw;
  end_effector_pose.M.GetQuaternion(qx, qy, qz, qw);
  transform_stamped.transform.rotation.x = qx;
  transform_stamped.transform.rotation.y = qy;
  transform_stamped.transform.rotation.z = qz;
  transform_stamped.transform.rotation.w = qw;

  tf_broadcaster_->sendTransform(transform_stamped);

  return controller_interface::return_type::OK;
} 

}
// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(my_simple_controllers::StateController,
    controller_interface::ControllerInterface)


