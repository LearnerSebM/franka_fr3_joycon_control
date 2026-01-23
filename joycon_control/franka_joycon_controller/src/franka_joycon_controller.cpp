#include <franka_joycon_controller/franka_joycon_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <chrono>

#include <custom_msgs/msg/joycon_command.hpp>
#include <franka_example_controllers/default_robot_behavior_utils.hpp>

using namespace std::chrono_literals;
using Vector7d = Eigen::Matrix<double, 7, 1>;

namespace franka_joycon_controller {

controller_interface::InterfaceConfiguration
FrankaJoyconController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
FrankaJoyconController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_state_interface_names();
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }

  config.names.push_back(arm_id_ + "/robot_time");

  return config;
}

void FrankaJoyconController::update_joint_states() {
  for (auto i = 0; i < num_joints_; ++i) {
    const auto& position_interface = state_interfaces_.at(16 + i);
    const auto& velocity_interface = state_interfaces_.at(23 + i);
    const auto& effort_interface = state_interfaces_.at(30 + i);
    joint_positions_current_[i] = position_interface.get_value();
    joint_velocities_current_[i] = velocity_interface.get_value();
    joint_efforts_current_[i] = effort_interface.get_value();
  }
}

std::shared_ptr<moveit_msgs::srv::GetPositionIK::Request>
FrankaJoyconController::create_ik_service_request(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation,
    const std::vector<double>& joint_positions_current,
    const std::vector<double>& joint_velocities_current,
    const std::vector<double>& joint_efforts_current) {
  auto service_request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();

  service_request->ik_request.group_name = arm_id_ + "_arm";
  service_request->ik_request.pose_stamped.header.frame_id = arm_id_ + "_link0";
  service_request->ik_request.pose_stamped.pose.position.x = position.x();
  service_request->ik_request.pose_stamped.pose.position.y = position.y();
  service_request->ik_request.pose_stamped.pose.position.z = position.z();
  service_request->ik_request.pose_stamped.pose.orientation.x = orientation.x();
  service_request->ik_request.pose_stamped.pose.orientation.y = orientation.y();
  service_request->ik_request.pose_stamped.pose.orientation.z = orientation.z();
  service_request->ik_request.pose_stamped.pose.orientation.w = orientation.w();
  service_request->ik_request.robot_state.joint_state.name = {
      arm_id_ + "_joint1", arm_id_ + "_joint2", arm_id_ + "_joint3", arm_id_ + "_joint4",
      arm_id_ + "_joint5", arm_id_ + "_joint6", arm_id_ + "_joint7"};
  service_request->ik_request.robot_state.joint_state.position = joint_positions_current;
  service_request->ik_request.robot_state.joint_state.velocity = joint_velocities_current;
  service_request->ik_request.robot_state.joint_state.effort = joint_efforts_current;

  if (is_gripper_loaded_) {
    service_request->ik_request.ik_link_name = arm_id_ + "_hand_tcp";
  }
  return service_request;
}

Vector7d FrankaJoyconController::compute_torque_command(
    const Vector7d& joint_positions_desired,
    const Vector7d& joint_positions_current,
    const Vector7d& joint_velocities_current) {
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  Vector7d coriolis(coriolis_array.data());
  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * joint_velocities_current;
  Vector7d q_error = joint_positions_desired - joint_positions_current;
  Vector7d tau_d_calculated =
      k_gains_.cwiseProduct(q_error) - d_gains_.cwiseProduct(dq_filtered_) + coriolis;

  return tau_d_calculated;
}

Eigen::Quaterniond FrankaJoyconController::eulerToQuaternion(double roll, double pitch, double yaw) {
  Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());

  Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;
  
  return q;
}

void FrankaJoyconController::joyconCommandCallback(const custom_msgs::msg::JoyconCommand::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(joycon_command_mutex_);
  
  // x_cartesian[6]: [x, y, z, roll, pitch, yaw]
  if (msg->x_cartesian.size() >= 6) {
    // position: [x, y, z] (offset from initial position)
    joycon_pos_ = Eigen::Vector3d(
      msg->x_cartesian[0],
      msg->x_cartesian[1],
      msg->x_cartesian[2]
    );
    
    // orientation: [roll, pitch, yaw]
    double roll = msg->x_cartesian[3];
    double pitch = msg->x_cartesian[4];
    double yaw = msg->x_cartesian[5];
    
    joycon_ort_ = eulerToQuaternion(roll, pitch, yaw);
    joycon_command_received_ = true;
  }
}

controller_interface::return_type FrankaJoyconController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  if (initialization_flag_) {
    std::tie(orientation_, position_) =
        franka_cartesian_pose_->getCurrentOrientationAndTranslation();

    initial_robot_time_ = state_interfaces_.back().get_value();
    new_pos_ = position_;
    new_ort_ = orientation_;
    RCLCPP_INFO(get_node()->get_logger(), "Initial position: %f, %f, %f", position_.x(), position_.y(), position_.z());
    elapsed_time_ = 0.0;
    initialization_flag_ = false;
  } else {
    robot_time_ = state_interfaces_.back().get_value();
    elapsed_time_ = robot_time_ - initial_robot_time_;
  }
  update_joint_states();


  std::lock_guard<std::mutex> lock(joycon_command_mutex_);
  if (joycon_command_received_) {
    new_pos_ = position_ + joycon_pos_;
    new_ort_ = joycon_ort_;
  }
  
  auto service_request =
      create_ik_service_request(new_pos_, new_ort_, joint_positions_current_,
                                joint_velocities_current_, joint_efforts_current_);

  using ServiceResponseFuture = rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture;
  auto response_received_callback =
      [&](ServiceResponseFuture future) {  // NOLINT(performance-unnecessary-value-param)
        const auto& response = future.get();

        if (response->error_code.val == response->error_code.SUCCESS) {
          joint_positions_desired_ = response->solution.joint_state.position;
        } else {
          RCLCPP_INFO(get_node()->get_logger(), "Inverse kinematics solution failed.");
        }
      };
  auto result_future_ =
      compute_ik_client_->async_send_request(service_request, response_received_callback);

  if (joint_positions_desired_.empty()) {
    return controller_interface::return_type::OK;
  }

  Vector7d joint_positions_desired_eigen(joint_positions_desired_.data());
  Vector7d joint_positions_current_eigen(joint_positions_current_.data());
  Vector7d joint_velocities_current_eigen(joint_velocities_current_.data());

  auto tau_d_calculated = compute_torque_command(
      joint_positions_desired_eigen, joint_positions_current_eigen, joint_velocities_current_eigen);

  for (int i = 0; i < num_joints_; i++) {
    command_interfaces_[i].set_value(tau_d_calculated(i));
  }

  return controller_interface::return_type::OK;
}

CallbackReturn FrankaJoyconController::on_init() {
  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
          franka_semantic_components::FrankaCartesianPoseInterface(k_elbow_activated_));

  return CallbackReturn::SUCCESS;
}

bool FrankaJoyconController::assign_parameters() {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  is_gripper_loaded_ = get_node()->get_parameter("load_gripper").as_bool();

  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return false;
  }
  if (k_gains.size() != static_cast<uint>(num_joints_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 num_joints_, k_gains.size());
    return false;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return false;
  }
  if (d_gains.size() != static_cast<uint>(num_joints_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints_, d_gains.size());
    return false;
  }
  for (int i = 0; i < num_joints_; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  return true;
}

CallbackReturn FrankaJoyconController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!assign_parameters()) {
    return CallbackReturn::FAILURE;
  }

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/" + k_robot_model_interface_name,
                                                   arm_id_ + "/" + k_robot_state_interface_name));

  auto collision_client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  compute_ik_client_ = get_node()->create_client<moveit_msgs::srv::GetPositionIK>("compute_ik");

  while (!compute_ik_client_->wait_for_service(1s) || !collision_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_node()->get_logger(), "service not available, waiting again...");
  }

  auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();
  auto future_result = collision_client->async_send_request(request);

  auto success = future_result.get();

  if (!success->success) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
  }

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  // topic subscriber: /joycon_command
  joycon_command_subscriber_ = get_node()->create_subscription<custom_msgs::msg::JoyconCommand>(
      "joycon_command", 10,
      std::bind(&FrankaJoyconController::joyconCommandCallback, this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaJoyconController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  elapsed_time_ = 0.0;
  dq_filtered_.setZero();
  joint_positions_desired_.reserve(num_joints_);
  joint_positions_current_.reserve(num_joints_);
  joint_velocities_current_.reserve(num_joints_);
  joint_efforts_current_.reserve(num_joints_);

  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  // get current joint_states when on_activate
  /*
  This fixes a bug caused by swtich_controller node:
  In original franka_ros2 repo, whether the ‘load-gripper’ is true or false, 
  the joint_impedance_with_ik_example_controller works fine.
  However, in our case, the franka_joycon_controller is activated by switch_controller node,
  and the robot moves to a position with a large offset.
  This might be caused by the congestion of ros2 rescources in switch_controller node.
  We fix this by manually setting the desired joint position to the current joint positions in on_active().
  */
  update_joint_states();
  joint_positions_desired_ = joint_positions_current_;

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaJoyconController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->release_interfaces();
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_joycon_controller

// export the controller as a plugin of ros2_control
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_joycon_controller::FrankaJoyconController,
                       controller_interface::ControllerInterface)
