#include <franka_joycon_controller/franka_joycon_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <custom_msgs/msg/joycon_command.hpp>
#include <franka_example_controllers/default_robot_behavior_utils.hpp>

namespace franka_joycon_controller {

controller_interface::InterfaceConfiguration
FrankaJoyconController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_command_interface_names();

  return config;
}

controller_interface::InterfaceConfiguration
FrankaJoyconController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_state_interface_names();

  config.names.push_back(arm_id_ + "/robot_time");
  return config;
}

Eigen::Quaterniond FrankaJoyconController::eulerToQuaternion(double roll, double pitch, double yaw) {
  // 将欧拉角 (roll, pitch, yaw) 转换为四元数
  // 使用 ZYX 顺序（固定轴旋转，ROS 标准约定）
  // 旋转顺序：先绕 Z 轴旋转 yaw，再绕 Y 轴旋转 pitch，最后绕 X 轴旋转 roll
  // 四元数乘法从右到左应用，所以顺序是：q = q_yaw * q_pitch * q_roll
  Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  
  // ZYX 顺序：yaw * pitch * roll（从右到左应用）
  Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;
  
  // 必须归一化四元数！libfranka 要求单位四元数，否则会导致角速度计算错误
  // 未归一化的四元数会导致 cartesian_motion_generator_velocity_limits_violation 错误
  //q.normalize();
  return q;
  
}

void FrankaJoyconController::joyconCommandCallback(const custom_msgs::msg::JoyconCommand::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(joycon_command_mutex_);
  
  // x_cartesian[6] 格式: [x, y, z, roll, pitch, yaw]
  if (msg->x_cartesian.size() >= 6) {
    // 位置: [x, y, z] (单位: 米)
    joycon_position_ = Eigen::Vector3d(
      msg->x_cartesian[0],
      msg->x_cartesian[1],
      msg->x_cartesian[2]
    );
    
    // 姿态: [roll, pitch, yaw] (单位: 弧度)
    double roll = msg->x_cartesian[3];
    double pitch = msg->x_cartesian[4];
    double yaw = msg->x_cartesian[5];
    
    joycon_orientation_ = eulerToQuaternion(roll, pitch, yaw);
    joycon_command_received_ = true;
  }
}

controller_interface::return_type FrankaJoyconController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  // 每次 update 都读取当前位置和姿态，确保状态是最新的
  std::tie(orientation_, position_) =
      franka_cartesian_pose_->getCurrentOrientationAndTranslation();
  
  if (initialization_flag_) {
    initial_robot_time_ = state_interfaces_.back().get_value();
    init_position_ = position_;
    elapsed_time_ = 0.0;

    initialization_flag_ = false;
    
    // 第一次 update 时，必须使用当前位置和姿态作为第一个命令
    // 这确保 libfranka 的起始姿态有效，避免 cartesian_position_motion_generator_start_pose_invalid 错误
    if (franka_cartesian_pose_->setCommand(orientation_, position_)) {
      return controller_interface::return_type::OK;
    } else {
      RCLCPP_FATAL(get_node()->get_logger(),
                   "Set command failed. Did you activate the elbow command interface?");
      return controller_interface::return_type::ERROR;
    }
  } else {
    robot_time_ = state_interfaces_.back().get_value();
    elapsed_time_ = robot_time_ - initial_robot_time_;
  }

  Eigen::Quaterniond new_orientation = orientation_;
  Eigen::Vector3d new_position = position_;

  // 检查是否收到 joycon 命令
  {
    //std::lock_guard<std::mutex> lock(joycon_command_mutex_);
    if (joycon_command_received_) {
      // 使用 joycon 命令
      new_position = init_position_ + joycon_position_;
      new_orientation = joycon_orientation_;
      // RCLCPP_INFO(
      //   get_node()->get_logger(), 
      //   "Calculated Command - position-delta: [%.4f, %.4f, %.4f] m, Quaternion: [%.4f, %.4f, %.4f, %.4f]",
      //   joycon_position_.x(), joycon_position_.y(), joycon_position_.z(),
      //   new_orientation.w(), new_orientation.x(), 
      //   new_orientation.y(), new_orientation.z()
      // );
      
    } else {
      // 如果没有收到 joycon 命令，使用当前位置和姿态（保持不动）
      new_position = position_;
      new_orientation = orientation_;
    }
  }
  // 输出日志：显示收到的命令值
  // RCLCPP_INFO(
  //   get_node()->get_logger(), 
  //   "Current Position - position: [%.4f, %.4f, %.4f] m, Quaternion: [%.4f, %.4f, %.4f, %.4f]",
  //   position_.x(), position_.y(), position_.z(),
  //   orientation_.w(), orientation_.x(), 
  //   orientation_.y(), orientation_.z()
  // );
  // RCLCPP_INFO(
  //   get_node()->get_logger(), 
  //   "Command Output - position: [%.4f, %.4f, %.4f] m, Quaternion: [%.4f, %.4f, %.4f, %.4f]",
  //   new_position.x(), new_position.y(), new_position.z(),
  //   new_orientation.w(), new_orientation.x(), 
  //   new_orientation.y(), new_orientation.z()
  // );
  if (franka_cartesian_pose_->setCommand(new_orientation, new_position)) {
    return controller_interface::return_type::OK;
  } else {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Set command failed. Did you activate the elbow command interface?");
    return controller_interface::return_type::ERROR;
  }
}

CallbackReturn FrankaJoyconController::on_init() {
  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
          franka_semantic_components::FrankaCartesianPoseInterface(k_elbow_activated_));

  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaJoyconController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();

  auto future_result = client->async_send_request(request);

  future_result.wait_for(robot_utils::time_out);
  auto success = future_result.get();
  if (!success) {
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

  // 创建 joycon 命令订阅者
  joycon_command_subscriber_ = get_node()->create_subscription<custom_msgs::msg::JoyconCommand>(
      "joycon_command", 10,
      std::bind(&FrankaJoyconController::joyconCommandCallback, this, std::placeholders::_1));
  
  RCLCPP_INFO(get_node()->get_logger(), "Joycon command subscriber created.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaJoyconController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  elapsed_time_ = 0.0;

  franka_cartesian_pose_->assign_loaned_command_interfaces(command_interfaces_);
  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaJoyconController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_joycon_controller

// export the controller as a plugin of ros2_control
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_joycon_controller::FrankaJoyconController,
                       controller_interface::ControllerInterface)
