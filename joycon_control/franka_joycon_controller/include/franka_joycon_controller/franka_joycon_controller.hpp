#pragma once

#include <string>
#include <mutex>

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka_example_controllers/robot_utils.hpp>
#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>
#include <custom_msgs/msg/joycon_command.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_joycon_controller {

/**
 * The franka joycon controller
 */
class FrankaJoyconController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;

  Eigen::Quaterniond orientation_;
  Eigen::Vector3d position_;
  
  const bool k_elbow_activated_{false};
  bool initialization_flag_{true};

  double elapsed_time_{0.0};
  double initial_robot_time_{0.0};
  double robot_time_{0.0};
  std::string robot_description_;
  std::string arm_id_;

  // Joycon command subscription
  rclcpp::Subscription<custom_msgs::msg::JoyconCommand>::SharedPtr joycon_command_subscriber_;
  std::mutex joycon_command_mutex_;
  bool joycon_command_received_{false};
  Eigen::Vector3d joycon_position_;
  Eigen::AngleAxisd joycon_orientation_;

  // Helper function to convert roll, pitch, yaw to quaternion
  Eigen::AngleAxisd eulerToAngleAxis(double roll, double pitch, double yaw);
  
  // Callback for joycon command
  void joyconCommandCallback(const custom_msgs::msg::JoyconCommand::SharedPtr msg);

};  // class FrankaJoyconController

}  // namespace franka_joycon_controller
