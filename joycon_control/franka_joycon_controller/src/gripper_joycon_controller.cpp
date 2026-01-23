#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "fmt/format.h"
#include "franka_example_controllers/default_robot_behavior_utils.hpp"
#include "franka_example_controllers/gripper_example_controller.hpp"
#include "franka_joycon_controller/gripper_joycon_controller.hpp"
#include "custom_msgs/msg/joycon_command.hpp"

#define RED "\033[1;31m"
#define GREEN "\033[1;32m"
#define YELLOW "\033[1;33m"
#define RESET "\033[0m"

namespace franka_joycon_controller {

controller_interface::InterfaceConfiguration
GripperJoyconController::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
GripperJoyconController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

CallbackReturn GripperJoyconController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperJoyconController::on_configure(const rclcpp_lifecycle::State&) {
  namespace_ = get_node()->get_namespace();
  gripper_grasp_action_client_ = rclcpp_action::create_client<franka_msgs::action::Grasp>(
      get_node(), fmt::format("{}/franka_gripper/grasp", namespace_));

  gripper_move_action_client_ = rclcpp_action::create_client<franka_msgs::action::Move>(
      get_node(), fmt::format("{}/franka_gripper/move", namespace_));

  gripper_stop_client_ = get_node()->create_client<std_srvs::srv::Trigger>(
      fmt::format("{}/franka_gripper/stop", namespace_));

  assignMoveGoalOptionsCallbacks();
  assignGraspGoalOptionsCallbacks();

  // topic subscriber: /joycon_command
  joycon_command_subscriber_ = get_node()->create_subscription<custom_msgs::msg::JoyconCommand>(
      "joycon_command", 10,
      std::bind(&GripperJoyconController::joyconCommandCallback, this, std::placeholders::_1));

  return nullptr != gripper_grasp_action_client_ && nullptr != gripper_move_action_client_ &&
                 nullptr != gripper_stop_client_
             ? CallbackReturn::SUCCESS
             : CallbackReturn::ERROR;
}

CallbackReturn GripperJoyconController::on_activate(const rclcpp_lifecycle::State&) {
  if (!gripper_move_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_node()->get_logger(), "Move Action server not available after waiting.");
    return CallbackReturn::ERROR;
  }
  if (!gripper_grasp_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_node()->get_logger(), "Grasp Action server not available after waiting.");
    return CallbackReturn::ERROR;
  }
  // Toggle the Gripper, initially we will order it to open
  toggleGripperState(true);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GripperJoyconController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (gripper_stop_client_->service_is_ready()) {
    std_srvs::srv::Trigger::Request::SharedPtr request =
        std::make_shared<std_srvs::srv::Trigger::Request>();

    auto result = gripper_stop_client_->async_send_request(request);
    if (result.get() && result.get()->success) {
      RCLCPP_INFO(get_node()->get_logger(), "Gripper stopped successfully.");
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to stop gripper.");
    }
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Gripper stop service is not available.");
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type GripperJoyconController::update(const rclcpp::Time&,
                                                                   const rclcpp::Duration&) {
  return controller_interface::return_type::OK;
}

void GripperJoyconController::assignMoveGoalOptionsCallbacks() {
  move_goal_options_.goal_response_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>&
                 goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_node()->get_logger(),
                       RED "Move Goal (i.e. open gripper) NOT accepted." RESET);
        } else {
          RCLCPP_INFO(get_node()->get_logger(), "Move Goal accepted");
        }
      };

  move_goal_options_.feedback_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>&,
             const std::shared_ptr<const franka_msgs::action::Move_Feedback>& feedback) {
      };

  move_goal_options_.result_callback =
      [this](
          const rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>::WrappedResult& result) {
        RCLCPP_INFO(get_node()->get_logger(), "Move Goal result %s.",
                    (rclcpp_action::ResultCode::SUCCEEDED == result.code ? YELLOW "SUCCESS" RESET
                                                                         : RED "FAIL" RESET));
      };
}

void GripperJoyconController::assignGraspGoalOptionsCallbacks() {
  grasp_goal_options_.goal_response_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>>&
                 goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_node()->get_logger(), RED "Grasp Goal NOT accepted." RESET);
        } else {
          RCLCPP_INFO(get_node()->get_logger(), "Grasp Goal accepted.");
        }
      };

  grasp_goal_options_.feedback_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>>&,
             const std::shared_ptr<const franka_msgs::action::Grasp_Feedback>& feedback) {
      };

  grasp_goal_options_.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>::WrappedResult&
                 result) {
        RCLCPP_INFO(get_node()->get_logger(), "Grasp Goal result %s.",
                    (rclcpp_action::ResultCode::SUCCEEDED == result.code ? GREEN "SUCCESS" RESET
                                                                         : RED "FAIL" RESET));
      };
}

void GripperJoyconController::joyconCommandCallback(const custom_msgs::msg::JoyconCommand::SharedPtr msg) {
  
  if (gripper_state_ != msg->gripper_state) {
    gripper_state_ = msg->gripper_state;
    toggleGripperState(gripper_state_);
  }
}

void GripperJoyconController::toggleGripperState(bool gr_state) {
  /*
   * toggle the existing gripper state according to gripper_state_.
   */
  if (gr_state) {
    openGripper();
  } else {
    graspGripper();
  }
}

bool GripperJoyconController::openGripper() {
  RCLCPP_INFO(get_node()->get_logger(), "Opening the gripper - Submitting a Move Goal");

  // define open gripper goal
  franka_msgs::action::Move::Goal move_goal;
  move_goal.width = 0.08;
  move_goal.speed = 0.2;

  std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>>
      move_goal_handle =
          gripper_move_action_client_->async_send_goal(move_goal, move_goal_options_);
  bool ret = move_goal_handle.valid();
  if (ret) {
    RCLCPP_INFO(get_node()->get_logger(), "Submited a Move Goal");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), RED "Failed to submit a Move Goal" RESET);
  }
  return ret;
}

void GripperJoyconController::graspGripper() {
  RCLCPP_INFO(get_node()->get_logger(), "Closing the gripper - Submitting a Grasp Goal");

  franka_msgs::action::Grasp::Goal grasp_goal;
  grasp_goal.width = 0.015;
  grasp_goal.speed = 0.05;
  grasp_goal.force = 100.0;
  grasp_goal.epsilon.inner = 0.005;  // 10mm or less == fail !
  grasp_goal.epsilon.outer = 0.010;  // 25mm or more == fail !

  std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>>>
      grasp_goal_handle =
          gripper_grasp_action_client_->async_send_goal(grasp_goal, grasp_goal_options_);

  bool ret = grasp_goal_handle.valid();
  if (ret) {
    RCLCPP_INFO(get_node()->get_logger(), "Submited a Grasp Goal");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), RED "Failed to submit a Grasp Goal" RESET);
  }
}

}  // namespace franka_joycon_controller
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_joycon_controller::GripperJoyconController,
                       controller_interface::ControllerInterface)
