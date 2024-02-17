/**
 *****************************Copyright (c) 2023  ZJU****************************
 * @file      : diff_test_controller.cpp
 * @brief     : 差速小车控制器例程
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0    2023-6-18       Hao Lion        1. <note>
 *******************************************************************************
 * @verbatim :
 *==============================================================================
 *
 *==============================================================================
 * @endverbatim :
 *****************************Copyright (c) 2023  ZJU****************************
 */

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "diff_test_controller.hpp"

namespace
{
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "/odom";
constexpr auto odom_frame_id = "odom";
constexpr auto base_frame_id = "base_link";
}  // namespace

namespace diff_test_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

DiffTestController::DiffTestController()
: controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn DiffTestController::on_init()
{
  RCLCPP_INFO(get_node()->get_logger(), "Loading controller...");
  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}


InterfaceConfiguration DiffTestController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto & joint_name : params_.right_wheel_names) {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : params_.left_wheel_names) {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration DiffTestController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto & joint_name : params_.right_wheel_names) {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : params_.left_wheel_names) {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::CallbackReturn DiffTestController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();
  RCLCPP_INFO(logger, "Configuring controller...");

  // update parameters if they have changed
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  if (params_.left_wheel_names.size() != params_.right_wheel_names.size())
  {
    RCLCPP_ERROR(
      logger, "The number of left wheels [%zu] and the number of right wheels [%zu] are different",
      params_.left_wheel_names.size(), params_.right_wheel_names.size());
    return controller_interface::CallbackReturn::ERROR;
  }


  if (params_.left_wheel_names.empty()) {
    RCLCPP_ERROR(logger, "Left wheel names parameters are empty!");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.right_wheel_names.empty()) {
    RCLCPP_ERROR(logger, "Right wheel names parameters are empty!");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!reset()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  const Twist empty_twist;
  received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));
  // Fill last two commands with default constructed commands
  previous_commands_.emplace(empty_twist);
  previous_commands_.emplace(empty_twist);

  // initialize command subscriber
  velocity_command_subscriber_ =
    get_node()->create_subscription<geometry_msgs::msg::Twist>(
    DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
    {
      if (!subscriber_is_active_) {
        RCLCPP_WARN(
          get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
        return;
      }
      received_velocity_msg_ptr_.set(std::move(msg));

    });


  // initialize odometry publisher and messasge
  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  auto & odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = odom_frame_id;
  odometry_message.child_frame_id = base_frame_id;
  // initialize odom values zeros
  odometry_message.twist =
    geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = 0.0;
    odometry_message.twist.covariance[diagonal_index] = 0.0;
  }

  odometry_.setWheelParams(params_.wheels_separation, params_.wheel_radius, params_.wheel_radius);

  // initialize transform publisher and message
  odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
    DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
      odometry_transform_publisher_);

  // keeping track of odom and base_link transforms only
  auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
  odometry_transform_message.transforms.resize(1);
  odometry_transform_message.transforms.front().header.frame_id = odom_frame_id;
  odometry_transform_message.transforms.front().child_frame_id = base_frame_id;

  RCLCPP_INFO(logger, "Configure over...");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DiffTestController::on_activate(
  const rclcpp_lifecycle::State &)
{
  const auto left_result =
    configure_side(
    "left", params_.left_wheel_names,
    registered_left_wheel_handles_);
  const auto right_result =
    configure_side(
    "right", params_.right_wheel_names,
    registered_right_wheel_handles_);

  if (
    left_result == controller_interface::CallbackReturn::ERROR ||
    right_result == controller_interface::CallbackReturn::ERROR)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.left_wheel_names.empty() ||
    params_.right_wheel_names.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "wheel interfaces are non existent");
    return controller_interface::CallbackReturn::ERROR;
  }

  is_halted = false;
  subscriber_is_active_ = true;


  RCLCPP_INFO(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DiffTestController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (!reset()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_.set(std::make_shared<Twist>());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DiffTestController::on_error(
  const rclcpp_lifecycle::State &)
{
  if (!reset()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DiffTestController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  if (!is_halted) {
    halt();
    is_halted = true;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DiffTestController::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::return_type DiffTestController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto logger = get_node()->get_logger();
  //if controller unload
  if (get_state().id() == State::PRIMARY_STATE_INACTIVE) {
    if (!is_halted) {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  //update param
  this->params_ = param_listener_->get_params();


  //update cmd
  std::shared_ptr<Twist> last_command_msg;
  received_velocity_msg_ptr_.get(last_command_msg);

  if (last_command_msg == nullptr) {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }


  // command may be limited
  Twist command = *last_command_msg;
  // Unit m/s
  linear_command = std::clamp(
    command.linear.x, -params_.linear.x.velocity_clamp,
    params_.linear.x.velocity_clamp);
  // Unit rad/s
  angular_command = std::clamp(
    command.angular.z, -params_.angular.z.velocity_clamp,
    params_.angular.z.velocity_clamp);

    //获取转速反馈,如果每边有多个车轮，计算平均值
    double left_feedback_mean = 0.0;
    double right_feedback_mean = 0.0;
    for (size_t index = 0; index < static_cast<size_t>(params_.wheels_per_side); ++index)
    {
      const double left_feedback = registered_left_wheel_handles_[index].feedback.get().get_value();
      const double right_feedback =
        registered_right_wheel_handles_[index].feedback.get().get_value();

      if (std::isnan(left_feedback) || std::isnan(right_feedback))
      {
        RCLCPP_ERROR(
          logger, "Either the left or right wheel %s is invalid for index [%zu]", "velocity",
          index);
        return controller_interface::return_type::ERROR;
      }

      left_feedback_mean += left_feedback;
      right_feedback_mean += right_feedback;
    }
    left_feedback_mean /= params_.wheels_per_side;
    right_feedback_mean /= params_.wheels_per_side;
    //里程计积分更新
    odometry_.update(
        left_feedback_mean * params_.wheel_radius,
        right_feedback_mean * params_.wheel_radius, time);

    //欧拉角转四元数
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, odometry_.getHeading());

    if (realtime_odometry_publisher_->trylock())
    {
        auto & odometry_message = realtime_odometry_publisher_->msg_;
        odometry_message.header.stamp = time;
        odometry_message.pose.pose.position.x = odometry_.getX();
        odometry_message.pose.pose.position.y = odometry_.getY();
        odometry_message.pose.pose.orientation.x = orientation.x();
        odometry_message.pose.pose.orientation.y = orientation.y();
        odometry_message.pose.pose.orientation.z = orientation.z();
        odometry_message.pose.pose.orientation.w = orientation.w();
        odometry_message.twist.twist.linear.x = odometry_.getLinear();
        odometry_message.twist.twist.angular.z = odometry_.getAngular();
        realtime_odometry_publisher_->unlockAndPublish();
    }

    if (realtime_odometry_transform_publisher_->trylock())
    {
        auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
        transform.header.stamp = time;
        transform.transform.translation.x = odometry_.getX();
        transform.transform.translation.y = odometry_.getY();
        transform.transform.rotation.x = orientation.x();
        transform.transform.rotation.y = orientation.y();
        transform.transform.rotation.z = orientation.z();
        transform.transform.rotation.w = orientation.w();
        realtime_odometry_transform_publisher_->unlockAndPublish();
    }


  // Compute wheels velocities:  Unit m/s
  const double velocity_right =
    (linear_command + angular_command * params_.wheels_separation / 2.0);
  const double velocity_left =
    (linear_command - angular_command * params_.wheels_separation / 2.0);

  // Set wheels velocities:
  for (size_t index = 0; index < static_cast<size_t>(params_.wheels_per_side); ++index) {
    registered_left_wheel_handles_[index].velocity.get().set_value(velocity_left / params_.wheel_radius);
    registered_right_wheel_handles_[index].velocity.get().set_value(velocity_right / params_.wheel_radius);
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn DiffTestController::configure_side(
  const std::string & wheel_kind,
  const std::vector<std::string> & wheel_names,
  std::vector<WheelHandle> & registered_handles)
{
  auto logger = get_node()->get_logger();

  if (wheel_names.empty()) {
    RCLCPP_ERROR(logger, "No '%s' motor names specified", wheel_kind.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  // register handles
  registered_handles.reserve(wheel_names.size());
  for (const auto & wheel_name : wheel_names) {
    const auto state_handle = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(),
      [&wheel_name](const auto & interface)
      {
        return interface.get_prefix_name() == wheel_name &&
        interface.get_interface_name() == HW_IF_VELOCITY;
      });

    if (state_handle == state_interfaces_.cend()) {
      RCLCPP_ERROR(logger, "Unable to obtain motor state handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    const auto command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&wheel_name](const auto & interface)
      {
        return interface.get_prefix_name() == wheel_name &&
        interface.get_interface_name() == HW_IF_VELOCITY;
      });

    if (command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(logger, "Unable to obtain motor command handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    registered_handles.emplace_back(
      WheelHandle{std::ref(*state_handle), std::ref(
          *command_handle)});
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

bool DiffTestController::reset()
{
  // release the old queue
  std::queue<Twist> empty;
  std::swap(previous_commands_, empty);

  registered_left_wheel_handles_.clear();
  registered_right_wheel_handles_.clear();

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  received_velocity_msg_ptr_.set(nullptr);

  is_halted = false;
  return true;
}

void DiffTestController::halt()
{
  //make wheels stop
  const auto halt_wheels = [](auto & wheel_handles)
    {
      for (const auto & wheel_handle : wheel_handles) {
        wheel_handle.velocity.get().set_value(0.0);
      }
    };

  halt_wheels(registered_left_wheel_handles_);
  halt_wheels(registered_right_wheel_handles_);
}

} // namespace diff_test_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  diff_test_controller::DiffTestController, controller_interface::ControllerInterface)
