/** 
 *****************************Copyright (c) 2023  ZJU****************************
 * @file      : diff_test_system.cpp
 * @brief     : 差速小车硬件接口
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0    2023-07-18       Hao Lion        1. <note>
 *******************************************************************************
 * @verbatim :
 *==============================================================================
 *                                                                              
 *                                                                              
 *==============================================================================
 * @endverbatim :
 *****************************Copyright (c) 2023  ZJU****************************
 */

#include "diff_test_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "diff_test_cmd";
constexpr auto DEFAULT_STATE_TOPIC = "diff_test_cmd";

}  // namespace

namespace diff_test_control
{
hardware_interface::CallbackReturn DiffTestSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{ 

  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

    if (info_.joints.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffTestSystemHardware"),
        "The number of crawlers is set not correactly,please check your urdf. 2 is expected,but now it is %zu.", info_.joints.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    else
    {
        RCLCPP_INFO(
        rclcpp::get_logger("DiffTestSystemHardware"), "Found 2 crawlers sucessfully!");
    }

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffTestSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffTestSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffTestSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffTestSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffTestSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffTestSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));

  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffTestSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffTestSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffTestSystemHardware"), "Activating ...please wait...");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }
  subscriber_is_active_ = true;

  this->node_ =  std::make_shared<rclcpp::Node>("hardware_node");

  std_msgs::msg::Float64MultiArray empty_int16array;
  for(std::size_t i = 0; i < hw_positions_.size(); i++)
  {
    empty_int16array.data.push_back(0.0);
  }
	received_fb_msg_ptr_.set(std::make_shared<std_msgs::msg::Float64MultiArray>(empty_int16array));

  fb_subscriber_ =
	this->node_->create_subscription<std_msgs::msg::Float64MultiArray>(
		DEFAULT_STATE_TOPIC, rclcpp::SystemDefaultsQoS(),
		[this](const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg) -> void
		{
			if (!subscriber_is_active_)
			{
				RCLCPP_WARN(
				this->node_->get_logger(), "Can't accept new commands. subscriber is inactive");
				return;
			}
			received_fb_msg_ptr_.set(std::move(msg));
		
		});


    //创建实时Publisher
    cmd_publisher = this->node_->create_publisher<std_msgs::msg::Float64MultiArray>(DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_cmd_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(cmd_publisher);

  RCLCPP_INFO(rclcpp::get_logger("DiffTestSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffTestSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffTestSystemHardware"), "Deactivating ...please wait...");

  subscriber_is_active_ = false;
  fb_subscriber_.reset();
  received_fb_msg_ptr_.set(nullptr);
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  RCLCPP_INFO(rclcpp::get_logger("DiffTestSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type diff_test_control::DiffTestSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  std::shared_ptr<std_msgs::msg::Float64MultiArray> fb_msg;
  received_fb_msg_ptr_.get(fb_msg);
  rclcpp::spin_some(this->node_);
  for (std::size_t i = 0; i < hw_positions_.size(); i++)
  {
    // Update the joint status: this is a revolute joint without any limit.
    if(i < hw_velocities_.size())
    {
      hw_velocities_[i] = fb_msg->data[i];
      hw_positions_ [i] += period.seconds() * hw_velocities_[i];
    }
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type diff_test_control::DiffTestSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
      if(realtime_cmd_publisher_->trylock())
      {
          auto & cmd_msg = realtime_cmd_publisher_->msg_;
          cmd_msg.data.resize(hw_commands_.size());
          for (auto i = 0u; i < hw_commands_.size(); i++)
          {
            cmd_msg.data[i] = hw_commands_[i];
          }
          realtime_cmd_publisher_->unlockAndPublish();
      }

  return hardware_interface::return_type::OK;
}

}  // namespace diff_test_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diff_test_control::DiffTestSystemHardware, hardware_interface::SystemInterface)