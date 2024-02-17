#ifndef DIFF_TEST_SYSTEM_HPP_
#define DIFF_TEST_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "visibility_control.h"

namespace diff_test_control
{
class DiffTestSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffTestSystemHardware);

  DIFF_TEST_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  DIFF_TEST_CONTROL_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DIFF_TEST_CONTROL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DIFF_TEST_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFF_TEST_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFF_TEST_CONTROL_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  DIFF_TEST_CONTROL_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::shared_ptr<rclcpp::Node> node_;
  //subscribe crawler feed back
  bool subscriber_is_active_ = false;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr fb_subscriber_ = nullptr;
  realtime_tools::RealtimeBox<std::shared_ptr<std_msgs::msg::Float64MultiArray>> received_fb_msg_ptr_{nullptr};
  //publishe crawler conmand
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> realtime_cmd_publisher_ = nullptr;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> cmd_publisher = nullptr;
};

}  // namespace diff_test_control

#endif  // DIFF_TEST_SYSTEM_HPP_