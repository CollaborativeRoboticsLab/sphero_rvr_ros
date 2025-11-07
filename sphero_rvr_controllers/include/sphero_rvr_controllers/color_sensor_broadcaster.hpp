#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace sphero_rvr_controllers
{

class ColorSensorBroadcaster : public controller_interface::ControllerInterface
{
public:
  ColorSensorBroadcaster();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  std::string sensor_name_;
  std::string frame_id_;
  double max_color_value_;  // For normalization (e.g., 65535 for 16-bit sensors)

  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::ColorRGBA>> realtime_publisher_;
};

}  // namespace sphero_rvr_controllers
