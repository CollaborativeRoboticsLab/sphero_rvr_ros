#pragma once

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <sphero_rvr_control/rvr_driver.hpp>

namespace sphero_rvr_control
{

class SpheroRvrHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SpheroRvrHardwareInterface)

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // Store the command and state for each joint
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;

  // Parameters
  std::string device_port_;
  int baud_rate_;
  double max_wheel_velocity_rad_s_{ 10.0 };
  // Encoder calibration: ticks per full wheel revolution
  double ticks_per_revolution_{ 0.0 };
  // Simulated mode: bypass hardware, integrate commands into states
  bool simulated_{ false };

  // Previous encoder tick counts to compute deltas
  int32_t prev_left_ticks_{ 0 };
  int32_t prev_right_ticks_{ 0 };
  bool have_prev_ticks_{ false };

  // Hardware driver (serial protocol implementation)
  std::unique_ptr<RvrDriver> rvr_;
};

}  // namespace sphero_rvr_control
