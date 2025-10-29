#include <sphero_rvr_control/sphero_rvr_hardware_interface.hpp>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sphero_rvr_control
{

hardware_interface::CallbackReturn
SpheroRvrHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
{
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize storage for joint states and commands
  hw_commands_velocities_.resize(params.hardware_info.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_positions_.resize(params.hardware_info.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(params.hardware_info.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Read hardware parameters from urdf
  device_port_ = params.hardware_info.hardware_parameters.at("device_port");
  baud_rate_ = std::stoi(params.hardware_info.hardware_parameters.at("baud_rate"));
  if (params.hardware_info.hardware_parameters.count("max_wheel_velocity_rad_s"))
  {
    try
    {
      max_wheel_velocity_rad_s_ = std::stod(params.hardware_info.hardware_parameters.at("max_wheel_velocity_rad_s"));
    }
    catch (...)
    {
      RCLCPP_WARN(rclcpp::get_logger("SpheroRvrHardwareInterface"),
                  "Invalid max_wheel_velocity_rad_s; using default %.2f", max_wheel_velocity_rad_s_);
    }
  }
  // Encoder calibration: ticks per revolution
  if (params.hardware_info.hardware_parameters.count("ticks_per_revolution"))
  {
    try
    {
      ticks_per_revolution_ = std::stod(params.hardware_info.hardware_parameters.at("ticks_per_revolution"));
    }
    catch (...)
    {
      RCLCPP_WARN(rclcpp::get_logger("SpheroRvrHardwareInterface"), "Invalid ticks_per_revolution; using fallback 1.0");
      ticks_per_revolution_ = 1.0;
    }
  }
  else
  {
    // Fallback to 1.0 to avoid division by zero; user should set a real value
    ticks_per_revolution_ = 1.0;
    RCLCPP_WARN(rclcpp::get_logger("SpheroRvrHardwareInterface"), "ticks_per_revolution not specified; defaulting to "
                                                                  "1.0. Set this parameter for correct odometry.");
  }

  // simulated mode
  if (params.hardware_info.hardware_parameters.count("simulated"))
  {
    std::string simulated_str = params.hardware_info.hardware_parameters.at("simulated");
    simulated_ = (simulated_str == "true" || simulated_str == "1");
  }

  RCLCPP_INFO(rclcpp::get_logger("SpheroRvrHardwareInterface"),
              "Initialized hardware interface with device: %s, baud: %d, simulated: %s", device_port_.c_str(),
              baud_rate_, simulated_ ? "true" : "false");

  for (const hardware_interface::ComponentInfo& joint : params.hardware_info.joints)
  {
    // Verify joint has required command and state interfaces
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(rclcpp::get_logger("SpheroRvrHardwareInterface"),
                   "Joint '%s' has %zu command interfaces. 1 expected (velocity).", joint.name.c_str(),
                   joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("SpheroRvrHardwareInterface"),
                   "Joint '%s' has '%s' command interface. Expected '%s'.", joint.name.c_str(),
                   joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(rclcpp::get_logger("SpheroRvrHardwareInterface"),
                   "Joint '%s' has %zu state interfaces. 2 expected (position, velocity).", joint.name.c_str(),
                   joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // info_ = params.hardware_info;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
SpheroRvrHardwareInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  auto node = get_node();

  // register parameters
  node->declare_parameter<std::string>("hardware.device_port", device_port_);
  node->declare_parameter<int>("hardware.baud_rate", baud_rate_);
  node->declare_parameter<bool>("hardware.simulated", simulated_);
  node->declare_parameter<double>("hardware.max_wheel_velocity_rad_s", max_wheel_velocity_rad_s_);
  node->declare_parameter<int>("hardware.ticks_per_revolution", ticks_per_revolution_);

  // get parameters
  node->get_parameter("hardware.device_port", device_port_);
  node->get_parameter("hardware.baud_rate", baud_rate_);
  node->get_parameter("hardware.simulated", simulated_);
  node->get_parameter("hardware.max_wheel_velocity_rad_s", max_wheel_velocity_rad_s_);
  ticks_per_revolution_ = node->get_parameter("hardware.ticks_per_revolution").as_int();

  // log changed parameters
  RCLCPP_INFO(rclcpp::get_logger("SpheroRvrHardwareInterface"),
              "Using parameters - device_port: %s, baud_rate: %d, simulated: %s, max_wheel_velocity_rad_s: %.2f, "
              "ticks_per_revolution: %.2f",
              device_port_.c_str(), baud_rate_, simulated_ ? "true" : "false", max_wheel_velocity_rad_s_,
              ticks_per_revolution_);

  if (!simulated_)
  {
    // Initialize connection to Sphero RVR hardware (serial protocol)
    rvr_ = std::make_unique<RvrDriver>();
    if (!rvr_->connect(device_port_, baud_rate_))
    {
      RCLCPP_ERROR(rclcpp::get_logger("SpheroRvrHardwareInterface"), "Failed to open serial port %s at %d baud",
                   device_port_.c_str(), baud_rate_);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Reset all values
  for (size_t i = 0; i < hw_commands_velocities_.size(); i++)
  {
    hw_commands_velocities_[i] = 0.0;
    hw_states_positions_[i] = 0.0;
    hw_states_velocities_[i] = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("SpheroRvrHardwareInterface"), "Successfully configured hardware interface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SpheroRvrHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SpheroRvrHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn
SpheroRvrHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SpheroRvrHardwareInterface"), "Activating hardware interface...");

  RCLCPP_INFO(rclcpp::get_logger("SpheroRvrHardwareInterface"), "Successfully activated hardware interface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
SpheroRvrHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SpheroRvrHardwareInterface"), "Deactivating hardware interface...");

  // Stop motors
  if (rvr_)
  {
    rvr_->stop();
    rvr_->disconnect();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SpheroRvrHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                                 const rclcpp::Duration& period)
{
  // Simulated: integrate commands directly into states
  if (simulated_)
  {
    const double dt = std::max(0.0, period.seconds());
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      const double v = std::isfinite(hw_commands_velocities_[i]) ? hw_commands_velocities_[i] : 0.0;
      double pos = std::isfinite(hw_states_positions_[i]) ? hw_states_positions_[i] : 0.0;
      pos += v * dt;  // Euler integration
      hw_states_positions_[i] = pos;
      hw_states_velocities_[i] = v;
    }
    return hardware_interface::return_type::OK;
  }

  // Get encoder counts directly from hardware
  if (rvr_)
  {
    int32_t left_ticks, right_ticks;
    if (!rvr_->get_encoder_counts(left_ticks, right_ticks))
    {
      // Failed to read encoders - keep previous values
      return hardware_interface::return_type::OK;
    }
    // Ensure valid ticks_per_revolution_
    const double ticks_per_rev = (ticks_per_revolution_ > 0.0) ? ticks_per_revolution_ : 1.0;

    // Initialize previous ticks on first read to avoid spikes
    if (!have_prev_ticks_)
    {
      prev_left_ticks_ = left_ticks;
      prev_right_ticks_ = right_ticks;
      have_prev_ticks_ = true;
      // Set initial positions based on current ticks
      const double left_pos_rad = (static_cast<double>(left_ticks) * 2.0 * M_PI) / ticks_per_rev;
      const double right_pos_rad = (static_cast<double>(right_ticks) * 2.0 * M_PI) / ticks_per_rev;
      // Apply to both joints on each side
      for (size_t i = 0; i < info_.joints.size(); ++i)
      {
        const auto& name = info_.joints[i].name;
        if (name.find("left_") != std::string::npos)
          hw_states_positions_[i] = left_pos_rad;
        else if (name.find("right_") != std::string::npos)
          hw_states_positions_[i] = right_pos_rad;
        hw_states_velocities_[i] = 0.0;
      }
      return hardware_interface::return_type::OK;
    }

    // Compute deltas
    const int32_t dleft = left_ticks - prev_left_ticks_;
    const int32_t dright = right_ticks - prev_right_ticks_;
    prev_left_ticks_ = left_ticks;
    prev_right_ticks_ = right_ticks;

    const double dt = period.seconds();
    const double left_delta_rad = (static_cast<double>(dleft) * 2.0 * M_PI) / ticks_per_rev;
    const double right_delta_rad = (static_cast<double>(dright) * 2.0 * M_PI) / ticks_per_rev;

    // Update positions and velocities for each joint on the corresponding side
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      const auto& name = info_.joints[i].name;
      if (name.find("left_") != std::string::npos)
      {
        // Accumulate position
        double pos = std::isfinite(hw_states_positions_[i]) ? hw_states_positions_[i] : 0.0;
        pos += left_delta_rad;
        hw_states_positions_[i] = pos;
        // Velocity
        hw_states_velocities_[i] = (dt > 0.0) ? (left_delta_rad / dt) : 0.0;
      }
      else if (name.find("right_") != std::string::npos)
      {
        double pos = std::isfinite(hw_states_positions_[i]) ? hw_states_positions_[i] : 0.0;
        pos += right_delta_rad;
        hw_states_positions_[i] = pos;
        hw_states_velocities_[i] = (dt > 0.0) ? (right_delta_rad / dt) : 0.0;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SpheroRvrHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                                  const rclcpp::Duration& /*period*/)
{
  if (simulated_)
  {
    // Ignore writes to hardware in simulated mode
    return hardware_interface::return_type::OK;
  }

  // Aggregate per-side wheel commands and send to hardware using raw motors
  double left_sum = 0.0, right_sum = 0.0;
  int left_cnt = 0, right_cnt = 0;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto& name = info_.joints[i].name;
    double v = hw_commands_velocities_[i];
    if (name.find("left_") != std::string::npos)
    {
      left_sum += v;
      left_cnt++;
    }
    else if (name.find("right_") != std::string::npos)
    {
      right_sum += v;
      right_cnt++;
    }
  }
  double left_rad_s = (left_cnt > 0) ? (left_sum / left_cnt) : 0.0;
  double right_rad_s = (right_cnt > 0) ? (right_sum / right_cnt) : 0.0;

  // Convert rad/s to normalized [-1, 1] then scale to duty cycle [0, 255]
  float left_norm = 0.0f;
  float right_norm = 0.0f;
  if (max_wheel_velocity_rad_s_ > 0.0)
  {
    left_norm = static_cast<float>(std::max(-1.0, std::min(1.0, left_rad_s / max_wheel_velocity_rad_s_)));
    right_norm = static_cast<float>(std::max(-1.0, std::min(1.0, right_rad_s / max_wheel_velocity_rad_s_)));
  }

  // Determine motor mode and duty cycle for each side
  // mode: 0=off, 1=forward, 2=reverse
  uint8_t left_mode = (std::abs(left_norm) < 0.01) ? 0 : (left_norm > 0.0) ? 1 : 2;
  uint8_t right_mode = (std::abs(right_norm) < 0.01) ? 0 : (right_norm > 0.0) ? 1 : 2;
  uint8_t left_duty = static_cast<uint8_t>(std::abs(left_norm) * 255.0f);
  uint8_t right_duty = static_cast<uint8_t>(std::abs(right_norm) * 255.0f);

  if (rvr_)
  {
    (void)rvr_->set_raw_motors(left_mode, left_duty, right_mode, right_duty);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace sphero_rvr_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sphero_rvr_control::SpheroRvrHardwareInterface, hardware_interface::SystemInterface)
