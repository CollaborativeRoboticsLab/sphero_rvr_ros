#include <sphero_rvr_control/sphero_rvr_hardware_interface.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
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
  // Optional sensor enables
  if (params.hardware_info.hardware_parameters.count("enable_imu"))
  {
    std::string val = params.hardware_info.hardware_parameters.at("enable_imu");
    enable_imu_ = (val == "true" || val == "1");
  }
  if (params.hardware_info.hardware_parameters.count("enable_light_sensors"))
  {
    std::string val = params.hardware_info.hardware_parameters.at("enable_light_sensors");
    enable_light_sensors_ = (val == "true" || val == "1");
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
  node->declare_parameter<double>("hardware.sensor_poll_hz", sensor_poll_hz_);
  node->declare_parameter<int>("hardware.imu_hz", imu_hz_);

  // get parameters
  node->get_parameter("hardware.device_port", device_port_);
  node->get_parameter("hardware.baud_rate", baud_rate_);
  node->get_parameter("hardware.simulated", simulated_);
  node->get_parameter("hardware.max_wheel_velocity_rad_s", max_wheel_velocity_rad_s_);
  ticks_per_revolution_ = node->get_parameter("hardware.ticks_per_revolution").as_int();
  sensor_poll_hz_ = node->get_parameter("hardware.sensor_poll_hz").as_double();
  imu_hz_ = node->get_parameter("hardware.imu_hz").as_int();

  // log changed parameters
  RCLCPP_INFO(rclcpp::get_logger("SpheroRvrHardwareInterface"),
              "Using parameters - device_port: %s, baud_rate: %d, simulated: %s, max_wheel_velocity_rad_s: %.2f, "
              "ticks_per_revolution: %.2f, imu_hz: %d, sensor_poll_hz: %.2f",
              device_port_.c_str(), baud_rate_, simulated_ ? "true" : "false", max_wheel_velocity_rad_s_,
              ticks_per_revolution_, imu_hz_, sensor_poll_hz_);

  // Create LED control services (realtime-safe via buffer)
  // Only user-controllable LEDs have services; hardware-managed LEDs are automatic
  set_headlight_service_ = node->create_service<sphero_rvr_msgs::srv::SetLED>(
      "~/set_headlight", std::bind(&SpheroRvrHardwareInterface::set_headlight_callback, this, std::placeholders::_1,
                                   std::placeholders::_2));

  set_status_led_service_ = node->create_service<sphero_rvr_msgs::srv::SetLED>(
      "~/set_status_led", std::bind(&SpheroRvrHardwareInterface::set_status_led_callback, this, std::placeholders::_1,
                                    std::placeholders::_2));

  set_headlight_sub_ = node->create_subscription<std_msgs::msg::ColorRGBA>(
      "~/headlight", 10,
      std::bind(&SpheroRvrHardwareInterface::set_headlight_topic_callback, this, std::placeholders::_1));

  if (!simulated_)
  {
    // Initialize connection to Sphero RVR hardware
    rvr_ = std::make_unique<sphero_rvr_driver_cpp::RvrDriver>();
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

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SpheroRvrHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Joint position and velocity states
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
  }

  // Sensor state interfaces (match names defined in URDF)
  // Supported:
  //  - imu_sensor: orientation/velocity/acceleration
  //  - ambient_light_sensor: ambient_light
  //  - color_sensor: color_r, color_g, color_b, color_c
  for (const auto& sensor : info_.sensors)
  {
    if (sensor.name == "imu_sensor" && enable_imu_)
    {
      // Orientation quaternion (w, x, y, z)
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(sensor.name, "orientation.w", &imu_orientation_[0]));
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(sensor.name, "orientation.x", &imu_orientation_[1]));
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(sensor.name, "orientation.y", &imu_orientation_[2]));
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(sensor.name, "orientation.z", &imu_orientation_[3]));

      // Angular velocity (rad/s)
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(sensor.name, "angular_velocity.x", &imu_angular_velocity_[0]));
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(sensor.name, "angular_velocity.y", &imu_angular_velocity_[1]));
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(sensor.name, "angular_velocity.z", &imu_angular_velocity_[2]));

      // Linear acceleration (m/s²)
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(sensor.name, "linear_acceleration.x", &imu_linear_acceleration_[0]));
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(sensor.name, "linear_acceleration.y", &imu_linear_acceleration_[1]));
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(sensor.name, "linear_acceleration.z", &imu_linear_acceleration_[2]));

      RCLCPP_INFO(rclcpp::get_logger("SpheroRvrHardwareInterface"), "Exported IMU sensor state interfaces");
    }
    else if (sensor.name == "ambient_light_sensor" && enable_light_sensors_)
    {
      // Ambient-only interface for ambient_light_sensor
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "ambient_light", &light_ambient_));

      RCLCPP_INFO(rclcpp::get_logger("SpheroRvrHardwareInterface"),
                  "Exported ambient light sensor state interface for '%s'", sensor.name.c_str());
    }
    else if (sensor.name == "color_sensor" && enable_light_sensors_)
    {
      // Color-only interfaces for color_sensor (underscore naming)
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "color_r", &light_r_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "color_g", &light_g_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "color_b", &light_b_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "color_c", &light_c_));

      RCLCPP_INFO(rclcpp::get_logger("SpheroRvrHardwareInterface"), "Exported color sensor state interfaces for '%s'",
                  sensor.name.c_str());
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SpheroRvrHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Joint velocity commands
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

  // Reset recovery state on activation
  sleep_imminent_.store(false);
  sleeping_.store(false);

  // Initialize hardware-managed LEDs
  if (rvr_ && !simulated_)
  {
    // wake and set longer timeout
    rvr_->wake();                                          // wake the RVR from soft sleep
    std::this_thread::sleep_for(std::chrono::seconds(2));  // Wait for wake
    // after waking, reset your control system timeout
    if (!rvr_->set_custom_control_system_timeout(10000))
    {
      RCLCPP_ERROR(rclcpp::get_logger("SpheroRvrHardwareInterface"), "FAILED to set control system timeout!");
    }

    // Set rear brakelights to red (indicate reverse direction)
    rvr_->set_brakelights_rgb(255, 0, 0);

    // Set undercarriage white LED (illuminate floor for color sensor)
    rvr_->set_undercarriage_white(255);

    // Initialize battery door LEDs based on current battery level
    // Will be updated periodically in read()
    rvr_->set_battery_leds_rgb(0, 255, 0);  // Start with green

    // DETECT imminent sleep (wake() in callback doesn't prevent it due to RVR firmware behavior)
    rvr_->set_will_sleep_callback([this]() {
      RCLCPP_WARN(rclcpp::get_logger("SpheroRvrHardwareInterface"), "RVR sleep imminent - will recover after sleep "
                                                                    "cycle completes");
      sleep_imminent_.store(true);
    });

    // DETECT did sleep
    rvr_->set_did_sleep_callback([this]() {
      RCLCPP_WARN(rclcpp::get_logger("SpheroRvrHardwareInterface"), "RVR has entered sleep mode");
      sleeping_.store(true);
      wake_initiated_time_ = std::chrono::steady_clock::now();
    });

    // Configure and start IMU streaming with realtime-safe callback
    if (enable_imu_)
    {
      using ImuSample = sphero_rvr_driver_cpp::RvrDriver::ImuSample;
      rvr_->set_imu_callback([this](const ImuSample& d) {
        // Convert degrees to radians
        const double deg2rad = M_PI / 180.0;
        const double roll = static_cast<double>(d.roll_deg) * deg2rad;    // x
        const double pitch = static_cast<double>(d.pitch_deg) * deg2rad;  // y
        const double yaw = static_cast<double>(d.yaw_deg) * deg2rad;      // z

        // Build quaternion from ZYX (yaw, pitch, roll)
        const double cy = std::cos(yaw * 0.5);
        const double sy = std::sin(yaw * 0.5);
        const double cp = std::cos(pitch * 0.5);
        const double sp = std::sin(pitch * 0.5);
        const double cr = std::cos(roll * 0.5);
        const double sr = std::sin(roll * 0.5);

        sphero_rvr_control::SpheroRvrHardwareInterface::ImuRtState s;
        // Quaternion (w, x, y, z)
        s.q[0] = cr * cp * cy + sr * sp * sy;  // w
        s.q[1] = sr * cp * cy - cr * sp * sy;  // x
        s.q[2] = cr * sp * cy + sr * cp * sy;  // y
        s.q[3] = cr * cp * sy - sr * sp * cy;  // z

        // Angular velocity (deg/s -> rad/s)
        s.w[0] = static_cast<double>(d.gx_dps) * deg2rad;
        s.w[1] = static_cast<double>(d.gy_dps) * deg2rad;
        s.w[2] = static_cast<double>(d.gz_dps) * deg2rad;

        // Linear acceleration (g -> m/s^2)
        const double g_to_ms2 = 9.80665;
        s.a[0] = static_cast<double>(d.ax_g) * g_to_ms2;
        s.a[1] = static_cast<double>(d.ay_g) * g_to_ms2;
        s.a[2] = static_cast<double>(d.az_g) * g_to_ms2;

        // Write to realtime buffer (lock-free)
        imu_rt_buffer_.writeFromNonRT(s);
      });

      // Enable full IMU suite (angles + gyro + accel) at configured Hz
      const int hz = std::max(1, imu_hz_);
      const uint16_t period_ms = static_cast<uint16_t>(std::max(33, 1000 / hz));
      if (!rvr_->enable_imu_suite_streaming(period_ms))
      {
        RCLCPP_WARN(rclcpp::get_logger("SpheroRvrHardwareInterface"), "Failed to start IMU streaming");
      }
      else
      {
        RCLCPP_INFO(rclcpp::get_logger("SpheroRvrHardwareInterface"), "IMU streaming started at ~%d Hz",
                    1000 / period_ms);
      }
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
SpheroRvrHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SpheroRvrHardwareInterface"), "Deactivating hardware interface...");

  // Stop motors
  if (rvr_)
  {
    rvr_->drive_stop();

    // Stop IMU streaming if enabled
    if (enable_imu_)
    {
      (void)rvr_->stop_imu_streaming();
      (void)rvr_->clear_imu_streaming();
    }

    // Release all LED control back to default behavior
    rvr_->release_led_requests();

    // reset custom control timeout
    rvr_->restore_default_control_system_timeout();

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
    // Simulated light sensors: Gaussian noise around fixed means (no time dependence)
    if (enable_light_sensors_)
    {
      static thread_local std::mt19937 rng{ std::random_device{}() };
      auto clamp = [](double v, double lo, double hi) {
        if (v < lo)
          return lo;
        if (v > hi)
          return hi;
        return v;
      };
      auto gaussian = [&](double mean, double stddev, double lo, double hi) {
        std::normal_distribution<double> dist(mean, stddev);
        return clamp(dist(rng), lo, hi);
      };

      // Means and stddevs chosen to be stable but non-constant
      light_ambient_ = gaussian(100.0, 5.0, 0.0, 500.0);
      light_r_ = gaussian(1000.0, 50.0, 0.0, 4095.0);
      light_g_ = gaussian(2000.0, 50.0, 0.0, 4095.0);
      light_b_ = gaussian(3000.0, 50.0, 0.0, 4095.0);
      light_c_ = gaussian(3500.0, 80.0, 0.0, 4095.0);
    }
    return hardware_interface::return_type::OK;
  }

  // Real hardware: read encoders
  if (rvr_)
  {
    // Initialize clock for all sensor timing in this cycle
    using clock = std::chrono::steady_clock;
    static const auto min_interval = std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(1.0 / std::max(0.1, sensor_poll_hz_)));
    const auto now_tp = clock::now();

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

    // Read light sensors staggered to avoid blocking multiple transacts per cycle
    if (enable_light_sensors_)
    {
      // Ambient light with offset
      if (last_ambient_light_read_.time_since_epoch().count() == 0)
      {
        // Initialize with offset on first read (fraction of min_interval)
        auto offset = std::chrono::duration_cast<clock::duration>(min_interval * ambient_offset_fraction_);
        last_ambient_light_read_ = now_tp - min_interval + offset;
      }
      if ((now_tp - last_ambient_light_read_) >= min_interval)
      {
        float ambient = 0.0f;
        if (rvr_->get_ambient_light(ambient))
        {
          light_ambient_ = static_cast<double>(ambient);
        }
        last_ambient_light_read_ = now_tp;
      }

      // RGBC color with offset
      if (last_rgbc_read_.time_since_epoch().count() == 0)
      {
        // Initialize with offset on first read (fraction of min_interval)
        auto offset = std::chrono::duration_cast<clock::duration>(min_interval * rgbc_offset_fraction_);
        last_rgbc_read_ = now_tp - min_interval + offset;
      }
      if ((now_tp - last_rgbc_read_) >= min_interval)
      {
        uint16_t r = 0, g = 0, b = 0, c = 0;
        if (rvr_->get_rgbc(r, g, b, c))
        {
          light_r_ = static_cast<double>(r);
          light_g_ = static_cast<double>(g);
          light_b_ = static_cast<double>(b);
          light_c_ = static_cast<double>(c);
        }
        last_rgbc_read_ = now_tp;
      }
    }

    // Read battery and update indicator LEDs at <= sensor_poll_hz_ with offset
    {
      // Initialize with offset on first read
      if (last_battery_read_.time_since_epoch().count() == 0)
      {
        // Battery has no offset (fires first)
        auto offset = std::chrono::duration_cast<clock::duration>(min_interval * battery_offset_fraction_);
        last_battery_read_ = now_tp - min_interval + offset;
      }

      if ((now_tp - last_battery_read_) >= min_interval)
      {
        uint8_t battery_pct = 0;
        if (rvr_->get_battery_percentage(battery_pct))
        {
          battery_percentage_.store(battery_pct);

          // Color code battery door LEDs based on percentage
          uint8_t led_r = 0, led_g = 0, led_b = 0;
          if (battery_pct > 60)
          {
            // Green: good battery
            led_g = 255;
          }
          else if (battery_pct > 40)
          {
            // Yellow: medium battery
            led_r = 255;
            led_g = 255;
          }
          else if (battery_pct > 20)
          {
            // Orange: low battery
            led_r = 255;
            led_g = 128;
          }
          else
          {
            // Red: critical battery
            led_r = 255;
          }

          rvr_->set_battery_leds_rgb(led_r, led_g, led_b);
        }
        last_battery_read_ = now_tp;
      }
    }

    // IMU: copy latest realtime sample (if streaming enabled)
    if (enable_imu_)
    {
      auto sample = *imu_rt_buffer_.readFromRT();
      imu_orientation_[0] = sample.q[0];
      imu_orientation_[1] = sample.q[1];
      imu_orientation_[2] = sample.q[2];
      imu_orientation_[3] = sample.q[3];
      imu_angular_velocity_[0] = sample.w[0];
      imu_angular_velocity_[1] = sample.w[1];
      imu_angular_velocity_[2] = sample.w[2];
      imu_linear_acceleration_[0] = sample.a[0];
      imu_linear_acceleration_[1] = sample.a[1];
      imu_linear_acceleration_[2] = sample.a[2];
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

  // Handle recovery state machine (non-blocking)
  if (sleeping_.load() && rvr_)
  {
    auto now = std::chrono::steady_clock::now();

    // Wait for wake to complete
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - wake_initiated_time_).count();
    if (elapsed >= WAKE_WAIT_MS)
    {
      reinitialize_device();
    }
  }

  // periodically send wake command when sleep is imminent
  if (sleep_imminent_.load() && rvr_)
  {
    rvr_->wake();  // Keepalive to NODE_NORDIC
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
    (void)rvr_->drive_raw_motors(left_mode, left_duty, right_mode, right_duty);
  }

  // Process user-controllable LED commands from realtime buffer (non-blocking)
  LEDCommand led_cmd = *led_command_buffer_.readFromRT();
  if (led_cmd.has_command && rvr_)
  {
    switch (led_cmd.target)
    {
      case LEDCommand::Target::HEADLIGHTS:
        (void)rvr_->set_headlights_rgb(led_cmd.r, led_cmd.g, led_cmd.b);
        RCLCPP_DEBUG(rclcpp::get_logger("SpheroRvrHardwareInterface"), "Headlights set to RGB(%u, %u, %u)", led_cmd.r,
                     led_cmd.g, led_cmd.b);
        break;

      case LEDCommand::Target::STATUS_LED:
        (void)rvr_->set_status_rgb(led_cmd.r, led_cmd.g, led_cmd.b);
        RCLCPP_DEBUG(rclcpp::get_logger("SpheroRvrHardwareInterface"), "Status LED set to RGB(%u, %u, %u)", led_cmd.r,
                     led_cmd.g, led_cmd.b);
        break;

      case LEDCommand::Target::NONE:
      default:
        break;
    }

    // Clear the command flag after processing
    led_cmd.has_command = false;
    led_command_buffer_.writeFromNonRT(led_cmd);
  }

  return hardware_interface::return_type::OK;
}

void SpheroRvrHardwareInterface::set_headlight_callback(
    const std::shared_ptr<sphero_rvr_msgs::srv::SetLED::Request> request,
    std::shared_ptr<sphero_rvr_msgs::srv::SetLED::Response> response)
{
  LEDCommand cmd;
  cmd.target = LEDCommand::Target::HEADLIGHTS;
  cmd.r = request->r;
  cmd.g = request->g;
  cmd.b = request->b;
  cmd.has_command = true;

  // Write to realtime buffer (lock-free, safe to call from service)
  led_command_buffer_.writeFromNonRT(cmd);

  response->success = true;
  response->message = "Headlight command queued";

  RCLCPP_DEBUG(rclcpp::get_logger("SpheroRvrHardwareInterface"), "Headlight command queued: RGB(%u, %u, %u)",
               request->r, request->g, request->b);
}

void SpheroRvrHardwareInterface::set_status_led_callback(
    const std::shared_ptr<sphero_rvr_msgs::srv::SetLED::Request> request,
    std::shared_ptr<sphero_rvr_msgs::srv::SetLED::Response> response)
{
  LEDCommand cmd;
  cmd.target = LEDCommand::Target::STATUS_LED;
  cmd.r = request->r;
  cmd.g = request->g;
  cmd.b = request->b;
  cmd.has_command = true;

  // Write to realtime buffer (lock-free, safe to call from service)
  led_command_buffer_.writeFromNonRT(cmd);

  response->success = true;
  response->message = "Status LED command queued";

  RCLCPP_DEBUG(rclcpp::get_logger("SpheroRvrHardwareInterface"), "Status LED command queued: RGB(%u, %u, %u)",
               request->r, request->g, request->b);
}

void SpheroRvrHardwareInterface::set_headlight_topic_callback(const std_msgs::msg::ColorRGBA::SharedPtr msg)
{
  auto clamp01 = [](float v) {
    if (v < 0.0f)
      return 0.0f;
    if (v > 1.0f)
      return 1.0f;
    return v;
  };
  auto to_u8 = [&](float v) { return static_cast<uint8_t>(std::lround(clamp01(v) * 255.0f)); };

  LEDCommand cmd;
  cmd.target = LEDCommand::Target::HEADLIGHTS;
  cmd.r = to_u8(msg->r);
  cmd.g = to_u8(msg->g);
  cmd.b = to_u8(msg->b);
  cmd.has_command = true;

  // Write to realtime buffer (lock-free, safe to call from subscription)
  led_command_buffer_.writeFromNonRT(cmd);

  RCLCPP_DEBUG(rclcpp::get_logger("SpheroRvrHardwareInterface"), "Headlight topic command queued: RGB(%u, %u, %u)",
               cmd.r, cmd.g, cmd.b);
}

void SpheroRvrHardwareInterface::reinitialize_device()
{
  RCLCPP_INFO(rclcpp::get_logger("SpheroRvrHardwareInterface"), "Reinitializing device after wake");

  // Reset control system timeout
  if (!rvr_->set_custom_control_system_timeout(10000))
  {
    RCLCPP_WARN(rclcpp::get_logger("SpheroRvrHardwareInterface"), "Failed to set control system timeout during "
                                                                  "recovery");
  }

  // Restore hardware-managed LEDs
  rvr_->set_brakelights_rgb(255, 0, 0);
  rvr_->set_undercarriage_white(255);
  rvr_->set_battery_leds_rgb(0, 255, 0);

  // Restart IMU streaming if enabled
  if (enable_imu_)
  {
    const int hz = std::max(1, imu_hz_);
    const uint16_t period_ms = static_cast<uint16_t>(std::max(33, 1000 / hz));
    if (!rvr_->enable_imu_suite_streaming(period_ms))
    {
      RCLCPP_WARN(rclcpp::get_logger("SpheroRvrHardwareInterface"), "Failed to restart IMU streaming during recovery");
    }
  }

  // Clear sleep detection flags
  sleep_imminent_.store(false);
  sleeping_.store(false);
}

}  // namespace sphero_rvr_control

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(sphero_rvr_control::SpheroRvrHardwareInterface, hardware_interface::SystemInterface)
