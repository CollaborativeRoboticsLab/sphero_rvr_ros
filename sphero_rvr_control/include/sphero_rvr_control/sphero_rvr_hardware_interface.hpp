#pragma once

#include <atomic>
#include <chrono>
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
#include <realtime_tools/realtime_buffer.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <sphero_rvr_driver_cpp/rvr_driver.hpp>
#include <sphero_rvr_msgs/srv/set_led.hpp>

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
  // Service callbacks (non-realtime) for user-controllable LEDs
  void set_headlight_callback(const std::shared_ptr<sphero_rvr_msgs::srv::SetLED::Request> request,
                              std::shared_ptr<sphero_rvr_msgs::srv::SetLED::Response> response);

  void set_status_led_callback(const std::shared_ptr<sphero_rvr_msgs::srv::SetLED::Request> request,
                               std::shared_ptr<sphero_rvr_msgs::srv::SetLED::Response> response);

  // topic callback for headlights
  void set_headlight_topic_callback(const std_msgs::msg::ColorRGBA::SharedPtr msg);

  // Store the command and state for each joint
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;

  // Sensor state storage (exported as state interfaces)
  // IMU: 9-DOF (orientation quaternion, angular velocity, linear acceleration)
  std::array<double, 4> imu_orientation_{ 1.0, 0.0, 0.0, 0.0 };     // w, x, y, z
  std::array<double, 3> imu_angular_velocity_{ 0.0, 0.0, 0.0 };     // x, y, z (rad/s)
  std::array<double, 3> imu_linear_acceleration_{ 0.0, 0.0, 0.0 };  // x, y, z (m/s²)

  // Light sensors: ambient + RGBC color sensor
  double light_ambient_{ 0.0 };
  double light_r_{ 0.0 };
  double light_g_{ 0.0 };
  double light_b_{ 0.0 };
  double light_c_{ 0.0 };  // clear channel

  // LED control via realtime buffer (commanded from service)
  struct LEDCommand
  {
    enum class Target
    {
      NONE,
      HEADLIGHTS,
      STATUS_LED
    };

    Target target{ Target::NONE };
    uint8_t r{ 0 };
    uint8_t g{ 0 };
    uint8_t b{ 0 };
    bool has_command{ false };
  };
  realtime_tools::RealtimeBuffer<LEDCommand> led_command_buffer_;

  // LED service interfaces (user-controllable LEDs only)
  rclcpp::Service<sphero_rvr_msgs::srv::SetLED>::SharedPtr set_headlight_service_;
  rclcpp::Service<sphero_rvr_msgs::srv::SetLED>::SharedPtr set_status_led_service_;
  // topic interface (headlights)
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr set_headlight_sub_;

  // Battery state for LED indication
  std::atomic<uint8_t> battery_percentage_{ 100 };

  // Parameters
  std::string device_port_;
  int baud_rate_;
  double max_wheel_velocity_rad_s_{ 10.0 };
  // Encoder calibration: ticks per full wheel revolution
  double ticks_per_revolution_{ 0.0 };
  // IMU streaming rate (Hz)
  int imu_hz_{ 10 };
  // Sensor polling cap for light/battery (Hz)
  double sensor_poll_hz_{ 2.0 };
  // Simulated mode: bypass hardware, integrate commands into states
  bool simulated_{ false };
  // Enable sensor reads (can be disabled for performance if not needed)
  bool enable_imu_{ true };
  bool enable_light_sensors_{ true };

  // Previous encoder tick counts to compute deltas
  int32_t prev_left_ticks_{ 0 };
  int32_t prev_right_ticks_{ 0 };
  bool have_prev_ticks_{ false };

  // Hardware driver (serial protocol implementation)
  std::unique_ptr<sphero_rvr_driver_cpp::RvrDriver> rvr_;

  // Realtime buffer for IMU data coming from async callbacks
  struct ImuRtState
  {
    double q[4]{ 1.0, 0.0, 0.0, 0.0 };  // w, x, y, z
    double w[3]{ 0.0, 0.0, 0.0 };       // angular velocity rad/s (x,y,z)
    double a[3]{ 0.0, 0.0, 0.0 };       // linear acceleration m/s^2 (x,y,z)
  };
  realtime_tools::RealtimeBuffer<ImuRtState> imu_rt_buffer_;

  // Time-based throttles for slow sensors (cap at sensor_poll_hz_)
  // Split to stagger reads and prevent multiple blocking transacts per cycle
  std::chrono::steady_clock::time_point last_ambient_light_read_{};
  std::chrono::steady_clock::time_point last_rgbc_read_{};
  std::chrono::steady_clock::time_point last_battery_read_{};

  // Stagger offsets as fractions of sensor poll interval (0.0 to 1.0)
  // These scale with sensor_poll_hz_, ensuring proper distribution at any control loop rate
  const double battery_offset_fraction_ = 0.0;   // No offset, fires first
  const double ambient_offset_fraction_ = 0.33;  // 1/3 through the interval
  const double rgbc_offset_fraction_ = 0.67;     // 2/3 through the interval

  std::chrono::steady_clock::time_point wake_initiated_time_{};
  static constexpr int WAKE_WAIT_MS = 3000;    // Time to wait after wake command
  std::atomic<bool> sleep_imminent_{ false };  // Set by will_sleep callback
  std::atomic<bool> sleeping_{ false };        // Set by did_sleep callback

  // Helper methods for recovery
  void reinitialize_device();
};

}  // namespace sphero_rvr_control
