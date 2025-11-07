#include <sphero_rvr_controllers/color_sensor_broadcaster.hpp>

#include <memory>
#include <string>

namespace sphero_rvr_controllers
{

ColorSensorBroadcaster::ColorSensorBroadcaster() : controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn ColorSensorBroadcaster::on_init()
{
  try
  {
    auto_declare<std::string>("sensor_name", "color_sensor");
    auto_declare<std::string>("frame_id", "color_sensor_link");
    auto_declare<double>("max_color_value", 65535.0);  // 16-bit ADC typical
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ColorSensorBroadcaster::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  frame_id_ = get_node()->get_parameter("frame_id").as_string();
  max_color_value_ = get_node()->get_parameter("max_color_value").as_double();

  try
  {
    // Create publisher first, then wrap in RealtimePublisher
    auto publisher = get_node()->create_publisher<std_msgs::msg::ColorRGBA>("~/color", 10);
    realtime_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::ColorRGBA>>(publisher);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during configure: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Configured color sensor broadcaster for '%s'", sensor_name_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ColorSensorBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{ controller_interface::interface_configuration_type::NONE };
}

controller_interface::InterfaceConfiguration ColorSensorBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(sensor_name_ + "/color_r");
  config.names.push_back(sensor_name_ + "/color_g");
  config.names.push_back(sensor_name_ + "/color_b");
  config.names.push_back(sensor_name_ + "/color_c");
  return config;
}

controller_interface::CallbackReturn
ColorSensorBroadcaster::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ColorSensorBroadcaster::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ColorSensorBroadcaster::update(const rclcpp::Time& /*time*/,
                                                                 const rclcpp::Duration& /*period*/)
{
  if (realtime_publisher_ && realtime_publisher_->trylock())
  {
    auto& msg = realtime_publisher_->msg_;

    // Read raw values from state interfaces using get_optional()
    auto r_opt = state_interfaces_[0].get_optional();
    auto g_opt = state_interfaces_[1].get_optional();
    auto b_opt = state_interfaces_[2].get_optional();
    auto c_opt = state_interfaces_[3].get_optional();

    if (r_opt && g_opt && b_opt && c_opt)
    {
      // Normalize to [0, 1] range for ColorRGBA message
      msg.r = static_cast<float>(r_opt.value() / max_color_value_);
      msg.g = static_cast<float>(g_opt.value() / max_color_value_);
      msg.b = static_cast<float>(b_opt.value() / max_color_value_);

      // Store clear channel in alpha (non-standard but practical)
      // Or you could publish to a separate topic
      msg.a = static_cast<float>(c_opt.value() / max_color_value_);

      realtime_publisher_->unlockAndPublish();
    }
    else
    {
      realtime_publisher_->unlock();
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace sphero_rvr_controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(sphero_rvr_controllers::ColorSensorBroadcaster, controller_interface::ControllerInterface)
