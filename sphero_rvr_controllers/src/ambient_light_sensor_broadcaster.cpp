#include <sphero_rvr_controllers/ambient_light_sensor_broadcaster.hpp>

#include <memory>
#include <string>

namespace sphero_rvr_controllers
{

AmbientLightSensorBroadcaster::AmbientLightSensorBroadcaster() : controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn AmbientLightSensorBroadcaster::on_init()
{
  try
  {
    auto_declare<std::string>("sensor_name", "ambient_light_sensor");
    auto_declare<std::string>("frame_id", "ambient_light_link");
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
AmbientLightSensorBroadcaster::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  frame_id_ = get_node()->get_parameter("frame_id").as_string();

  try
  {
    // Create publisher first, then wrap in RealtimePublisher
    auto publisher = get_node()->create_publisher<sensor_msgs::msg::Illuminance>("~/ambient_light", 10);
    realtime_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::Illuminance>>(publisher);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during configure: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration AmbientLightSensorBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{ controller_interface::interface_configuration_type::NONE };
}

controller_interface::InterfaceConfiguration AmbientLightSensorBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(sensor_name_ + "/ambient_light");
  return config;
}

controller_interface::CallbackReturn
AmbientLightSensorBroadcaster::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
AmbientLightSensorBroadcaster::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AmbientLightSensorBroadcaster::update(const rclcpp::Time& time,
                                                                        const rclcpp::Duration& /*period*/)
{
  if (realtime_publisher_ && realtime_publisher_->trylock())
  {
    auto& msg = realtime_publisher_->msg_;
    msg.header.stamp = time;
    msg.header.frame_id = frame_id_;

    // Read from state interface using get_optional()
    auto illuminance_opt = state_interfaces_[0].get_optional();

    if (illuminance_opt)
    {
      msg.illuminance = illuminance_opt.value();

      // Variance unknown - set to 0
      msg.variance = 0.0;

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

PLUGINLIB_EXPORT_CLASS(sphero_rvr_controllers::AmbientLightSensorBroadcaster, controller_interface::ControllerInterface)
