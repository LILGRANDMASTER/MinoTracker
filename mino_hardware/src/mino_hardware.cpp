#include "mino_hardware/mino_hardware.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace mino_hardware
{
  hardware_interface::CallbackReturn MinoHardware::on_init(const hardware_interface::HardwareInfo & info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger_, "Hardware initialization...");

    /* Initialize configuration parameters from urdf file */
    config_.left_wheel_name       = info_.hardware_parameters[lw_name_param];
    config_.right_wheel_name      = info_.hardware_parameters[rw_name_param];
    config_.left_motor_i2c_addr   = info_.hardware_parameters[laddr_param];
    config_.right_motor_i2c_addr  = info_.hardware_parameters[raddr_param];
    config_.left_rpt              = std::stod(info_.hardware_parameters[lrpt_param]);
    config_.right_rpt             = std::stod(info_.hardware_parameters[rrpt_param]);

    RCLCPP_DEBUG(logger_, (lw_name_param  + static_cast<std::string>(": ") + config_.left_wheel_name).c_str());
    RCLCPP_DEBUG(logger_, (rw_name_param  + static_cast<std::string>(": ") + config_.right_wheel_name).c_str());
    RCLCPP_DEBUG(logger_, (laddr_param    + static_cast<std::string>(": ") + config_.left_motor_i2c_addr).c_str());
    RCLCPP_DEBUG(logger_, (raddr_param    + static_cast<std::string>(": ") + config_.right_motor_i2c_addr).c_str());
    
    for (const hardware_interface::ComponentInfo & joint : info.joints) {
      if (joint.command_interfaces.size() != 1) {
        RCLCPP_FATAL(logger_, "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    for (const hardware_interface::ComponentInfo & joint : info.joints) {
      if (joint.state_interfaces.size() != 2) {
        RCLCPP_FATAL(logger_, "Joint '%s' has %zu state interfaces found. 1 expected.", joint.name.c_str(), joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    uint8_t laddr = std::stoi(config_.left_motor_i2c_addr, nullptr, 16);
    uint8_t raddr = std::stoi(config_.right_motor_i2c_addr, nullptr, 16);

    double rpt1   = config_.left_rpt;
    double rpt2   = config_.right_rpt;

    motor_left.setup(config_.left_wheel_name, laddr, rpt1);
    motor_right.setup(config_.right_wheel_name, raddr, rpt2);

    RCLCPP_INFO(logger_, "Initialization finished");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn MinoHardware::on_configure(const rclcpp_lifecycle::State  & previous_state)
  {
    RCLCPP_INFO(logger_, "Hardware configuration...");
    RCLCPP_INFO(logger_, "Configuration finished.");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> MinoHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(motor_left.get_wheel_data().name_, hardware_interface::HW_IF_VELOCITY, &motor_left.get_wheel_data().vel_));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(motor_left.get_wheel_data().name_, hardware_interface::HW_IF_POSITION, &motor_left.get_wheel_data().pos_));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(motor_right.get_wheel_data().name_, hardware_interface::HW_IF_VELOCITY, &motor_right.get_wheel_data().vel_));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(motor_right.get_wheel_data().name_, hardware_interface::HW_IF_POSITION, &motor_right.get_wheel_data().pos_));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> MinoHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(motor_left.get_wheel_data().name_, hardware_interface::HW_IF_VELOCITY, &motor_left.get_wheel_data().cmd_));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(motor_right.get_wheel_data().name_, hardware_interface::HW_IF_VELOCITY, &motor_right.get_wheel_data().cmd_));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn MinoHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    RCLCPP_INFO(logger_, "On activate...");
    RCLCPP_INFO(logger_, "Activation finished");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn MinoHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    RCLCPP_INFO(logger_, "On deactivate...");
    RCLCPP_INFO(logger_, "Deactivation finished");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type MinoHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
  {

    uint32_t enc1 = motor_left.get_encoder_ticks();
    uint32_t enc2 = motor_right.get_encoder_ticks();

    double rpt1 = motor_left.get_wheel_data().rpt_;
    double rpt2 = motor_right.get_wheel_data().rpt_;

    motor_left.get_wheel_data().vel_ = motor_left.get_speed();
    motor_right.get_wheel_data().vel_ = motor_right.get_speed();



    if (motor_left.get_wheel_data().cmd_ > 0)
      motor_left.get_wheel_data().pos_ += (enc1 - motor_left.get_wheel_data().enc_) * rpt1;
    else
      motor_left.get_wheel_data().pos_ -= (enc1 - motor_left.get_wheel_data().enc_) * rpt1;

    if (motor_right.get_wheel_data().cmd_ > 0)
      motor_right.get_wheel_data().pos_ += (enc2 - motor_right.get_wheel_data().enc_) * rpt2;
    else
      motor_right.get_wheel_data().pos_ -= (enc2 - motor_right.get_wheel_data().enc_) * rpt2;


    motor_left.get_wheel_data().enc_ = enc1;
    motor_right.get_wheel_data().enc_ = enc2;

    return hardware_interface::return_type::OK;
  }


  hardware_interface::return_type MinoHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    RCLCPP_DEBUG(logger_, "Setting up motors speed");
    RCLCPP_DEBUG(logger_, "Left motor speed: %f", motor_left.get_wheel_data().cmd_);
    RCLCPP_DEBUG(logger_, "Right motor speed: %f", motor_right.get_wheel_data().cmd_);

    double rpm1 = (-1) * motor_left.get_wheel_data().cmd_; // must be invert, check motor docks
    double rpm2 = motor_right.get_wheel_data().cmd_;

    motor_left.set_speed(rpm1);
    motor_right.set_speed(rpm2);
    


    RCLCPP_DEBUG(logger_, "Setup finished");
    
    return hardware_interface::return_type::OK;
  }
}

PLUGINLIB_EXPORT_CLASS(mino_hardware::MinoHardware, hardware_interface::SystemInterface)