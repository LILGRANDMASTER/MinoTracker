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

    config_.left_wheel_name = info_.hardware_parameters[lw_name_param];
    config_.right_wheel_name = info_.hardware_parameters[rw_name_param];
    config_.left_motor_i2c_addr = info_.hardware_parameters[laddr_param];
    config_.right_motor_i2c_addr = info_.hardware_parameters[raddr_param];

    RCLCPP_DEBUG(logger_, (lw_name_param + static_cast<std::string>(": ") + config_.left_wheel_name).c_str());
    RCLCPP_DEBUG(logger_, (rw_name_param + static_cast<std::string>(": ") + config_.right_wheel_name).c_str());
    RCLCPP_DEBUG(logger_, (laddr_param + static_cast<std::string>(": ") + config_.left_motor_i2c_addr).c_str());
    RCLCPP_DEBUG(logger_, (raddr_param + static_cast<std::string>(": ") + config_.right_motor_i2c_addr).c_str());

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

    motor_left.setup(config_.left_wheel_name, laddr);
    motor_right.setup(config_.right_wheel_name, raddr);

    /* TODO : info_.hardware_parameters[lrpt_param] */
    motor_left.wheel_.rpt_ = 0.0628;
    motor_right.wheel_.rpt_ = 0.0628;

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
      hardware_interface::StateInterface(motor_left.wheel_.name_, hardware_interface::HW_IF_VELOCITY, &motor_left.wheel_.vel_));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(motor_left.wheel_.name_, hardware_interface::HW_IF_POSITION, &motor_left.wheel_.pos_));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(motor_right.wheel_.name_, hardware_interface::HW_IF_VELOCITY, &motor_right.wheel_.vel_));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(motor_right.wheel_.name_, hardware_interface::HW_IF_POSITION, &motor_right.wheel_.pos_));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> MinoHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(motor_left.wheel_.name_, hardware_interface::HW_IF_VELOCITY, &motor_left.wheel_.cmd_));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(motor_right.wheel_.name_, hardware_interface::HW_IF_VELOCITY, &motor_right.wheel_.cmd_));

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

    double rpt1 = motor_left.wheel_.rpt_;
    double rpt2 = motor_right.wheel_.rpt_;

    motor_left.wheel_.vel_ = motor_left.get_speed();
    motor_right.wheel_.vel_ = motor_right.get_speed();



    if (motor_left.wheel_.cmd_ > 0)
      motor_left.wheel_.pos_ += (enc1 - motor_left.wheel_.enc_) * rpt1;
    else
      motor_left.wheel_.pos_ -= (enc1 - motor_left.wheel_.enc_) * rpt1;

    if (motor_right.wheel_.cmd_ > 0)
      motor_right.wheel_.pos_ += (enc2 - motor_right.wheel_.enc_) * rpt2;
    else
      motor_right.wheel_.pos_ -= (enc2 - motor_right.wheel_.enc_) * rpt2;


    motor_left.wheel_.enc_ = enc1;
    motor_right.wheel_.enc_ = enc2;

    return hardware_interface::return_type::OK;
  }


  hardware_interface::return_type MinoHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    RCLCPP_DEBUG(logger_, "Setting up motors speed");
    RCLCPP_DEBUG(logger_, "Left motor speed: %f", motor_left.wheel_.cmd_);
    RCLCPP_DEBUG(logger_, "Right motor speed: %f", motor_right.wheel_.cmd_);

    double rpm1 = motor_left.wheel_.cmd_;
    double rpm2 = motor_right.wheel_.cmd_;

    motor_left.set_speed(rpm1);
    motor_right.set_speed(rpm2);

    RCLCPP_DEBUG(logger_, "Setup finished");
    
    return hardware_interface::return_type::OK;
  }
}

PLUGINLIB_EXPORT_CLASS(mino_hardware::MinoHardware, hardware_interface::SystemInterface)