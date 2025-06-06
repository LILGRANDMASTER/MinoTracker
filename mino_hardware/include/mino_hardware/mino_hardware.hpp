#ifndef MINO_HARDWARE_HPP_
#define MINO_HARDWARE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "mino_motors.hpp"

namespace mino_hardware 
{
  class MinoHardware : public hardware_interface::SystemInterface
  {
    public:
    MinoHardware() = default;

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
    
    /* Joints names parameters*/
    const std::string lw_name_param   {"left_wheel_name"};
    const std::string rw_name_param   {"right_wheel_name"};

    /* Motor module i2c address parameters*/
    const std::string laddr_param     {"left_motor_i2c_addr"};
    const std::string raddr_param     {"right_motor_i2c_addr"};

    /* Radians per tick parameters*/
    const std::string lrpt_param      {"left_rpt"};
    const std::string rrpt_param      {"right_rpt"};


    struct Config {
      std::string left_wheel_name       = "left_wheel_joint";
      std::string right_wheel_name      = "right_wheel_joint";

      std::string left_motor_i2c_addr   = "0x0C";
      std::string right_motor_i2c_addr  = "0x0A";

      double left_rpt                   = 0.0628;
      double right_rpt                  = 0.0628;
    };

    
    MinoMotor motor_left;
    MinoMotor motor_right;
    
    Config config_;
    rclcpp::Logger logger_ = rclcpp::get_logger("MinoHardware");
  };
}
#endif // MINO_HARDWARE_HPP_