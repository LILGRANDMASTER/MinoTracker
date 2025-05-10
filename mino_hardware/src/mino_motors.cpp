#include "mino_hardware/mino_motors.hpp"
#include "mino_hardware/mino_mot_reg.hpp"

#include <wiringPiI2C.h>
#include <algorithm>
#include <cmath>
#include <cstring> 


namespace mino_hardware
{

  MinoMotor::MinoMotor() : fd_(-1) {}

  MinoMotor::~MinoMotor()
  {
    if (fd_ >= 0)
    {
      close(fd_);
      fd_ = -1;
    }
  }

  MinoMotor::MinoMotor(MinoMotor&& other) noexcept
    : wheel_(std::move(other.wheel_)), fd_(other.fd_) {
    other.fd_ = -1;
  }

  MinoMotor& MinoMotor::operator=(MinoMotor&& other) noexcept {
    if (this != &other) {
      if (fd_ >= 0) {
        close(fd_);
      }
      wheel_ = std::move(other.wheel_);
      fd_ = other.fd_;
      other.fd_ = -1;
    }
    return *this;
  }

  MinoError MinoMotor::setup(const std::string& name, uint8_t i2c_addr, double rpt)
  {
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }

    wheel_.name_ = name;
    wheel_.cmd_ = 0.0;
    wheel_.vel_ = 0.0;
    wheel_.pos_ = 0.0;
    wheel_.enc_ = 0;
    wheel_.rpt_ = rpt;

    fd_ = wiringPiI2CSetup(i2c_addr);
    if (fd_ < 0)
    {
      return MinoError::I2C_SETUP_FAILED;
    }
    return MinoError::SUCCESS;
  }

  // Внутренние I2C хелперы
  MinoError MinoMotor::i2c_write_byte_internal(byte reg, byte value) const {
      if (fd_ < 0) return MinoError::NOT_INITIALIZED;
      if (wiringPiI2CWriteReg8(fd_, reg, value) == -1) {
          return MinoError::I2C_WRITE_FAILED;
      }
      return MinoError::SUCCESS;
  }

  MinoError MinoMotor::i2c_write_block_internal(byte reg, const byte* buffer, int length) const {
    if (fd_ < 0) return MinoError::NOT_INITIALIZED;
      
    for (int i = 0; i < length; i++) {
      if (i2c_write_byte_internal(reg++, buffer[i]) == MinoError::I2C_WRITE_FAILED) {
        return MinoError::I2C_WRITE_FAILED;
      }
    }
     
    return MinoError::SUCCESS;
  }

  MinoError MinoMotor::i2c_read_byte_internal(byte reg, byte& out_value) const {
      if (fd_ < 0) return MinoError::NOT_INITIALIZED;

      int result = wiringPiI2CReadReg8(fd_, reg);
      if (result == -1) {
          return MinoError::I2C_READ_FAILED;
      }
      out_value = static_cast<byte>(result);
      return MinoError::SUCCESS;
  }

  MinoError MinoMotor::i2c_read_block_internal(byte reg, byte* buffer, int length) const {
      if (fd_ < 0) return MinoError::NOT_INITIALIZED;
      // wiringPiI2CReadI2CBlockData возвращает количество прочитанных байт или -1 при ошибке.
      for (int i = 0; i < length; i++) {
        if (i2c_read_byte_internal(reg++, buffer[i]) == MinoError::I2C_READ_FAILED) {
          return MinoError::I2C_READ_FAILED;
        }
      }

      return MinoError::SUCCESS;
  }

  // Публичные методы
  MinoError MinoMotor::reset()
  {
    if (!is_initialized()) return MinoError::NOT_INITIALIZED;
    byte reg = REG_BITS_0;
    byte data = SET_RESET;
    return i2c_write_byte_internal(reg, data);
  }

  MinoError MinoMotor::set_speed(double rpm)
  {
    if (!is_initialized()) return MinoError::NOT_INITIALIZED;

    #if __cplusplus >= 201703L
    int16_t rpm_val = static_cast<int16_t>(std::round(std::clamp(rpm, MIN_RPM_VALUE, MAX_RPM_VALUE)));
    #else
    double clamped_rpm = rpm;
    if (clamped_rpm < MIN_RPM_VALUE) clamped_rpm = MIN_RPM_VALUE;
    if (clamped_rpm > MAX_RPM_VALUE) clamped_rpm = MAX_RPM_VALUE;
    int16_t rpm_val = static_cast<int16_t>(std::round(clamped_rpm));
    #endif

    byte reg = static_cast<byte>(REG_MOT_SET_RPM_L);
    byte data_to_write[2] = {0};
    data_to_write[0] = static_cast<byte>(rpm_val & 0xFF);
    data_to_write[1] = static_cast<byte>((rpm_val >> 8) & 0xFF);

    return i2c_write_block_internal(reg, data_to_write, 2);
  }

  MinoError MinoMotor::hard_stop()
  {
    if (!is_initialized()) return MinoError::NOT_INITIALIZED;
    byte reg = static_cast<byte>(REG_MOT_STOP);
    byte data_to_write = static_cast<byte>(BIT_STOP);
    return i2c_write_byte_internal(reg, data_to_write);
  }

  double MinoMotor::get_speed(void)
  {
    if (!is_initialized()) return 0.0;

    byte reg = static_cast<byte>(REG_MOT_GET_RPM_L);
    byte read_data[2] = {0};

    MinoError status = i2c_read_block_internal(reg, read_data, 2);
    if (status != MinoError::SUCCESS) {
        return 0.0;
    }

    int16_t raw_vel = static_cast<int16_t>((static_cast<int16_t>(read_data[1]) << 8) | read_data[0]);
    return static_cast<double>(raw_vel);
  }

  uint32_t MinoMotor::get_encoder_ticks(void)
  {
    if (!is_initialized()) return 0;

    byte reg = REG_MOT_GET_REV_L;
    byte read_data[3] = {0};

    MinoError status = i2c_read_block_internal(reg, read_data, 3);
    if (status != MinoError::SUCCESS) {
        return 0;
    }

    return (static_cast<uint32_t>(read_data[2]) << 16) |
                (static_cast<uint32_t>(read_data[1]) << 8)  |
                (static_cast<uint32_t>(read_data[0]));
  }

  MinoError MinoMotor::set_direction(bool clockwise)
  {
    if (!is_initialized()) return MinoError::NOT_INITIALIZED;
    byte reg = REG_BITS_2;
    byte current_val;

    MinoError status = i2c_read_byte_internal(reg, current_val);
    if (status != MinoError::SUCCESS) {
        return status; // Не удалось прочитать текущее значение
    }

    byte new_val;
    if (clockwise) {
      new_val = current_val | BIT_DIR_CKW;
    } else {
      new_val = current_val & (~BIT_DIR_CKW);
    }

    return i2c_write_byte_internal(reg, new_val);
  }

  bool MinoMotor::get_direction(void)
  {
    if (!is_initialized()) return false;
    byte reg = REG_BITS_2;
    byte read_data;

    MinoError status = i2c_read_byte_internal(reg, read_data);
    if (status != MinoError::SUCCESS) {
        return false;
    }

    return (read_data & BIT_DIR_CKW) != 0;
  }

  const char* to_string(MinoError err) {
    switch (err) {
      case MinoError::SUCCESS: return "SUCCESS";
      case MinoError::NOT_INITIALIZED: return "NOT_INITIALIZED";
      case MinoError::I2C_SETUP_FAILED: return "I2C_SETUP_FAILED";
      case MinoError::I2C_WRITE_FAILED: return "I2C_WRITE_FAILED";
      case MinoError::I2C_READ_FAILED: return "I2C_READ_FAILED";
      default: return "UNKNOWN_ERROR";
    }
  }

} // namespace mino_hardware
