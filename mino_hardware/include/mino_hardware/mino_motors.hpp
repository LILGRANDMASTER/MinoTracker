#ifndef MINO_MOTORS_HPP_
#define MINO_MOTORS_HPP_

#include <cstdint>
#include <string>
#include <unistd.h>

namespace mino_hardware
{
  using byte = uint8_t;

  /* Коды ошибок */
  enum class MinoError : int
  {
    SUCCESS = 0,
    NOT_INITIALIZED = -1,
    I2C_SETUP_FAILED = -2,
    I2C_WRITE_FAILED = -3,
    I2C_READ_FAILED = -4,
  };

  class MinoMotor
  {
  public:
    MinoMotor();
    ~MinoMotor();

    MinoMotor(const MinoMotor&) = delete;
    MinoMotor& operator=(const MinoMotor&) = delete;
    MinoMotor(MinoMotor&& other) noexcept;
    MinoMotor& operator=(MinoMotor&& other) noexcept;

    /* Инициализация и перезагрузка модуля */
    MinoError setup(const std::string& name, byte i2c_addr, double rpt);
    MinoError reset();

    /* Управление моторов */
    MinoError set_speed(double rpm);
    MinoError set_direction(bool clockwise);
    MinoError hard_stop();

    /* Получение данных */
    double get_speed(void);
    uint32_t get_encoder_ticks(void);
    bool get_direction(void);

    struct Wheel
    {
      std::string name_ = "";
      double rpt_ = 0;
      double cmd_ = 0;
      double vel_ = 0;
      double pos_ = 0;
      uint32_t enc_ = 0;
    };

    static constexpr double MAX_RPM_VALUE = 32767.0;
    static constexpr double MIN_RPM_VALUE = -32768.0;

    Wheel& get_wheel_data() { return wheel_; }
    bool is_initialized() const { return fd_ >= 0; }

  private:
    Wheel wheel_;
    int fd_ = -1;

    MinoError i2c_write_byte_internal(byte reg, byte value) const;
    MinoError i2c_write_block_internal(byte reg, const byte* buffer, int length) const;
    MinoError i2c_read_byte_internal(byte reg, byte& out_value) const;
    MinoError i2c_read_block_internal(byte reg, byte* buffer, int length) const;
  };

  // (Опционально) Вспомогательная функция для преобразования MinoError в строку
  const char* to_string(MinoError err);

} // namespace mino_hardware

#endif // MINO_MOTORS_HPP_
