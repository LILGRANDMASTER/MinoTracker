#ifndef MINO_MOTORS_HPP_
#define MINO_MOTORS_HPP_

#include <cstdint>
#include <string>

namespace mino_hardware
{
  class MinoMotor
  {
    public:
    MinoMotor() = default;
    ~MinoMotor() = default;

    /* Functions for initialization and reset */
    void setup(const std::string & name, uint8_t i2c_addr);
    void reset(void);

    
    void set_speed(double rpm);
    void set_direction(bool dir);

    double get_speed(void);
    uint32_t get_encoder_ticks(void);
    bool get_direction(void);

    struct Wheel {
      std::string name_ = "";     /* Joint name */
      double rpt_ = 0;            /* Radians per tick*/

      double cmd_ = 0;            /* Command interface */
      double vel_ = 0;            /* Velocity */
      double pos_ = 0;            /* Position (angle in radians) */
      
      uint32_t enc_ = 0;          /* Encoder ticks */
    };

    const double max_speed = 32767.0;
    const double min_speed = -32767.0;
    
    Wheel wheel_;
    int fd_;                      /* I2C address */
  };
}

#endif // MINO_MOTORS_HPP_