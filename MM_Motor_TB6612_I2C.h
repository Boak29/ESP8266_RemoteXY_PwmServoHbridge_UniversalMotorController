#ifndef __MM_MOTOR_TB6612_I2C_H
#define __MM_MOTOR_TB6612_I2C_H

#define TB6612_MOTOR_A 0
#define TB6612_MOTOR_B 1

#include <Arduino.h>

class MM_Motor_TB6612_I2C
{
  public:
    MM_Motor_TB6612_I2C(uint8_t address, uint8_t motor, uint32_t freq);
    MM_Motor_TB6612_I2C(uint8_t address, uint8_t motor, uint32_t freq, uint8_t STBY_IO);

    void run(int16_t speed);

    void halt();
    void standby();

    uint16_t getMaxSpeed() const
    {
      return 10000;
    }
    void loop();

  private:
    void setmotor(uint8_t dir, uint16_t pwm_val);
    void setfreq(uint32_t freq);

    uint8_t _address;
    uint8_t _motor;
    bool _use_STBY_IO = false;
    uint8_t _STBY_IO;

    uint8_t current_dir;
    float current_pwm_val;

    unsigned long last_update_time; // houdt bij wanneer de wemos motor shield zijn laatste commando gekregen heeft, om 10-seconden "feature" te counteren (motor shield blokkeert wanneer hij 10 seconden geen commando krijgt)
};

#endif

