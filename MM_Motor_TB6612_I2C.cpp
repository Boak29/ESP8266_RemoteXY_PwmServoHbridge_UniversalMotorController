#include "MM_Motor_TB6612_I2C.h"

#include "Wire.h"

#define TB6612_SHORT_BRAKE 0
#define TB6612_CCW  1
#define TB6612_CW   2
#define TB6612_STOP 3
#define TB6612_STANDBY 4


MM_Motor_TB6612_I2C::MM_Motor_TB6612_I2C(uint8_t address, uint8_t motor, uint32_t freq)
{
  _use_STBY_IO = false;

  if (motor == TB6612_MOTOR_A)
    _motor = TB6612_MOTOR_A;
  else
    _motor = TB6612_MOTOR_B;

  Wire.begin();

  _address = address;

  setfreq(freq);
  standby(); // initialises current_pwm_val, current_dir, last_update_time
}


MM_Motor_TB6612_I2C::MM_Motor_TB6612_I2C(uint8_t address, uint8_t motor, uint32_t freq, uint8_t STBY_IO)
{
  _use_STBY_IO = true;
  _STBY_IO = STBY_IO;

  if (motor == TB6612_MOTOR_A)
    _motor = TB6612_MOTOR_A;
  else
    _motor = TB6612_MOTOR_B;

  Wire.begin();

  _address = address;

  setfreq(freq);

  pinMode(_STBY_IO, OUTPUT);
  digitalWrite(_STBY_IO, LOW);
  standby(); // initialises current_pwm_val, current_dir, last_update_time
}


/* setfreq() -- set PWM's frequency
  freq:
  PWM's frequency
*/
void MM_Motor_TB6612_I2C::setfreq(uint32_t freq)
{
  Wire.beginTransmission(_address);
  Wire.write(((byte)(freq >> 16)) & (byte)0x0f);
  Wire.write((byte)(freq >> 16));
  Wire.write((byte)(freq >> 8));
  Wire.write((byte)freq);
  Wire.endTransmission();     // stop transmitting
  delay(100);
}

/* setmotor() -- set motor
  motor:
  _MOTOR_A  0 Motor A
  _MOTOR_B  1 Motor B
  dir:
  _SHORT_BRAKE  0
  _CCW      1
  _CW      2
  _STOP     3
  _STANDBY    4
  pwm_val:
  0.00 - 100.00  (%)
*/

void MM_Motor_TB6612_I2C::run(int16_t speed)
{
  speed = speed > getMaxSpeed() ? getMaxSpeed() : speed;
  speed = speed < -getMaxSpeed() ? -getMaxSpeed() : speed;

  if (speed > 0)
  {
    setmotor(TB6612_CW, speed);
  }
  else
  {
    setmotor(TB6612_CCW, -speed);
  }
}


void MM_Motor_TB6612_I2C::setmotor(uint8_t dir, uint16_t pwm_val)
{
  uint16_t _pwm_val;

  last_update_time = millis();
  current_dir = dir;
  current_pwm_val = pwm_val;

  if (_use_STBY_IO == true) {

    if (dir == TB6612_STANDBY)
    {
      digitalWrite(_STBY_IO, LOW);
      return;
    }
    else
      digitalWrite(_STBY_IO, HIGH);
  }

  Wire.beginTransmission(_address);
  Wire.write(_motor | (byte)0x10);
  Wire.write(dir);

 // _pwm_val = uint16_t(pwm_val * 100);

  if (_pwm_val > 10000)
    _pwm_val = 10000;

  Wire.write((byte)(_pwm_val >> 8));
  Wire.write((byte)_pwm_val);
  Wire.endTransmission();     // stop transmitting


  delay(10); // was oorspronkelijk 100, maar is veel te traag
}

void MM_Motor_TB6612_I2C::halt()
{
  setmotor(TB6612_STOP, 0);
}

void MM_Motor_TB6612_I2C::standby()
{
  setmotor(TB6612_STANDBY, 0);
}

void MM_Motor_TB6612_I2C::loop()
{
  if ( millis() - last_update_time  > 1000)
  {
    setmotor(current_dir,current_pwm_val);
  }
}

