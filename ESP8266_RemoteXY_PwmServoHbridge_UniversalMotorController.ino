/*
   -- RemoteXY Configuratble Motor & Servo controller on ESP8266 based chips --

   To compile this code using RemoteXY library 2.3.3 or later version
   download by link http://remotexy.com/en/library/
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/
     - for ANDROID 4.1.1 or later version;
     - for iOS 1.2.1 or later version;

*/


//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// RemoteXY select connection mode and include library
#define REMOTEXY_MODE__ESP8266WIFI_LIB_POINT
#include <ESP8266WiFi.h>

// #define REMOTEXY__DEBUGLOGS Serial

#include <RemoteXY.h>

// RemoteXY configurate
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
{ 255, 7, 0, 0, 0, 45, 0, 8, 8, 0,
  5, 32, 1, 10, 42, 42, 2, 26, 31, 4,
  0, 44, 1, 11, 61, 2, 26, 5, 37, 56,
  9, 42, 42, 2, 26, 31, 4, 160, 3, 54,
  37, 6, 2, 26, 4, 160, 62, 55, 35, 5,
  2, 26
};

// this structure defines all the variables of your control interface
struct {

  // input variable
  int8_t joystickLeft_x; // =-100..100 x-coordinate joystick position
  int8_t joystickLeft_y; // =-100..100 y-coordinate joystick position
  int8_t slider; // =0..100 slider position
  int8_t joystickRight_x; // =-100..100 x-coordinate joystick position
  int8_t joystickRight_y; // =-100..100 y-coordinate joystick position
  int8_t trimLeft; // =-100..100 slider position
  int8_t trimRight; // =-100..100 slider position

  // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY, RemoteXYprev;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

#include <Servo.h>

typedef enum {
  MOTORCLASS_SERVO = 0,
  MOTORCLASS_HBRIDGE,
  MOTORCLASS_PWM,      // PWM (slider)
  MOTORCLASS_PWM_HALF, // PWM only half of the range (joystick)
  MOTORCLASS_ONOFF,    // digital value
} motorclass_t;


typedef enum {
  HBRIDGETYPE_UNDEFINED = 0,
  HBRIDGETYPE_L293D,
  HBRIDGETYPE_TB6612_2PIN, // keep 3rd pin HIGH
  HBRIDGETYPE_HG7881,
  HBRIDGETYPE_TB6612_I2C_A, // Wemos D1 motor shield motor A, don't define pins: must be on standard I2C port
  HBRIDGETYPE_TB6612_I2C_B, // Wemos D1 motor shield motor B, don't define pins: must be on standard I2C port
} hbridge_type_t;

typedef enum {
  MOTORSETUP_UNDEFINED = 0,
  MOTORSETUP_LEFTRIGHT,
  MOTORSETUP_FORWARDTURN, // is acutally the same as MOTORSETUP_UNDEFINED
} motorsetup_t;

#define PIN_NOT_USED   -1

typedef enum {
  CONTROLLER_LEFT_X = 0,
  CONTROLLER_LEFT_Y,
  CONTROLLER_RIGHT_X,
  CONTROLLER_RIGHT_Y,
  CONTROLLER_SLIDER,
  NUM_CONTROLLERS
} controller_id_t;

typedef struct
{
  motorclass_t motorclass;
  hbridge_type_t hbridgetype;
  int pin1;
  int pin2;
  bool reverse; // reverse joystick
  int translat;
  int factor;
} control_config_t;

struct
{
  //  char ssid[20]; TODO
  motorsetup_t joystickleft;
  motorsetup_t joystickright;
  int connectionpin;
  control_config_t control[NUM_CONTROLLERS];
} configuration;

Servo servos[NUM_CONTROLLERS];

#include "MM_Motor_TB6612_I2C.h"
MM_Motor_TB6612_I2C *tb6612_i2c_motor_a = NULL;
MM_Motor_TB6612_I2C *tb6612_i2c_motor_b = NULL;

#include "config.h"

void setup_pin_mode_output(int pin)
{
#ifdef ESP8266
  if ((pin == 1) || (pin == 3)) // RX & TX ESP-01
  {
    pinMode (pin, FUNCTION_3);
  }
#endif
  pinMode(pin, OUTPUT);
}

void init_motors()
{
  for (int count = 0; count < NUM_CONTROLLERS; ++count)
  {
    control_config_t *motor = &(configuration.control[count]);

    switch (motor->motorclass)
    {
      case MOTORCLASS_SERVO:
        setup_pin_mode_output(motor->pin1);
        servos[count].attach(motor->pin1);
        break;

      case MOTORCLASS_PWM:
      case MOTORCLASS_PWM_HALF:
        setup_pin_mode_output(motor->pin1);
        // TODO check if pin can work as PWM
        break;

      case MOTORCLASS_HBRIDGE:
        // TODO check if pins can work as PWM if necessary
        setup_pin_mode_output(motor->pin1);
        setup_pin_mode_output(motor->pin2);
        switch (motor->hbridgetype)
        {
          case HBRIDGETYPE_TB6612_I2C_A :
            if (!tb6612_i2c_motor_a)
            {
              tb6612_i2c_motor_a = new MM_Motor_TB6612_I2C(0x30, TB6612_MOTOR_A, 1000);
            }
            break;

          case HBRIDGETYPE_TB6612_I2C_B :
            if (!tb6612_i2c_motor_b)
            {
              tb6612_i2c_motor_b = new MM_Motor_TB6612_I2C(0x30, TB6612_MOTOR_B, 1000);
            }
            break;
        }
        break;

      case MOTORCLASS_ONOFF:
        setup_pin_mode_output(motor->pin1);
        // TODO check if pin can work as PWM
        break;

    }
    motor_halt(count);
  }
}

int16_t hbridge_getmaxvalue(hbridge_type_t hbridgetype)
{
  switch (hbridgetype)
  {
    case HBRIDGETYPE_L293D:
    case HBRIDGETYPE_TB6612_2PIN:
    case HBRIDGETYPE_HG7881:
      return PWMRANGE;
      break;

    case HBRIDGETYPE_TB6612_I2C_A :
    case HBRIDGETYPE_TB6612_I2C_B :
      return tb6612_i2c_motor_a->getMaxSpeed();
      break;
  }
}

int16_t motor_getmaxvalue(control_config_t *motor)
{
  switch (motor->motorclass)
  {
    case MOTORCLASS_SERVO:
      return SERVO_ANGLE_MAX;

    case MOTORCLASS_PWM:
    case MOTORCLASS_PWM_HALF:
      return PWMRANGE;

    case MOTORCLASS_HBRIDGE:
      return hbridge_getmaxvalue(motor->hbridgetype);

    case MOTORCLASS_ONOFF:
      return 100;
  }
}

long motor_getminvalue(control_config_t *motor)
{
  switch (motor->motorclass)
  {
    case MOTORCLASS_SERVO:
      return SERVO_ANGLE_MIN;

    case MOTORCLASS_PWM:
      return 0;

    case MOTORCLASS_PWM_HALF:
      return -PWMRANGE;

    case MOTORCLASS_HBRIDGE:
      return -hbridge_getmaxvalue(motor->hbridgetype);

    case MOTORCLASS_ONOFF:
      return -100;
  }
}

void hbridge_setspeed(hbridge_type_t hbridgetype, int pin1, int pin2, long motorspeed)
{
#ifdef PRINT_DEBUG
  PRINT_DEBUG.print("hbridge_setspeed: hbridgetype = ");
  PRINT_DEBUG.print(hbridgetype);
  PRINT_DEBUG.print(" pin1 = ");
  PRINT_DEBUG.print(pin1);
  PRINT_DEBUG.print(" pin2 = ");
  PRINT_DEBUG.print(pin2);
  PRINT_DEBUG.print(" motorspeed = ");
  PRINT_DEBUG.println(motorspeed);
#endif


  switch (hbridgetype)
  {
    case HBRIDGETYPE_L293D:
      if (motorspeed > 0)
      {
        digitalWrite(pin1, HIGH);
        analogWrite(pin2, motorspeed);
      }
      else
      {
        digitalWrite(pin1, LOW);
        analogWrite(pin2, -motorspeed);
      }
      break;

    case HBRIDGETYPE_TB6612_2PIN:
      if (motorspeed >= 0)
      {
        analogWrite(pin1, 0);
        analogWrite(pin2, motorspeed);
      }
      else
      {
        analogWrite(pin1, -motorspeed);
        analogWrite(pin2, 0);
      }
      break;

    case HBRIDGETYPE_HG7881:
      if (motorspeed >= 0)
      {
        digitalWrite(pin1, HIGH);
        analogWrite(pin2, PWMRANGE - motorspeed);
      }
      else
      {
        digitalWrite(pin1, LOW);
        analogWrite(pin2, -motorspeed);
      }
      break;

    case HBRIDGETYPE_TB6612_I2C_A :
      tb6612_i2c_motor_a->run(motorspeed);
      break;

    case HBRIDGETYPE_TB6612_I2C_B :
      tb6612_i2c_motor_b->run(motorspeed);
      break;
  }
}

void motor_halt(int motorid)
{
  control_config_t *motor = &(configuration.control[motorid]);
  switch (motor->motorclass)
  {
    case MOTORCLASS_SERVO:
      servos[motorid].write(map(0, -100, 100, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX));
      break;

    case MOTORCLASS_PWM:
    case MOTORCLASS_PWM_HALF:
      analogWrite(motor->pin1, 0);
      break;

    case MOTORCLASS_HBRIDGE:
      switch (motor->hbridgetype)
      {
        case HBRIDGETYPE_L293D:
        case HBRIDGETYPE_TB6612_2PIN:
        case HBRIDGETYPE_HG7881:
          // HG7881: LOW, LOW;
          // L293D: LOW, LOW
          // TB6612-2PIN: LOW,LOW
          digitalWrite(motor->pin1, LOW);
          digitalWrite(motor->pin2, LOW);
          break;

        case HBRIDGETYPE_TB6612_I2C_A :
          tb6612_i2c_motor_a->halt();
          break;

        case HBRIDGETYPE_TB6612_I2C_B :
          tb6612_i2c_motor_b->halt();
          break;
      }
      break;

    case MOTORCLASS_ONOFF:
      digitalWrite(motor->pin1, LOW);
      break;
  }
}

void motor_setvalue(controller_id_t motorid, int motorvalue)
{
#ifdef PRINT_DEBUG
  PRINT_DEBUG.print("motor_setvalue: motorid = ");
  PRINT_DEBUG.print(motorid);
  PRINT_DEBUG.print(" motorvalue = ");
  PRINT_DEBUG.println(motorvalue);
#endif

  control_config_t *motor = &(configuration.control[motorid]);

  switch (motor->motorclass)
  {
    case MOTORCLASS_SERVO:
      // no constrain
      servos[motorid].write(motorvalue);
      break;

    case MOTORCLASS_PWM:
    case MOTORCLASS_PWM_HALF:
      motorvalue = constrain(motorvalue, 0, motor_getmaxvalue(motor));
      analogWrite(motor->pin1, motorvalue);
      break;

    case MOTORCLASS_HBRIDGE:
      motorvalue = constrain(motorvalue, -hbridge_getmaxvalue(motor->hbridgetype), hbridge_getmaxvalue(motor->hbridgetype));
      hbridge_setspeed(motor->hbridgetype, motor->pin1, motor->pin2, motorvalue);
      break;

    case MOTORCLASS_ONOFF:
      if (motorvalue > 70)
      {
        digitalWrite(motor->pin1, HIGH);
      }
      if (motorvalue < -70)
      {
        digitalWrite(motor->pin1, LOW);
      }
      break;
  }
}

inline float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void onChangeJoystickTrim(float x, float y, float trim_x, controller_id_t control_x_id, controller_id_t control_y_id, motorsetup_t motorsetup)
{
  control_config_t *control_x = &(configuration.control[control_x_id]);
  control_config_t *control_y = &(configuration.control[control_y_id]);

  if (RemoteXY.connect_flag)
  {

    if (control_x->reverse)
    {
      x = -x;
      trim_x = -trim_x;
    }
    if (control_y->reverse)
    {
      y = -y;
    }

    x = ((x * (float)control_x->factor) / 100.0);
    y = ((y * (float)control_y->factor) / 100.0);

    x += (float)control_x->translat;
    y += (float)control_y->translat;

    switch (control_x->motorclass)
    {
      case MOTORCLASS_SERVO:
        x += trim_x;
        break;

      case MOTORCLASS_HBRIDGE:
      case MOTORCLASS_PWM:
      case MOTORCLASS_PWM_HALF:
        x += (trim_x * fabs(y / 100.0));
        break;

      case MOTORCLASS_ONOFF:
        // ignore trim_x
        break;
    }

    if (motorsetup == MOTORSETUP_LEFTRIGHT)
    {
      float temp1 = y + x;
      float temp2 = y - x;

      x = temp1;
      y = temp2;
    }
  }
  else
  {
    // when disconnected
#ifdef PRINT_DEBUG
    PRINT_DEBUG.println("onChangeJoystick disconnected ");
#endif
    x = 0;
    y = 0;
  }

  x = mapf(x, -100, 100, motor_getminvalue(control_x), motor_getmaxvalue(control_x));
  y = mapf(y, -100, 100, motor_getminvalue(control_y), motor_getmaxvalue(control_y));

  motor_setvalue(control_x_id, x);
  motor_setvalue(control_y_id, y);
}

void onChangeJoystickTrimLeft()
{
#ifdef PRINT_DEBUG
  PRINT_DEBUG.println("onChangeJoystickTrimLeft ");
#endif

  onChangeJoystickTrim(RemoteXY.joystickLeft_x, RemoteXY.joystickLeft_y, RemoteXY.trimLeft, CONTROLLER_LEFT_X, CONTROLLER_LEFT_Y, configuration.joystickleft);
}

void onChangeJoystickTrimRight()
{
#ifdef PRINT_DEBUG
  PRINT_DEBUG.println("onChangeJoystickTrimRight ");
#endif

  onChangeJoystickTrim(RemoteXY.joystickRight_x, RemoteXY.joystickRight_y, RemoteXY.trimRight, CONTROLLER_RIGHT_X, CONTROLLER_RIGHT_Y, configuration.joystickright);
}

void onChangeSlider()
{
  float slider_value = RemoteXY.slider;
  control_config_t *control = &(configuration.control[CONTROLLER_SLIDER]);

#ifdef PRINT_DEBUG
  PRINT_DEBUG.print("onChangeSlider: ");
  PRINT_DEBUG.println(slider_value);
#endif

  if (control->reverse)
  {
    slider_value = 100.0 - slider_value; // 0 .. 100
  }

  slider_value = ((slider_value * (float)control->factor) / 100.0);

  slider_value += (float)control->translat;

  motor_setvalue(CONTROLLER_SLIDER, mapf(slider_value, 0, 100, motor_getminvalue(control), motor_getmaxvalue(control)));
}

void updateAll()
{
  onChangeJoystickTrimLeft();
  onChangeSlider();
  onChangeJoystickTrimRight();
}

void onChangeConnection()
{
  if (RemoteXY.connect_flag)
  {
#ifdef PRINT_DEBUG
    PRINT_DEBUG.println("Connected");
#endif

    if (configuration.connectionpin != PIN_NOT_USED)
    {
      digitalWrite(configuration.connectionpin, LOW); // // LED of NodeMCU is on when value is LOW
    }
    updateAll();
  }
  else
  {
#ifdef PRINT_DEBUG
    PRINT_DEBUG.println("Disconnected");
#endif
    for (int count = 0; count < NUM_CONTROLLERS; ++count)
    {
      control_config_t *motor = &(configuration.control[count]);
      motor_halt(count);
    }
    // detach servo
  }
}

void setup()
{
#ifdef PRINT_DEBUG
  PRINT_DEBUG.begin(115200);
  PRINT_DEBUG.println("");
  PRINT_DEBUG.println("Remote XY setup start");
#endif

  defaultconfig();
  init_motors();

  if (configuration.connectionpin != PIN_NOT_USED)
  {
    pinMode(configuration.connectionpin, OUTPUT);
    digitalWrite(configuration.connectionpin, HIGH); // LED of NodeMCU is off when value is HIGH
  }
  updateAll();

  RemoteXY_Init ();

  memcpy(&RemoteXYprev, &RemoteXY, sizeof(RemoteXY));

#ifdef PRINT_DEBUG
  PRINT_DEBUG.println("Remote XY setup end");
#endif
}



void loop()
{
  RemoteXY_Handler ();

  if (RemoteXY.connect_flag != RemoteXYprev.connect_flag) {
    onChangeConnection();
  }

  if ((!RemoteXY.connect_flag) && (configuration.connectionpin != PIN_NOT_USED))
  {
    digitalWrite(configuration.connectionpin, (millis() % 1000) > 500 ? LOW : HIGH); // LED of NodeMCU is off when value is HIGH
  }

  if ((RemoteXY.joystickLeft_x != RemoteXYprev.joystickLeft_x) || (RemoteXY.joystickLeft_y != RemoteXYprev.joystickLeft_y) || (RemoteXY.trimLeft != RemoteXYprev.trimLeft))
  {
    onChangeJoystickTrimLeft();
  }

  if (RemoteXY.slider != RemoteXYprev.slider) {
    onChangeSlider();
  }

  if ((RemoteXY.joystickRight_x != RemoteXYprev.joystickRight_x) || (RemoteXY.joystickRight_y != RemoteXYprev.joystickRight_y) || (RemoteXY.trimRight != RemoteXYprev.trimRight))
  {
    onChangeJoystickTrimRight();
  }

  if (tb6612_i2c_motor_a)
  {
    tb6612_i2c_motor_a->loop();
  }

  if (tb6612_i2c_motor_b)
  {
    tb6612_i2c_motor_b->loop();
  }

  memcpy(&RemoteXYprev, &RemoteXY, sizeof(RemoteXY));
  delay(5); // in fact not necessary, just for lower power consumption
}
