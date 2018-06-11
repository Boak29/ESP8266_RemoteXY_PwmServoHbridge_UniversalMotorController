# ESP8266_RemoteXY_PwmServoHbridge_UniversalMotorController
Configurable RemoteXY based app for controlling motors, servo, led's.. on ESP8266 using Wifi

Using the Arduino IDE, RemoteXY app on Android or iPhone, you can control a combination of
- Servo
- PWM (LED, DC motor, ...)
- H-bridge

All ESP8266 based boards are supported, including NodeMCU, Wemos D1, ESP-01 (4 pin's!), ...

Supported H-bridges are
- L293D (NodeMCU motor shield)
- TB6612 (using 2 PWM channels, the 3rd pin is kept HIGH)
- TB6612 I2C (Wemos D1 motor shield)
- HG7881 (L9110)

The user interface is fixed and contains 
- a joystick and horizontal slider to trim the motor/servo
- a vertical slider
- a joystick and a horizontal slider to trim the motor/servo

The motor setup in case of 2 H-bridge controlled motors can be
- one motor left, one motor richt
- one motor to go forward-backward, another motor to go left-right

The definition of which joystick/slider is controlling which motor/servo/pwm on which pin, is done in a config.h file, no need to adapt the code itself.
The purpose is to be able to develop a remote control for small cars, boats, zeppelins, ... in minutes time and be able to change the hardware without having to change a lot of source code.

