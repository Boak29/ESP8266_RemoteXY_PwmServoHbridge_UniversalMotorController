// RemoteXY connection settings
#define REMOTEXY_WIFI_SSID "Machien01"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377

// Default servo minimum & maximum angle (but no constrain)
#define SERVO_ANGLE_MIN 45
#define SERVO_ANGLE_MAX 135

#define PRINT_DEBUG Serial // uncomment if no debug info necessary or if RX/TX pins are used as gpio 

void defaultconfig()
{
  // Setup example: NodeMCU + NodeMCU motorshield (left joystick) + 1x servo (right joystick x-as), 1 pwm on richt joystick y-as , 1 PWM control (slider), connection indication using LEDBUILTIN
  control_config_t *control;
  // TODO SSID 
  configuration.connectionpin = D0; // Builtin LED. PIN_NOT_USED if not used

  // Joystick Left
  configuration.joystickleft  = MOTORSETUP_LEFTRIGHT;

  control = &(configuration.control[CONTROLLER_LEFT_X ]);
  // joystick left x setup
  control->reverse = false;
  control->factor = 40; // 40%
  control->translat = 0; // no translation
  // motor setup : Motor Left
  control->motorclass = MOTORCLASS_HBRIDGE;
  control->hbridgetype = HBRIDGETYPE_L293D; // notemcu motor shield
  control->pin1 = D3;
  control->pin2 = D1;

  control = &(configuration.control[CONTROLLER_LEFT_Y]);
  // joystick left y setup
  control->reverse = false;
  control->factor = 100; // 100%
  control->translat = 0;
  // motor setup: Motor Right
  control->motorclass = MOTORCLASS_HBRIDGE;
  control->hbridgetype = HBRIDGETYPE_L293D;
  control->pin1 = D4;
  control->pin2 = D2;

  // Joystick right
  configuration.joystickright = MOTORSETUP_UNDEFINED;

  control = &(configuration.control[CONTROLLER_RIGHT_X]);
  // joystick right x setup
  control->reverse = false;
  control->factor = 100;
  control->translat = 0;
  // motor setup x (left-right)
  control->motorclass = MOTORCLASS_SERVO;
  control->hbridgetype = HBRIDGETYPE_UNDEFINED;
  control->pin1 = D5;
  control->pin2 = PIN_NOT_USED;


  control = &(configuration.control[CONTROLLER_RIGHT_Y]);
  // joystick right y setup
  control->reverse = false;
  control->factor = 100;
  control->translat = 0;
  // motor setup y (forward-backward)
  control->motorclass = MOTORCLASS_PWM_HALF;
  control->hbridgetype = HBRIDGETYPE_UNDEFINED;
  control->pin1 = D6;
  control->pin2 = PIN_NOT_USED;

  // Slider
  control = &(configuration.control[CONTROLLER_SLIDER]);
  // joystick setup
  control->reverse = false;
  control->factor = 100;
  control->translat = 0;
  // motor setup
  control->motorclass = MOTORCLASS_PWM;
  control->hbridgetype = HBRIDGETYPE_UNDEFINED;
  control->pin1 = D7;
  control->pin2 = PIN_NOT_USED;
}
