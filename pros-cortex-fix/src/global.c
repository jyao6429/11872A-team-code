#include "main.h"

// Drive motor ports
const int PORT_leftMotor0 = 9;
const int PORT_leftMotor1 = 8;
const int PORT_rightMotor0 = 2;
const int PORT_rightMotor1 = 3;

// Other motor ports
const int PORT_tray = -1;
const int PORT_leftArm = -1;
const int PORT_rightArm = -1;
const int PORT_leftRoller = -1;
const int PORT_rightRoller = -1;

// Encoder ports
const int PORT_leftEncoder = 5;
const int PORT_rightEncoder = 1;
const int PORT_backEncoder = 3;

// Encoders themselves
Encoder leftEncoder;
Encoder rightEncoder;
Encoder backEncoder;

// Sensor ports
const int PORT_armPot = 2;

// Ultrasonic ports
const int PORT_leftSonarOrange = 7;
const int PORT_leftSonarYellow = 8;

// Ultrasonics themselves
Ultrasonic leftSonar;

// Digital switches
const int PORT_startTestButton = 11;
const int PORT_stopTestButton = 12;
const int PORT_verticalTrayLimit = -1;
const int PORT_angledTrayLimit = -1;

// Mutexes
Mutex mutexes[4];
