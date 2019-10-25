#include "main.h"

// Drive motor ports
const int PORT_leftMotor0 = 2;
const int PORT_leftMotor1 = 3;
const int PORT_rightMotor0 = 9;
const int PORT_rightMotor1 = 8;

// Other motor ports
const int PORT_tray = 6;
const int PORT_leftArm = 1;
const int PORT_rightArm = 10;
const int PORT_leftRoller = 4;
const int PORT_rightRoller = 7;

// Encoder ports
const int PORT_leftEncoder = 1;
const int PORT_rightEncoder = 5;
const int PORT_backEncoder = 3;

// Encoders themselves
Encoder leftEncoder;
Encoder rightEncoder;
Encoder backEncoder;

// Sensor ports
const int PORT_armPot = 1;
const int PORT_dialPot = 2;
const int PORT_trayPot = 3;

// Ultrasonic ports
const int PORT_leftSonarOrange = 7;
const int PORT_leftSonarYellow = 8;

// Ultrasonics themselves
Ultrasonic leftSonar;

// Digital switches
const int PORT_interactButton = 9;
//const int PORT_verticalTrayLimit = -1;
//const int PORT_angledTrayLimit = -1;

// Led ports
const int PORT_redAllianceLED = 10;
const int PORT_smallGoalLED = 11;
const int PORT_skillsLED = 12;

// Mutexes
Mutex mutexes[4];
