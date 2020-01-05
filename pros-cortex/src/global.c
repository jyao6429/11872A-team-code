#include "main.h"

// Drive motor ports
const int PORT_leftMotor0 = 2;
const int PORT_leftMotor1 = 3;
const int PORT_rightMotor0 = 9;
const int PORT_rightMotor1 = 8;

// Other motor ports
const int PORT_leftTray = 1;
const int PORT_rightTray = 10;
const int PORT_leftArm = 6;
const int PORT_rightArm = 6;
const int PORT_leftRoller = 7;
const int PORT_rightRoller = 4;

// Encoder ports
const int PORT_leftEncoder = 3;
const int PORT_rightEncoder = 5;
const int PORT_backEncoder = 1;

// Encoders themselves
Encoder leftEncoder;
Encoder rightEncoder;
Encoder backEncoder;

// Sensor ports
const int PORT_armPot = 2;
const int PORT_dialPot = 3;
const int PORT_trayPot = 1;

// Ultrasonic ports
const int PORT_leftSonarOrange = 7;
const int PORT_leftSonarYellow = 8;

// Ultrasonics themselves
Ultrasonic leftSonar;

// Digital switches
const int PORT_interactButton = 9;

// Led ports
const int PORT_redAllianceLED = 10;
const int PORT_smallGoalLED = 11;
const int PORT_skillsLED = 12;

// Booleans for state
bool isMainConnected = false;
bool isPartnerConnected = false;

// Mutexes
Mutex mutexes[4];
