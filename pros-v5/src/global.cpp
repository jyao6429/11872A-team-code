#include "main.h"

// Drive motor ports
const int PORT_leftMotor0 = 2;
const int PORT_leftMotor1 = 3;
const int PORT_rightMotor0 = 9;
const int PORT_rightMotor1 = 8;

// Other motor ports
const int PORT_tray = 1;
const int PORT_arm = 6;
const int PORT_leftRoller = 7;
const int PORT_rightRoller = 4;

// Encoder ports
const char PORT_leftEncoder = 'A';
const char PORT_rightEncoder = 'C';
const char PORT_backEncoder = 'E';

// Encoders themselves
//ADIEncoder leftEncoder;
//ADIEncoder rightEncoder;
//ADIEncoder backEncoder;

// Sensor ports
const char PORT_trayPot = 'G';

// Booleans for state
bool isMainConnected = false;
bool isPartnerConnected = false;

// Mutexes
//pros::Mutex mutexes[4];
