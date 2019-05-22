/** @file init.c
 * @brief File for initialization code
 *
 * This file should contain the user initialize() function and any functions related to it.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"
#include "aps.h"
#include "global.h"
#include "NERD_Gyro.h"

/*
 * Runs pre-initialization code. This function will be started in kernel mode one time while the
 * VEX Cortex is starting up. As the scheduler is still paused, most API functions will fail.
 *
 * The purpose of this function is solely to set the default pin modes (pinMode()) and port
 * states (digitalWrite()) of limit switches, push buttons, and solenoids. It can also safely
 * configure a UART port (usartOpen()) but cannot set up an LCD (lcdInit()).
 */
void initializeIO()
{
}

/*
 * Runs user initialization code. This function will be started in its own task with the default
 * priority and stack size once when the robot is starting up. It is possible that the VEXnet
 * communication link may not be fully established at this time, so reading from the VEX
 * Joystick may fail.
 *
 * This function should initialize most sensors (gyro, encoders, ultrasonics), LCDs, global
 * variables, and IMEs.
 *
 * This function must exit relatively promptly, or the operatorControl() and autonomous() tasks
 * will not start. An autonomous mode selection menu like the pre_auton() in other environments
 * can be implemented in this task if desired.
 */
void initialize()
{
  // Temporary while testing with current robot
  int IMECount = imeInitializeAll();
  if (IMECount != 2)
    printf("IMEs are not all detected!\n");
  //leftEncoder = encoderInit(PORT_leftEncoder, PORT_leftEncoder + 1, false);
  //rightEncoder = encoderInit(PORT_rightEncoder, PORT_rightEncoder + 1, true);
  backEncoder = encoderInit(PORT_backEncoder, PORT_backEncoder + 1, false);

  // Initialize Gyro
  gyro_init(gyro, PORT_gyro, false);

  initializeAPS(0.0, 0.0, 0.0);
}
