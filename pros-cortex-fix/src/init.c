/** @file init.c
 * @brief File for initialization code
 *
 * This file should contain the user initialize() function and any functions related to it.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

void initializeIO()
{
  pinMode(PORT_interactButton, INPUT);
  //pinMode(PORT_stopTestButton, INPUT);
  //pinMode(PORT_verticalTilterLimit, INPUT);
  //pinMode(PORT_angledTilterLimit, INPUT);
}

void initialize()
{
  // initialize JINX
  /*
  initJINX(stdout);
  delay(100);
  taskCreate(JINXRun, TASK_DEFAULT_STACK_SIZE, NULL, (TASK_PRIORITY_DEFAULT + 3));
  delay(100);
  */

  // Create mutexes
  mutexes[MUTEX_POSE] = mutexCreate();
  mutexes[MUTEX_ASYNC_CHASSIS] = mutexCreate();
  mutexes[MUTEX_ASYNC_TRAY] = mutexCreate();
  mutexes[MUTEX_ASYNC_ARM] = mutexCreate();

  // Initialize encoders
  leftEncoder = encoderInit(PORT_leftEncoder, PORT_leftEncoder + 1, false);
  rightEncoder = encoderInit(PORT_rightEncoder, PORT_rightEncoder + 1, true);
  backEncoder = encoderInit(PORT_backEncoder, PORT_backEncoder + 1, true);

  // Initialize sonar
  leftSonar = ultrasonicInit(PORT_leftSonarOrange, PORT_leftSonarYellow);

  chosenAuto = AUTO_NONE;
  autoChooser();
}
