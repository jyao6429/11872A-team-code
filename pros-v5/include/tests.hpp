#ifndef TESTS_H
#define TESTS_H

// boolean to control toggling between starting and stopping tests
extern bool isTesting;
/**
 * Starts the testing task
 */
void startTesting();
/**
 * Kills the testing task and stops all motors
 */
void stopTesting();

#endif
