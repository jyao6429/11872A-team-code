#ifndef DEBUG_H
#define DEBUG_H

/**
 * Wrapper for JINX that logs a double value
 *
 * @param *name - a pointer to a char array (aka a String) to name the logged variable
 * @param value - the double value to log
 */
void logDataDouble(const char *name, double value);
/**
 * Wrapper for JINX that logs an int value
 *
 * @param *name - a pointer to a char array (aka a String) to name the logged variable
 * @param value - the int value to log
 */
void logDataInt(const char *name, int value);

#endif
