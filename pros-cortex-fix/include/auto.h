#ifndef AUTO_H
#define AUTO_H

// Enums for each autonomous routine
typedef enum AutoOptions
{
  AUTO_NONE,
  AUTO_BLUE_SMALL,
  AUTO_BLUE_LARGE,
  AUTO_RED_SMALL,
  AUTO_RED_LARGE,
  AUTO_SKILLS
} AutoOptions;

AutoOptions chosenAuto;

/**
 * Chooses the autonomous program to run based on the potentiometer knob
 */
void autoChooser();

#endif
