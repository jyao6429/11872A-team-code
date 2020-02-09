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

extern AutoOptions chosenAuto;

/**
 * Action in the beginning to deploy
 */
void deploy();
void autoSkillsSuperSafe();
void autoBlueSmallSafe();
void autoBlueSmallSuperSafe();
void autoBlueLargeSuperSafe();
void autoRedSmallSafe();
void autoRedSmallSuperSafe();
void autoRedLargeSuperSafe();

#endif
