#ifndef LCD_SELECTOR_HPP
#define LCD_SELECTOR_HPP

// Enums for autonomous selection
typedef enum AutoColor
{
  AUTO_COLOR_BLUE = 0,
  AUTO_COLOR_RED,
  AUTO_COLOR_SKILLS
} AutoColor;
typedef enum AutoSide
{
  SIDE_SMALL = 0,
  SIDE_LARGE,
  SIDE_SKILLS
} AutoSide;
typedef enum AutoSmallScripts
{
  SMALL_5PT = 0,
  SMALL_7PT,
  SMALL_9PT,
  SMALL_6PT,
  SMALL_8PT,
  SMALL_1PT
} AutoSmallScripts;
typedef enum AutoLargeScripts
{
  LARGE_5PT = 0,
  LARGE_1PT
} AutoLargeScripts;

// Variables to hold chosen auto
extern AutoColor color;
extern AutoSide side;
extern int smallGoalAuto;
extern int largeGoalAuto;
extern bool isConfirmed;

void updateText();
void initLCD();

#endif
