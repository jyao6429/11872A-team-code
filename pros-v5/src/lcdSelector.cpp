#include "main.h"

// Variables to hold chosen auto
AutoColor color;
AutoSide side;
int smallGoalAuto;
int largeGoalAuto;
bool isConfirmed;

// Variables for lcd use
int leftButtonCounter;

#define NUM_SMALL_SCRIPTS 6
#define NUM_LARGE_SCRIPTS 2

void updateText()
{
  // Text for confirmation
  if (!isConfirmed)
  {
    pros::lcd::set_text(1, "SELECTING AUTON: UNCONFIRMED");
    pros::lcd::set_text(7, "Color & Side\t\tScript\t\tConfirm?");
  }
  else
  {
    pros::lcd::set_text(1, "SELECTING AUTON: CONFIRMED");
    pros::lcd::set_text(7, "Color & Side\t\tScript\t\tCancel?");
  }

  // Text for skills
  if (color == AUTO_COLOR_SKILLS)
  {
    pros::lcd::set_text(2, "SKILLS");
    pros::lcd::clear_line(3);
    pros::lcd::clear_line(4);
    return;
  }

  // Text for alliance color
  if (color == AUTO_COLOR_BLUE)
  {
    pros::lcd::set_text(2, "Alliance: BLUE");
  }
  else
  {
    pros::lcd::set_text(2, "Alliance: RED");
  }

  // Text for side of field and scripts
  if (side == SIDE_SMALL)
  {
    pros::lcd::set_text(3, "Side: SMALL GOAL");
    switch (smallGoalAuto)
    {
      case SMALL_9PT:
        pros::lcd::set_text(4, "Script: 9 POINTS");
        break;
      case SMALL_8PT:
        pros::lcd::set_text(4, "Script: 8 POINTS");
        break;
      case SMALL_7PT:
        pros::lcd::set_text(4, "Script: 7 POINTS");
        break;
      case SMALL_6PT:
        pros::lcd::set_text(4, "Script: 6 POINTS");
        break;
      case SMALL_5PT:
        pros::lcd::set_text(4, "Script: 5 POINTS");
        break;
      case SMALL_1PT:
        pros::lcd::set_text(4, "Script: 1 POINT");
        break;
    }
  }
  else
  {
    pros::lcd::set_text(3, "Side: LARGE GOAL");
    switch (largeGoalAuto)
    {
      case LARGE_5PT:
        pros::lcd::set_text(4, "Script: 5 POINTS");
        break;
      case LARGE_1PT:
        pros::lcd::set_text(4, "Script: 1 POINT");
        break;
    }
  }
}
void onLeftButton()
{
  if (isConfirmed)
    return;

  // Increment counter and check bounds
  leftButtonCounter++;
  if (leftButtonCounter > 4)
    leftButtonCounter = 0;

  // Switch through each situation for counter
  switch (leftButtonCounter)
  {
    case 0:
      color = AUTO_COLOR_BLUE;
      side = SIDE_SMALL;
      break;
    case 1:
      color = AUTO_COLOR_RED;
      side = SIDE_SMALL;
      break;
    case 2:
      color = AUTO_COLOR_BLUE;
      side = SIDE_LARGE;
      break;
    case 3:
      color = AUTO_COLOR_RED;
      side = SIDE_LARGE;
      break;
    case 4:
      color = AUTO_COLOR_SKILLS;
      side = SIDE_SKILLS;
      break;
  }
  updateText();
}
void onCenterButton()
{
  if (isConfirmed)
    return;

  // No need to change for skills
  if (side == SIDE_SKILLS)
    return;

  // Increment the respective side
  if (side == SIDE_SMALL)
  {
    smallGoalAuto++;

    if (smallGoalAuto >= NUM_SMALL_SCRIPTS)
      smallGoalAuto = 0;
  }
  else
  {
    largeGoalAuto++;

    if (largeGoalAuto >= NUM_LARGE_SCRIPTS)
      largeGoalAuto = 0;
  }
  updateText();
}
void onRightButton()
{
  isConfirmed = !isConfirmed;
  updateText();
}
void initLCD()
{
  // Initialize LCD and all variables
  pros::lcd::initialize();
  leftButtonCounter = 0;
  isConfirmed = false;

  color = AUTO_COLOR_BLUE;
  side = SIDE_SMALL;
  smallGoalAuto = SMALL_9PT;
  largeGoalAuto = LARGE_1PT;

  // Set all the default text for the screen
  updateText();

  pros::lcd::register_btn0_cb(onLeftButton);
  pros::lcd::register_btn1_cb(onCenterButton);
  pros::lcd::register_btn2_cb(onRightButton);
}
