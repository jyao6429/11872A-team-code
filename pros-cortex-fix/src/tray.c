#include "main.h"

void setTray(int power)
{
  motorSet(PORT_tray, power);
}
void stopTray()
{
  motorStop(PORT_tray);
}
void moveTrayVertical()
{
  /*
  while (digitalRead(PORT_verticalTrayLimit) != LOW)
  {
    setTray(127);
    delay(40);
  }
  delay(250);
  setTray(10);
  */

  setTray(100);
  delay(400);
  setTray(50);
  delay(800);
  setTray(10);
}
void moveTrayAngled()
{
  /*
  while (digitalRead(PORT_angledTrayLimit) != LOW)
  {
    setTray(-127);
    delay(40);
  }
  delay(250);
  stopTray();
  */
  setTray(-127);
  delay(400);
  setTray(-20);
  delay(400);
  setTray(-5);
  delay(500);
  stopTray();
}
