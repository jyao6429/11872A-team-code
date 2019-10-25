#include "main.h"

PID trayPID;

void moveTrayVertical()
{
  print("Starting moveTrayVertical");
  /*
  while (digitalRead(PORT_verticalTrayLimit) != LOW)
  {
    setTray(127);
    delay(40);
  }
  delay(250);
  setTray(10);
  */
  /*
  setTray(100);
  delay(400);
  setTray(50);
  delay(800);
  setTray(10);
  */
  pidInit(&trayPID, 0.3, 0.0, 0.0);
  bool isAtTarget = false;

  while (!isAtTarget)
  {
    // Calculate and set power for arms
    int power = pidCalculate(&trayPID, TRAY_VERTICAL, getTrayPot());
    setTray(power);

    // Debug
    printf("trayPot: %d\tpower: %d\ttarget: %d\n", getTrayPot(), power, TRAY_VERTICAL);

    // Disengages if tray is within 20 ticks
    isAtTarget = abs(getTrayPot() - TRAY_VERTICAL) < 20;

    delay(20);
  }

  print("Finished moveTrayVertical");
}
void moveTrayAngled()
{
  print("Starting moveTrayAngled");
  /*
  while (digitalRead(PORT_angledTrayLimit) != LOW)
  {
    setTray(-127);
    delay(40);
  }
  delay(250);
  stopTray();
  */
  /*
  setTray(-127);
  delay(400);
  setTray(-20);
  delay(400);
  setTray(-5);
  delay(500);
  stopTray();
  */
  pidInit(&trayPID, 0.7, 0.0, 0.0);
  bool isAtTarget = false;

  while (!isAtTarget)
  {
    // Calculate and set power for arms
    int power = pidCalculate(&trayPID, TRAY_ANGLED, getTrayPot());
    setTray(power);

    // Debug
    printf("trayPot: %d\tpower: %d\ttarget: %d\n", getTrayPot(), power, TRAY_ANGLED);

    // Disengages if tray is within 20 ticks
    isAtTarget = abs(getTrayPot() - TRAY_ANGLED) < 20;

    delay(20);
  }
  print("Finished moveTrayAngled");
}
int getTrayPot()
{
  return analogRead(PORT_trayPot);
}
void setTray(int power)
{
  motorSet(PORT_tray, power);
}
void stopTray()
{
  motorStop(PORT_tray);
}
