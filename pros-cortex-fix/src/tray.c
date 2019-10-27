#include "main.h"

PID trayPID;

void moveTrayVertical()
{
  print("Starting moveTrayVertical\n");

  //moveArmsScoreAsync();

  pidInit(&trayPID, 0.0007, 0.0, 0.0);
  bool isAtTarget = false;

  while (!isAtTarget)
  {
    // Calculate and set power for arms
    int power = pidCalculate(&trayPID, TRAY_VERTICAL, getTrayPot()) * 100 + 40;
    setTray(power);

    // Debug
    printf("trayPot: %d\tpower: %d\ttarget: %d\n", getTrayPot(), power, TRAY_VERTICAL);

    // Disengages if tray is within 20 ticks
    isAtTarget = abs(getTrayPot() - TRAY_VERTICAL) < 20;

    delay(20);
  }

  print("Finished moveTrayVertical\n");
}
void moveTrayAngled()
{
  print("Starting moveTrayAngled\n");

  pidInit(&trayPID, 0.0007, 0.0, 0.0);
  bool isAtTarget = false;

  while (!isAtTarget)
  {
    // Calculate and set power for arms
    int power = pidCalculate(&trayPID, TRAY_ANGLED, getTrayPot()) * 127;
    setTray(power);

    // Debug
    printf("trayPot: %d\tpower: %d\ttarget: %d\n", getTrayPot(), power, TRAY_ANGLED);

    // Disengages if tray is within 20 ticks
    isAtTarget = abs(getTrayPot() - TRAY_ANGLED) < 100;

    delay(20);
  }
  print("Finished moveTrayAngled\n");
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
