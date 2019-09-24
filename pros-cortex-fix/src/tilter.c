#include "main.h"

void setTilter(int power)
{
  motorSet(PORT_tilter, power);
}
void stopTilter()
{
  motorStop(PORT_tilter);
}
void moveTilterVertical()
{
  while (digitalRead(PORT_verticalTilterLimit) != LOW)
  {
    setTilter(127);
    delay(40);
  }
  delay(250);
  setTilter(10);
}
void moveTilterAngled()
{
  while (digitalRead(PORT_angledTilterLimit) != LOW)
  {
    setTilter(-127);
    delay(40);
  }
  delay(250);
  stopTilter();
}
