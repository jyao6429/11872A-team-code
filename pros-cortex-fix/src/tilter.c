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
  /*
  while (digitalRead(PORT_verticalTilterLimit) != LOW)
  {
    setTilter(127);
    delay(40);
  }
  delay(250);
  setTilter(10);
  */

  setTilter(100);
  delay(400);
  setTilter(50);
  delay(400);
  setTilter(5);
}
void moveTilterAngled()
{
  /*
  while (digitalRead(PORT_angledTilterLimit) != LOW)
  {
    setTilter(-127);
    delay(40);
  }
  delay(250);
  stopTilter();
  */
  setTilter(-127);
  delay(200);
  setTilter(-5);
  delay(200);
  stopTilter();
}
