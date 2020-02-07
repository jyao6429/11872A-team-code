#include "main.h"

void logDataDouble(const char *name, double value)
{
  char temp[25];
  sprintf(temp, "%3.3f", value);
  writeJINXData(name, temp);
}
void logDataInt(const char *name, int value)
{
  char temp[25];
  sprintf(temp, "%d", value);
  writeJINXData(name, temp);
}
