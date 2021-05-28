#pragma once

#include <string>

//selector configuration
#define HUE 180
#define DEFAULT 0
#define AUTONS "Front-HR", "Front-Half", "Back-Half", "None"

namespace selector{

extern int auton;
const char *b[] = {AUTONS, ""};
void init(int hue = HUE, int default_auton = DEFAULT, const char **autons = b);

}
