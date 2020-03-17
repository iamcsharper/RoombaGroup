#pragma once
#include "global.h"

int getRightPeak(int minimum);


void sonar_init(uint16_t period);
void sonar_loop();
int sonar_get(uint8_t id);
bool CheckTimer(uint8_t id);