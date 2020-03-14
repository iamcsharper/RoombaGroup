#pragma once
#include "global.h"

void sonar_init(uint16_t period);
void sonar_loop();
int sonar_get(uint8_t id);
bool CheckTimer(uint8_t id);