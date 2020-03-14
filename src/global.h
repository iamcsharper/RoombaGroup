#pragma once
#include <Arduino.h>
#include "state_manager.h"

#define default_state glass_corridor

#define DEBUG_ENABLED true
#define ROOMBA_NUM 0

byte noop();

#define print(text) DEBUG_ENABLED ? Serial.print(text) : noop()
#define print_f(...) DEBUG_ENABLED ? Serial.printf(__VA_ARGS__) : noop()

void debug_init();

// Есть объект спереди (уз + ик)
bool hasObjectInFront();
// Объект ушел (гистерезис)
bool hasGoneObjectInFront();