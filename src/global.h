#pragma once
#include <Arduino.h>
#include "state_manager.h"

// PID SETTINGS
static const int pid_p = 2;
static const int pid_d = -20;
static const float pid_i = 0;
static const float pid_s = 0.7f;
// PID SETTINGS END

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