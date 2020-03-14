#pragma once
#include <Arduino.h>
#include "state_manager.h"

#define DEBUG_ENABLED true

#define ROOMBA_NUM 1
// 0 - Вика
// 1 - Марина

// PID SETTINGS
#if ROOMBA_NUM == 0
static const int pid_p = 2;
static const int pid_d = -20;
static const float pid_i = 0;
static const float pid_s = 0.7f;

#define default_state glass_corridor

// PID SETTINGS END
#elif ROOMBA_NUM == 1
static const int pid_p = 2;
static const int pid_d = -20;
static const float pid_i = 0;
static const float pid_s = 0.9f;

#define default_state long_corridor

#define MIN_WALL_THICKNESS 10
#define MAX_TIME_PASS_WALL 500 // ms of DIFFERENTIAL pulse

#endif


// CONSTS FOR STATE MANAGER

#if ROOMBA_NUM == 0

#define roomLength 170
#define roomWidth 20

#elif ROOMBA_NUM == 1

#define roomLength 600
#define roomWidth 100

#endif

byte noop();

#define print(text) DEBUG_ENABLED ? Serial.print(text) : noop()
#define print_f(...) DEBUG_ENABLED ? Serial.printf(__VA_ARGS__) : noop()

void debug_init();

// Есть объект спереди (уз + ик)
bool hasObjectInFront();
// Объект ушел (гистерезис)
bool hasGoneObjectInFront();