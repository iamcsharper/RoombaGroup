#pragma once
#include <Arduino.h>
#include "state_manager.h"
#include "bluetooth.h"

#define DEBUG_ENABLED true

// 0 - Вика
// 1 - Марина
// 2 - slave
#define ROOMBA_NUM 2

// PID SETTINGS
#if ROOMBA_NUM == 0
static const int pid_p = 2;
static const int pid_d = -20;
static const float pid_i = 0;
static const float pid_s = 0.7f;

#define roomLength 1000 // 15 метров коридор
#define roomWidth 90 // ~1 метр свободного пробега

#define default_state glass_corridor

// PID SETTINGS END
#elif ROOMBA_NUM == 1
static const int pid_p = 1;
static const int pid_d = -25;
static const float pid_i = 0;
static const float pid_s = 0.9f;

#define default_state long_corridor

#define MIN_WALL_THICKNESS 5
#define MAX_TIME_PASS_WALL 200 // ms of DIFFERENTIAL pulse

#define roomLength 330
#define roomWidth 120

#elif ROOMBA_NUM == 2
#define default_state slaves_move
#endif

byte noop();

#define print(text) DEBUG_ENABLED ? Serial.print(text) : noop()
#define print_f(...) DEBUG_ENABLED ? getBTSerial().printf(__VA_ARGS__) : noop()

void debug_init();

// Ожидаем уход объекта спереди
void awaitObstacleLeave();
// Ожидаем объект спереди
void awaitObstacle(float until, float trig);

// Есть объект спереди (уз + ик)
bool hasObjectInFront();
// Объект ушел (гистерезис)
bool hasGoneObjectInFront();

int sign(float val);