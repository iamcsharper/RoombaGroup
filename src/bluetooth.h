#pragma once
#include "global.h"

#include "BluetoothSerial.h"      // https://github.com/espressif/arduino-esp32/tree/master/libraries/BluetoothSerial

#include "roomba.h"

extern int joyX;       // obtain the Int from the ASCII representation
extern int joyY;
extern bool flag;

void bluetooth_init();
void bluetooth_loop();
void getJoystickState(byte data[8]);
void bluetooth_move();
bool bluetooth_timeout();