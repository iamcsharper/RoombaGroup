#pragma once

#include <Arduino.h>

#define clamp(value, min, max) (value < min ? min : value > max ? max : value)
#define bytes_to_u16(MSB, LSB) (((uint16_t)MSB)) << 8 | (((uint8_t)LSB))

#define roomba Serial1
#define RX1_pin 19
#define TX1_pin 18

#define GYRO_ENABLED false

#define TICKS_ONE_TURN 1675.6f /// 8378/5 = 1676
#define TICKS_TO_CM 0.0444339623f

#define DRIVE_PERIOD 20

bool doSeeVirtualWall();
void roomba_init();
void roomba_loop();
void waitForInput();
byte readByte();
byte handlePacket(byte id);
bool handleSensors();
void wakeUp();
void startFull();
void reset();
void startStream(const uint8_t *arr, uint8_t len);
void setLED(bool debrisLED = false, bool spotLED = false, bool dockLED = false, bool warningLED = false, byte color = 0xFF, byte intensity = 0xFF);
void drive(int velocity, int radius);
void driveDirect(int right, int left);
void motors(int bits);
void drivePWM(int mainBrush, int sideBrush, int vacuum);
void driveForwardWithRegulation(float speed, float dLeft, float dRight);
void driveRotateWithRegulation(float speed, int dir, float dl, float dr);
void playSoundGood();
void playSoundBad();

// Установить отсекаемые числа энкодеров в последние достоверные
void resetInitEncoders();

extern const int ddPin; //device detect

// const int w = 235;
// const int diam = 72;
// const int tick = 509;

extern unsigned long driveTime;

extern uint8_t bumpsWheeldrops;     //7
extern uint8_t cliffLeft;           //9
extern uint8_t cliffFrontLeft;      //10
extern uint8_t cliffFrontRight;     //11
extern uint8_t cliffRight;          //12
extern uint8_t virtualWall;         //13
extern uint8_t buttons;             //18
extern uint16_t voltage;            //22
extern int encoderCountsLeft;  //43
extern int encoderCountsRight; //44
extern uint8_t lightBumper;         //45
extern uint8_t isDock;
extern uint8_t isClean;

extern uint8_t lbCenterLeft;
extern uint8_t lbCenterRight;

extern uint8_t isStop;

extern int initEncoderLeft; // вычитаем накопленное значение
extern int initEncoderRight;

extern int encoderLeft;
extern int encoderRight;

extern int lastLeftEnc; // последнее значение энкодера для вычисления погрешности и тд
extern int lastRightEnc;

extern float lastCoveredDistance;
extern float lastRotationDelta;

extern bool hasInitEncoders;