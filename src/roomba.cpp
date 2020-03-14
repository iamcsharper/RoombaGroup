#include "global.h"
#include "roomba.h"
#include "sonar.h"



const int ddPin = 22;
unsigned long driveTime = 0;
uint8_t bumpsWheeldrops = 0; //7
uint8_t cliffLeft = 0;       //9
uint8_t cliffFrontLeft = 0;  //10
uint8_t cliffFrontRight = 0; //11
uint8_t cliffRight = 0;      //12
uint8_t virtualWall = 0;     //13
uint8_t buttons = 0;         //18
uint16_t voltage = 0;        //22
int encoderCountsLeft = 0;   //43
int encoderCountsRight = 0;  //44
int _encoderCountsLeft = 0;
int _encoderCountsRight = 0;
uint8_t lightBumper = 0;  //45
uint8_t _lightBumper = 0; //45

uint8_t lbCenterLeft = 0; // 48
uint8_t _lbCenterLeft = 0;
uint8_t lbCenterRight = 0; // 49
uint8_t _lbCenterRight = 0;

uint8_t isDock = 0;
uint8_t isClean = 0;
uint8_t isStop = 0;
uint8_t _isStop = 0;

bool _seeVirtualWall = false;
bool seeVirtualWall = false;

int initEncoderLeft = 0; // вычитаем накопленное значение
int initEncoderRight = 0;
int encoderLeft = 0;
int encoderRight = 0;

bool hasInitEncoders = false;

int lastLeftEnc = 0; // последнее значение энкодера для вычисления погрешности и тд
int lastRightEnc = 0;

// Расстояние пройденное с последнего сброса энкодеров
float lastCoveredDistance = 0;
// Изменение угла с последнего сброса энкодеров в радианах
float lastRotationDelta = 0;

void roomba_init()
{
  pinMode(ddPin, OUTPUT);
  roomba.begin(115200, SERIAL_8N1, RX1_pin, TX1_pin);

  delay(1000);
  wakeUp();
  startFull();
  driveDirect(0, 0);

  uint8_t arr[] = {48, 49, 43, 44, 18, 13};
  startStream(arr, 6);

  setLED(false, false, false);

  delay(200);

  setLED(true, true, true);

  delay(200);

  setLED(false, false, false);

  delay(200);

  setLED(true, true, true);
}

void resetInitEncoders()
{
  initEncoderLeft = encoderCountsLeft;
  initEncoderRight = encoderCountsRight;
}

void roomba_loop()
{
  bool hasGoodPackets = handleSensors();

  if (hasGoodPackets)
  {
    // Устанавливаем реальные нулевые положения энкодеров (!= 0 значит что дошли хорошие пакеты)}
    if (!hasInitEncoders)
    {
      resetInitEncoders();
      hasInitEncoders = true;
    }

    lastLeftEnc = encoderLeft;
    lastRightEnc = encoderRight; // delta enc

    encoderLeft = encoderCountsLeft - initEncoderLeft;
    encoderRight = encoderCountsRight - initEncoderRight;

    lastCoveredDistance = (encoderLeft + encoderRight) / 2.0f * TICKS_TO_CM;
    lastRotationDelta = (float)(abs(encoderLeft - encoderRight)) / TICKS_ONE_TURN * PI;
  }
}

int lastPlayTime = millis();

void playSoundBad()
{
  if (millis() - lastPlayTime > 1000)
  {
    roomba.write(140);
    roomba.write(0);
    roomba.write(4); // 4 notes

    roomba.write(58); //A1
    roomba.write(8);

    roomba.write(64); //E1
    roomba.write(8);

    roomba.write(64 + 7); //E1
    roomba.write(8);

    roomba.write(64); //E1
    roomba.write(8);

    roomba.write(141);
    roomba.write(0);

    lastPlayTime = millis();
  }
}

void playSoundGood()
{
  if (millis() - lastPlayTime > 1000)
  {
    roomba.write(140);
    roomba.write(1);
    roomba.write(2); // 4 notes

    roomba.write(84);
    roomba.write(16);

    roomba.write(88);
    roomba.write(32);

    roomba.write(141);
    roomba.write(1);

    lastPlayTime = millis();
  }
}

void waitForInput()
{
  while (!roomba.available())
    yield();
}

byte readByte()
{
  waitForInput();
  byte c = roomba.read();
  return c;
}

int waitSize = 0;

byte handlePacket(byte id)
{
  if (id == 7)
  {
    byte r = readByte();
    if (r & 1 || r & 0)
    {
      //state = STOP;
    }
    return r;
  }
  if (id == 13)
  {
    byte r = readByte();
    waitSize -= 1;
    _seeVirtualWall = r;
    return r;
  }
  if (id == 18)
  {
    byte r = readByte();
    waitSize -= 1;
    if (r & 1)
      _isStop = 1;
    // if (r & 4)
    //   isReset = 1;
    return r;
  }
  if (id == 43)
  {
    byte lbmm = readByte();
    byte rbmm = readByte();
    _encoderCountsLeft = (signed short)((signed short)lbmm << 8 | rbmm);
    waitSize -= 2;
    return lbmm + rbmm;
  }
  if (id == 44)
  {
    byte lbmm = readByte();
    byte rbmm = readByte();
    _encoderCountsRight = (signed short)((signed short)lbmm << 8 | rbmm);
    waitSize -= 2;
    return lbmm + rbmm;
  }
  if (id == 48)
  {
    byte lbmm = readByte();
    byte rbmm = readByte();
    _lbCenterLeft = lbmm << 8 | rbmm;
    waitSize -= 2;
    return lbmm + rbmm;
  }
  if (id == 49)
  {
    byte lbmm = readByte();
    byte rbmm = readByte();
    _lbCenterRight = lbmm << 8 | rbmm;
    waitSize -= 2;
    return lbmm + rbmm;
  }
}

bool handleSensors()
{
  int packetId = 0;
  if (roomba.available())
  {
    if (readByte() == 19)
    {
      waitSize = readByte();
      byte sum = 19 + waitSize;

      while (waitSize > 0)
      {
        packetId = readByte();
        //Serial.print("Have read packet #");
        //Serial.println(packetId);
        sum += packetId;
        waitSize--;
        sum += handlePacket(packetId);
      }

      byte last = readByte(); // TODO CHECKSUM

      if ((sum + last) % 256 == 0)
      {
        //if (packetId == 18)
        isStop = _isStop;
        lightBumper = _lightBumper;
        lbCenterRight = _lbCenterRight;
        lbCenterLeft = _lbCenterLeft;

        encoderCountsLeft = _encoderCountsLeft;
        encoderCountsRight = _encoderCountsRight;

        seeVirtualWall = _seeVirtualWall;

        return true;
      }
      else
      {
        //print("[ROOMBA] bad hashsum\n");
        return false;
      }

      packetId = 0;
      waitSize = 0;
    }
  }

  return false;
}

float lastGz = 0;

int aix, aiy, aiz;
int gix, giy, giz;
float ax, ay, az;
float gx, gy, gz;
float roll, pitch, heading;

//static int rightCompensation = 10;

float convertRawAcceleration(int aRaw)
{
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw)
{
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

void driveRotateWithRegulation(float speed, int dir, float dl, float dr)
{
#if GYRO_ENABLED
  // read raw data from CurieIMU
  BMI160.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

  // convert from raw data to gravity and degrees/second units
  ax = convertRawAcceleration(aix);
  ay = convertRawAcceleration(aiy);
  az = convertRawAcceleration(aiz);
  gx = convertRawGyro(gix);
  gy = convertRawGyro(giy);
  gz = convertRawGyro(giz);

  float deltaGz = gz - lastGz;
  lastGz = gz;
#else
  float deltaGz = 0;
#endif

  int d_regulator = 8 * (dl - dr) + 3 * deltaGz;

  int speedLeft = speed;
  int speedRight = speed;

  speedRight += d_regulator;
  speedLeft -= d_regulator;

  driveDirect(speedRight * dir, speedLeft * -dir);
}

void driveForwardWithRegulation(float speed, float dLeft, float dRight)
{
#if GYRO_ENABLED
  // read raw data from CurieIMU
  BMI160.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

  // convert from raw data to gravity and degrees/second units
  ax = convertRawAcceleration(aix);
  ay = convertRawAcceleration(aiy);
  az = convertRawAcceleration(aiz);
  gx = convertRawGyro(gix);
  gy = convertRawGyro(giy);
  gz = convertRawGyro(giz);

  float delta = (encoderLeft - encoderRight) * 2.0f + gz * 3.0f;
#else
  float delta = (encoderLeft - encoderRight) * 2.0f;
#endif

  //float delta = (encoderLeft - encoderRight) * 2.5f;

  driveDirect(speed + (int)(delta), (speed - (int)(delta)));
}

bool doSeeVirtualWall()
{
  return seeVirtualWall;
}

bool hasFrontObstacle()
{
  return lbCenterLeft > 20 || lbCenterRight > 20;
}

float reg = 0;
float prev_dis = 0;
float integral = 0;

void drivePIDWall(float minimum)
{
  float dis = (sonar_get(0) - minimum) * 0.01;

  reg = dis * pid_p - (dis - prev_dis) * pid_d - integral * pid_i;

  integral += dis;

  driveDirect(250 * clamp(-reg + pid_s, -pid_s, pid_s), 250 * clamp(reg + pid_s, -pid_s, pid_s));

  prev_dis = dis;
}

// END PID

bool isFrontClear() 
{
  return lbCenterLeft < 30 && lbCenterRight < 30;
}

void wakeUp()
{
  digitalWrite(ddPin, HIGH);
  delay(100);
  digitalWrite(ddPin, LOW);
  delay(500);
  digitalWrite(ddPin, HIGH);
  delay(2000);
}

void startFull()
{
  roomba.write(128);
  roomba.write(132);
  delay(1000);
}

void reset()
{
  roomba.write(7);
}

void startStream(const uint8_t *arr, uint8_t len)
{
  roomba.write(148);
  roomba.write(len);
  for (int i = 0; i < len; i++)
  {
    roomba.write(arr[i]);
  }
}

void setLED(bool debrisLED, bool spotLED, bool dockLED, bool warningLED, byte color, byte intensity)
{
  roomba.write(139);
  roomba.write((debrisLED ? 1 : 0) + (spotLED ? 2 : 0) + (dockLED ? 4 : 0) + (warningLED ? 8 : 0));
  roomba.write(color);
  roomba.write(intensity);
}

void drive(int velocity, int radius)
{
  clamp(velocity, -500, 500); //def max and min velocity in mm/s
  clamp(radius, -2000, 2000); //def max and min radius in mm

  roomba.write(137);
  roomba.write(velocity >> 8);
  roomba.write(velocity);
  roomba.write(radius >> 8);
  roomba.write(radius);
}

int lastRightSpeed = 0;
int lastLeftSpeed = 0;

void driveDirect(int right, int left)
{
  clamp(right, -500, 500);
  clamp(left, -500, 500);

  // upd 02.03.2020
  // Одно и то же не посылаем!
  if (lastLeftSpeed != left || lastRightSpeed != right)
  {
    roomba.write(145);
    roomba.write(right >> 8);
    roomba.write(right);
    roomba.write(left >> 8);
    roomba.write(left);
  }

  lastRightSpeed = right;
  lastLeftSpeed = left;
}

void motors(int bits)
{
  roomba.write(138);
  roomba.write(bits);
}

void drivePWM(int mainBrush, int sideBrush, int vacuum)
{
  roomba.write(144);
  roomba.write(mainBrush);
  roomba.write(sideBrush);
  roomba.write(vacuum);
}