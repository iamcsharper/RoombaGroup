#include "bluetooth.h"

#define STX 0x02
#define ETX 0x03

byte cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // bytes received
byte buttonStatus = 0;                  // first Byte sent to Android device
long previousMillis = 0;                // will store last time Buttons status was updated
long sendInterval = 750;                // interval between Buttons status transmission (milliseconds)
String displayStatus = "";              // message to Android device

BluetoothSerial SerialBT;

int joyX; // obtain the Int from the ASCII representation
int joyY;
bool flag; // Флаг ожидания новых данных от Bluetooth

void bluetooth_init()
{
  SerialBT.begin("Roomba " + String(ROOMBA_NUM)); // BlueTooth device name
  while (SerialBT.available())
    SerialBT.read(); // empty RX buffer
}

void getJoystickState(byte data[8])
{
  joyX = (data[1] - 48) * 100 + (data[2] - 48) * 10 + (data[3] - 48); // obtain the Int from the ASCII representation
  joyY = (data[4] - 48) * 100 + (data[5] - 48) * 10 + (data[6] - 48);
  joyX = joyX - 200; // Offset to avoid
  joyY = joyY - 200; // transmitting negative numbers

  if (joyX < -100 || joyX > 100 || joyY < -100 || joyY > 100)
    return; // commmunication error

  flag = true; // (данные получены)

  joyX *= -5; // инверсия для удержания вертикально
  joyY *= 5;
  joyX = constrain(joyX, -500, 500);
  joyY = constrain(joyY, -500, 500);
}

unsigned long pingTime = 0;
void bluetooth_loop()
{
  if (!bluetooth_timeout() && globalState != bt_control)
  {
    prevState = globalState;
    globalState = bt_control;
  }
  if (SerialBT.available())
  {
    pingTime = millis();
    // data received from smartphone

    delay(2);
    cmd[0] = SerialBT.read();
    if (cmd[0] == STX)
    {
      int i = 1;
      while (SerialBT.available())
      {
        delay(1);
        cmd[i] = SerialBT.read();
        if (cmd[i] > 127 || i > 7)
          break; // Communication error
        if ((cmd[i] == ETX) && (i == 2 || i == 7))
          break; // Button or Joystick data
        i++;
      }
      if (i == 7)
        getJoystickState(cmd); // 6 Bytes  ex: < STX "200" "180" ETX >
    }
  }
}

void bluetooth_move()
{
  int speedL, speedR;
  if (flag)
  {

    if (joyY < 0)
    {
      joyY = abs(joyY);

      speedL = joyX - joyY;
      speedR = joyX;

      speedL = constrain(speedL, -500, joyX);
    }
    else if (joyY > 0)
    {
      speedL = joyX;
      speedR = joyX - joyY;

      speedR = constrain(speedR, -500, joyX);
    }
    else
    {
      speedL = joyX;
      speedR = joyX;
    }

    driveDirect(speedL, speedR);
    flag = false; // Флаг ожидания новых данных от Bluetooth
  }
}

bool bluetooth_timeout()
{
  return millis() - pingTime > 1000;
}