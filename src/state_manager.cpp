#include "global.h"
#include "state_manager.h"
#include "bluetooth.h"
#include "sonar.h"
#include "HMC5883L.h"
#include "roomba.h"
#include "network.h"

GlobalState globalState = default_state;
GlobalState prevState = default_state;

bool on = true;
unsigned long lastBlinkTime = 0;

enum MoveState
{
  Stop,
  Forward,
  GoingForward,
  Left,
  RotatingLeft,
  Right,
  RotatingRight,
  AlongWallUntilObstacle,
  AlongWallUntilDistance,

  AwaitObstacleLeave
};

MoveState moveState = AlongWallUntilObstacle;

float initCompassAngle = .0f;

int beginStateInMs = 0;

bool calibrateFlag = false;

void state_manager_init()
{
}

#define roomLength 170
#define roomWidth 20

MoveState stateAfterRotation = AlongWallUntilObstacle;
int wallIndex = 0;
float untilDistance = 0;

float minimum = -1;

void resetGlassCorridor()
{
}

MoveState stateAfterObstacle = Stop;

void awaitObstacle(float until, float trig)
{
  if (lastCoveredDistance < until - trig && hasObjectInFront())
  {
    driveDirect(0, 0);
    stateAfterObstacle = moveState;
    moveState = AwaitObstacleLeave;
  }
}

void awaitObstacleLeave()
{
  if (hasGoneObjectInFront())
  {
    moveState = stateAfterObstacle;
  }
}

bool areWeAtDestination()
{
  return (lastCoveredDistance > untilDistance || abs(lastCoveredDistance - untilDistance) < 4);
}

float extraRot = 0;

bool areWeAtAngle()
{
  return abs(lastRotationDelta - extraRot - HALF_PI) < 0.01f;
}

enum MSState
{
  Stop,
  WaitingForObstacleToLeave,
  Rotate,
  Rotating,
  Go_Forward,
  Going_Forward,
  Avoid_Obstacle_Left,
  Avoiding_Obstacle_Left,
  Avoid_Obstacle_Right,
  Avoiding_Obstacle_Right,
  SimpleRotate,
  WaitingForNewPoint,
  BT_Control
} MSstate,
    MSprevState;

bool rotEpsilonParity = true;

#define POINTS_COUNT 4
int next = 0;
float points_x[POINTS_COUNT] = {50, -50, 50, 0};
float points_y[POINTS_COUNT] = {100, 200, 100, 0};

bool going_inwards = true;

float target_x = 0, target_y = 0;
float x = 0, y = 0;
float old_x = 0, old_y = 0;

float old_rotation = 0;
float rotation = 0;
float rotation_target = 0;
float extra_rotation = 0;
int rotation_dir = 1;

int lastTimeMovingForward = 0;
int lastMoveTime = 0;

bool had_obstacle_front = false;
bool had_right_ping = false;

int sign(float val)
{
  if (val < 0)
  {
    return -1;
  }

  return 1;
}

float calc_angle_to_target()
{
  float dir_x = target_x - x;
  float dir_y = target_y - y;

  float dir_len = sqrtf(dir_x * dir_x + dir_y * dir_y);

  //dir_x = (int)(dir_x/dir_len);
  dir_y = dir_y / dir_len;

  // front = (0, 1)
  // dot = dir • front
  // float dot = dir_y;
  ///////////  = dir_x * 0 + dir_y * 1

  return acos(dir_y) * sign(dir_x);
}

float tx = 0, ty = 300;

#define ROBOTS_NUM 1

bool canMove[ROBOTS_NUM];

bool should_all_stop()
{
    for (byte i = 0; i < ROBOTS_NUM; i++)
    {
        if (!canMove[i])
            return true;
    }

    return false;
}

void sendHendehoh(byte exceptForId)
{
    bool stopping = should_all_stop();
    // себе ничего не отправляем поэтому и с 1
    for (byte i = 1; i < ROBOTS_NUM; i++)
    {
        // Зачем говорить той, что уже стоит, - "стоять"? о_о
        if (i == exceptForId)
            continue;

        if (stopping)
        {
            Serial.println("Sending all stop");

            Udp.beginPacket(IPAddress(192, 168, 1, 201), 4210);
            Udp.write(0);
            Udp.endPacket();
        }
    }
}

void handleMaster()
{
    if (millis() - lastTime > 2000 && !should_all_stop())
    {
        int iX = (int)tx;
        int iY = (int)ty;

        Serial.println("Sending coordinates...");
        for (int i = 1; i < ROBOTS_NUM; i++)
        {
            Udp.beginPacket(idToIp(i), port);
            Udp.write(1);
            Udp.write((byte)(iX >> 8));
            Udp.write((byte)(iX & 0xff));
            Udp.write((byte)(iY >> 8));
            Udp.write((byte)(iY & 0xff));
            Udp.endPacket();
        }
        lastTime = millis();
    }

    int packetSize = Udp.parsePacket();

    if (packetSize)
    {
        IPAddress senderIP = Udp.remoteIP();
        Serial.printf("Received %d bytes from %s, port %d\n", packetSize, senderIP.toString().c_str(), Udp.remotePort());

        int len = Udp.read(packetBuffer, 255);

        if (len > 0)
        {
            packetBuffer[len] = '\0';

            byte roombaId = ipToId(senderIP);
            byte packetId = packetBuffer[0];

            // Все чисто
            if (packetId == 2)
            {
                Serial.print("All is clear for Robot#");
                Serial.println(roombaId);
                canMove[roombaId] = true;

                sendHendehoh(roombaId);
            }

            // Видит препятствие
            if (packetId == 3)
            {
                Serial.print("OBSTACLE!!! for Robot#");
                Serial.println(roombaId);
                canMove[roombaId] = false;

                sendHendehoh(roombaId);
            }

            for (int i = 0; i < 255; i++)
            {
                packetBuffer[i] = 0;
            }
        }
    }
}

unsigned long slaveTimer = 0;

void state_manager_loop()
{
  if (globalState == calibrate_compass)
  {
    // if (!calibrateFlag)
    // {
    //   driveDirect(50, -50);
    //   initCompassAngle = GetCompass();
    //   initCompassAngle += GetCompass();
    //   initCompassAngle += GetCompass();
    //   initCompassAngle /= 3.0f;
    //   calibrateFlag = true;
    // }
    // else
    // {
    //   float curCompass = GetCompass() - initCompassAngle;
    //   if (abs(curCompass) > 2*PI)
    //   {
    //     driveDirect(0, 0);
    //     globalState = waiting_for_commands;
    //     moveState = Stop;
    //     playSoundGood();
    //   }
    // }
  }
  else if (globalState == waiting_for_commands)
  {
    if (millis() - lastBlinkTime > 1000)
    {
      lastBlinkTime = millis();
    }
  }
  else if (globalState == bt_control)
  {
    bluetooth_move();

    if (bluetooth_timeout())
    {
      resetInitEncoders();
      globalState = prevState;
    }
  }
  // Только вика ездит в стеклянном корридоре
#if ROOMBA_NUM == 0
  else if (globalState == glass_corridor)
  {
    if (minimum < 0)
    {
      minimum = sonar_get(0);
    }

    if (millis() >= lastBlinkTime)
    {
      lastBlinkTime = millis() + 200;
      int left = sonar_get(3);
      int right = sonar_get(2);

      if (left == 0)
        left = 200;
      if (right == 0)
        right = 200;

      int dist = (int)(2.0f / (1.0f / left + 1.0f / right));

      print_f("0:%i     2=%i    3:%i      medium:%i\n", sonar_get(0), sonar_get(2), sonar_get(3), dist);
      //print_f("front:%i     encoderLeft:%i    encoderRight=%i    dist:%f     until:%f\n", sonar_get(2), encoderLeft, encoderRight, lastCoveredDistance, untilDistance);
    }

    int frontDistance = sonar_get(2);

    if (frontDistance < 1)
    {
      frontDistance = 200;
    }

    if (moveState == AwaitObstacleLeave)
    {
      awaitObstacleLeave(); //меняем состояние назад при удачном стечении обстоятельств
    }
    else if (moveState == Stop)
    {
      driveDirect(0, 0);
    }
    else if (moveState == AlongWallUntilObstacle)
    {
      drivePIDWall(minimum);
      awaitObstacle(roomLength, 50);

      //print_f("0: %i      1: %i    2: %i    3: %i    4: %i     5: %i\n", sonar_get(0), sonar_get(1), sonar_get(2), sonar_get(3), sonar_get(4), sonar_get(5));

      if (lastCoveredDistance > roomLength && frontDistance < 30 && wallIndex == 0)
      {
        driveDirect(0, 0);
        moveState = Left;
        stateAfterRotation = Forward;
        untilDistance = roomWidth;
      }
    }
    else if (moveState == AlongWallUntilDistance)
    {
      drivePIDWall(minimum);
      awaitObstacle(untilDistance, 40);

      if (areWeAtDestination() || doSeeVirtualWall())
      {
        //movementType = Stop;
        moveState = Left;
        stateAfterRotation = Forward;
        untilDistance = roomWidth;
      }
    }
    else if (moveState == Forward)
    {
      driveDirect(50, 50);
      moveState = GoingForward;
    }
    else if (moveState == GoingForward)
    {
      driveForwardWithRegulation(100, encoderLeft - lastLeftEnc, encoderRight - lastRightEnc);
      awaitObstacle(untilDistance, 50);

      if (frontDistance < 22 && areWeAtDestination())
      {
        if (wallIndex == 1)
        {
          untilDistance = roomLength;
        }

        minimum = minimum * 0.4 + 0.6 * frontDistance;
        moveState = Left;
        stateAfterRotation = AlongWallUntilObstacle;
      }
    }
    else if (moveState == Left)
    {
      extraRot = (encoderLeft - encoderRight) / TICKS_ONE_TURN * PI;
      resetInitEncoders();
      moveState = RotatingLeft;
    }
    else if (moveState == RotatingLeft)
    {
      driveRotateWithRegulation(70, 1, encoderLeft - lastLeftEnc, encoderRight - lastRightEnc);

      if (areWeAtAngle())
      {
        driveDirect(0, 0);
        //minimum = sonar_get(0);
        moveState = stateAfterRotation;
        resetInitEncoders();
        wallIndex = (wallIndex + 1) % 4;
      }

      lastLeftEnc = encoderLeft;
      lastRightEnc = encoderRight;
    }

    // end glass corridor
  }
#endif
  else if (globalState == long_corridor)
  {
    // end long corridor
  }
  else if (globalState == darkness_to_scene)
  {
    // end darkness to scene
  }
  else if (globalState == dance_roomba)
  {
    // end dance
  }
  else if (globalState == masters_move)
  {
    float sqrDistanceToTarget = (target_x - x) * (target_x - x) + (target_y - y) * (target_y - y);
    if (MSstate != Stop)
    {
      handleMaster();
    }

    if (sqrDistanceToTarget < 10 * 10)
    {
      target_x = points_x[next];
      target_y = points_y[next];
      next = (next + 1) % POINTS_COUNT;
      MSstate = Rotate;
    }
  }
  else if (globalState == slaves_move)
  {
    float sqrDistanceToTarget = (target_x - x) * (target_x - x) + (target_y - y) * (target_y - y);
    bool iSeeLegs = false;
    if (udp_get())
    {
      // Read STOP!
      if (incomingPacket[0] == 0)
      {
        MSstate = Stop;
        Serial.println("Stop");

        for (int i = 0; i < NTP_PACKET_SIZE; i++)
        {
          incomingPacket[i] = 255;
        }
      }
      // Read MOVE TO!
      else if (incomingPacket[0] == 1)
      {
        target_x = (signed short)((signed short)(incomingPacket[1]) << 8 | incomingPacket[2]);
        target_y = (signed short)((signed short)(incomingPacket[3]) << 8 | incomingPacket[4]);
        if (((target_x - x) * (target_x - x) + (target_y - y) * (target_y - y)) > 10)
        {
          if (iSeeLegs)
          {
            uint8_t arr[] = {3};
            udp_send(arr, 1, IPAddress(192, 168, 1, 200));
          }
          else
          {
            MSstate = Rotate;
          }
        }

        for (int i = 0; i < NTP_PACKET_SIZE; i++)
        {
          incomingPacket[i] = 255;
        }
      }
    }

    if (millis() - slaveTimer > 200)
    {
      int right = sonar_get(0);
      int quiteright = sonar_get(1);
      int frontright = sonar_get(2);
      int frontleft = sonar_get(3);
      int quiteleft = sonar_get(4);
      int left = sonar_get(5);
      int front = (frontleft + frontright) / 2;

      int encoderLeft = encoderCountsLeft - initEncoderLeft;
      int encoderRight = encoderCountsRight - initEncoderRight;
      float distance = (encoderLeft + encoderRight) / 2.0f * TICKS_TO_CM;

      iSeeLegs = (frontleft > 0 && frontleft < 35);
      iSeeLegs |= (frontright > 0 && frontright < 35);
      iSeeLegs |= (quiteleft > 0 && quiteleft < 35);
      iSeeLegs |= (quiteright > 0 && quiteright < 35);
      iSeeLegs |= lbCenterLeft > 40;
      iSeeLegs |= lbCenterRight > 40;

      if (MSstate == Stop)
      {
        driveDirect(0, 0);
      }
      else if (MSstate == SimpleRotate)
      {
        driveDirect(150, -150);
      }
      else if (MSstate == WaitingForObstacleToLeave)
      {
        driveDirect(0, 0);

        bool allIsClear = (frontleft == 0 || frontleft > 50);
        allIsClear &= (frontright == 0 || frontright > 50);

        if (allIsClear && lbCenterRight < 15 && lbCenterLeft < 15)
        {
          MSstate = MSprevState;
          uint8_t arr[] = {2};
          udp_send(arr, 1, IPAddress(192, 168, 1, 200));
        }
      }
      else if (MSstate == Rotate)
      {
        // Вычисляем поворот
        rotation_target = calc_angle_to_target();

        rotation_dir = 1;

        if (rotation_target < rotation)
        {
          rotation_dir = -1;
        }

        driveRotateWithRegulation(30, rotation_dir, abs(encoderLeft - lastLeftEnc), abs(encoderRight - lastRightEnc));

        initEncoderLeft = encoderCountsLeft;
        initEncoderRight = encoderCountsRight;
        old_rotation = rotation; // Сохраняем текущее вращение

        MSstate = Rotating;
      }
      else if (MSstate == Rotating)
      {
        lastTimeMovingForward = millis();

        // На сколько радиан мы повернули
        ///////////////////// <------------------------------------------>   -   сколько полных оборотов мы совершили (в виде дроби)
        float deltaRot = (float)(abs(encoderLeft - encoderRight)) / one_turn_ticks * PI;

        float rotError = (rotation - rotation_target);

        driveRotateWithRegulation(120 * clamp(abs(rotError), 0.2f, 1) + 5, rotation_dir, abs(encoderLeft - lastLeftEnc), abs(encoderRight - lastRightEnc));

        rotation = old_rotation + rotation_dir * deltaRot; // Текущее вращение есть то что было плюс то что "набежало" к текущему моменту

        bool shouldWeStopRotating = false;

        if (rotEpsilonParity)
        {
          // Паритет в сторону повернуть больше чем надо
          shouldWeStopRotating = rotation_dir * rotError > 0.001f;
        }
        else
        {
          // Паритет в сторону повернуть меньше чем надо
          shouldWeStopRotating = abs(rotError) < 0.001f;
        }

        rotEpsilonParity = !rotEpsilonParity;

        if (shouldWeStopRotating)
        {
          driveDirect(0, 0);
          MSstate = Go_Forward;

          initEncoderLeft = encoderCountsLeft;
          initEncoderRight = encoderCountsRight;
          old_rotation = rotation; // Сохраняем текущее вращение
        }
      }
      else if (MSstate == Go_Forward)
      {
        journeyLength = sqrDistanceToTarget;

        driveForwardWithRegulation(250, encoderLeft, encoderRight, abs(encoderLeft - lastLeftEnc), abs(encoderRight - lastRightEnc));

        old_x = x;
        old_y = y;

        initEncoderLeft = encoderCountsLeft;
        initEncoderRight = encoderCountsRight;

        MSstate = Going_Forward;
      }
      else if (MSstate == Going_Forward)
      {
        // Видим препятствие
        if (iSeeLegs)
        {
          MSstate = WaitingForObstacleToLeave;
          MSprevState = Going_Forward;
          uint8_t arr[] = {3};
          udp_send(arr, 1, IPAddress(192, 168, 1, 200));
        }

        float speedCoef = 1;

        if (sqrDistanceToTarget / journeyLength < 0.01f)
          speedCoef = clamp(sqrDistanceToTarget / journeyLength, 0.4f, 1);

        driveForwardWithRegulation(250 * speedCoef, encoderLeft, encoderRight, encoderLeft - lastLeftEnc, encoderRight - lastRightEnc);

        float dx = distance * (float)sin(rotation);
        float dy = distance * (float)cos(rotation);

        x = old_x + dx;
        y = old_y + dy;

        // Раз в 2.5 секунды поворачиваем заново в сторону цели) Чтобы получилась своеобразная дуга состоящая из отрезков
        if (!had_obstacle_front && millis() - lastTimeMovingForward > 2200 && sqrDistanceToTarget > 40 * 40)
        {
          // driveDirect(0, 0);
          // _state = Rotate;
        }

        // Уже увидели спереди препятствие
        if (had_obstacle_front)
        {
          // Правым краем видим препятствие на расстоянии от 1 до 20 см
          if ((right < 30 && right > 0) || (quiteright < 30 && quiteright > 0))
          {
            had_right_ping = true;
            lastTimeMovingForward = millis();
          }

          // 1) время достаточное для объезда соблюдено
          // 2) правым глазом мы видели препятствие
          // 3) а теперь не видим
          if (millis() - lastTimeMovingForward > 400 && had_right_ping && (right > 90 || right == 0))
          {
            had_right_ping = false;
            had_obstacle_front = false;

            // old_rotation = rotation;
            // old_x = x;
            // old_y = y;

            lastTimeMovingForward = millis();

            driveDirect(0, 0);

            MSstate = Rotate; // DEBUG !!! Rotate
          }
        }
      }
      else if (MSstate == Avoid_Obstacle_Left)
      {
        rotation_dir = 1;

        driveRotateWithRegulation(100, rotation_dir, abs(encoderLeft - lastLeftEnc), abs(encoderRight - lastRightEnc));

        initEncoderLeft = encoderCountsLeft;
        initEncoderRight = encoderCountsRight;
        // old_x = x;
        // old_y = y;
        old_rotation = rotation; // Сохраняем текущее вращение

        MSstate = Avoiding_Obstacle_Left;
      }
      else if (MSstate == Avoiding_Obstacle_Left)
      {
        float deltaRot = (float)(abs(encoderLeft - encoderRight)) / one_turn_ticks * PI;

        rotation = old_rotation + rotation_dir * deltaRot;

        driveRotateWithRegulation(100, rotation_dir, abs(encoderLeft - lastLeftEnc), abs(encoderRight - lastRightEnc));

        if (frontleft > 90 || frontleft == 0)
        {
          initEncoderLeft = encoderCountsLeft;
          initEncoderRight = encoderCountsRight;
          old_rotation = rotation; // Сохраняем текущее вращение
          MSstate = Rotating;
          extra_rotation = atan(35 / last_front);
          rotation_target = rotation + extra_rotation;
          had_obstacle_front = true;
        }

        last_front = front;
        if (iSeeLegs)
        {
          if (millis() - sawLegs > 1000)
          {
            uint8_t arr[] = {3};
            udp_send(arr, 1, IPAddress(192, 168, 1, 200));
            sawLegs = millis();
          }
        }
        else
        {
          if (millis() - sawLegs > 1000)
          {
            uint8_t arr[] = {2};
            udp_send(arr, 1, IPAddress(192, 168, 1, 200));
            sawLegs = millis();
          }
        }
      }

      lastLeftEnc = encoderLeft;
      lastRightEnc = encoderRight;

      slaveTimer = millis();
    }
  }
  else if (globalState == free_move)
  {
    // end free move
  }
  else if (globalState == return_darkness)
  {
    // end return darkness
  }
  else if (globalState == return_base)
  {
    // end return base
  }
}