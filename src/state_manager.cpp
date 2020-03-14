#include "global.h"
#include "state_manager.h"
#include "bluetooth.h"
#include "sonar.h"
#include "HMC5883L.h"
#include "roomba.h"

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

// PID
#define roomLength 170
#define roomWidth 20

int pid_p = 2;
int pid_d = -20;
float pid_i = 0;
float pid_s = 0.7f;
float reg = 0;
float prev_dis = 0;
float integral = 0;

int minimum = -1;
MoveState stateAfterRotation = AlongWallUntilObstacle;
int wallIndex = 0;
float untilDistance = 0;

void resetGlassCorridor()
{
}

void driveNearWall()
{
  float dis = (sonar_get(0) - minimum) * 0.01;

  reg = dis * pid_p - (dis - prev_dis) * pid_d - integral * pid_i;

  integral += dis;

  driveDirect(250 * clamp(-reg + pid_s, -pid_s, pid_s), 250 * clamp(reg + pid_s, -pid_s, pid_s));

  prev_dis = dis;
}

// END PID

MoveState stateAfterObstacle = Stop;

void awaitObstacle()
{
  int frontDistance = (sonar_get(3) + sonar_get(2)) / 2;

  if (frontDistance > 0 && frontDistance < 40)
  {
    setLED(false, false, false, true);
    stateAfterObstacle = moveState;
    moveState = AwaitObstacleLeave;
  }
}

void awaitObstacleLeave()
{
  int frontDistance = (sonar_get(3) + sonar_get(2)) / 2;

  if (frontDistance == 0 || frontDistance > 50)
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
    // if (millis() - lastBlinkTime > 1000)
    // {
    //   setLED(on, on, on);
    //   on = !on;
    //   lastBlinkTime = millis();
    //   print_f("Sonar: %i\n", sonar_get(0));
    //   print_f("Compass: %f\n", GetCompass());
    // }
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
  else if (globalState == glass_corridor)
  {
    if (minimum < 0)
    {
      minimum = sonar_get(0);
    }

    if (millis() >= lastBlinkTime)
    {
      lastBlinkTime = millis() + 200;

      print_f("0:%i     1:%i    2=%i    3:%i     4:%i     5:%i\n", sonar_get(0), sonar_get(1), sonar_get(2), sonar_get(3), sonar_get(4), sonar_get(5));
      //print_f("front:%i     encoderLeft:%i    encoderRight=%i    dist:%f     until:%f\n", sonar_get(2), encoderLeft, encoderRight, lastCoveredDistance, untilDistance);
    }

    if (moveState != Left && moveState != RotatingLeft && moveState != AwaitObstacleLeave)
    {
      // Если видим препятствие ОП по тормозам, запоминаем состояние
      awaitObstacle();
    }

    int frontDistance = sonar_get(2);

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
      driveNearWall();

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
      driveNearWall();
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

      if (wallIndex == 1)
      {
        if (frontDistance < 22 && areWeAtDestination())
        {
          minimum = minimum * 0.6 + 0.4 * frontDistance;
          moveState = Left;
          stateAfterRotation = AlongWallUntilDistance;
          untilDistance = roomLength;
        }
      }
      else
      {
        if (areWeAtDestination() && frontDistance < 22)
        {
          minimum = minimum * 0.6 + 0.4 * frontDistance;
          moveState = Left;
          stateAfterRotation = AlongWallUntilObstacle;
        }
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
    // end masters move
  }
  else if (globalState == slaves_move)
  {
    // end slaves mode
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