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