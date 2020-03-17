#include "global.h"
#include "state_manager.h"
#include "bluetooth.h"
#include "sonar.h"
#include "HMC5883L.h"
#include "roomba.h"
#include "network.h"
#include "group_move.h"

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

#if ROOMBA_NUM == 0
MoveState stateAfterRotation = AlongWallUntilObstacle;
#elif ROOMBA_NUM == 1
MoveState stateAfterRotation = Forward;
int wallIndex = 0;
float untilDistance = roomLength;

float minimum = -1;

bool areWeAtDestination()
{
  return (lastCoveredDistance > untilDistance || abs(lastCoveredDistance - untilDistance) < 4);
}

float extraRot = 0;

float howCloseIsAngle()
{
  return abs(lastRotationDelta - extraRot - HALF_PI);
}

bool areWeAtAngle()
{
  return howCloseIsAngle() < 0.01f;
}
#endif
MoveState moveState = AlongWallUntilDistance;
MoveState stateAfterHitObstacle = Stop;

bool calibrateFlag = false;

void state_manager_init()
{
}

void resetGlassCorridor()
{
}

void awaitObstacle(float until, float trig)
{
  if (lastCoveredDistance >= until - trig)
    return;

  if (hasObjectInFront())
  {
    driveDirect(0, 0);
    stateAfterHitObstacle = moveState;
    moveState = AwaitObstacleLeave;
  }
}

void awaitObstacleLeave()
{
  if (hasGoneObjectInFront())
  {
    moveState = stateAfterHitObstacle;
  }
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

    // For debugging with console :)
    if (millis() >= lastBlinkTime)
    {
      lastBlinkTime = millis() + 200;
      //print_f("front:%i     encoderLeft:%i    encoderRight=%i    dist:%f     until:%f\n", sonar_get(2), encoderLeft, encoderRight, lastCoveredDistance, untilDistance);
    }

    // Get center-right distance from about 5 to 300 cm
    int frontDistance = sonar_get(2);

    if (frontDistance < 1)
    {
      frontDistance = 300;
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
          stateAfterRotation = AlongWallUntilDistance;
        }
        else
        {
          stateAfterRotation = AlongWallUntilObstacle;
        }

        untilDistance = roomLength;
        moveState = Left;
        minimum = minimum * 0.4 + 0.6 * frontDistance;
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
    }

    // end glass corridor
  }
#elif ROOMBA_NUM == 1
  else if (globalState == long_corridor)
  {
    // Set initial PID min
    if (minimum <= 0)
    {
      minimum = sonar_get(0);
    }

    // Get center-right distance from about 5 to 300 cm
    int frontDistance = sonar_get(2) < 1 ? 300 : sonar_get(2);

    if (moveState == AwaitObstacleLeave)
    {
      awaitObstacleLeave(); //меняем состояние назад при удачном стечении обстоятельств
    }
    else if (moveState == Stop)
    {
      driveDirect(0, 0);
    }
    else if (moveState == AlongWallUntilDistance) // 0, 2
    {
      drivePIDWall(minimum);

      // For debugging with console :)
      if (millis() >= lastBlinkTime)
      {
        lastBlinkTime = millis() + 200;
        print_f("Journey: %i ; Target distance: %i ; Right: %i ; Min: %i\n", (int)lastCoveredDistance, sonar_get(0), (int)minimum);
      }

      ///  |
      ///  |
      ///  |
      ///  |____       <-->
      ///^     |
      // ()    |

      minimum += getRightPeak(minimum); // обратно меняем

      awaitObstacle(untilDistance, 30 + 2 * minimum);

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
      driveForwardWithRegulation(100, encoderLeft - lastLeftEnc, encoderRight - lastRightEnc);
      moveState = GoingForward;
    }
    else if (moveState == GoingForward)
    {
      driveForwardWithRegulation(100, encoderLeft - lastLeftEnc, encoderRight - lastRightEnc);
      awaitObstacle(untilDistance, 35 + 2 * minimum);

      if (frontDistance < minimum + 10 && areWeAtDestination())
      {
        stateAfterRotation = AlongWallUntilDistance;
        untilDistance = roomLength;
        moveState = Left;
      }
    }
    else if (moveState == Left)
    {
      extraRot = (encoderLeft - encoderRight) / TICKS_ONE_TURN * PI;
      resetInitEncoders();
      moveState = RotatingLeft;
      driveDirect(0, 0);
    }
    else if (moveState == RotatingLeft)
    {
      driveRotateWithRegulation(100 * clamp(howCloseIsAngle(), 0.5f, 1), 1, encoderLeft - lastLeftEnc, encoderRight - lastRightEnc);

      if (areWeAtAngle())
      {
        minimum = sonar_get(0);

        driveDirect(0, 0);
        //minimum = sonar_get(0);
        moveState = stateAfterRotation;
        resetInitEncoders();
        wallIndex = (wallIndex + 1) % 4;
      }
    }
    // end long corridor
  }
#endif
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
    master_move_loop();
  }
  else if (globalState == slaves_move)
  {
    slave_move_loop();
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