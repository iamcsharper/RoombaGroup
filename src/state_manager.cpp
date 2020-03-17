#include "global.h"
#include "state_manager.h"
#include "bluetooth.h"
#include "sonar.h"
#include "HMC5883L.h"
#include "roomba.h"
#include "network.h"
#include "group_move.h"

#include "corridor_state.h"

GlobalState globalState = default_state;
GlobalState prevState = default_state;

bool on = true;

#if ROOMBA_NUM == 0
MoveState stateAfterRotation = AlongWallUntilObstacle;
#endif

MoveState moveState = AlongWallUntilDistance;
MoveState stateAfterHitObstacle = Stop;

bool calibrateFlag = false;

#if ROOMBA_NUM == 0 || ROOMBA_NUM == 1
int wallIndex = 0;
float untilDistance = roomLength;
float minimum = -1;
float extraRot = 0;
#endif

void state_manager_init()
{
}

void resetGlassCorridor()
{

}

bool areWeAtDestination()
{
    return (lastCoveredDistance > untilDistance || abs(lastCoveredDistance - untilDistance) < 4);
}

float howCloseIsAngle()
{
    return abs(lastRotationDelta - extraRot - HALF_PI);
}

bool areWeAtAngle()
{
    return howCloseIsAngle() < 0.01f;
}

#if ROOMBA_NUM == 0 || ROOMBA_NUM == 1

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

#endif

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
    corridor_state_loop();
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