#pragma once
#include "global.h"

enum GlobalState
{
  calibrate_compass,
  waiting_for_commands,
  bt_control,
  glass_corridor,
  long_corridor,
  darkness_to_scene,
  dance_roomba,
  masters_move,
  slaves_move,
  free_move,
  return_darkness,
  return_base
};

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
  AwaitObstacleLeave,

  RotateToTarget,
  RotatingToTarget,
};

extern GlobalState globalState;
extern GlobalState prevState;

extern MoveState moveState;
extern MoveState stateAfterHitObstacle;

void state_manager_init();
void state_manager_loop();


bool areWeAtDestination();
float howCloseIsAngle();
bool areWeAtAngle();