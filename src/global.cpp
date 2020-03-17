#include "global.h"
#include "sonar.h"
#include "roomba.h"

void debug_init()
{
#if DEBUG_ENABLED
  Serial.begin(9600);
#endif
}

bool hasObjectInFront()
{
  int left = sonar_get(3);
  int right = sonar_get(2);

  if (left == 0)
    left = 200;
  if (right == 0)
    right = 200;

  // Мы еще не доехали до until
  return (left < 30 || right < 30 || hasFrontObstacle());
}

bool hasGoneObjectInFront()
{
  int left = sonar_get(3);
  int right = sonar_get(2);

  if (left == 0)
    left = 200;
  if (right == 0)
    right = 200;

  int dist = (int)(2.0f / (1.0f / left + 1.0f / right));

  return (dist > 55 && isFrontClear());
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

byte noop() { return 0; }

int sign(float val)
{
  if (val < 0)
  {
    return -1;
  }
  return 1;
}