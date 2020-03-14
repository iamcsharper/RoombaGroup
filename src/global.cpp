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

  int dist = (int)(2.0f / (1.0f / left + 1.0f / right));

  // Мы еще не доехали до until
  return (dist < 40 || hasFrontObstacle());
}

bool hasGoneObjectInFront() {
  int left = sonar_get(3);
  int right = sonar_get(2);

  if (left == 0)
    left = 200;
  if (right == 0)
    right = 200;

  int dist = (int)(2.0f / (1.0f / left + 1.0f / right));

  return (dist > 55 && isFrontClear());
}

byte noop() { return 0; }