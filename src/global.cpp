#include "global.h"

void debug_init()
{
#if DEBUG_ENABLED
  Serial.begin(9600);
#endif
}

byte noop() { return 0; }