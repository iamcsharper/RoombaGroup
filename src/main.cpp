#include "global.h"
#include "state_manager.h"
#include "roomba.h"
#include "bluetooth.h"
#include "network.h"
#include "sonar.h"
#include "HMC5883L.h"

void setup()
{
  //xTaskCreatePinnedToCore(readSensors, "sensorRead", 12000, NULL, 0, &sensorRead, 0);

  debug_init();
  roomba_init();
  bluetooth_init();
  wifi_init();
  udp_init();
  sonar_init(30);
  //compass_init();

  state_manager_init();
}

void loop()
{
  roomba_loop();
  bluetooth_loop();

  udp_get();

  sonar_loop();

  state_manager_loop();

  //playSoundGood();
}