#include "network.h"

WiFiUDP Udp;

IPAddress myIP(192, 168, 1, 200 + ROOMBA_NUM);
IPAddress masterIP(192, 168, 1, 200);

// char ssid[] = "LAB-NET"; // название вашей сети EnoT_Master
// char pass[] = "+74957208715";
char ssid[] = "Mi"; // название вашей сети EnoT_Master
char pass[] = "";

IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

unsigned int localUdpPort = 4210;    // Порт, с которого принимаем UDP пакеты
char incomingPacket[NTP_PACKET_SIZE]; // Буфер хранения пакетов

void wifi_init() //статический IP)
{
  while (WiFi.status() != WL_CONNECTED)
  {
    WiFi.mode(WIFI_STA);
    WiFi.config(myIP, gateway, subnet, IPAddress(10,0,1,1));
    WiFi.begin(ssid, pass);

    int secs = 0;

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED && secs < 10)
    {
      yield();
      delay(500);
      print(".");
      secs++;
    }
  }

  print("\nWifi connected!\n");
}

void udp_init()
{
  Udp.begin(localUdpPort);
  print_f("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);

  for (int i = 0; i < NTP_PACKET_SIZE; i++)
  {
    incomingPacket[i] = 255;
  }
}

void udp_print(const uint8_t *arr, int length, IPAddress ip)
{
  Udp.beginPacket(ip, localUdpPort); // общение

  for (int i = 0; i < length; i++)
    Udp.write(arr[i]);

  Udp.endPacket();
}

void udp_print(const uint8_t *arr, int length, IPAddress ip, int port)
{

  Udp.beginPacket(ip, port); // общение

  for (int i = 0; i < length; i++)
    Udp.write(arr[i]);
  Udp.endPacket();
}

IPAddress lastRemoteIP;

int udp_get()
{
  int packetSize = Udp.parsePacket();

  if (packetSize)
  {
    print_f("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());

    lastRemoteIP = Udp.remoteIP();
    int len = Udp.read(incomingPacket, NTP_PACKET_SIZE);

    // очищаю лишние элементы массива
    if (len > 0)
    {
      incomingPacket[len] = 0;
    }

    print(incomingPacket);
    print("\n");
  }

  return packetSize;
}
