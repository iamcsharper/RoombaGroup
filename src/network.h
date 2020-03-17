#pragma once
#include "global.h"
#include <WiFi.h>
#include <WiFiUdp.h>

extern WiFiUDP Udp;

extern IPAddress myIP;
extern IPAddress lastRemoteIP;
extern unsigned int localUdpPort; // Порт, с которого принимаем UDP пакеты

#define NTP_PACKET_SIZE 128
extern char incomingPacket[NTP_PACKET_SIZE];

void wifi_init();

void udp_init(); // Инициализация

int udp_get(); // возвращает массив символов

void udp_print(const uint8_t *arr, int length, IPAddress ip); // отправляет массив символов на заданный IP и порт поумолчанию

void udp_print(const uint8_t *arr, int length, IPAddress ip, int port); // отправляет массив символов на заданный IP и порт
