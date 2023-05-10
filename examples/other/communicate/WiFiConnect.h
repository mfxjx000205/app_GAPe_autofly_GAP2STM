#pragma once

#include <stdint.h>
#include "cpx.h"

#define LOGGING_MSG_LIMIT 950

typedef enum {
  WIFI_CTRL_SET_SSID                = 0x10,
  WIFI_CTRL_SET_KEY                 = 0x11,

  WIFI_CTRL_WIFI_CONNECT            = 0x20,

  WIFI_CTRL_STATUS_WIFI_CONNECTED   = 0x31,
  WIFI_CTRL_STATUS_CLIENT_CONNECTED = 0x32,
} __attribute__((packed)) WiFiCTRLType;

typedef struct {
  WiFiCTRLType cmd;
  uint8_t data[50];
} __attribute__((packed)) WiFiCTRLPacket_t;

typedef struct
{
    uint8_t seq;
    uint8_t data[WIFI_STREAM_LIMIT];
} WiFi_Stream_Packet_t;

