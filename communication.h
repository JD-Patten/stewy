// communication.h
#pragma once
#include <WString.h>

void wifiSetup(const char* ssid = nullptr, const char* password = nullptr);
void wifiRun();

// Generic HTTP command callback:
// - path: e.g. "/walk", "/pose1", "/setOffsets"
// - params: the raw params string from '?params=...'
// The callback must be fast (don't block); if heavy work is needed, set flags for main loop.
using HttpCommandCallback = void(*)(const String& path, const String& params);

// register the single generic callback
void onHttpCommand(HttpCommandCallback cb);

// optional: register UDP packet callback (if you want)
struct JoystickPacket {
  int16_t joyX;
  int16_t joyY;
  uint8_t clicked;
  uint8_t padding;
  uint16_t distance;
};
using UdpCallback = void(*)(const JoystickPacket& pkt);
void onUdpPacket(UdpCallback cb);
