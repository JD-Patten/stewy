// communication.cpp
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include "user_interface.h"
#include "communication.h"

static WebServer server(80);
static WiFiUDP udp;
const int localPort = 4210;

// callback pointers
static HttpCommandCallback httpCmdCb = nullptr;
static UdpCallback udpCb = nullptr;

void onHttpCommand(HttpCommandCallback cb) { httpCmdCb = cb; }
void onUdpPacket(UdpCallback cb) { udpCb = cb; }

void wifiSetup(const char* ssid_arg, const char* pass_arg) {
  const char* ss = ssid_arg ? ssid_arg : "ESP32-UDP";
  const char* pw = pass_arg ? pass_arg : "12345678";

  WiFi.softAP(ss, pw);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  udp.begin(localPort);
  Serial.println("UDP server started");

  if (!MDNS.begin("robot")) {
    Serial.println("Error starting mDNS");
  }

  // Serve the UI at root
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", USER_INTERFACE_HTML);
  });

  // Generic handler for any other GET path -> forward to main callback
  server.onNotFound([]() {
    String path = server.uri(); // requested path, e.g. "/walk"
    String params = "";
    if (server.hasArg("params")) params = server.arg("params");
    Serial.print("HTTP received: "); Serial.print(path); Serial.print(" params: "); Serial.println(params);

    // Call the registered callback (if any). Do this quickly — avoid long blocking tasks.
    if (httpCmdCb) {
      httpCmdCb(path, params);
    }

    // Quick response to client
    server.send(200, "text/plain", "OK");
  });

  server.begin();
  Serial.println("HTTP server started");
}

void wifiRun() {
  // UDP: check for joystick packets; forward to UDP callback when size matches
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    // If your sender uses the same struct size:
    if (packetSize == (int)sizeof(JoystickPacket)) {
      JoystickPacket pkt;
      int r = udp.read((uint8_t*)&pkt, sizeof(pkt));
      if (r == (int)sizeof(pkt)) {
        if (udpCb) udpCb(pkt);
        else {
          // Default logging if no callback registered
          Serial.print("received UDP packet of size "); Serial.print(r);
      }
    } else {
      // consume/ignore unexpected-sized packet
      uint8_t buf[256];
      int r = udp.read(buf, min(packetSize, (int)sizeof(buf)));
      Serial.printf("Ignored UDP packet of size %d (read %d)\n", packetSize, r);
    }
  }

  // HTTP
  server.handleClient();
}
}
