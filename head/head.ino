#include <WiFi.h>
#include <WiFiUdp.h>

// ---- Wi-Fi Network Config ----
const char* ssid = "stewyAP";
const char* password = "stewy123";
const IPAddress receiverIP(192, 168, 4, 1);  // AP’s IP (the UDP receiver)
const int receiverPort = 4210;

WiFiUDP udp;

// ---- Joystick Pins ----
#define VRX_PIN     3   // Joystick X
#define VRY_PIN     1   // Joystick Y
#define SWITCH_PIN  0   // Joystick Button

// ---- Ultrasonic Sensor Pins ----
#define TRIG_PIN    4   // HC-SR04 Trig
#define ECHO_PIN    5   // HC-SR04 Echo



// ---- Packet Structure ----
struct JoystickPacket {
  int16_t joyX;
  int16_t joyY;
  uint8_t clicked;
  uint8_t padding;
  uint16_t distance;
};

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Connect to Access Point
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);

  Serial.println("Connecting to WiFi...");

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > 10000) {
      Serial.println("\nConnection timeout!");
      return;
    }
    delay(200);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(4211);
}

uint16_t getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 20000); // 20 ms timeout
  uint16_t distance = duration * 0.034 / 2; // cm
  return constrain(distance, 0, 400);
}

void loop() {

  static unsigned long lastAttempt = 0;

  if (WiFi.status() == WL_DISCONNECTED && millis() - lastAttempt > 3000) {
    Serial.println("Reconnecting...");
    WiFi.begin(ssid, password);
    lastAttempt = millis();
  }

  if (WiFi.status() != WL_CONNECTED) {
    delay(50);
    return;
  }

  JoystickPacket pkt;

  pkt.joyX = analogRead(VRX_PIN);
  pkt.joyY = analogRead(VRY_PIN);
  pkt.clicked = !digitalRead(SWITCH_PIN);
  pkt.distance = getDistance();

  // Send binary UDP packet
  udp.beginPacket(receiverIP, receiverPort);
  udp.write((uint8_t*)&pkt, sizeof(pkt));
  udp.endPacket();

  // Debug print
  Serial.print("Sent: X=");
  Serial.print(pkt.joyX);
  Serial.print(" Y=");
  Serial.print(pkt.joyY);
  Serial.print(" Clicked=");
  Serial.print(pkt.clicked);
  Serial.print(" Distance=");
  Serial.println(pkt.distance);

  delay(50); // 20 Hz update rate
}
