#include "inverse_kinematics.h"
#include "controller.h"

#include <ESP32Servo.h>
// removed ESPmDNS, WiFiClient, WiFiAP, WiFiServer includes because comms moved
#include "communication.h"   // new communication module (wifiSetup, wifiRun, onHttpCommand)

#define LED_BUILTIN 48   

// Set these to your desired credentials.
const char *ssid = "stewyAP";
const char *password = "stewy123";

// Servo setup
Servo servos[6];
int servoPins[6] = {3,2,1,18,17,10};
// vector<float> servoOffsets = {5, -3, 2, -2, 8, 6}; // plastic gear offsets
vector<float> servoOffsets = {0, -2, 7, -4, 4, 0};  // metal gear offsets

// Create IK solver instance
IKSolver ikSolver(
    50,     // servoArmLength
    100,    // arm2Length
    73,     // servoOffset1
    15.5,   // servoOffset2
    0,     // servoZOffset
    15,     // draftAngle
    40,     // topPlateOffset1
    20,     // topPlateOffset2
    -10     // topPlateZOffset
);

float maxAcceleration = 10;
float maxRotationalAcceleration = 10;
Controller controller(servoPins, ikSolver, maxAcceleration, maxRotationalAcceleration );  

bool isProcessingRequest = false;  // kept for compatibility (not used by comms here)

//
// Simple HTTP command handler that just prints path and params.
// Replace with your full parser later (this is intentionally minimal).
//
void httpCommandHandler(const String& path, const String& params) {
  Serial.print("HTTP command received. Path: ");
  Serial.print(path);
  Serial.print("  Params: ");
  Serial.println(params);

  if (path == "/pose1") {
    Serial.print("Lets GOOOOOOOO!!");
    Pose poseToSet;
    if (poseToSet.fromString(params)) {
      controller.setGoalPose(poseToSet);
    } else {
      Serial.println("Invalid pose format in params");
    }
  }

  digitalWrite(LED_BUILTIN, HIGH);
  delay(10);
  digitalWrite(LED_BUILTIN, LOW);
}

void udpPacketHandler(const JoystickPacket& pkt) {
    // Example: just print the received joystick packet
    Serial.print("UDP Packet - JoyX: "); Serial.print(pkt.joyX);
    Serial.print(" JoyY: "); Serial.print(pkt.joyY);
    Serial.print(" Clicked: "); Serial.print(pkt.clicked);
    Serial.print(" Distance: "); Serial.println(pkt.distance);

    controller.updateSensorState(pkt.distance, pkt.joyX, pkt.joyY, pkt.clicked);
}
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting up...");
  
  Serial.println();
  Serial.println("Configuring access point...");

  // Start comms (AP, mDNS and HTTP server, UDP etc are handled inside communication.*).
  // wifiSetup will use the SSID/password provided.
  wifiSetup(ssid, password);
  Serial.println("wifiSetup() called");

  // Register generic HTTP command callback so communication module forwards commands here.
  onHttpCommand(httpCommandHandler);
  onUdpPacket(udpPacketHandler);

  // start the controller (unchanged)
  controller.setOffsets(servoOffsets);
  controller.begin(Pose(0, 0, 130, 0, 0, 0));
  controller.setGoalPose(Pose(0, 0, 110, 0, 0, 0));
}

void loop() {
  // keep controller running as before
  controller.update();

  // Let communication module handle HTTP/UDP and forward requests to our callback.
  wifiRun();

  // (Optionally) other robot logic can go here
}
