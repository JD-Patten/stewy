#include "inverse_kinematics.h"
#include "controller.h"
#include <ESP32Servo.h>
#include "communication.h"   

// change these to fix mechanical offsets in the arms
vector<float> servoOffsets = {-5, 10, 5, 0, 0, 10};  
float deadzone = 100;

#define LED_BUILTIN 48   

// Set these to your desired credentials.
const char *ssid = "stewyAP";
const char *password = "stewy123";

// Servo setup
Servo servos[6];
int servoPins[6] = {3,2,1,18,17,10};

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
    Pose poseToSet;
    if (poseToSet.fromString(params)) {
      controller.setGoalPose(poseToSet);
    } else {
      Serial.println("Invalid pose format in params");
    }
  } else if (path == "/setOffsets") {
    Serial.println("Handling servo offsets update");
    vector<float> newOffsets(6, 0.0f);
    int count = 0;
    int start = 0;
    String paramStr = params;
    bool valid = true;
    for (int i = 0; i < 6; i++) {
      int comma = paramStr.indexOf(',', start);
      if (comma == -1) {
        comma = paramStr.length();
        if (i < 5) valid = false;  // Need 6 values
      }
      String valStr = paramStr.substring(start, comma);
      valStr.trim();  // Remove whitespace
      newOffsets[i] = valStr.toFloat();
      start = (comma < paramStr.length() && i < 5) ? comma + 1 : -1;
      count++;
    }
    if (valid && count == 6) {
      servoOffsets = newOffsets;
      controller.setOffsets(servoOffsets);
      Serial.println("Offsets updated to:");
      for (int i = 0; i < 6; i++) {
        Serial.print("Servo "); Serial.print(i); Serial.print(": "); Serial.println(newOffsets[i]);
      }
    } else {
      Serial.println("Invalid offsets params: expected exactly 6 comma-separated floats");
    }
 } else if (path == "/setAngles") {
    Serial.println("Setting servo angles");
    vector<float> newAngles(6, 0.0f);
    int count = 0;
    int start = 0;
    String paramStr = params;
    bool valid = true;
    for (int i = 0; i < 6; i++) {
      int comma = paramStr.indexOf(',', start);
      if (comma == -1) {
        comma = paramStr.length();
        if (i < 5) valid = false;  // Need 6 values
      }
      String valStr = paramStr.substring(start, comma);
      valStr.trim();  // Remove whitespace
      newAngles[i] = valStr.toFloat();
      start = (comma < paramStr.length() && i < 5) ? comma + 1 : -1;
      count++;
    }
    if (valid && count == 6) {
      controller.setAngles(newAngles);
      Serial.println("Angles updated to:");
      for (int i = 0; i < 6; i++) {
        Serial.print("Servo "); Serial.print(i); Serial.print(": "); Serial.println(newAngles[i]);
      }
    } else {
      Serial.println("Invalid angles params: expected exactly 6 comma-separated floats");
    }

  } else if (path == "/walk") {
    // Example params: "direction,speedMultiplier"
    int commaIndex = params.indexOf(',');
    if (commaIndex != -1) {
      String speedStr = params.substring(0, commaIndex);
      String direction = params.substring(commaIndex + 1);
      float speedMultiplier = speedStr.toFloat();
      controller.startWalking(direction, speedMultiplier);
      Serial.println("Started walking " + direction + " at speed multiplier " + String(speedMultiplier));
    } else {
      Serial.println("Invalid startWalk params");
    }

  }else if (path == "/setMaxAccel") {
    // Example params: "transAccel,rotAccel"
    int commaIndex = params.indexOf(',');
    if (commaIndex != -1) {
      String transStr = params.substring(0, commaIndex);
      String rotStr = params.substring(commaIndex + 1);
      float transAccel = transStr.toFloat();
      float rotAccel = rotStr.toFloat();
      controller.setAccelerationLimits(transAccel, rotAccel);
      Serial.println("Set max accelerations - Translational: " + String(transAccel) + ", Rotational: " + String(rotAccel));
    } else {
      Serial.println("Invalid setMaxAccel params");
    }
  }
  // Blink built-in LED to indicate request processed 
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


// Find home position
Pose homePose;

vector<float> targetAngles = {-23, 23, -23, 23, -23, 23};
float zMin = 0, zMax = 300;

while(zMax - zMin > 0.1) {
    float zMid = (zMin + zMax) / 2.0;
    auto result = ikSolver.solveInverseKinematics(Pose(0, 0, zMid, 0, 0, 0));
    
    if(!result.success) {
        zMax = zMid;
        continue;
    }
    
    float maxError = 0;
    for(int i = 0; i < 6; i++) {
        maxError = max(maxError, abs(result.angles[i] - targetAngles[i]));
    }
    
    if(maxError < 0.1) {
        Serial.println("Found homePose: (0,0," + String(zMid) + ",0,0,0) with angles: " + String(result.angles[0]) + ", " + String(result.angles[1]) + ", " + String(result.angles[2]) + ", " + String(result.angles[3]) + ", " + String(result.angles[4]) + ", " + String(result.angles[5]));
        homePose = Pose(0, 0, zMid, 0, 0, 0);
        break;
    }
    
    (result.angles[0] < targetAngles[0]) ? zMin = zMid : zMax = zMid;
}

// start the controller
controller.setOffsets(servoOffsets);
controller.setDeadzone(deadzone);
controller.begin(homePose);
}

void loop() {
  // keep controller running as before
  controller.update();

  // Let communication module handle HTTP/UDP and forward requests to our callback.
  wifiRun();

  // (Optionally) other robot logic can go here
}
