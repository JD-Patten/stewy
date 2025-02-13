#include "inverse_kinematics.h"
#include "controller.h"
#include <ESP32Servo.h>
#include <ESPmDNS.h>  // Include the mDNS library

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>

#define LED_BUILTIN 13   // Set the GPIO pin where you connected your test LED or comment this line out if your dev board has a built-in LED

// Set these to your desired credentials.
const char *ssid = "stewyAP";
const char *password = "stewy123";

WiFiServer server(80);


// Servo setup
Servo servos[6];
int servoPins[6] = {18, 2, 10, 1, 3, 17};

// Create IK solver instance
IKSolver ikSolver(
    50,     // servoArmLength
    100,    // arm2Length
    20,     // servoOffset1
    72,     // servoOffset2
    0,      // servoZOffset
    15,     // draftAngle
    20,     // topPlateOffset1
    35,     // topPlateOffset2
    -10     // topPlateZOffset
);

Controller controller(servoPins, ikSolver, 0, 0, 0, 10, 10 );  

// Variable to track the last button pressed
String lastButtonPressed = "";

bool isProcessingRequest = false;  // Added to handle multiple requests

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting up...");
  
  Serial.println();
  Serial.println("Configuring access point...");

  // You can remove the password parameter if you want the AP to be open.
  // a valid password must have more than 7 characters
  if (!WiFi.softAP(ssid, password)) {
    log_e("Soft AP creation failed.");
    while(1);
  }
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.begin();

  Serial.println("Server started");

  if (!MDNS.begin("robot")) {  // Start the mDNS responder for "robot.local"
    Serial.println("Error setting up MDNS responder!");
    while(1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");


  // start the controller
  controller.begin(Pose(0, 0, 100, 0, 0, 0));
  controller.setGoalPose(Pose(0, 0, 130, 0, 0, 0));
}

void loop() {
  controller.update();

  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {
    String currentLine = "";                // make a String to hold incoming data from the client
    String completeRequest = "";            // String to hold the complete request
    bool requestComplete = false;           // Flag to indicate if we have a complete request
    
    while (client.connected() && !requestComplete) {  // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        if (c == '\n') {                    // if the byte is a newline character
          if (currentLine.length() == 0) {
            requestComplete = true;          // Mark request as complete
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("<!DOCTYPE html>");
            client.print("<html lang=\"en\">");
            client.print("<head>");
            client.print("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">");
            client.print("<style>");
            client.print("body { display: flex; flex-direction: column; justify-content: center; align-items: center; height: 100vh; margin: 0; font-size: 24px; text-align: center; }");
            client.print("button { padding: 20px; margin: 10px; background:rgb(0, 207, 135); color: white; border: none; border-radius: 10px; font-size: 24px; cursor: pointer; }");
            client.print("button:hover { background:rgb(0, 90, 63); }");
            client.print("input { width: 50px; margin: 5px; }");
            client.print("</style>");
            client.print("<script>");
            client.print("function sendRequest(command, inputs) {");
            client.print("  const params = Array.from(inputs).map(input => input.value).join(',');");
            client.print("  fetch(`${command}?params=${params}`);");  // Send GET request with parameters
            client.print("}");
            client.print("</script>");
            client.print("</head>");
            client.print("<body>");
            client.print("<div>");
            client.print("<input type='number' id='pose1_x' placeholder='X' value='0.0'>");
            client.print("<input type='number' id='pose1_y' placeholder='Y' value='0.0'>");
            client.print("<input type='number' id='pose1_z' placeholder='Z' value='0.0'>");
            client.print("<input type='number' id='pose1_roll' placeholder='Roll' value='0.0'>");
            client.print("<input type='number' id='pose1_pitch' placeholder='Pitch' value='0.0'>");
            client.print("<input type='number' id='pose1_yaw' placeholder='Yaw' value='0.0'>");
            client.print("<button onclick=\"sendRequest('/pose1', document.querySelectorAll('#pose1_x, #pose1_y, #pose1_z, #pose1_roll, #pose1_pitch, #pose1_yaw'))\">Set Pose 1</button>");
            client.print("</div>");
            client.print("<div>");
            client.print("<input type='number' id='pose2_x' placeholder='X' value='0.0'>");
            client.print("<input type='number' id='pose2_y' placeholder='Y' value='0.0'>");
            client.print("<input type='number' id='pose2_z' placeholder='Z' value='0.0'>");
            client.print("<input type='number' id='pose2_roll' placeholder='Roll' value='0.0'>");
            client.print("<input type='number' id='pose2_pitch' placeholder='Pitch' value='0.0'>");
            client.print("<input type='number' id='pose2_yaw' placeholder='Yaw' value='0.0'>");
            client.print("<button onclick=\"sendRequest('/pose2', document.querySelectorAll('#pose2_x, #pose2_y, #pose2_z, #pose2_roll, #pose2_pitch, #pose2_yaw'))\">Set Pose 2</button>");
            client.print("</div>");
            client.print("<div>");
            client.print("<input type='number' id='max_accel_trans' placeholder='Max Trans Accel' value='10000'>");
            client.print("<input type='number' id='max_accel_ang' placeholder='Max Angular Accel' value='10000'>");
            client.print("<button onclick=\"sendRequest('/setMaxAccel', document.querySelectorAll('#max_accel_trans, #max_accel_ang'))\">Set Max Acceleration</button>");
            client.print("</div>");
            client.print("<div>");
            client.print("<input type='number' id='offset_x' placeholder='X Offset' value='0.0'>");
            client.print("<input type='number' id='offset_y' placeholder='Y Offset' value='0.0'>");
            client.print("<input type='number' id='offset_z' placeholder='Z Offset' value='0.0'>");
            client.print("<input type='number' id='offset_roll' placeholder='Roll Offset' value='0.0'>");
            client.print("<input type='number' id='offset_pitch' placeholder='Pitch Offset' value='0.0'>");
            client.print("<input type='number' id='offset_yaw' placeholder='Yaw Offset' value='0.0'>");
            client.print("<button onclick=\"sendRequest('/setOffsets', document.querySelectorAll('#offset_x, #offset_y, #offset_z, #offset_roll, #offset_pitch, #offset_yaw'))\">Set Offsets</button>");
            client.print("</div>");
            client.print("</body>");
            client.print("</html>");

            client.println();
            break;
          } else {    
            if (currentLine.indexOf("HTTP/1.1") > 0 || currentLine.indexOf("HTTP/1.0") > 0) {
              completeRequest = currentLine;  // Save the complete request line
            }
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }

    // Only process if we have a complete request
    if (!isProcessingRequest && requestComplete && completeRequest.length() > 0) {
      if (completeRequest.startsWith("GET /pose1?params=")) {
        // Only process if this wasn't the last pose button pressed
        if (lastButtonPressed != "pose1") {
          isProcessingRequest = true;
          String params = completeRequest.substring(completeRequest.indexOf('=') + 1);
          if (params.indexOf("HTTP") > 0) {
            params = params.substring(0, params.indexOf(" HTTP"));
          }
          if (params.indexOf(',') != -1) {  // Ensure we have at least one comma (multiple values)
            int values[6] = {0};
            int index = 0;
            while (params.length() > 0 && index < 6) {
              int commaIndex = params.indexOf(',');
              if (commaIndex == -1) {
                values[index++] = params.toInt();
                break;
              } else {
                values[index++] = params.substring(0, commaIndex).toInt();
                params = params.substring(commaIndex + 1);
              }
            }
            controller.setGoalPose(Pose(values[0], values[1], values[2], values[3], values[4], values[5]));
            lastButtonPressed = "pose1";  // Update last button pressed
          }
          isProcessingRequest = false;
        }
      }
      if (completeRequest.startsWith("GET /pose2?params=")) {
        // Only process if this wasn't the last pose button pressed
        if (lastButtonPressed != "pose2") {
          isProcessingRequest = true;
          String params = completeRequest.substring(completeRequest.indexOf('=') + 1);
          if (params.indexOf("HTTP") > 0) {
            params = params.substring(0, params.indexOf(" HTTP"));
          }
          if (params.indexOf(',') != -1) {
            int values[6] = {0};
            int index = 0;
            while (params.length() > 0 && index < 6) {
              int commaIndex = params.indexOf(',');
              if (commaIndex == -1) {
                values[index++] = params.toInt();
                break;
              } else {
                values[index++] = params.substring(0, commaIndex).toInt();
                params = params.substring(commaIndex + 1);
              }
            }
            controller.setGoalPose(Pose(values[0], values[1], values[2], values[3], values[4], values[5]));
            lastButtonPressed = "pose2";  // Update last button pressed
          }
          isProcessingRequest = false;
        }
      }
      if (completeRequest.startsWith("GET /setMaxAccel?params=")) {
        isProcessingRequest = true;
        String params = completeRequest.substring(completeRequest.indexOf('=') + 1);
        if (params.indexOf("HTTP") > 0) {
          params = params.substring(0, params.indexOf(" HTTP"));
        }
        if (params.indexOf(',') != -1) {
          float maxTransAccel = params.substring(0, params.indexOf(',')).toFloat();
          float maxAngularAccel = params.substring(params.indexOf(',') + 1).toFloat();
          controller.setAccelerationLimits(maxTransAccel, maxAngularAccel);
        }
        isProcessingRequest = false;
      }
      if (completeRequest.startsWith("GET /setOffsets?params=")) {
        isProcessingRequest = true;
        String params = completeRequest.substring(completeRequest.indexOf('=') + 1);
        if (params.indexOf("HTTP") > 0) {
          params = params.substring(0, params.indexOf(" HTTP"));
        }
        if (params.indexOf(',') != -1) {
          vector<float> offsets;  // Create a vector to hold the offsets
          while (params.length() > 0) {
            int commaIndex = params.indexOf(',');
            if (commaIndex == -1) {
              offsets.push_back(params.toFloat());  // Add the last value
              break;
            } else {
              offsets.push_back(params.substring(0, commaIndex).toFloat());  // Add each value
              params = params.substring(commaIndex + 1);
            }
          }
          controller.setOffsets(offsets);  // Pass the vector to setOffsets
        }
        isProcessingRequest = false;
      }
    }
    client.stop();
  }
}

   
