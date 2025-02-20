#include "inverse_kinematics.h"
#include "controller.h"
#include <ESP32Servo.h>
#include <ESPmDNS.h>  // Include the mDNS library

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>

#define LED_BUILTIN 48   

// Set these to your desired credentials.
const char *ssid = "stewyAP";
const char *password = "stewy123";
WiFiServer server(80);

// Servo setup
Servo servos[6];
int servoPins[6] = {2,3,18,17,10, 1};
vector<float> servoOffsets = {4, 0, 0, 5, 0, -3};

// Create IK solver instance
IKSolver ikSolver(
    50,     // servoArmLength
    100,    // arm2Length
    15.5,     // servoOffset1
    70,     // servoOffset2
    0,      // servoZOffset
    15,     // draftAngle
    20,     // topPlateOffset1
    35,     // topPlateOffset2
    -10     // topPlateZOffset
);

float maxAcceleration = 10;
float maxRotationalAcceleration = 10;
Controller controller(servoPins, ikSolver, maxAcceleration, maxRotationalAcceleration );  

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
  controller.setOffsets(servoOffsets);
  controller.begin(Pose(0, 0, 100, 0, 0, 0));
  controller.setGoalPose(Pose(0, 0, 80, 0, 0, 0));
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
client.print("<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">");
client.print("<!-- CSS and JS will be injected here by the combine script -->\n<link rel=\"stylesheet\" href=\"styles.css\">");
client.print("<script src=\"script.js\"></script>\n<style>\nbody {\ndisplay: flex;\nflex-direction: row;\njustify-content: center;");
client.print("align-items: center;\nheight: 100vh;\nmargin: 0;\nfont-size: 30px;\ntext-align: center;\ngap: 100px;\nbackground-color: #0099a109;");
client.print("}\nlabel {\nfont-size: 30px;\ncolor: #000;\nfont-family: sans-serif;\ntext-align: right;\npadding-right: 0px;");
client.print("padding-left: 0px;\nline-height: 45px; /* This helps align labels vertically with inputs */\n}\nbutton {");
client.print("width: 100%;\npadding: 20px;\nmargin: 10px;\nbackground: #0099a1;\ncolor: rgb(255, 255, 255);\nborder: none;");
client.print("border-radius: 10px;\nfont-size: 50px;\nfont-family: sans-serif;\ncursor: pointer;\ntransition: all 0.3s ease;");
client.print("}\nbutton:hover {\nbackground: #0099a17f;\ntransform: scale(1.02);\n}\ninput {\nwidth: 120px;\nmargin: 5px;");
client.print("font-size: 30px;\nfont-family: sans-serif;\nbackground-color: #0099a121;\ncolor: #000;\n}\nhtml, body {");
client.print("margin: 0;\npadding: 0;\nwidth: 100%;\nheight: 100%;\noverflow: hidden;\n}\n.container {\nwidth: 100%;");
client.print("height: 100%;\ndisplay: flex;\nflex-direction: column;\n}\n/* Custom Hexagon Button */\n.button.custom-hexagon {");
client.print("width: 300px;\naspect-ratio: 1;\nclip-path: polygon( 50%     50%,\n87.5%   71.65%,\n75%     93.3%,\n25%     93.3%,");
client.print("12.5%   71.65%);\ncolor: white;\nposition: absolute;\ntransform-origin: 50% 49%;\ndisplay: flex;\nalign-items: center;");
client.print("justify-content: center;\nborder-radius: 15px;\n}\n/* Update the active state to combine with rotations */");
client.print(".button.custom-hexagon.active {\ntransform: scale(0.95) rotate(60deg) !important;  /* For hexagon 1 */");
client.print("}\n/* Container for the rotating hexagons */\n.walking-buttons {\nposition: relative;\nwidth: 300px;\nheight: 300px;");
client.print("display: flex;\njustify-content: center;  /* Center horizontally */\nalign-items: center;      /* Center vertically */");
client.print("}\n.walking-container {\ndisplay: flex;\nflex-direction: column;\nalign-items: center;\njustify-content: center;");
client.print("}\n/* Position the hexagons absolutely within container */\n.button.custom-hexagon::after {\ncontent: '\\2193';  /* Unicode Down Arrow U+2193 */");
client.print("color: white;\nfont-size: 80px;\nposition: absolute;\ntop: 170px;\ntransform: rotate(0deg);\n}\n/* Update the class names in the rotation styles */");
client.print(".walk-left {\ntransform: rotate(60deg);\n}\n.walk-forward {\ntransform: rotate(180deg);\n}\n.walk-right {");
client.print("transform: rotate(300deg);\n}\n/* Update hover states with new class names */\n.walk-left:hover {\ntransform: scale(1.03) rotate(60deg);");
client.print("}\n.walk-forward:hover {\ntransform: scale(1.03) rotate(179.99deg);\n}\n.walk-right:hover {\ntransform: scale(1.03) rotate(300deg);");
client.print("}\n/* Update active states with new class names */\n.walk-left.active {\ntransform: rotate(60deg);\n}");
client.print(".walk-forward.active {\ntransform: rotate(180deg);\n}\n.walk-right.active {\ntransform: rotate(300deg);");
client.print("}\n/* Remove old class references */\n/* Delete or comment out:\n.custom-hexagon-1 { ... }\n.custom-hexagon-2 { ... }");
client.print(".custom-hexagon-3 { ... }\n.custom-hexagon-1:hover { ... }\n.custom-hexagon-2:hover { ... }\n.custom-hexagon-3:hover { ... }");
client.print(".custom-hexagon-1.active { ... }\n.custom-hexagon-2.active { ... }\n.custom-hexagon-3.active { ... }\n*/");
client.print(".pose-inputs {\ndisplay: flex;\ngap: 40px;\njustify-content: center;\nmargin-top: 20px;\n}\n.pose-grid {");
client.print("display: grid;\ngrid-template-columns: auto auto auto auto;\ngap: 20px;\njustify-content: center;\nmargin-top: 20px;");
client.print("column-gap: 20px;\n}\n.pose-grid label {\ntext-align: right;\npadding-right: 0px;\npadding-left: 20px;");
client.print("}\n.offsets-grid {\ndisplay: grid;\ngrid-template-columns: auto 100px;\ngap: 20px;\njustify-content: center;");
client.print("margin-top: 20px;\n}\n/* Add these new styles at the end of the file */\n.toggle-button {\nwidth: 90px;");
client.print("height: 90px;\nborder-radius: 50%;\nbackground: #0099a1;\ncolor: white;\nfont-size: 45px;\npadding: 0;");
client.print("margin: 10px;\ndisplay: flex;\nalign-items: center;\njustify-content: center;\nposition: absolute;\nright: 20px;");
client.print("top: 20px;\ntext-shadow: 0 0 20px rgb(255, 255, 255);  /* Added for better visibility */\n}\n.hidden {");
client.print("display: none;\n}\n.accel-grid {\ndisplay: grid;\ngrid-template-columns: auto 100px;\ngap: 20px;\njustify-content: center;");
client.print("margin-top: 20px;\n}\n</style>\n<script>");
client.print("function sendRequest(command, inputs) { const params=inputs.map(input => typeof input==='string' ? input : input.value).join(','); fetch(`\${command}?params=\${params}`); } function toggleOffsets() { const settingsContainers=document.querySelectorAll('.offsets-container, .accel-container'); const mainContainers=document.querySelectorAll('.walking-container, .pose-container'); const settingsHidden=document.querySelector('.offsets-container').classList.contains('hidden'); const toggleButton=document.querySelector('.toggle-button'); toggleButton.textContent=settingsHidden ? '\\u{1F3AE}' : '\\u{2699}'; settingsContainers.forEach(container => { container.classList.toggle('hidden',!settingsHidden); }); mainContainers.forEach(container => { container.classList.toggle('hidden', settingsHidden); }); }");
client.print("</script>\n</head>\n<body>\n<div class=\"walking-container\">\n<div class=\"walking-buttons\">");
client.print("<button class=\"button custom-hexagon walk-left\" onclick=\"sendRequest('/walk', [document.getElementById('speed_multiplier'), 'left'])\"></button>");
client.print("<button class=\"button custom-hexagon walk-forward\" onclick=\"sendRequest('/walk', [document.getElementById('speed_multiplier'), 'forward'])\"></button>");
client.print("<button class=\"button custom-hexagon walk-right\" onclick=\"sendRequest('/walk', [document.getElementById('speed_multiplier'), 'right'])\"></button>");
client.print("</div>\n<div>\n<label>Speed Multiplier</label>\n</div>\n<div>\n<input type='number' id='speed_multiplier' placeholder='speed multiplier' value='1.0'>");
client.print("</div>\n</div>\n<div class=\"pose-container\">\n<div>\n<button onclick=\"sendRequest('/pose1', [\ndocument.getElementById('pose_x'),");
client.print("document.getElementById('pose_y'),\ndocument.getElementById('pose_z'),\ndocument.getElementById('pose_roll'),");
client.print("document.getElementById('pose_pitch'),\ndocument.getElementById('pose_yaw')\n])\">Set Pose</button>\n</div>");
client.print("<div class=\"pose-grid\">\n<label>X</label>\n<input type='number' id='pose_x' placeholder='X' value='0.0'>");
client.print("<label>Roll</label>\n<input type='number' id='pose_roll' placeholder='Roll' value='0.0'>\n<label>Y</label>");
client.print("<input type='number' id='pose_y' placeholder='Y' value='0.0'>\n<label>Pitch</label>\n<input type='number' id='pose_pitch' placeholder='Pitch' value='0.0'>");
client.print("<label>Z</label>\n<input type='number' id='pose_z' placeholder='Z' value='0.0'>\n<label>Yaw</label>\n<input type='number' id='pose_yaw' placeholder='Yaw' value='0.0'>");
client.print("</div>\n</div>\n<button class=\"toggle-button\" onclick=\"toggleOffsets()\">&#x2699;</button>\n<div class=\"offsets-container hidden\">");
client.print("<div>\n<button onclick=\"sendRequest('/setOffsets', [\ndocument.getElementById('offset_1').value,\ndocument.getElementById('offset_2').value,");
client.print("document.getElementById('offset_3').value,\ndocument.getElementById('offset_4').value,\ndocument.getElementById('offset_5').value,");
client.print("document.getElementById('offset_6').value\n])\">Set Offsets</button>\n</div>\n<div class=\"offsets-grid\">");
client.print("<label>Servo 1</label>\n<input type='number' id='offset_1' placeholder='offset1' value='0.0'>\n<label>Servo 2</label>");
client.print("<input type='number' id='offset_2' placeholder='offset2' value='0.0'>\n<label>Servo 3</label>\n<input type='number' id='offset_3' placeholder='offset3' value='0.0'>");
client.print("<label>Servo 4</label>\n<input type='number' id='offset_4' placeholder='offset4' value='0.0'>\n<label>Servo 5</label>");
client.print("<input type='number' id='offset_5' placeholder='offset5' value='0.0'>\n<label>Servo 6</label>\n<input type='number' id='offset_6' placeholder='offset6' value='0.0'>");
client.print("</div>\n</div>\n<div class=\"accel-container hidden\">\n<div>\n<button onclick=\"sendRequest('/setMaxAccel', [");
client.print("document.getElementById('max_trans_accel'),\ndocument.getElementById('max_angular_accel')\n])\">Set Accelerations</button>");
client.print("</div>\n<div class=\"accel-grid\">\n<label>Translational</label>\n<input type='number' id='max_trans_accel' placeholder='translational' value='50.0'>");
client.print("<label>Rotational</label>\n<input type='number' id='max_angular_accel' placeholder='rotational' value='50.0'>");
client.print("</div>\n</div>\n</body>\n</html>");

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

      if (completeRequest.startsWith("GET /walk?params=")) {
        isProcessingRequest = true;
        String params = completeRequest.substring(completeRequest.indexOf('=') + 1);
        if (params.indexOf("HTTP") > 0) {
          params = params.substring(0, params.indexOf(" HTTP"));
        }
        
        // Split the params into speed and direction
        float speed = params.substring(0, params.indexOf(',')).toFloat();
        String direction = params.substring(params.indexOf(',') + 1);
        
        Serial.print("Walk command received - Speed: ");
        Serial.print(speed);
        Serial.print(", Direction: ");
        Serial.println(direction);

        controller.startWalking(direction);
        
        isProcessingRequest = false;
      }

      if (completeRequest.startsWith("GET /pose1?params=")) {

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
        }
        isProcessingRequest = false;
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

        digitalWrite(LED_BUILTIN, HIGH);
        delay(10);
        digitalWrite(LED_BUILTIN, LOW);
    }
    client.stop();
  }
}

   
