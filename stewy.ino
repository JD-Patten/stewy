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

Controller controller(servoPins, ikSolver, 0, 0, 0, 10, 10 );  

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
            client.print("<!-- CSS and JS will be injected here by the combine script -->");
            client.print("<style>");
            client.print("body {");
            client.print("display: flex;");
            client.print("flex-direction: row;");
            client.print("justify-content: center;");
            client.print("align-items: center;");
            client.print("height: 100vh;");
            client.print("margin: 0;");
            client.print("font-size: 30px;");
            client.print("text-align: center;");
            client.print("gap: 100px;");
            client.print("background-color: #0099a109;");
            client.print("}");
            client.print("label {");
            client.print("font-size: 30px;");
            client.print("color: #000;");
            client.print("font-family: sans-serif;");
            client.print("text-align: right;");
            client.print("padding-right: 0px;");
            client.print("padding-left: 0px;");
            client.print("line-height: 45px; /* This helps align labels vertically with inputs */");
            client.print("}");
            client.print("button {");
            client.print("width: 100%;");
            client.print("padding: 20px;");
            client.print("margin: 10px;");
            client.print("background: #0099a1;");
            client.print("color: rgb(255, 255, 255);");
            client.print("border: none;");
            client.print("border-radius: 10px;");
            client.print("font-size: 50px;");
            client.print("font-family: sans-serif;");
            client.print("cursor: pointer;");
            client.print("transition: all 0.3s ease;");
            client.print("}");
            client.print("button:hover {");
            client.print("background: #0099a17f;");
            client.print("transform: scale(1.02);");
            client.print("}");
            client.print("input {");
            client.print("width: 120px;");
            client.print("margin: 5px;");
            client.print("font-size: 30px;");
            client.print("font-family: sans-serif;");
            client.print("background-color: #0099a121;");
            client.print("color: #000;");
            client.print("}");
            client.print("html, body {");
            client.print("margin: 0;");
            client.print("padding: 0;");
            client.print("width: 100%;");
            client.print("height: 100%;");
            client.print("overflow: hidden;");
            client.print("}");
            client.print(".container {");
            client.print("width: 100%;");
            client.print("height: 100%;");
            client.print("display: flex;");
            client.print("flex-direction: column;");
            client.print("}");
            client.print("/* Custom Hexagon Button */");
            client.print(".button.custom-hexagon {");
            client.print("width: 300px;");
            client.print("aspect-ratio: 1;");
            client.print("clip-path: polygon( 50%     50%,");
            client.print("87.5%   71.65%,");
            client.print("75%     93.3%,");
            client.print("25%     93.3%,");
            client.print("12.5%   71.65%);");
            client.print("color: white;");
            client.print("position: absolute;");
            client.print("transform-origin: 50% 49%;");
            client.print("display: flex;");
            client.print("align-items: center;");
            client.print("justify-content: center;");
            client.print("border-radius: 15px;");
            client.print("}");
            client.print("/* Update the active state to combine with rotations */");
            client.print(".button.custom-hexagon.active {");
            client.print("transform: scale(0.95) rotate(60deg) !important;  /* For hexagon 1 */");
            client.print("}");
            client.print("/* Container for the rotating hexagons */");
            client.print(".walking-buttons {");
            client.print("position: relative;");
            client.print("width: 300px;");
            client.print("height: 300px;");
            client.print("display: flex;");
            client.print("justify-content: center;  /* Center horizontally */");
            client.print("align-items: center;      /* Center vertically */");
            client.print("}");
            client.print(".walking-container {");
            client.print("display: flex;");
            client.print("flex-direction: column;");
            client.print("align-items: center;");
            client.print("justify-content: center;");
            client.print("}");
            client.print("/* Position the hexagons absolutely within container */");
            client.print(".button.custom-hexagon::after {");
            client.print("content:'\\2193';"); 
            client.print("color: white;");
            client.print("font-size: 80px;");
            client.print("position: absolute;");
            client.print("top: 170px;");
            client.print("transform: rotate(0deg);");
            client.print("}");
            client.print("/* Keep original rotations without hover changes */");
            client.print(".custom-hexagon-1 {");
            client.print("transform: rotate(60deg);");
            client.print("}");
            client.print(".custom-hexagon-2 {");
            client.print("transform: rotate(180deg);");
            client.print("}");
            client.print(".custom-hexagon-3 {");
            client.print("transform: rotate(300deg);");
            client.print("}");
            client.print("/* Add these new hover states that combine scale with rotation */");
            client.print(".custom-hexagon-1:hover {");
            client.print("transform: scale(1.03) rotate(60deg);");
            client.print("}");
            client.print(".custom-hexagon-2:hover {");
            client.print("transform: scale(1.03) rotate(179.99deg);");
            client.print("}");
            client.print(".custom-hexagon-3:hover {");
            client.print("transform: scale(1.03) rotate(300deg);");
            client.print("}");
            client.print("/* Active states with fixed rotations */");
            client.print(".custom-hexagon-1.active {");
            client.print("transform: rotate(60deg);");
            client.print("}");
            client.print(".custom-hexagon-2.active {");
            client.print("transform: rotate(180deg);");
            client.print("}");
            client.print(".custom-hexagon-3.active {");
            client.print("transform: rotate(300deg);");
            client.print("}");
            client.print("/* Counter-rotate arrows to keep them pointing down */");
            client.print(".custom-hexagon-1::after {");
            client.print("transform: rotate(0deg);");
            client.print("}");
            client.print(".custom-hexagon-3::after {");
            client.print("transform: rotate(-240deg);  /* Counter-rotate by negative angle */");
            client.print("}");
            client.print(".pose-inputs {");
            client.print("display: flex;");
            client.print("gap: 40px;");
            client.print("justify-content: center;");
            client.print("margin-top: 20px;");
            client.print("}");
            client.print(".pose-grid {");
            client.print("display: grid;");
            client.print("grid-template-columns: auto auto auto auto;");
            client.print("gap: 20px;");
            client.print("justify-content: center;");
            client.print("margin-top: 20px;");
            client.print("column-gap: 20px;");
            client.print("}");
            client.print(".pose-grid label {");
            client.print("text-align: right;");
            client.print("padding-right: 0px;");
            client.print("padding-left: 20px;");
            client.print("}");
            client.print(".offsets-grid {");
            client.print("display: grid;");
            client.print("grid-template-columns: auto 100px;");
            client.print("gap: 20px;");
            client.print("justify-content: center;");
            client.print("margin-top: 20px;");
            client.print("}");
            client.print("/* Add these new styles at the end of the file */");
            client.print(".toggle-button {");
            client.print("width: 90px;");
            client.print("height: 90px;");
            client.print("border-radius: 50%;");
            client.print("background: #0099a1;");
            client.print("color: white;");
            client.print("font-size: 45px;");
            client.print("padding: 0;");
            client.print("margin: 10px;");
            client.print("display: flex;");
            client.print("align-items: center;");
            client.print("justify-content: center;");
            client.print("position: absolute;");
            client.print("right: 20px;");
            client.print("top: 20px;");
            client.print("text-shadow: 0 0 20px rgb(255, 255, 255);  /* Added for better visibility */");
            client.print("}");
            client.print(".hidden {");
            client.print("display: none;");
            client.print("}");
            client.print(".accel-grid {");
            client.print("display: grid;");
            client.print("grid-template-columns: auto 100px;");
            client.print("gap: 20px;");
            client.print("justify-content: center;");
            client.print("margin-top: 20px;");
            client.print("}");
            client.print("</style>");
            client.print("<script>");
            client.print("function sendRequest(command,inputs){const params=Array.from(inputs).map(input=>input.value).join(',');fetch(`${command}?params=${params}`);}");

            client.print("function toggleOffsets(){");
            client.print("const settingsContainers=document.querySelectorAll('.offsets-container,.accel-container');");
            client.print("const mainContainers=document.querySelectorAll('.walking-container,.pose-container');");
            client.print("const settingsHidden=document.querySelector('.offsets-container').classList.contains('hidden');");
            client.print("const toggleButton=document.querySelector('.toggle-button');");
            client.print("toggleButton.textContent=settingsHidden?'\\u{1F3AE}':'\\u{2699}';");
            client.print("settingsContainers.forEach(container=>{container.classList.toggle('hidden',!settingsHidden);});");
            client.print("mainContainers.forEach(container=>{container.classList.toggle('hidden',settingsHidden);});");
            client.print("}");
            client.print("</script>");
            client.print("</head>");
            client.print("<body>");
            client.print("<div class=\"walking-container\">");
            client.print("<div class=\"walking-buttons\">");
            client.print("<button class=\"button custom-hexagon custom-hexagon-1\"></button>");
            client.print("<button class=\"button custom-hexagon custom-hexagon-2\"></button>");
            client.print("<button class=\"button custom-hexagon custom-hexagon-3\"></button>");
            client.print("</div>");
            client.print("<div>");
            client.print("<label>Speed Multiplier</label>");
            client.print("</div>");
            client.print("<div>");
            client.print("<input type='number' id='speed_multiplier' placeholder='speed multiplier' value='1.0'>");
            client.print("</div>");
            client.print("</div>");
            client.print("<div class=\"pose-container\">");
            client.print("<div>");
            client.print("<button onclick=\"sendRequest('/pose1', [");
            client.print("document.getElementById('pose_x'),");
            client.print("document.getElementById('pose_y'),");
            client.print("document.getElementById('pose_z'),");
            client.print("document.getElementById('pose_roll'),");
            client.print("document.getElementById('pose_pitch'),");
            client.print("document.getElementById('pose_yaw')");
            client.print("])\">Set Pose</button>");
            client.print("</div>");
            client.print("<div class=\"pose-grid\">");
            client.print("<label>X</label>");
            client.print("<input type='number' id='pose_x' placeholder='X' value='0.0'>");
            client.print("<label>Roll</label>");
            client.print("<input type='number' id='pose_roll' placeholder='Roll' value='0.0'>");
            client.print("<label>Y</label>");
            client.print("<input type='number' id='pose_y' placeholder='Y' value='0.0'>");
            client.print("<label>Pitch</label>");
            client.print("<input type='number' id='pose_pitch' placeholder='Pitch' value='0.0'>");
            client.print("<label>Z</label>");
            client.print("<input type='number' id='pose_z' placeholder='Z' value='0.0'>");
            client.print("<label>Yaw</label>");
            client.print("<input type='number' id='pose_yaw' placeholder='Yaw' value='0.0'>");
            client.print("</div>");
            client.print("</div>");
            client.print("<button class=\"toggle-button\" onclick=\"toggleOffsets()\">⚙️</button>");
            client.print("<div class=\"offsets-container hidden\">");
            client.print("<div>");
            client.print("<button onclick=\"sendRequest('/setOffsets', document.querySelectorAll('#offset_1, #offset_2, #offset_3, #offset_4, #offset_5, #offset_6'))\">Set Offsets</button>");
            client.print("</div>");
            client.print("<div class=\"offsets-grid\">");
            client.print("<label>Servo 1</label>");
            client.print("<input type='number' id='offset_1' placeholder='offset1' value='0.0'>");
            client.print("<label>Servo 2</label>");
            client.print("<input type='number' id='offset_2' placeholder='offset2' value='0.0'>");
            client.print("<label>Servo 3</label>");
            client.print("<input type='number' id='offset_3' placeholder='offset3' value='0.0'>");
            client.print("<label>Servo 4</label>");
            client.print("<input type='number' id='offset_4' placeholder='offset4' value='0.0'>");
            client.print("<label>Servo 5</label>");
            client.print("<input type='number' id='offset_5' placeholder='offset5' value='0.0'>");
            client.print("<label>Servo 6</label>");
            client.print("<input type='number' id='offset_6' placeholder='offset6' value='0.0'>");
            client.print("</div>");
            client.print("</div>");
            client.print("<div class=\"accel-container hidden\">");
            client.print("<div>");
            client.print("<button onclick=\"sendRequest('/setMaxAccel', [");
            client.print("document.getElementById('max_trans_accel'),");
            client.print("document.getElementById('max_angular_accel')");
            client.print("])\">Set Accelerations</button>");
            client.print("</div>");
            client.print("<div class=\"accel-grid\">");
            client.print("<label>Translational</label>");
            client.print("<input type='number' id='max_trans_accel' placeholder='translational' value='50.0'>");
            client.print("<label>Rotational</label>");
            client.print("<input type='number' id='max_angular_accel' placeholder='rotational' value='50.0'>");
            client.print("</div>");
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
    }
    client.stop();
  }
}

   
