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


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
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
}

void loop() {

  controller.update();

  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    //Serial.println("New Client.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        //Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
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
            client.print("button { padding: 20px; margin: 10px; background:rgb(20, 142, 91); color: white; border: none; border-radius: 10px; font-size: 24px; cursor: pointer; }");
            client.print("button:hover { background:rgb(11, 107, 78); }");
            client.print("</style>");
            client.print("<script>");
            client.print("function sendRequest(command) {");
            client.print("  fetch(command);");  // Send GET request without reloading the page
            client.print("}");
            client.print("</script>");
            client.print("</head>");
            client.print("<body>");
            client.print("<button onclick=\"sendRequest('/pose1')\">Set Pose 1</button>");
            client.print("<button onclick=\"sendRequest('/pose2')\">Set Pose 2</button>");
            client.print("</body>");
            client.print("</html>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /pose1" or "GET /pose2":
        if (currentLine.endsWith("GET /pose1")) {
          controller.setGoalPose(Pose(0, 0, 90, 0, 0, 0));  // Set goal pose to 0, 0, 90, 0, 0, 0
        }
        if (currentLine.endsWith("GET /pose2")) {
          controller.setGoalPose(Pose(0, 0, 130, 0, 0, 0)); // Set goal pose to 0, 0, 130, 0, 0, 0
        }
      }
    }
    // close the connection:
    client.stop();
    //Serial.println("Client Disconnected.");
  }
}

   
