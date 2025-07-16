//////////////////////////////////////////////////////////////////////////////////////
//LIBRARIES
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "AsyncJson.h"
#include "ArduinoJson.h"
//////////////////////////////////////////////////////////////////////////////////////

//VARIABLES AND PORTS
// Define the UART pins (ESP32 default pins)
#define RXD2 16 // RX pin for UART2
#define TXD2 17 // TX pin for UART2
#define BUTTON 18

// WiFi Credentials
const char* ssid = "IDMind";
const char* password = "df%645;A";
//Server Address
const char* serverAddress = "192.168.2.30"; 

// Define the AsyncWebServer port
AsyncWebServer server(80);

// Timer to track last received message time
unsigned long lastMessageTime = 0;  // Stores the time when the last message was received
unsigned long timeoutInterval = 500; // 500 milliseconds timeout

//////////////////////////////////////////////////////////////////////////////////////

void setup() {
//////////////////////////////////////////////////////////////////////////////////////
//INITIALIZATION
  // Initialize Serial for debugging
  Serial.begin(115200);
  // Initialize Serial2 for UART communication
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  pinMode(BUTTON, INPUT);
  
  // Set ESP32 WiFi to Station Mode
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.println("Connecting to WiFi...");

//////////////////////////////////////////////////////////////////////////////////////
//CONNECTION

  Serial.println("Connecting to WiFi...");
  // Waiting for the connection
  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED && attempt < 20) {
    Serial.printf("WiFi Connection Attempt %d Failed. Retrying...\n", ++attempt);
    delay(1000);
  }

  // Check connection result
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP()); // IP Address
  } else {
    Serial.println("WiFi connection failed. Please check your credentials and try again.");
    return; // Exit if connection failed
  }
//////////////////////////////////////////////////////////////////////////////////////
//TESTING

  //Test GET request
  server.on("/get_message/test", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Test GET successful");
  });
  //Test POST request
  server.on("/post_message/test", HTTP_POST, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Test POST successful");
  });

//////////////////////////////////////////////////////////////////////////////////////

/*// Handle POST request for sending sonar data
  server.on("/post_message", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("message", true)) {
      String message = request->getParam("message", true)->value();
      Serial.printf("Received message: %s\n", message.c_str());
      request->send(200, "text/plain", "Message sent successfully to ESP32");
      Serial2.println(message);

      // Reset the last message time
      lastMessageTime = millis();
    } 
    else {
      request->send(400, "text/plain", "Message parameter missing");
    }
  });
*/
 server.on("/button_state", HTTP_GET, [](AsyncWebServerRequest *request) {
    bool isPressed = (digitalRead(BUTTON) == LOW);
    String payload = "{\"pressed\":";
    payload += (isPressed ? "true" : "false");
    payload += "}";
    request->send(200, "application/json", payload);
  });


server.on("/post_message", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
[](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
  Serial.printf("Received %u bytes via POST\n", len);

  String msgHex = "Sent to Arduino: ";
  for (size_t i = 0; i < len; i++) {
    Serial2.write(data[i]);
    msgHex += "0x" + String(data[i], HEX) + " ";
  }
  Serial.println(msgHex);
  request->send(200, "text/plain", "Bytes sent to Arduino.");
  lastMessageTime = millis();
});





#define BUMPER_BYTE 0x66  // match our Arduino’s bumper-status request

server.on("/bumper_state", HTTP_GET, [](AsyncWebServerRequest *request) {
  // 0) clear out any leftover bytes
  while (Serial2.available()) Serial2.read();

  // 1) ask the Arduino for its two-byte bumper status
  Serial2.write(BUMPER_BYTE);

  // 2) wait up to 50 ms for both bytes to arrive
  unsigned long t0 = millis();
  while (Serial2.available() < 2 && millis() - t0 < 50) {
    delay(1);
  }

  // 3) read them (0 = released, 1 = pressed)
  int rightB  = Serial2.available() ? Serial2.read()  : 0;
  int leftB = Serial2.available() ? Serial2.read()  : 0;
  Serial.println(rightB);
  Serial.println(leftB);
  bool left  = (leftB  != 0);
  bool right = (rightB != 0);

  // 4) toss any extras just in case
  while (Serial2.available()) Serial2.read();

  // 5) build and send JSON
  String js = "{\"hit\":";
     js += (left || right) ? "true" : "false";
     js += ",\"which\":\"";
     if ( left &&  right) js += "BOTH";
     else if ( left)      js += "L";
     else if ( right)     js += "R";
     js += "\"}";
  request->send(200, "application/json", js);
});


//////////////////////////////////////////////////////////////////////////////////////

  // Start server
  server.begin();
}

void loop() {
  // Check if more than 500ms have passed since the last message
  if (millis() - lastMessageTime >= timeoutInterval) {
    // Send the message b'\x86\x00\x00\x00\x00' to the Arduino
    Serial.println("Timeout: Sending b'\\x86\\x00\\x00\\x00\\x00' to Arduino");
    Serial2.write(0x86); // Sending binary values
    Serial2.write(0x00);
    Serial2.write(0x00);
    Serial2.write(0x00);
    Serial2.write(0x00);

    // Update lastMessageTime to prevent repeated sends
    lastMessageTime = millis();
  }
}
