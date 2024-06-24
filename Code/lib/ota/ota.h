#ifndef _oth_h
#define _oth_h
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include <ESPmDNS.h> // Include the mDNS library

#define ENABLE_PIN 4 // Pin where the enable signal is connected

AsyncWebServer server(80);

const char* ssid = "MANIP-OTA";
const char* password = "12345678";

bool wifiStarted = false;
bool previousPinState = LOW;

const char* serverIndex = R"(
<!DOCTYPE html>
<html>
  <body>
    <form method='POST' action='/update' enctype='multipart/form-data'>
      <input type='file' name='update'>
      <input type='submit' value='Update'>
    </form>
  </body>
</html>
)";

void startOTA() {
  if (!MDNS.begin("esp32")) 
  {
    Serial.println("Error starting mDNS");
    return;
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", serverIndex);
  });

  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    response->addHeader("Connection", "close");
    request->send(response);
    delay(100);
    ESP.restart();
  }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
    if (!index) 
    {
        Serial.printf("Update Start: %s\n", filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN))  //start with max available size
            Update.printError(Serial);
      
    }
    if (Update.write(data, len) != len) 
        Update.printError(Serial);
    if (final) 
    {
        if (Update.end(true))  //true to set the size to the current progress
            Serial.printf("Update Success: %u\nRebooting...\n", index + len);
        else 
            Update.printError(Serial);
      
    }
  });

  server.begin();
}

void startWiFi() {
  if (!wifiStarted) {
    Serial.println("Starting WiFi...");
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);

    // If IP is acquired, start the web server and OTA
    if (IP) {
      startOTA();
      wifiStarted = true;
    }
  }
}

void stopWiFi() {
  if (wifiStarted) {
    Serial.println("Stopping WiFi...");
    server.end();
    WiFi.softAPdisconnect(true);
    MDNS.end();
    wifiStarted = false;
    Serial.println("WiFi stopped");
  }
}

void setup_ota() {
  pinMode(ENABLE_PIN, INPUT);
  Serial.begin(115200);
  Serial.println("Ready");
}

void loop_ota() {
  bool currentPinState = digitalRead(ENABLE_PIN);

  // Check if the enable pin is now HIGH and was LOW before (rising edge)
  if (currentPinState == HIGH && previousPinState == LOW) 
    startWiFi();
  
  else if (currentPinState == LOW && previousPinState == HIGH) 
    stopWiFi();
  
  // Update the previous state
  previousPinState = currentPinState;
}

#endif