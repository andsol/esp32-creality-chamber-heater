#include <Arduino.h>
#include "esp_timer.h"
#include <SPI.h>
#include "I2CScanner.h"
#include <Wire.h>
#include <LiquidTWI2.h>
#include <DFRobot_DHT20.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>

I2CScanner scanner;

// PWM settings
#define HEATER_PIN 16  //GPIO pin for heater control
#define FAN_PIN 23     // GPIO pin for fan control
#define TACH_PIN 18    // GPIO pin for tachometer input
#define PWM_FREQ 25000 // 25 kHz frequency for Noctua fan
#define PWM_RESOLUTION 8 // 8-bit resolution
#define PWM_HEATER_LIMIT 200 // Do not reach 100%

// Web server
WebServer server(80);

// NVS (Non-Volatile Storage)
Preferences preferences;

websockets::WebsocketsClient webSocket; // Use the ArduinoWebsockets client

DFRobot_DHT20 dht(&Wire); // Initialize the DHT20 object

float targetTemperature = 25.0; // Default target temperature
const float fullLoadThreshold = 5.0; // Temperature threshold for full load
float currentTemperature = 0.0;
float currentHumidity = 0.0;
float chamberTemperature = 0.0;
int fanSpeed = 0;
volatile unsigned long tachCount = 0;
unsigned long lastTachTime = 0;
int fanRPM = 0;
bool isFanRunning = false; // Fan state
unsigned long fanStopTime = 0; // Time when the fan should stop

String moonrakerIp = "";
String moonrakerAuth = "";
bool isPrinting = false;

unsigned long lastConnectionAttempt = 0;
const unsigned long connectionInterval = 10000; // 10 seconds

bool waitingForTemperature = false;
bool useDHT20 = true; // Flag to switch between DHT20 and Moonraker
bool manualMode = false; // Flag to switch between DHT20 and Moonraker

#define TIMER_INTERVAL_SEC 20 // 20 seconds
String topText = "";
String bottomText = "";
LiquidTWI2 display(0x20);

// Function declarations
void handleRoot();
void handleSetTargetTemp();
void handleSetMoonrakerConfig();
void handleSetTemperatureSource();
void handleSetManualMode();
void updatePWM();
void updateDisplay();
void IRAM_ATTR handleTach();

void onMessageCallback(websockets::WebsocketsMessage message);
void onEventsCallback(websockets::WebsocketsEvent event, String data);


void subscribeMoonraker();
void attemptConnection();
void sendContinueCommand();


// Display update limiter
bool canUpdateDisplay = true;
esp_timer_handle_t timer;

void IRAM_ATTR resetDisplayTimerCallback(void* arg) {
  canUpdateDisplay = true;
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  if (Wire.begin()) {
    Serial.println("I2C Started");
  } else {
    Serial.println(F("I2C Not Started"));
  }

  // Initialize the DHT20 sensor
  while(dht.begin()){
    Serial.println("Initialize sensor failed");
    delay(1000);
  }

  Serial.print("Temperature: ");
  Serial.print(dht.getTemperature());
  Serial.println(" *C");

  scanner.Init();
  scanner.Scan();

  display.setMCPType(LTI_TYPE_MCP23008);
  display.begin(16, 2);
  // Print a message to the LCD.
  display.print("Boot...");
  display.setBacklight(HIGH);

  // Initialize WiFi using WiFiManager
  WiFiManager wifiManager;
  if (!wifiManager.autoConnect("ESP32_AP")) {
    Serial.println("Failed to connect and hit timeout");
    ESP.restart();
  }

  // Initialize NVS
  preferences.begin("moonraker", false);
  moonrakerIp = preferences.getString("ip", "");
  moonrakerAuth = preferences.getString("auth", "");
  useDHT20 = preferences.getBool("useDHT20", true);
  manualMode = preferences.getBool("manualMode", false);

  // Initialize the web server
  server.on("/", handleRoot);
  server.on("/setTargetTemp", handleSetTargetTemp);
  server.on("/setMoonrakerConfig", handleSetMoonrakerConfig);
  server.on("/setTemperatureSource", handleSetTemperatureSource);
  server.on("/setManualMode",   handleSetManualMode);

  server.begin();
  Serial.println("HTTP server started");

  // Initialize PWM for heater and fan
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION); // 25 kHz PWM, 8-bit resolution
  ledcAttachPin(HEATER_PIN, 0);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION); // 25 kHz PWM, 8-bit resolution
  ledcAttachPin(FAN_PIN, 1);

  // Initialize tachometer input
  pinMode(TACH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), handleTach, FALLING);

  // Attempt initial connection to WebSocket
  attemptConnection();

  // Create the timer
  const esp_timer_create_args_t timerArgs = {
    .callback = &resetDisplayTimerCallback,
    .name = "reset_timer"
  };
  esp_timer_create(&timerArgs, &timer);

  updatePWM();
}

void loop() {
  // Handle web server requests
  server.handleClient();

  // Handle WebSocket communication
  if (webSocket.available()) {
    webSocket.poll();
  }
  

  // Attempt to connect to WebSocket if not connected
  unsigned long currentTime = millis();
  if (!webSocket.available() && currentTime - lastConnectionAttempt >= connectionInterval) {
    attemptConnection();
    lastConnectionAttempt = currentTime;
  }

  // Read current temperature and humidity from DHT20
  if (useDHT20) {
    currentTemperature = dht.getTemperature();
    currentHumidity = dht.getHumidity();
  }

  // Check if waiting for temperature to reach target
  if (waitingForTemperature && currentTemperature >= targetTemperature) {
    sendContinueCommand();
    waitingForTemperature = false;
  }

  // Update PWM values based on temperature
  updatePWM();

  // Calculate fan RPM
  if (currentTime - lastTachTime >= 1000) {
    fanRPM = (tachCount * 60) / 2; // Tachometer gives 2 pulses per revolution
    tachCount = 0;
    lastTachTime = currentTime;
  }

  // Update the OLED display
  if (canUpdateDisplay) {
    updateDisplay();
    canUpdateDisplay = false;
    // Start the timer to reset canExecuteFunction after 20 seconds
    esp_timer_start_once(timer, TIMER_INTERVAL_SEC * 1000000);
  }

  // Add a small delay to avoid reading too frequently
  delay(2000);
}

void handleRoot() {
  String html = "<html><head><link rel=\"stylesheet\" href=\"https://cdn.simplecss.org/simple.min.css\"></head><body>";
  html += "<header><h1>Moonraker chamber Heater Control</h1>";
  html += "<p>Current Temperature: " + String(currentTemperature) + " &deg;C</p>";
  html += "<p>Current Humidity: " + String(currentHumidity) + " %</p>";
  html += "<p>Target Temperature: " + String(targetTemperature) + " &deg;C</p>";
  html += "<p>Chamber Temperature: " + String(chamberTemperature) + " &deg;C</p>";
  html += "</header>";
  html += "<main><form action=\"/setTargetTemp\" method=\"POST\">";
  html += "Set Target Temperature: <input type=\"text\" name=\"targetTemp\"><br>";
  html += "<input type=\"submit\" value=\"Set\">";
  html += "</form>";
  html += "<h2>Moonraker Configuration</h2>";
  html += "<form action=\"/setMoonrakerConfig\" method=\"POST\">";
  html += "Moonraker IP: <input type=\"text\" name=\"ip\" value=\"" + moonrakerIp + "\"><br>";
  html += "Auth Token: <input type=\"text\" name=\"auth\" value=\"" + moonrakerAuth + "\"><br>";
  html += "<input type=\"submit\" value=\"Save\">";
  html += "</form>";
  html += "<h2>Temperature Source</h2>";
  html += "<form action=\"/setTemperatureSource\" method=\"POST\">";
  html += "Use DHT20 Sensor: <input type=\"radio\" name=\"source\" value=\"DHT20\" " + String(useDHT20 ? "checked" : "") + "><br>";
  html += "Use Moonraker API: <input type=\"radio\" name=\"source\" value=\"Moonraker\" " + String(!useDHT20 ? "checked" : "") + "><br>";
  html += "<input type=\"submit\" value=\"Set\">";
  html += "</form>";
  html += "<h2>Manual mode</h2>";
  html += "<form action=\"/setManualMode\" method=\"POST\">";
  html += "Manual: <input type=\"radio\" name=\"mode\" value=\"Yes\" " + String(manualMode ? "checked" : "") + "><br>";
  html += "Use Moonraker API: <input type=\"radio\" name=\"mode\" value=\"No\" " + String(!manualMode ? "checked" : "") + "><br>";
  html += "<input type=\"submit\" value=\"Set\">";
  html += "</form></main>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleSetTargetTemp() {
  if (server.hasArg("targetTemp")) {
    targetTemperature = server.arg("targetTemp").toFloat();
    Serial.println("New target temperature set to: " + String(targetTemperature));
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleSetMoonrakerConfig() {
  if (server.hasArg("ip") && server.hasArg("auth")) {
    moonrakerIp = server.arg("ip");
    moonrakerAuth = server.arg("auth");
    preferences.putString("ip", moonrakerIp);
    preferences.putString("auth", moonrakerAuth);
    Serial.println("Moonraker IP set to: " + moonrakerIp);
    Serial.println("Moonraker Auth set to: " + moonrakerAuth);
    attemptConnection();
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleSetManualMode() {
  if (server.hasArg("mode")) {
    String mode = server.arg("mode");
    manualMode = (mode == "Yes");
    preferences.putBool("manualMode", manualMode);
    Serial.println("Manual mode: " + String(manualMode ? "Yes" : "No"));
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleSetTemperatureSource() {
  if (server.hasArg("source")) {
    String source = server.arg("source");
    useDHT20 = (source == "DHT20");
    preferences.putBool("useDHT20", useDHT20);
    Serial.println("Temperature source set to: " + String(useDHT20 ? "DHT20" : "Moonraker"));
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void updatePWM() {
  // Heating and fan control logic
  if (isPrinting || manualMode) {
    if (currentTemperature < targetTemperature) {
      fanStopTime = 0; // Reset fan stop time
      if (currentTemperature < targetTemperature - fullLoadThreshold) {
        // Temperature is much lower than target, full load
        ledcWrite(0, PWM_HEATER_LIMIT);
        ledcWrite(1, 255);
        isFanRunning = true;
      } else {
        // Temperature is closer to target, reduce heater power
        float pwmValue = 255 * (targetTemperature - currentTemperature) / fullLoadThreshold;
        pwmValue = constrain((int)pwmValue, 0, PWM_HEATER_LIMIT);
        ledcWrite(0, pwmValue);
        ledcWrite(1, 255);
        isFanRunning = true;
      }
    } else {
      // Target temperature reached, turn off heater and fan
      ledcWrite(0, 0);
      if (isFanRunning) {
        // Set the time when the fan should stop
        if (fanStopTime == 0) {
          fanStopTime = millis();
        }
        // Check if 30 seconds have passed
        if (millis() - fanStopTime >= 30000) {
          ledcWrite(1, 0);
          isFanRunning = false;
        } else {
          // Half speed
          ledcWrite(1, 128);
        }
      }
    }
  } else {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  }
}

void updateDisplay() {

  topText = "";
  bottomText = "";

  // Draw WiFi icon if connected
  if (WiFi.status() == WL_CONNECTED) {
    //display.drawBitmap(0, 0, wifi_icon, WIFI_ICON_WIDTH, WIFI_ICON_HEIGHT, SSD1306_WHITE);
  }

  topText += "Wifi:";
  topText += WiFi.status() == WL_CONNECTED ? "Yes" : "No";
  topText += " ";

  // Draw 3D printer icon if connected to Moonraker
  if (webSocket.available()) {
    //display.drawBitmap(20, 0, printer_icon, PRINTER_ICON_WIDTH, PRINTER_ICON_HEIGHT, SSD1306_WHITE);
  }

  topText += "MoonR:";
  topText += webSocket.available() ? "Yes" : "No";
  topText += " ";

  topText += "CurrT:";
  topText += currentTemperature;
  topText += " ";

  topText += "TargetT:";
  topText += targetTemperature;
  topText += " ";

  topText += "Humid:";
  topText += currentHumidity;
  topText += "% ";

  bottomText += "FanSp:";
  bottomText += fanSpeed;
  bottomText += " ";

  bottomText += "FanRpm:";
  bottomText += fanRPM;
  bottomText += " ";

  bottomText += "CamberT:";
  bottomText += chamberTemperature;
  bottomText += " ";

  bottomText += ("Printing: ");
  bottomText += isPrinting ? "Yes" : "No";

  int topTextLen = topText.length();
  int bottomTextLen = bottomText.length();
  int maxScroll = max(topTextLen, bottomTextLen);

  for (int i = 0; i < maxScroll - 16; i++) {
    display.clear();
    display.setCursor(0, 0);
    for (int j = 0; j < 16; j++) {
      if (i + j < topTextLen) {
        display.print(topText[i + j]);
      } else {
        display.print(' ');
      }
    }

    display.setCursor(0, 1);
    for (int s = 0; s < 16; s++) {
      if (i + s < bottomTextLen) {
        display.print(bottomText[i + s]);
      } else {
        display.print(' ');
      }
    }
    delay(500);
  }
}

void IRAM_ATTR handleTach() {
  tachCount++;
}

void onMessageCallback(websockets::WebsocketsMessage message) {
  Serial.printf("Received: %s\n", message.data());
  JsonDocument doc;
  deserializeJson(doc, message.data());
  if (doc.containsKey("event")) {
    String event = doc["event"].as<String>();
    if (event == "PrintStarted") {
      isPrinting = true;
      // Get target temperature from Moonraker API when print starts
      if (WiFi.status() == WL_CONNECTED && moonrakerIp != "") {
        HTTPClient http;
        String url = "http://" + moonrakerIp + "/printer/gcode/script?script=M191";
        http.begin(url);
        http.addHeader("Authorization", "Bearer " + moonrakerAuth);
        int httpResponseCode = http.GET();

        if (httpResponseCode == 200) {
          String payload = http.getString();
          JsonDocument doc;
          deserializeJson(doc, payload);
          targetTemperature = doc["result"]["target_temperature"].as<float>();
          Serial.println("Target Temperature: " + String(targetTemperature));
          waitingForTemperature = true;
        } else {
          Serial.println("Error on HTTP request: " + String(httpResponseCode));
        }
        http.end();
      } else {
        Serial.println("WiFi Disconnected or Moonraker IP not set");
      }
    } else if (event == "PrintDone" || event == "PrintFailed") {
      isPrinting = false;
    }
  } else if (doc.containsKey("params") && doc["params"].containsKey("temperature")) {
    chamberTemperature = doc["params"]["temperature"]["chamber"]["actual"].as<float>();
    if (!useDHT20) {
      currentTemperature = chamberTemperature;
    }
  }
}

void webSocketEvent(websockets::WebsocketsEvent event, String data) {
  switch (event) {
    case websockets::WebsocketsEvent::ConnectionClosed:
      Serial.println("Disconnected from Moonraker");
      break;
    case websockets::WebsocketsEvent::ConnectionOpened:
      Serial.println("Connected to Moonraker");
      subscribeMoonraker();
      break;
    default:
      break;
  }
}

void subscribeMoonraker() {
  if (WiFi.status() == WL_CONNECTED && moonrakerIp != "") {
    String auth = "Bearer " + moonrakerAuth;
    webSocket.addHeader("Authorization", auth.c_str());
    webSocket.send("{\"jsonrpc\":\"2.0\",\"method\":\"printer.subscribe\",\"params\":{\"topics\":[\"event.state\",\"temperature\"]},\"id\":1}");
    
  } else {
    Serial.println("WiFi Disconnected or Moonraker IP not set");
  }
}

void attemptConnection() {
  if (WiFi.status() == WL_CONNECTED && moonrakerIp != "") {
    webSocket.connect(moonrakerIp.c_str(), 80, "/websocket");
    webSocket.onEvent(webSocketEvent);
    webSocket.onMessage(onMessageCallback);
    // Send a ping
    webSocket.ping();
    lastConnectionAttempt = millis();
  } else {
    Serial.println("WiFi Disconnected or Moonraker IP not set");
  }
}

void sendContinueCommand() {
  if (WiFi.status() == WL_CONNECTED && moonrakerIp != "") {
    HTTPClient http;
    String url = "http://" + moonrakerIp + "/printer/gcode/script?script=CONTINUE";
    http.begin(url);
    http.addHeader("Authorization", "Bearer " + moonrakerAuth);
    int httpResponseCode = http.GET();

    if (httpResponseCode == 200) {
      Serial.println("Sent continue command to Moonraker");
    } else {
      Serial.println("Error on HTTP request: " + String(httpResponseCode));
    }
    http.end();
  } else {
    Serial.println("WiFi Disconnected or Moonraker IP not set");
  }
}
