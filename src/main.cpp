#include <Arduino.h>
#include <Wire.h>
#include "I2CScanner.h"
#include "SSD1306Wire.h"
#include <DHT20.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Ticker.h>

// Create a Ticker object
Ticker pollTicker;
Ticker displayTicker;
Ticker temperatureTicker;

I2CScanner scanner;

// OLED display settings
#define I2C_ADDRESS 0x3C
SSD1306Wire display(I2C_ADDRESS, SDA, SCL);

// PWM settings
#define HEATER_PIN 16  // GPIO pin for heater control
#define FAN_PIN 23    // GPIO pin for fan control
#define PWM_FREQ 25000 // 25 kHz frequency for Noctua fan
#define PWM_RESOLUTION 8 // 8-bit resolution
#define PWM_HEATER_LIMIT 200

// Web server
WebServer server(80);

// NVS (Non-Volatile Storage)
Preferences preferences;

DHT20 dht(&Wire); // Initialize the DHT20 object

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
bool isConnected = false;
bool isPrinting = false;

unsigned long lastConnectionAttempt = 0;
const unsigned long connectionInterval = 10000; // 10 seconds

bool waitingForTemperature = false;
bool useDHT20 = true; // Flag to switch between DHT20 and Moonraker
bool manualMode = false; // Flag to switch between DHT20 and Moonraker

// WiFi and 3D printer icons
#define ICON_WIDTH 16
#define ICON_HEIGHT 16
static const unsigned char PROGMEM wifi_icon[] = {
  0x00,0x00,0x00,0x00,0xc0,0x07,0xf8,0x3f,0xfc,0x7f,0x1c,0x70,0x00,0x01,0xe0,
0x0f,0xf0,0x1f,0x60,0x0c,0x00,0x00,0x80,0x03,0xc0,0x07,0xc0,0x07,0x80,0x03,
0x00,0x00
};

static const unsigned char PROGMEM printer_icon[] = {
  0x00,0x00,0x00,0x02,0x00,0x07,0x00,0x0f,0x40,0x1e,0xe0,0x3c,0xf0,0x79,0xf8,
0x73,0xec,0x27,0xc4,0x0f,0x8c,0x07,0x1c,0x03,0xbc,0x01,0xfc,0x00,0x00,0x00,
0x00,0x00
};

static const unsigned char PROGMEM printing_icon[] = {
  0xc0,0x01,0xc0,0x01,0xc0,0x01,0xc0,0x01,0xc0,0x01,0xfc,0x1f,0xf8,0x0f,0xe0,
0x03,0xc3,0x61,0x83,0x60,0x03,0x60,0x03,0x60,0x07,0x70,0xfe,0x3f,0xfc,0x1f,
0x00,0x00
};

// Function declarations
void handleRoot();
void handleSetTargetTemp();
void handleSetManualMode();
void handleSetMoonrakerConfig();
void handleSetTemperatureSource();
void updatePWM();
void attemptConnection();
void subscribeMoonraker();

void updateDisplay() {
  display.clear();
  display.setFont(ArialMT_Plain_10);

  // Draw WiFi icon if connected
  if (WiFi.status() == WL_CONNECTED) {
    display.drawXbm(0, 0, ICON_WIDTH, ICON_HEIGHT, wifi_icon);
  }

  // Draw 3D printer icon if connected to Moonraker
  if (isConnected) {
    display.drawXbm(0, 20, ICON_WIDTH, ICON_HEIGHT, printer_icon);
  }

  if (isPrinting) {
    display.drawXbm(0, 40, ICON_WIDTH, ICON_HEIGHT, printing_icon);
  }

  String temp = "Current Temp: ";
  temp += currentTemperature;
  temp += " C";
  display.drawString(20, 0, temp);

  String targetTemp = "Target Temp: ";
  targetTemp += targetTemperature;
  targetTemp += " C";
  display.drawString(20, 10, targetTemp);

  String humidity = "Humidity: ";
  humidity += currentHumidity;
  humidity += " %";
  display.drawString(20, 20, humidity);

  String fanSpeedText = "Fan Speed: ";
  fanSpeedText += fanSpeed;
  fanSpeedText += " %";
  display.drawString(20, 30, fanSpeedText);

  String chambTempText = "Chamber Temp: ";
  chambTempText += chamberTemperature;
  chambTempText += " C";
  display.drawString(20, 40, chambTempText);

  display.display();
}

void onMessageCallback(websockets::WebsocketsMessage message) {
  Serial.printf("Received: %s\n", message.data());
  JsonDocument doc;
  deserializeJson(doc, message.data());
  // Status
  if (doc.containsKey("result") && doc["params"].containsKey("status") && doc["result"]["status"].containsKey("print_stats")) {
    String event = doc["result"]["status"]["print_stats"]["state"].as<String>();
    if (event == "printing") {
      isPrinting = true;
    } else {
      isPrinting = false;
    }
  } else if (doc.containsKey("params") && doc["params"].containsKey("temperature_sensor chamber_temp")) {
    chamberTemperature = doc["params"]["temperature_sensor chamber_temp"]["temperature"].as<float>();
  } else if (doc.containsKey("method") && doc.containsKey("params") && doc["method"].as<String>() == "notify_gcode_response") {
    JsonArray params = doc["params"];
    for (const char* param : params) {
      Serial.println(param);
      // Check if the string contains the desired substring
      if (strstr(param, "// Set chamber heater to: ") != nullptr) {
        // Extract the temperature value from the string
        float temperature = 0;
        sscanf(param, "// Set chamber heater to: %f", &temperature);
        targetTemperature = temperature;
      }
    }
  }
}

void onEventsCallback(websockets::WebsocketsEvent event, String data) {
  switch (event) {
    case websockets::WebsocketsEvent::ConnectionClosed:
      isConnected = false;
      server.begin();
      Serial.println("Disconnected from Moonraker");
      break;
    case websockets::WebsocketsEvent::ConnectionOpened:
      Serial.println("Connected to Moonraker");
      isConnected = true;
      server.stop();
      subscribeMoonraker();
      break;
    case websockets::WebsocketsEvent::GotPing:
      Serial.println("Got a Ping!");
      isConnected = true;
      break;
    case websockets::WebsocketsEvent::GotPong:
      Serial.println("Got a Pong!");
      isConnected = true;
      break;
    default:
      break;
  }
}

void updateTempareture() {
    dht.read(); // Read values from DHT20
    currentTemperature = dht.getTemperature();
    currentHumidity = dht.getHumidity();
}

websockets::WebsocketsClient webSocket; // Use the ArduinoWebsockets client

// Function to be called every 500 milliseconds
void pollWebSocket() {
  // Handle WebSocket communication
  if (isConnected) {
    Serial.println("Poll");
    webSocket.poll();
  }
}


void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Wire.begin(SDA, SCL);

  scanner.Init();
  scanner.Scan();

  // Initialize the DHT20 sensor
  dht.begin();

  // Initialize the OLED display
  display.init();
  display.displayOn(); 
  display.resetDisplay();

  display.setFont(ArialMT_Plain_10);
  display.drawString(20, 0, "Boot...");

  // Initialize WiFi using WiFiManager
  WiFiManager wifiManager;
  if (!wifiManager.autoConnect("Chamber Heater AP")) {
    Serial.println("Failed to connect and hit timeout");
    ESP.restart();
  }

  if (WiFi.status() == WL_CONNECTED) {
    display.drawString(20, 10, "IP Address: " + String(WiFi.localIP()));
  } else {
    display.drawString(20, 10, "Acces Point: " + String(WiFi.softAPIP()));
  }
  
  display.display();

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
  server.on("/setManualMode", handleSetManualMode);
  server.begin();
  Serial.println("HTTP server started");

  // Initialize PWM for heater and fan
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION); // 25 kHz PWM, 8-bit resolution
  ledcAttachPin(HEATER_PIN, 0);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION); // 25 kHz PWM, 8-bit resolution
  ledcAttachPin(FAN_PIN, 1);

  // Attempt initial connection to WebSocket
  webSocket.onMessage(onMessageCallback);
  webSocket.onEvent(onEventsCallback);
  
  attemptConnection();
  updatePWM();

  pollTicker.attach_ms(500, pollWebSocket);
  displayTicker.attach(1, updateDisplay);
  temperatureTicker.attach(1, updateTempareture);


    // Create the wifiTask on core 0
  xTaskCreatePinnedToCore(
    wifiTask,    // Function to implement the task
    "wifiTask",  // Name of the task
    10000,       // Stack size in words
    NULL,        // Task input parameter
    1,           // Priority of the task
    NULL,        // Task handle
    0);          // Core where the task should run

  // Create the i2cTask on core 1
  xTaskCreatePinnedToCore(
    i2cTask,    // Function to implement the task
    "i2cTask",  // Name of the task
    10000,      // Stack size in words
    NULL,       // Task input parameter
    1,          // Priority of the task
    NULL,       // Task handle
    1);         // Core where the task should run
}

void loop() {
  // Handle web server requests
  server.handleClient();

  // Attempt to connect to WebSocket if not connected
  unsigned long currentTime = millis();
  if (!webSocket.available() && currentTime - lastConnectionAttempt >= connectionInterval) {
    attemptConnection();
    lastConnectionAttempt = currentTime;
  }

  if (useDHT20) {
      chamberTemperature = currentTemperature;
  }

  // Update PWM values based on temperature
  updatePWM();
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

void updatePWM() {
  // Heating and fan control logic
  if (isPrinting || manualMode) {
    if (chamberTemperature < targetTemperature) {
      fanStopTime = 0; // Reset fan stop time
      if (chamberTemperature < targetTemperature - fullLoadThreshold) {
        // Temperature is much lower than target, full load
        ledcWrite(0, PWM_HEATER_LIMIT);
        ledcWrite(1, 255);
        fanSpeed = 100;
        isFanRunning = true;
      } else {
        // Temperature is closer to target, reduce heater power
        float pwmValue = 255 * (targetTemperature - chamberTemperature) / fullLoadThreshold;
        pwmValue = constrain((int)pwmValue, 0, PWM_HEATER_LIMIT);
        ledcWrite(0, pwmValue);
        ledcWrite(1, 255);
        fanSpeed = 100;
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
          fanSpeed = 0;
          isFanRunning = false;
        } else {
          // Half speed
          fanSpeed = 50;
          ledcWrite(1, 128);
        }
      }
    }
  } else {
    fanSpeed = 0;
    isFanRunning = false;
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  }
}

void subscribeMoonraker() {
  if (isConnected) {
    Serial.println("Subscribing");
    webSocket.send("{\"jsonrpc\": \"2.0\",\"method\": \"printer.objects.subscribe\",\"params\": {\"objects\": {\"print_stats\": [\"state\"]}},\"id\": 1}");
    webSocket.send("{\"jsonrpc\": \"2.0\",\"method\": \"printer.objects.subscribe\",\"params\": {\"objects\": {\"webhooks\": [\"gcode_response\"]}},\"id\": 2}");
    webSocket.send("{\"jsonrpc\": \"2.0\",\"method\": \"printer.objects.subscribe\",\"params\": {\"objects\": {\"temperature_sensor chamber_temp\": [\"temperature\"]}},\"id\": 3}");
  } else {
    Serial.println("WiFi Disconnected or Moonraker IP not set");
  }
}

void attemptConnection() {
  if (WiFi.status() == WL_CONNECTED && moonrakerIp != "") {
    if (moonrakerAuth != "") {
      String auth = "Bearer " + moonrakerAuth;
      webSocket.addHeader("Authorization", auth.c_str());
    }
    if(!webSocket.connect(moonrakerIp.c_str(), (uint16_t) 7125, "/websocket")) {
      Serial.println("Could not connect to websocket: " + moonrakerIp);
    } else {
      // Send a ping
      webSocket.ping();
    }
    lastConnectionAttempt = millis();
  } else {
    Serial.println("WiFi Disconnected or Moonraker IP not set");
  }
}
