#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <DHT20.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <Preferences.h>
#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>

// OLED display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SSD1306_I2C_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// PWM settings
#define HEATER_PIN 16  // GPIO pin for heater control
#define FAN_PIN 18    // GPIO pin for fan control
#define TACH_PIN 23    // GPIO pin for tachometer input
#define PWM_FREQ 25000 // 25 kHz frequency for Noctua fan
#define PWM_RESOLUTION 8 // 8-bit resolution

// Web server
WebServer server(80);

// NVS (Non-Volatile Storage)
Preferences preferences;

websockets::WebsocketsClient webSocket; // Use the ArduinoWebsockets client

DHT20 dht(&Wire); // Initialize the DHT20 object

float targetTemperature = 25.0; // Default target temperature
float currentTemperature = 0.0;
float currentHumidity = 0.0;
float chamberTemperature = 0.0;
int fanSpeed = 0;
volatile unsigned long tachCount = 0;
unsigned long lastTachTime = 0;
int fanRPM = 0;

String moonrakerIp = "";
String moonrakerAuth = "";
bool isPrinting = false;

unsigned long lastConnectionAttempt = 0;
const unsigned long connectionInterval = 10000; // 10 seconds

bool waitingForTemperature = false;
bool useDHT20 = true; // Flag to switch between DHT20 and Moonraker

// WiFi and 3D printer icons
#define WIFI_ICON_WIDTH 16
#define WIFI_ICON_HEIGHT 16
static const unsigned char PROGMEM wifi_icon[] = {
  0x00, 0x00, 0x03, 0xc0, 0x07, 0xe0, 0x0c, 0x30, 0x18, 0x18, 0x30, 0x0c, 0x63, 0xe6, 0xc6, 0x73, 
  0x8c, 0x31, 0x18, 0x18, 0x30, 0x0c, 0x60, 0x06, 0xc0, 0x03, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00 
};

#define PRINTER_ICON_WIDTH 16
#define PRINTER_ICON_HEIGHT 16
static const unsigned char PROGMEM printer_icon[] = {
  0x3c, 0x3c, 0x7e, 0x7e, 0x66, 0x66, 0xc3, 0xc3, 0x81, 0x81, 0x99, 0x99, 0xbd, 0xbd, 0xff, 0xff, 
  0x81, 0x81, 0xbd, 0xbd, 0xbd, 0xbd, 0x99, 0x99, 0x81, 0x81, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00 
};

// Function declarations
void handleRoot();
void handleSetTargetTemp();
void handleSetMoonrakerConfig();
void handleSetTemperatureSource();
void updatePWM();
void updateDisplay();
void IRAM_ATTR handleTach();

void onMessageCallback(websockets::WebsocketsMessage message);
void onEventsCallback(websockets::WebsocketsEvent event, String data);


void subscribeMoonraker();
void attemptConnection();
void sendContinueCommand();

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize the DHT20 sensor
  dht.begin();

  // Initialize the OLED display
  if (!display.begin(SSD1306_I2C_ADDRESS, OLED_RESET)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.display();
  delay(2000);
  display.clearDisplay();

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

  // Initialize the web server
  server.on("/", handleRoot);
  server.on("/setTargetTemp", handleSetTargetTemp);
  server.on("/setMoonrakerConfig", handleSetMoonrakerConfig);
  server.on("/setTemperatureSource", handleSetTemperatureSource);
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

  // Update display and PWM initially
  updateDisplay();
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
    dht.read(); // Read values from DHT20
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
  updateDisplay();

  // Add a small delay to avoid reading too frequently
  delay(2000);
}

void handleRoot() {
  String html = "<html><body>";
  html += "<h1>ESP32 PWM Heater Control</h1>";
  html += "<p>Current Temperature: " + String(currentTemperature) + " &deg;C</p>";
  html += "<p>Current Humidity: " + String(currentHumidity) + " %</p>";
  html += "<p>Target Temperature: " + String(targetTemperature) + " &deg;C</p>";
  html += "<p>Chamber Temperature: " + String(chamberTemperature) + " &deg;C</p>";
  html += "<form action=\"/setTargetTemp\" method=\"POST\">";
  html += "Set Target Temperature: <input type=\"text\" name=\"targetTemp\">";
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

void updatePWM() {
  // Calculate the PWM value for the heater
  int heaterPWM = map(currentTemperature, 0, targetTemperature, 255, 0);
  heaterPWM = constrain(heaterPWM, 0, 255);
  if (isPrinting) {
    ledcWrite(0, heaterPWM);
  } else {
    ledcWrite(0, 0);
  }

  // Calculate the PWM value for the fan
  fanSpeed = map(currentTemperature, targetTemperature, targetTemperature + 10, 0, 255);
  fanSpeed = constrain(fanSpeed, 0, 255);
  if (isPrinting) {
    ledcWrite(1, fanSpeed);
  } else {
    ledcWrite(1, 0);
  }
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Draw WiFi icon if connected
  if (WiFi.status() == WL_CONNECTED) {
    display.drawBitmap(0, 0, wifi_icon, WIFI_ICON_WIDTH, WIFI_ICON_HEIGHT, SSD1306_WHITE);
  }

  // Draw 3D printer icon if connected to Moonraker
  if (webSocket.available()) {
    display.drawBitmap(20, 0, printer_icon, PRINTER_ICON_WIDTH, PRINTER_ICON_HEIGHT, SSD1306_WHITE);
  }

  display.setCursor(40, 0);
  display.print("Current Temp: ");
  display.print(currentTemperature);
  display.print(" C");

  display.setCursor(40, 10);
  display.print("Target Temp: ");
  display.print(targetTemperature);
  display.print(" C");

  display.setCursor(40, 20);
  display.print("Humidity: ");
  display.print(currentHumidity);
  display.print(" %");

  display.setCursor(40, 30);
  display.print("Fan Speed: ");
  display.print(fanSpeed);
  display.print(" %");

  display.setCursor(40, 40);
  display.print("Fan RPM: ");
  display.print(fanRPM);

  display.setCursor(40, 50);
  display.print("Chamber Temp: ");
  display.print(chamberTemperature);
  display.print(" C");

  display.setCursor(40, 60);
  display.print("Printing: ");
  display.print(isPrinting ? "Yes" : "No");

  display.display();
}

void IRAM_ATTR handleTach() {
  tachCount++;
}

void onMessageCallback(websockets::WebsocketsMessage message) {
  Serial.printf("Received: %s\n", message.data());
  StaticJsonDocument<1024> doc;
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
          StaticJsonDocument<200> doc;
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
