
#include <Arduino.h>
#include <Wire.h>
#include "I2CScanner.h"
#include "SSD1306Wire.h"
#include <DHT20.h>
#include <WiFiManager.h>
#include <Preferences.h>



#include <WebSocketsClient.h>
#include <WebSocketsServer.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <PID_v1.h>
#include <Ticker.h>


WiFiManager wifiManager;
WebServer server(80);
WebSocketsClient webSocket;
WebSocketsServer webSocketServer = WebSocketsServer(81);

// Create a Ticker object
Ticker displayTicker;
Ticker temperatureTicker;
Ticker metricsTicker;
Ticker pwmTicker;

double targetTemperature = 22.0; // Default target temperature
float maxTemperature = 70.0; // Default target temperature
const float fullLoadThreshold = 40.0; // Default target temperature
const float halfLoadThreshold = 5.0; // Temperature threshold for full load

float dhtTemperature = 0.0;
float dhtHumidity = 0.0;

float chamberTemperature = 0.0;
double currentTemperature = 0.0;

// PID parameters for heater
double heaterOutput;
String heaterPIDStr = "1,0,0";
double Kp = 1, Ki = 0, Kd = 0; // Initial PID values for heater
PID heaterPID(&currentTemperature, &heaterOutput, &targetTemperature, Kp, Ki, Kd, DIRECT);

// PID parameters for fan
double fanOutput;
String fanPIDStr = "1,0,0";
double fanKp = 1, fanKi = 0, fanKd = 0; // Initial PID values for fan
PID fanPID(&currentTemperature, &fanOutput, &targetTemperature, fanKp, fanKi, fanKd, DIRECT);

I2CScanner scanner;

// OLED display settings
#define I2C_ADDRESS 0x3C
SSD1306Wire display(I2C_ADDRESS, SDA, SCL);

// PWM settings
#define FAN_TACH_PIN 4
#define FAN_POWER_PIN 13
#define FAN_PIN 23    // GPIO pin for fan control
#define HEATER_PIN 16  // GPIO pin for heater control
#define PWM_FREQ 25000 // 25 kHz frequency for Noctua fan
#define PWM_RESOLUTION 8 // 8-bit resolution
#define PWM_HEATER_LIMIT 220

// NVS (Non-Volatile Storage)
Preferences preferences;

DHT20 dht(&Wire); // Initialize the DHT20 object

int maxFanSpeed = 100;
volatile unsigned long tachCounter;
volatile unsigned long lastTachTime= 0;
bool isFanRunning = false; // Fan state
unsigned long fanStopTime = 0; // Time when the fan should stop
volatile unsigned long pulseInterval = 0;
int fanRPM = 0; // the average

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

// socket page content
const char* htmlMetrics = R"rawliteral(
  <div id="metrics">
    <div class="metric">Top Temperature: <span id="chamberTemperature">--</span> &deg;C</div>
    <div class="metric">Bottom Temperature: <span id="dhtTemperature">--</span> &deg;C</div>
    <div class="metric">Bottom Humidity: <span id="dhtHumiduty">--</span> %</div>
    <div class="metric">Heater Duty: <span id="heaterDuty">--</span> %</div>
    <div class="metric">Fan Duty: <span id="fanDuty">--</span> %</div>
  </div>
  <div id="chart-container" style="height:250px">
    <canvas id="temperatureChart"></canvas>
  </div>
  <script>
    const socket = new WebSocket('ws://' + window.location.hostname + ':81');

    const ctx = document.getElementById('temperatureChart').getContext('2d');
    const temperatureChart = new Chart(ctx, {
      type: 'line',
      data: {
        labels: [], // Time labels
        datasets: [
          {
            label: 'Top Temperature (C)',
            borderColor: '#ff6384',
            data: [],
            fill: false,
          },
          {
            label: 'Bottom Temperature (C)',
            borderColor: '#ff9f40',
            data: [],
            fill: false,
          },
          {
            label: 'Humidity (%)',
            borderColor: '#36a2eb',
            data: [],
            fill: false,
          },
          {
            label: 'Heater duty (%)',
            borderColor: '#4bc0c0',
            data: [],
            fill: false,
          },
          {
            label: 'Fan Duty (%)',
            borderColor: '#9966ff',
            data: [],
            fill: false,
          },
          {
            label: 'Target Temperature (C)',
            borderColor: '#cc65fe',
            data: [],
            fill: false,
          }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        pointStyle: false,
        scales: {
          x: {
            type: 'realtime',
            realtime: {
              duration: 200000,
              refresh: 1000,
              delay: 1000
            }
          },
          y: {
            beginAtZero: false
          }
        },
        plugins: {
          // Change options for ALL axes of THIS CHART
          streaming: {
          }
        }
      }
    });

    socket.onopen = function() {
      console.log('WebSocket connection established');
    };

    socket.onmessage = function(event) {
      const metrics = JSON.parse(event.data);
      document.getElementById('chamberTemperature').innerText = metrics.chamberTemperature;
      document.getElementById('dhtTemperature').innerText = metrics.dhtTemperature;
      document.getElementById('dhtHumiduty').innerText = metrics.dhtHumiduty;
      document.getElementById('heaterDuty').innerText = metrics.heaterDuty;
      document.getElementById('fanDuty').innerText = metrics.fanDuty;

      // Add data to chart
      temperatureChart.data.datasets[0].data.push({x: Date.now(), y: metrics.chamberTemperature});
      temperatureChart.data.datasets[1].data.push({x: Date.now(), y: metrics.dhtTemperature});
      temperatureChart.data.datasets[2].data.push({x: Date.now(), y: metrics.dhtHumiduty});
      temperatureChart.data.datasets[3].data.push({x: Date.now(), y: metrics.heaterDuty});
      temperatureChart.data.datasets[4].data.push({x: Date.now(), y: metrics.fanDuty});
      temperatureChart.data.datasets[5].data.push({x: Date.now(), y: metrics.targetTemperature});
      temperatureChart.update('quiet');
    };

    socket.onclose = function() {
      console.log('WebSocket connection closed');
    };

    socket.onerror = function(error) {
      console.error('WebSocket error:', error);
    };
  </script>
)rawliteral";

// Function declarations
void handleRoot();
void handleSetTargetTemp();
void handleSetMaxFanSpeed();
void handleSetManualMode();
void handleSetMoonrakerConfig();
void handleSetTemperatureSource();
void handleSetPID();
void updatePWM();
void attemptConnection();
void subscribeMoonraker();

void IRAM_ATTR tachISR() {
  tachCounter++;
}

void updateMetrics() {
  // Prepare metrics JSON
  String metrics = "{";
  metrics += "\"chamberTemperature\": " + String(chamberTemperature, 2) + ",";
  metrics += "\"dhtTemperature\": " + String(dhtTemperature, 2) + ",";
  metrics += "\"dhtHumiduty\": " + String(dhtHumidity, 2) + ",";
  metrics += "\"heaterDuty\": " + String(heaterOutput, 2) + ",";
  metrics += "\"fanDuty\": " + String(fanOutput, 2) + ",";
  metrics += "\"targetTemperature\": " + String(targetTemperature, 2);
  metrics += "}";

  webSocketServer.broadcastTXT(metrics);
}

void updateDisplay() {
  display.clear();
  display.setFont(ArialMT_Plain_10);

  // Draw WiFi icon if connected
  if (WiFi.status() == WL_CONNECTED) {
    display.drawXbm(0, 0, ICON_WIDTH, ICON_HEIGHT, wifi_icon);
  }

  // Draw 3D printer icon if connected to Moonraker
  if (webSocket.isConnected()) {
    display.drawXbm(0, 20, ICON_WIDTH, ICON_HEIGHT, printer_icon);
  }

  if (webSocket.isConnected() && isPrinting) {
    display.drawXbm(0, 40, ICON_WIDTH, ICON_HEIGHT, printing_icon);
  }

  String targetTemp = "Target Temp: ";
  targetTemp += targetTemperature;
  targetTemp += " C";
  display.drawString(20, 0, targetTemp);

  String temp = "Heater Temp: ";
  temp += dhtTemperature;
  temp += " C";
  display.drawString(20, 10, temp);

  String humidity = "Heater Humidity: ";
  humidity += dhtHumidity;
  humidity += " %";
  display.drawString(20, 20, humidity);

  String chambTempText = "Chamber Temp: ";
  chambTempText += chamberTemperature;
  chambTempText += " C";
  display.drawString(20, 30, chambTempText);

  String fanSpeedText = "Fan Speed: ";
  fanSpeedText += fanOutput;
  fanSpeedText += " %";
  display.drawString(20, 40, fanSpeedText);

  // String surrentSensor = "Sensor: ";
  // surrentSensor += useDHT20? "Heater" : "Chamber";
  // display.drawString(20, 50, surrentSensor);

  String currentFanRpm = "Fan RPM: ";
  currentFanRpm += fanRPM;
  display.drawString(20, 50, currentFanRpm);

  display.display();
}

void onEventsCallback(WStype_t type, uint8_t * payload, size_t length) {
  String payloadStr = "";
      for (size_t i = 0; i < length; i++) {
        payloadStr += (char)payload[i];
      }
      // Serial.print("Received message: ");
      // Serial.println(payloadStr);
  switch (type) {
    case WStype_DISCONNECTED:
      //isConnected = false;
      Serial.println("Disconnected from Moonraker");
      break;
    case WStype_CONNECTED:
      Serial.println("Connected to Moonraker");
      //isConnected = true;
      subscribeMoonraker();
      break;
    case WStype_TEXT:
      JsonDocument doc;
      deserializeJson(doc, payloadStr);
      // Status
      if (doc.containsKey("result") && doc["params"].containsKey("status") && doc["result"]["status"].containsKey("print_stats")) {
        String event = doc["result"]["status"]["print_stats"]["state"].as<String>();
        if (event == "printing") {
          isPrinting = true;
        } else {
          isPrinting = false;
        }
      } else if (doc.containsKey("params") && doc.containsKey("method") && doc["method"] == "notify_status_update"
        && doc["params"].is<JsonArray>() && doc["params"].size() > 0) {
        // Iterate through the "params" array
        for (JsonVariant value : doc["params"].as<JsonArray>()) {
          // Check if the current element contains the key
          if (value.is<JsonObject>() && value.containsKey("temperature_sensor chamber_temp")) {
            // Extract the temperature value
            chamberTemperature = (float) value["temperature_sensor chamber_temp"]["temperature"];
            continue;
          }
        }
      } else if (doc.containsKey("method") && doc.containsKey("params") && doc["method"] == "notify_gcode_response"
          && doc["params"].is<JsonArray>() && doc["params"].size() > 0) {
        Serial.println("Found gCode history");
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
      break;

  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_DISCONNECTED:
            Serial.println("Disconnected from webpage");
            break;
        case WStype_CONNECTED:
            Serial.println("Connected to webpage");
            break;
        case WStype_TEXT:
            //Serial.println("[%u] get Text: %s\n", num, payload);
            // send message to client
            // webSocket.sendTXT(num, "message here");

            // send data to all connected clients
            // webSocket.broadcastTXT("message here");
            break;
        case WStype_BIN:
        case WStype_ERROR:			
        case WStype_FRAGMENT_TEXT_START:
        case WStype_FRAGMENT_BIN_START:
        case WStype_FRAGMENT:
        case WStype_FRAGMENT_FIN:
			      break;
    }

}

void updateTempareture() {
    dht.read(); // Read values from DHT20
    dhtTemperature = dht.getTemperature();
    dhtHumidity = dht.getHumidity();
}


void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  pinMode(FAN_POWER_PIN, OUTPUT);
  // Initially turn off the LM2596 module
  digitalWrite(FAN_POWER_PIN, HIGH);
  pinMode(FAN_TACH_PIN, INPUT_PULLUP);  // Enable internal pull-up resistor
  
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), tachISR, FALLING);
  
  ledcWrite(0, 0);
  ledcWrite(1, 0);

  Wire.begin(SDA, SCL);

  scanner.Init();
  scanner.Scan();

  // Initialize the DHT20 sensor
  dht.begin();

  // Initialize the OLED display
  display.init();
  display.flipScreenVertically();
  display.displayOn(); 
  display.resetDisplay();

  //display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "Boot...");

  // Initialize WiFi using WiFiManager
  if (!wifiManager.autoConnect("Chamber Heater AP")) {
    Serial.println("Failed to connect and hit timeout");
    ESP.restart();
  }
  display.display();

  if (WiFi.status() == WL_CONNECTED) {
    display.drawString(0, 10, "IP Address: " + String(WiFi.localIP().toString()));
  } else {
    display.drawString(0, 10, "Acces Point: " + String(WiFi.softAPIP().toString()));
  }
  
  display.display();

  // Initialize NVS
  preferences.begin("moonraker", false);
  moonrakerIp = preferences.getString("ip", "");
  moonrakerAuth = preferences.getString("auth", "");
  useDHT20 = preferences.getBool("useDHT20", true);
  manualMode = preferences.getBool("manualMode", false);
  maxFanSpeed = preferences.getInt("maxFanSpeed", 100);

  heaterPID.SetMode(AUTOMATIC);
  heaterPID.SetOutputLimits(0, 255); // Set output limits for heater PWM
  heaterPIDStr = preferences.getString("heaterPID", heaterPIDStr);
  sscanf(heaterPIDStr.c_str(), "%lf,%lf,%lf", &Kp, &Ki, &Kd);
  heaterPID.SetTunings(Kp, Ki, Kd);

  fanPID.SetMode(AUTOMATIC);
  fanPID.SetOutputLimits(0, 255); // Set output limits for fan PWM
  fanPIDStr = preferences.getString("fanPID", fanPIDStr);
  sscanf(fanPIDStr.c_str(), "%lf,%lf,%lf", &fanKp, &fanKi, &fanKd);
  heaterPID.SetTunings(fanKp, fanKi, fanKd);

  delay(5000);

  // Initialize the web server
  server.on("/", handleRoot);
  server.on("/setTargetTemp", handleSetTargetTemp);
  server.on("/setMaxFanSpeed", handleSetMaxFanSpeed);
  server.on("/setMoonrakerConfig", handleSetMoonrakerConfig);
  server.on("/setTemperatureSource", handleSetTemperatureSource);
  server.on("/setManualMode", handleSetManualMode);
  server.on("/setPID", handleSetPID);
  server.begin();
  Serial.println("HTTP server started");

  // Initialize PWM for heater and fan
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION); // 25 kHz PWM, 8-bit resolution
  ledcAttachPin(HEATER_PIN, 0);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION); // 25 kHz PWM, 8-bit resolution
  ledcAttachPin(FAN_PIN, 1);

  // Attempt initial connection to WebSocket
  webSocket.setReconnectInterval(5000);
  webSocket.onEvent(onEventsCallback);

  // Start WebSocket server
  webSocketServer.begin();
  webSocketServer.onEvent(webSocketEvent);

  displayTicker.attach(1, updateDisplay);
  metricsTicker.attach(1, updateMetrics);
  temperatureTicker.attach(1, updateTempareture);
  pwmTicker.attach(1, updatePWM);

  display.drawString(0, 20, "Boot complete");
  display.display();
}

void loop() {
  webSocketServer.loop();
  webSocket.loop();

  // Handle web server requests
  server.handleClient();

  // Attempt to connect to WebSocket if not connected
  unsigned long currentTime = millis();
  if (!webSocket.isConnected() && currentTime - lastConnectionAttempt >= connectionInterval) {
    Serial.println("Trying to connet to Moonraker");
    attemptConnection();
    lastConnectionAttempt = currentTime;
  }

  // Update every second
  unsigned long currentTimeMicros = micros();
  unsigned long timeDiff = currentTimeMicros - lastTachTime;
  if (timeDiff > 1000000) {
    noInterrupts();
    unsigned long pulses = tachCounter;
    tachCounter = 0;
    interrupts();
    fanRPM = (pulses * 60.0) / 2.0;
    lastTachTime = currentTimeMicros;
    Serial.println("RPM pulses: " + String(pulses) + " Seconds passed:" + String(timeDiff/1000000));
  }
}

void handleRoot() {
  String html = "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"><link rel=\"stylesheet\" href=\"https://cdn.simplecss.org/simple.min.css\">";
  html += "<script src=\"https://cdn.jsdelivr.net/npm/chart.js\"></script><script src=\"https://cdn.jsdelivr.net/npm/luxon\"></script><script src=\"https://cdn.jsdelivr.net/npm/chartjs-adapter-luxon\"></script><script src=\"https://cdn.jsdelivr.net/npm/chartjs-plugin-streaming\"></script></head><body>";
  html += "<header><h1>Moonraker chamber Heater Control</h1>";
  html += "<p>Target Temperature: " + String(targetTemperature) + " &deg;C</p>";
  html += htmlMetrics;
  html += "</header>";
  html += "<form action=\"/setPID\" method=\"post\">";
  html += "  <label for=\"heaterPID\">Heater PID (Kp,Ki,Kd):</label><br>";
  html += "  <input type=\"text\" id=\"heaterPID\" name=\"heaterPID\" value=\"" + String(heaterPIDStr) + "\"><br>";
  html += "  <label for=\"fanPID\">Fan PID (Kp,Ki,Kd):</label><br>";
  html += "  <input type=\"text\" id=\"fanPID\" name=\"fanPID\" value=\"" + String(fanPIDStr) + "\"><br>";
  html += "  <input type=\"submit\" value=\"Submit\">";
  html += "</form>";
  html += "<form action=\"/setMaxFanSpeed\" method=\"POST\">";
  html += "Set Max Fan Speed %: <input type=\"text\" name=\"maxFanSpeed\" value=\"" + String(maxFanSpeed) + "\"><br>";
  html += "<input type=\"submit\" value=\"Set\">";
  html += "</form>";
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
  html += "Heater sensor: <input type=\"radio\" name=\"source\" value=\"DHT20\" " + String(useDHT20 ? "checked" : "") + "><br>";
  html += "Moonraker: <input type=\"radio\" name=\"source\" value=\"Moonraker\" " + String(!useDHT20 ? "checked" : "") + "><br>";
  html += "<input type=\"submit\" value=\"Set\">";
  html += "</form>";
  html += "<h2>Mode</h2>";
  html += "<form action=\"/setManualMode\" method=\"POST\">";
  html += "Manual: <input type=\"radio\" name=\"mode\" value=\"Yes\" " + String(manualMode ? "checked" : "") + "><br>";
  html += "Moonraker: <input type=\"radio\" name=\"mode\" value=\"No\" " + String(!manualMode ? "checked" : "") + "><br>";
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

void handleSetMaxFanSpeed() {
  if (server.hasArg("maxFanSpeed")) {
    maxFanSpeed = server.arg("maxFanSpeed").toInt();
    Serial.println("New max fan speed set to: " + String(maxFanSpeed));
    preferences.putInt("maxFanSpeed", maxFanSpeed);
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
    if (webSocket.isConnected()) {
      webSocket.disconnect();
    }
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

void handleSetPID() {
  if (server.hasArg("heaterPID") && server.hasArg("fanPID")) {
    heaterPIDStr = server.arg("heaterPID");
    fanPIDStr = server.arg("fanPID");

    // Parse and set Heater PID values
    sscanf(heaterPIDStr.c_str(), "%lf,%lf,%lf", &Kp, &Ki, &Kd);
    heaterPID.SetTunings(Kp, Ki, Kd);
    preferences.putString("heaterPID", heaterPIDStr);

    // Parse and set Fan PID values
    sscanf(fanPIDStr.c_str(), "%lf,%lf,%lf", &fanKp, &fanKi, &fanKi);
    fanPID.SetTunings(fanKp, fanKi, fanKi);
    preferences.putString("fanPID", fanPIDStr);
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void updatePWM() {

  if (useDHT20) {
    currentTemperature = dhtTemperature;
  } else {
    currentTemperature = chamberTemperature;
  }

  // Temperature protection
  if (maxTemperature < chamberTemperature || maxTemperature < dhtTemperature) {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    digitalWrite(FAN_POWER_PIN, HIGH);
    Serial.println("Warning temperature limit reached");
    return;
  }

  // Primary PID control for heater
  heaterPID.Compute();
  Serial.println("Heater output: " + (String) heaterOutput);
  ledcWrite(0, heaterOutput);

  bool fanComputeResult = fanPID.Compute();
  Serial.println("Fan compute: " + fanComputeResult? "Updated": "Failed");
  Serial.println("Fan output: " + (String) fanOutput);
  if (fanOutput == 0) {
    digitalWrite(FAN_POWER_PIN, HIGH);
  } else {
    digitalWrite(FAN_POWER_PIN, LOW);
  }
  ledcWrite(1, fanOutput);

  // ----------------------------------------
}

void subscribeMoonraker() {
  if (webSocket.isConnected()) {
    Serial.println("Subscribing");
    webSocket.sendTXT("{\"jsonrpc\": \"2.0\",\"method\": \"printer.objects.subscribe\",\"params\": {\"objects\": {\"print_stats\": [\"state\"]}},\"id\": 1}");
    webSocket.sendTXT("{\"jsonrpc\": \"2.0\",\"method\": \"printer.objects.subscribe\",\"params\": {\"objects\": {\"webhooks\": [\"gcode_response\"]}},\"id\": 2}");
    webSocket.sendTXT("{\"jsonrpc\": \"2.0\",\"method\": \"printer.objects.subscribe\",\"params\": {\"objects\": {\"temperature_sensor chamber_temp\": [\"temperature\"]}},\"id\": 3}");
  } else {
    Serial.println("WiFi Disconnected or Moonraker IP not set");
  }
}

void attemptConnection() {
  if (WiFi.status() == WL_CONNECTED && moonrakerIp != "" && !webSocket.isConnected()) {
    if (moonrakerAuth != "") {
      String auth = "Authorization: Bearer " + moonrakerAuth;
      webSocket.setExtraHeaders(auth.c_str());
    }
    IPAddress addr = IPAddress();
    addr.fromString(moonrakerIp);
    webSocket.begin(addr, (uint16_t) 7125, "/websocket", "");
  } else {
    Serial.println("WiFi Disconnected or Moonraker IP not set");
  }
}
