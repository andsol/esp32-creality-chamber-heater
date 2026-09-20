
#include <Arduino.h>
#include <math.h>
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
#include "ThermalControl.h"
#include "NtcProcessing.h"
#include <esp_task_wdt.h>
#include <stdlib.h>


WiFiManager wifiManager;
WebServer server(80);
WebSocketsClient webSocket;
WebSocketsServer webSocketServer = WebSocketsServer(81);

thermal::Controller controller;
double targetTemperature = 0; // Explicit command required after each boot.
uint32_t lastLocalSample = 0, lastTopSample = 0;
uint32_t lastControl = 0, lastUi = 0, lastQuery = 0;
bool localSampled = false, topSampled = false;
bool moonQueryPending = false;
uint32_t moonQuerySent = 0;
constexpr uint32_t LOCAL_TIMEOUT_MS = 3000;
constexpr uint32_t TOP_TIMEOUT_MS = 15000;

float dhtTemperature = NAN;
float dhtHumidity = NAN;

// 3V3 -- 47k resistor -- GPIO34 -- NTC 100k B3950 -- GND.
// Place a 90-100 nF capacitor between GPIO34 and GND.
constexpr uint8_t NTC_PIN = 34; // ADC1: usable while Wi-Fi is active.
constexpr float NTC_SERIES_OHMS = 47000.0f;
constexpr float NTC_NOMINAL_OHMS = 100000.0f;
constexpr float NTC_BETA = 3950.0f;
constexpr float NTC_SUPPLY_MV = 3300.0f; // Adjust to measured 3V3 for calibration.
constexpr unsigned NTC_SAMPLES = 16;
float ntcTemperature = NAN;
float ntcRawTemperature = NAN;
float ntcOffset = 0;
ntc::Filter ntcFilter;
ntc::ReferenceWindow ntcReference;
float chamberTemperature = NAN;

I2CScanner scanner;

// OLED display settings
#define I2C_ADDRESS 0x3C
SSD1306Wire display(I2C_ADDRESS, SDA, SCL);

// PWM settings
#define FAN_POWER_PIN 13
#define FAN_PIN 23    // GPIO pin for fan control
#define HEATER_PIN 16  // GPIO pin for heater control
#define PWM_FREQ 25000 // 25 kHz frequency for Noctua fan
#define PWM_RESOLUTION 8 // 8-bit resolution

// NVS (Non-Volatile Storage)
Preferences preferences;

DHT20 dht(&Wire); // Initialize the DHT20 object

String moonrakerIp = "";
String moonrakerAuth = "";
bool isPrinting = false;

unsigned long lastConnectionAttempt = 0;
const unsigned long connectionInterval = 10000; // 10 seconds

bool useDHT20 = true; // Flag to switch between DHT20 and Moonraker
bool manualMode = true; // Local setpoint vs Moonraker G-code setpoint.

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

#include "WebDashboard.h"

// Function declarations
void handleRoot();
void handleSetTargetTemp();
void handleSetManualMode();
void handleSetMoonrakerConfig();
void handleSetTemperatureSource();
void handleSetControl();
void handleNtcCalibration();
void handleResetFault();
void updatePWM();
void attemptConnection();
void subscribeMoonraker();

thermal::Input controlInput() {
  const uint32_t now = millis();
  thermal::Input x;
  x.bottom = dhtTemperature; x.top = chamberTemperature; x.radiator = ntcTemperature;
  x.bottomValid = localSampled && now - lastLocalSample <= LOCAL_TIMEOUT_MS && isfinite(x.bottom);
  x.radiatorValid = localSampled && now - lastLocalSample <= LOCAL_TIMEOUT_MS && isfinite(x.radiator);
  x.topValid = topSampled && webSocket.isConnected() && now - lastTopSample <= TOP_TIMEOUT_MS && isfinite(x.top);
  x.useBottom = useDHT20; x.target = targetTemperature;
  return x;
}

void applyOutputs() {
  // Remove heat first. Establish fan output before increasing heater power.
  if (controller.heater <= 0) ledcWrite(0, 0);
  ledcWrite(1, lround(controller.fan * 255.0 / 100.0));
  digitalWrite(FAN_POWER_PIN, controller.fan > 0 ? LOW : HIGH);
  if (controller.heater > 0) ledcWrite(0, lround(controller.heater * 255.0 / 100.0));
}

bool parseNumber(const String& value, double& result) {
  const char* begin = value.c_str();
  char* end = nullptr;
  result = strtod(begin, &end);
  if (end == begin || !isfinite(result)) return false;
  while (*end == ' ' || *end == '\r' || *end == '\n' || *end == '\t') ++end;
  return *end == '\0';
}

bool setTarget(double value) {
  if (!isfinite(value) || value < 0 || value > 65) return false;
  if (value > 0 && (!controller.config.valid() || controller.state == thermal::State::Fault)) return false;
  if (value > 0) ntcReference.reset();
  targetTemperature = value;
  updatePWM(); // Stop immediately, or begin startup checks.
  return true;
}

void redirectHome() { server.sendHeader("Location", "/"); server.send(303); }

// Report the applied PWM duty, including zero after a protection shutdown.
float pwmDutyPercent(uint8_t channel) {
  constexpr uint32_t maxDuty = (1UL << PWM_RESOLUTION) - 1;
  const uint32_t duty = ledcRead(channel);
  return 100.0f * (duty > maxDuty ? maxDuty : duty) / maxDuty;
}

String ntcCalibrationStatus() {
  if (targetTemperature != 0 || controller.heater != 0 || controller.state == thermal::State::Spinup)
    return "Stop heating before calibration";
  if (controller.state == thermal::State::Fault) return "Clear the controller fault before calibration";
  if (!controller.localValid(controlInput())) return "DHT20 or NTC reading is missing or stale";
  if (ntcReference.sampleCount() < 60)
    return "Collecting measurements: " + String(ntcReference.sampleCount()) + "/60";
  if (!ntcReference.ready())
    return "Waiting for stable readings. NTC variation: " + String(ntcReference.ntcSpread(), 2) +
      " C (max 2.00 C); DHT20 variation: " + String(ntcReference.dhtSpread(), 2) + " C (max 0.50 C)";
  return "ready";
}

void updateMetrics() {
  const auto x = controlInput();
  JsonDocument doc;
  if (x.topValid) doc["chamberTemperature"] = x.top; else doc["chamberTemperature"] = nullptr;
  if (x.bottomValid) doc["dhtTemperature"] = x.bottom; else doc["dhtTemperature"] = nullptr;
  if (isfinite(dhtHumidity) && x.bottomValid) doc["dhtHumiduty"] = dhtHumidity; else doc["dhtHumiduty"] = nullptr;
  if (x.radiatorValid) doc["ntcTemperature"] = x.radiator; else doc["ntcTemperature"] = nullptr;
  if (x.topValid && x.bottomValid) doc["airDelta"] = fabs(x.top - x.bottom); else doc["airDelta"] = nullptr;
  doc["heaterDuty"] = pwmDutyPercent(0);
  doc["fanDuty"] = pwmDutyPercent(1);
  doc["targetTemperature"] = targetTemperature;
  doc["controlState"] = controller.stateName();
  doc["ntcCalibrationSamples"] = ntcReference.sampleCount();
  doc["ntcCalibrationReady"] = ntcCalibrationStatus() == "ready";
  doc["ntcCalibrationStatus"] = ntcCalibrationStatus();
  doc["fault"] = controller.fault;
  String metrics;
  serializeJson(doc, metrics);
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

  String temp = "Bottom Temp: ";
  temp += dhtTemperature;
  temp += " C";
  display.drawString(20, 10, temp);

  String humidity = "Humidity: ";
  humidity += dhtHumidity;
  humidity += " %";
  display.drawString(20, 20, humidity);

  String chambTempText = "Chamber Temp: ";
  chambTempText += chamberTemperature;
  chambTempText += " C";
  display.drawString(20, 30, chambTempText);

  String fanSpeedText = "Fan Speed: ";
  fanSpeedText += String(pwmDutyPercent(1), 1);
  fanSpeedText += " %";
  display.drawString(20, 40, fanSpeedText);

  display.drawString(20, 50, controller.stateName());

  display.display();
}

// Replies contain a snapshot; notifications contain only changed fields.
void updateMoonrakerStatus(JsonObjectConst status) {
  JsonVariantConst state = status["print_stats"]["state"];
  if (state.is<const char*>()) {
    isPrinting = (strcmp(state.as<const char*>(), "printing") == 0);
  }
  JsonVariantConst temperature = status["temperature_sensor chamber_temp"]["temperature"];
  if (temperature.is<float>()) {
    chamberTemperature = temperature.as<float>();
    if (isfinite(chamberTemperature) && chamberTemperature >= -20 && chamberTemperature <= 150) {
      topSampled = true;
      lastTopSample = millis();
    } else { topSampled = false; chamberTemperature = NAN; }
  }
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
      topSampled = false;
      moonQueryPending = false;
      updatePWM();
      Serial.println("Disconnected from Moonraker");
      break;
    case WStype_CONNECTED:
      Serial.println("Connected to Moonraker");
      topSampled = false;
      moonQueryPending = false;
      lastQuery = 0;
      subscribeMoonraker();
      break;
    case WStype_TEXT:
      JsonDocument doc;
      if (deserializeJson(doc, payloadStr)) {
        Serial.println("Invalid Moonraker JSON");
        break;
      }
      if (doc["error"].is<JsonObject>()) {
        topSampled = false;
        updatePWM();
        Serial.print("Moonraker RPC error: ");
        serializeJson(doc["error"], Serial);
        Serial.println();
        break;
      }
      if (doc["id"] == 2) {
        moonQueryPending = false;
        // A late/malformed snapshot must not make an old reading fresh.
        if (millis() - moonQuerySent > 5000 ||
            !doc["result"]["status"]["temperature_sensor chamber_temp"]["temperature"].is<float>()) {
          topSampled = false;
          break;
        }
      }
      if (doc["method"] == "notify_klippy_shutdown" || doc["method"] == "notify_klippy_disconnected") {
        topSampled = false;
        updatePWM();
        break;
      }
      if (doc["result"]["status"].is<JsonObject>()) {
        updateMoonrakerStatus(doc["result"]["status"].as<JsonObjectConst>());
      } else if (doc["method"] == "notify_status_update"
          && doc["params"][0].is<JsonObject>()) {
        updateMoonrakerStatus(doc["params"][0].as<JsonObjectConst>());
      } else if (doc.containsKey("method") && doc.containsKey("params") && doc["method"] == "notify_gcode_response"
          && doc["params"].is<JsonArray>() && doc["params"].size() > 0) {
        Serial.println("Found gCode history");
        JsonArray params = doc["params"];
        for (const char* param : params) {
          if (!param) continue;
          Serial.println(param);
          // Check if the string contains the desired substring
          const char* prefix = "// Set chamber heater to: ";
          if (!manualMode && strncmp(param, prefix, strlen(prefix)) == 0) {
            double temperature;
            if (!parseNumber(String(param + strlen(prefix)), temperature) || !setTarget(temperature))
              Serial.println("Rejected chamber temperature command");
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

float readNtcTemperature() {
    uint32_t sumMv = 0;
    for (unsigned i = 0; i < NTC_SAMPLES; ++i) {
        sumMv += analogReadMilliVolts(NTC_PIN);
    }
    const float mv = sumMv / static_cast<float>(NTC_SAMPLES);
    // Reject short/open circuits and readings outside the useful ADC range.
    // These are measurement limits, not safe heater temperature limits.
    if (mv < 150.0f || mv > 2450.0f || mv >= NTC_SUPPLY_MV) return NAN;
    const float resistance = NTC_SERIES_OHMS * mv / (NTC_SUPPLY_MV - mv);
    const float inverseKelvin = 1.0f / 298.15f
        + logf(resistance / NTC_NOMINAL_OHMS) / NTC_BETA;
    if (!isfinite(inverseKelvin) || inverseKelvin <= 0.0f) return NAN;
    const float temperature = 1.0f / inverseKelvin - 273.15f;
    return isfinite(temperature) ? temperature : NAN;
}

void updateTempareture() {
  if (localSampled && millis() - lastLocalSample > LOCAL_TIMEOUT_MS) {
    ntcFilter.reset();
    ntcReference.reset();
  }
  ntcRawTemperature = readNtcTemperature();
  const float filtered = ntcFilter.update(ntcRawTemperature);
  ntcTemperature = filtered + ntcOffset;
  const int result = dht.read();
  const float temperature = dht.getTemperature();
  if (result == DHT20_OK && isfinite(temperature) && temperature >= -20 && temperature <= 100) {
    dhtTemperature = temperature;
    dhtHumidity = dht.getHumidity();
  } else { dhtTemperature = NAN; dhtHumidity = NAN; }
  ntcReference.add(filtered, dhtTemperature,
      targetTemperature == 0 && controller.heater == 0 && controller.state != thermal::State::Fault);
  localSampled = true;
  lastLocalSample = millis();
}


void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  pinMode(NTC_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(NTC_PIN, ADC_11db);

  // Establish safe outputs before Wi-Fi or other potentially blocking setup.
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);
  pinMode(FAN_POWER_PIN, OUTPUT);
  digitalWrite(FAN_POWER_PIN, LOW);
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(HEATER_PIN, 0);
  ledcWrite(0, 0);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(FAN_PIN, 1);
  ledcWrite(1, 255); // Cool after a reboot until sensors confirm a cool radiator.

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
  wifiManager.setHttpPort(8080); // Separate from the heater UI on port 80.
  wifiManager.setConnectTimeout(3);
  wifiManager.setConfigPortalBlocking(false);
  if (!wifiManager.autoConnect("Chamber Heater AP")) {
    Serial.println("Wi-Fi portal active; local control continues");
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
  manualMode = preferences.getBool("manualMode", true);
  auto& cfg = controller.config;
  cfg.soft = preferences.getDouble("tcSoft", 0);
  cfg.hard = preferences.getDouble("tcHard", 0);
  cfg.minFan = preferences.getDouble("tcMinFan", 0);
  cfg.maxFan = preferences.getDouble("tcMaxFan", 100);
  cfg.maxHeat = preferences.getDouble("tcMaxHeat", 86);
  cfg.kp = preferences.getDouble("tcKp", 2);
  cfg.ki = preferences.getDouble("tcKi", 0.02);
  cfg.verifySeconds = preferences.getDouble("tcVerify", 600);
  ntcOffset = preferences.getFloat("ntcOffset", 0);
  if (!isfinite(ntcOffset) || fabsf(ntcOffset) > 15) ntcOffset = 0;

  // Initialize the web server
  server.on("/", handleRoot);
  server.on("/setTargetTemp", handleSetTargetTemp);
  server.on("/setMoonrakerConfig", handleSetMoonrakerConfig);
  server.on("/setTemperatureSource", handleSetTemperatureSource);
  server.on("/setManualMode", handleSetManualMode);
  server.on("/setControl", HTTP_POST, handleSetControl);
  server.on("/resetFault", HTTP_POST, handleResetFault);
  server.on("/calibrateNtc", HTTP_POST, handleNtcCalibration);
  server.begin();
  Serial.println("HTTP server started");

  // Attempt initial connection to WebSocket
  webSocket.setReconnectInterval(5000);
  webSocket.onEvent(onEventsCallback);

  // Start WebSocket server
  webSocketServer.begin();
  webSocketServer.onEvent(webSocketEvent);

  updateTempareture();
  updatePWM();
  lastControl = lastUi = millis();
  esp_task_wdt_init(5, true);
  esp_task_wdt_add(nullptr); // Reset if networking or I2C stalls the control loop.
  updateDisplay();
}

void loop() {
  esp_task_wdt_reset();
  const uint32_t now = millis();
  // Sensor acquisition and all state changes run in one task, not Ticker callbacks.
  if (now - lastLocalSample >= 1000) updateTempareture();
  if (now - lastControl >= 200) { lastControl = now; updatePWM(); }
  wifiManager.process();
  webSocketServer.loop();
  webSocket.loop();
  server.handleClient();
  if (!webSocket.isConnected() && now - lastConnectionAttempt >= connectionInterval) {
    lastConnectionAttempt = now;
    attemptConnection();
  }
  // Notifications contain only changes. Query snapshots to refresh unchanged temperatures.
  if (webSocket.isConnected() && !moonQueryPending && now - lastQuery >= 5000) {
    lastQuery = now;
    moonQuerySent = millis();
    moonQueryPending = true;
    webSocket.sendTXT("{\"jsonrpc\":\"2.0\",\"method\":\"printer.objects.query\",\"params\":{\"objects\":{\"temperature_sensor chamber_temp\":[\"temperature\"]}},\"id\":2}");
  }
  if (moonQueryPending && millis() - moonQuerySent > 5000) {
    topSampled = false;
    moonQueryPending = false;
    webSocket.disconnect(); // Prevent late replies being mistaken for a newer query.
    updatePWM();
  }
  if (now - lastUi >= 1000) { lastUi = now; updateMetrics(); updateDisplay(); }
}

void handleRoot() {
  String html;
  html.reserve(24000);
  html = "<!DOCTYPE html><html lang='en'><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'><title>Chamber control</title>";
  html += dashboardStyle;
  html += "</head><body><header><div class='topbar'><div><span class='eyebrow'>Creality K1C / Chamber heater</span><h1>Chamber control</h1></div><span id='connection' class='badge' role='status'>Connecting</span></div>";
  html += "<div class='controls'><form action='/setTargetTemp' method='post'><label for='targetTemp'>Target temperature (C)</label><input required id='targetTemp' type='number' min='0' max='65' step='0.1' name='targetTemp' value='" + String(targetTemperature, 1) + "'><button>Set target</button><small>0 = off. Range 0-65 C. Target resets to zero after reboot.</small></form>";
  html += "<div class='control-actions'><form action='/setTargetTemp' method='post'><input type='hidden' name='targetTemp' value='0'><button class='danger'>Stop heating</button></form>";
  html += "<form action='/resetFault' method='post'><button class='secondary'>Reset fault</button><small>Stop and let the radiator cool first.</small></form></div></div>";
  html += htmlMetrics;
  html += "</header>";
  html += "<details><summary>Thermal control settings</summary><p>Set radiator limits for your assembly and a tested minimum fan PWM. Tachometer is disabled; rotation is not verified.</p><form action='/setControl' method='post'>";
  auto field = [&](const char* name, const char* label, double value,
                   const char* note, const char* placeholder, bool unset) {
    html += "<label for='" + String(name) + "'>" + label + "</label>";
    html += "<input required type='number' step='any' id='" + String(name) +
      "' name='" + name + "' aria-describedby='" + name + "-note' placeholder='" +
      placeholder + "' value='" + (unset ? String("") : String(value, 3)) + "'>";
    html += "<small id='" + String(name) + "-note'>" + note + "</small>";
  };
  field("soft", "Radiator soft limit (C)", controller.config.soft,
    "NTC metal temperature where heater power starts decreasing. Fan boost starts 10 C earlier and reaches 100% here. Minimum 30 C. Example 90 C; enter a limit suitable for your assembly.", "90", controller.config.soft == 0);
  field("hard", "Radiator shutdown limit (C)", controller.config.hard,
    "NTC metal temperature that stops heating and latches a fault. At least 5 C above the soft limit, at most 135 C. Example 110 C; this is not a verified hardware rating.", "110", controller.config.hard == 0);
  field("minFan", "Minimum stable fan PWM (%)", controller.config.minFan,
    "Lowest tested PWM providing reliable airflow through the assembled heater. Range 10-100%; no higher than the normal maximum. PWM does not confirm rotation.", "", controller.config.minFan == 0);
  field("maxFan", "Normal maximum fan PWM (%)", controller.config.maxFan,
    "Upper end of the normal fan curve, between the minimum and 100%. Spin-up and thermal protection can override it to 100%.", "", false);
  field("maxHeat", "Maximum heater PWM (%)", controller.config.maxHeat,
    "Limits heater duty to more than 0 and at most 86%. Start low when commissioning. For a PTC heater, duty is not a fixed percentage of rated watts.", "", false);
  field("kp", "Heater Kp (%/C)", controller.config.kp,
    "Proportional response: PWM percentage points per degree of chamber temperature error. Range greater than 0 up to 100. Higher values respond more strongly; the initial value needs tuning.", "", false);
  field("ki", "Heater Ki (%/C/s)", controller.config.ki,
    "Integral response removes persistent temperature error. Range 0-10; 0 disables integration. Increase cautiously to avoid overshoot; the initial value needs tuning.", "", false);
  field("verify", "Heating verification interval (s)", controller.config.verifySeconds,
    "Range 60-3600 seconds. With continuous heater duty at least 50% and temperature at least 2 C below target, a rise of 1 C is required within this interval or heating faults.", "", false);
  html += "<p>Stop heating before editing. Cooling and spin-up may use 100% fan despite the normal limit.</p><button>Save settings</button></form>";
  html += "</details><details><summary>NTC calibration</summary>";
  html += "<p>Saved correction: " + String(ntcOffset, 2) + " C. Measurements: <span id='ntcCalibrationSamples'>" +
    String(ntcReference.sampleCount()) + "</span>/60. Calibration: <span id='ntcCalibrationStatus'>" +
    ntcCalibrationStatus() + "</span>.</p>";
  html += "<p>Let the heater cool completely to room temperature. DHT20 and the radiator must be at the same temperature. This is a one-point offset, not a calibration of the full NTC curve.</p>";
  html += "<form action='/calibrateNtc' method='post'><label><input type='checkbox' name='equilibrium' value='yes' required> Heater is fully cooled and both sensors are at the same temperature</label><button name='action' value='reference'>Calibrate NTC to DHT20</button></form>";
  html += "<form action='/calibrateNtc' method='post'><button class='secondary' name='action' value='reset'>Reset NTC correction</button></form>";
  html += "</details><details><summary>Connection &amp; temperature source</summary>";
  html += "<h2>Moonraker Configuration</h2>";
  html += "<form action=\"/setMoonrakerConfig\" method=\"POST\">";
  auto escapeHtml = [](String text) {
    text.replace("&", "&amp;"); text.replace("\"", "&quot;");
    text.replace("<", "&lt;"); text.replace(">", "&gt;"); return text;
  };
  html += "Moonraker IP: <input type=\"text\" name=\"ip\" value=\"" + escapeHtml(moonrakerIp) + "\"><br>";
  html += "<small>Printer IPv4 address running Moonraker on port 7125; do not include http:// or a port. Saving stops heating.</small>";
  html += "Auth Token: <input type=\"password\" name=\"auth\" value=\"" + escapeHtml(moonrakerAuth) + "\"><br>";
  html += "<small>Moonraker Bearer token, if required by your server. Leave empty when authentication is not required.</small>";
  html += "<input type=\"submit\" value=\"Save\">";
  html += "</form>";
  html += "<h2>Control temperature sensor</h2>";
  html += "<form action=\"/setTemperatureSource\" method=\"POST\">";
  html += "Bottom air (DHT20): <input type=\"radio\" name=\"source\" value=\"DHT20\" " + String(useDHT20 ? "checked" : "") + "><br>";
  html += "<small>Uses the lower DHT20 air temperature for PI control. Moonraker is optional; the radiator NTC remains required. Saving stops heating.</small>";
  html += "Upper air (Moonraker): <input type=\"radio\" name=\"source\" value=\"Moonraker\" " + String(!useDHT20 ? "checked" : "") + "><br>";
  html += "<small>Uses the upper chamber temperature received from Moonraker. DHT20 and radiator NTC remain required for protection. Missing or stale readings stop heating. Saving stops heating; set a new target afterward.</small>";
  html += "<input type=\"submit\" value=\"Set\">";
  html += "</form>";
  html += "<h2>Target command source</h2>";
  html += "<form action=\"/setManualMode\" method=\"POST\">";
  html += "Web only: <input type=\"radio\" name=\"mode\" value=\"Yes\" " + String(manualMode ? "checked" : "") + "><br>";
  html += "<small>Ignores Moonraker temperature commands; set the target on this page. This selects the command source, not the temperature sensor.</small>";
  html += "Web + Moonraker: <input type=\"radio\" name=\"mode\" value=\"No\" " + String(!manualMode ? "checked" : "") + "><br>";
  html += "<small>Accepts Moonraker chamber setpoint messages. Web controls remain available. The latest accepted command sets the target; a new Moonraker command can restart heating after Stop. Saving stops heating; a new command is required.</small>";
  html += "<input type=\"submit\" value=\"Set\">";
  html += "</form></details>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleSetTargetTemp() {
  double value;
  if (!parseNumber(server.arg("targetTemp"), value) || !setTarget(value)) {
    server.send(400, "text/plain", "Target must be 0..65 C; configure control and clear faults first."); return;
  }
  redirectHome();
}

void handleSetMoonrakerConfig() {
  if (server.hasArg("ip") && server.hasArg("auth")) {
    setTarget(0);
    topSampled = false;
    moonrakerIp = server.arg("ip");
    moonrakerAuth = server.arg("auth");
    preferences.putString("ip", moonrakerIp);
    preferences.putString("auth", moonrakerAuth);
    Serial.println("Moonraker IP set to: " + moonrakerIp);
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
    if (source != "DHT20" && source != "Moonraker") { server.send(400, "text/plain", "Invalid source"); return; }
    setTarget(0);
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
    if (mode != "Yes" && mode != "No") { server.send(400, "text/plain", "Invalid mode"); return; }
    setTarget(0);
    manualMode = (mode == "Yes");
    preferences.putBool("manualMode", manualMode);
    Serial.println("Manual mode: " + String(manualMode ? "Yes" : "No"));
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleSetControl() {
  if (targetTemperature != 0 || controller.state == thermal::State::Heating || controller.state == thermal::State::Spinup) {
    server.send(409, "text/plain", "Stop heating before changing settings"); return;
  }
  thermal::Config cfg;
  if (!parseNumber(server.arg("soft"), cfg.soft) || !parseNumber(server.arg("hard"), cfg.hard) ||
      !parseNumber(server.arg("minFan"), cfg.minFan) || !parseNumber(server.arg("maxFan"), cfg.maxFan) ||
      !parseNumber(server.arg("maxHeat"), cfg.maxHeat) || !parseNumber(server.arg("kp"), cfg.kp) ||
      !parseNumber(server.arg("ki"), cfg.ki) || !parseNumber(server.arg("verify"), cfg.verifySeconds) || !cfg.valid()) {
    server.send(400, "text/plain", "Invalid settings: 30<=soft; soft+5<=hard<=135; 10<=minFan<=maxFan<=100; 0<maxHeat<=86; 0<Kp<=100; 0<=Ki<=10; 60<=verification<=3600."); return;
  }
  controller.config = cfg;
  preferences.putDouble("tcSoft", cfg.soft); preferences.putDouble("tcHard", cfg.hard);
  preferences.putDouble("tcMinFan", cfg.minFan); preferences.putDouble("tcMaxFan", cfg.maxFan);
  preferences.putDouble("tcMaxHeat", cfg.maxHeat); preferences.putDouble("tcKp", cfg.kp);
  preferences.putDouble("tcKi", cfg.ki); preferences.putDouble("tcVerify", cfg.verifySeconds);
  updatePWM();
  redirectHome();
}

void handleNtcCalibration() {
  if (targetTemperature != 0 || controller.heater != 0 || controller.state == thermal::State::Fault ||
      controller.state == thermal::State::Spinup) {
    server.send(409, "text/plain", "Stop heating and clear faults before calibration."); return;
  }
  float correction = 0;
  if (server.arg("action") == "reference") {
    if (server.arg("equilibrium") != "yes") {
      server.send(409, "text/plain", "Confirm the checkbox that the heater is fully cooled and both sensors are at the same temperature."); return;
    }
    const String status = ntcCalibrationStatus();
    if (status != "ready") {
      server.send(409, "text/plain", status); return;
    }
    correction = ntcReference.offset();
    if (!isfinite(correction) || fabsf(correction) > 15) {
      server.send(400, "text/plain", "Correction exceeds +/-15 C. Check the sensor and let the heater cool."); return;
    }
  } else if (server.arg("action") != "reset") {
    server.send(400, "text/plain", "Unknown calibration action."); return;
  }
  if (preferences.putFloat("ntcOffset", correction) != sizeof(float)) {
    server.send(500, "text/plain", "Could not save NTC correction."); return;
  }
  ntcTemperature += correction - ntcOffset;
  ntcOffset = correction;
  ntcReference.reset();
  updatePWM();
  redirectHome();
}

void handleResetFault() {
  if (!controller.reset(controlInput())) {
    server.send(409, "text/plain", "Set target to zero, fix sensors/configuration and let the radiator cool before reset."); return;
  }
  applyOutputs();
  redirectHome();
}

void updatePWM() {
  // The unsmoothed, calibrated reading bypasses filter lag for hard shutdown.
  if (controller.config.valid() && localSampled && millis() - lastLocalSample <= LOCAL_TIMEOUT_MS &&
      isfinite(ntcRawTemperature) && ntcRawTemperature + ntcOffset >= controller.config.hard)
    controller.trip("Radiator overtemperature");
  controller.tick(controlInput(), millis());
  applyOutputs();
}

void subscribeMoonraker() {
  if (webSocket.isConnected()) {
    Serial.println("Subscribing");
    // A new request replaces the subscription. G-code responses need no subscription.
    webSocket.sendTXT("{\"jsonrpc\":\"2.0\",\"method\":\"printer.objects.subscribe\",\"params\":{\"objects\":{\"print_stats\":[\"state\"],\"temperature_sensor chamber_temp\":[\"temperature\"]}},\"id\":1}");
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
