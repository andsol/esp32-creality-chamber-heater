# ESP32 PWM Heater Control with Web Interface and WebSocket Integration

This project involves an ESP32 microcontroller controlling a PWM heater and fan, reading temperature and humidity from a DHT20 sensor, and integrating with a 3D printer via the Moonraker API using WebSockets. It includes a web interface for setting target temperatures and configuring the Moonraker API connection, and an OLED display for real-time monitoring.

# Model assemby

![Heater Case](https://github.com/andsol/esp32-creality-chamber-heater/blob/main/media/heater-assembly.png)

## Hardware Components:

ESP32 microcontroller
DHT20 temperature and humidity sensor
PWM heater
PWM fan (Noctua NF-A4x10 24V)
SSD1306 I2C OLED display
Tachometer for fan RPM measurement
WiFi connectivity
Software Components
Arduino IDE

## Libraries:

Wire
Adafruit_SSD1306
WiFi
WebServer
WiFiManager
Preferences
ArduinoJson
HTTPClient
ArduinoWebsockets by Gil Maimon
DHT20 by Rob Tillart

## Functional Description
### Temperature and Humidity Monitoring:
Uses the DHT20 sensor to read current temperature and humidity.
Displays these values on the OLED display.
### PWM Heater and Fan Control:
Controls a PWM heater and fan based on the target temperature.
Measures and displays fan RPM using a tachometer.
### Web Interface:
Serves a web page to configure target temperature, Moonraker API IP address, and authentication token.
Allows switching between DHT20 sensor and Moonraker API for temperature readings.
### WebSocket Integration:
Connects to the Moonraker API via WebSockets.
Subscribes to temperature and print state events.
Updates the current temperature and printing status based on received messages.
### OLED Display:
Shows current temperature, target temperature, humidity, fan speed, fan RPM, chamber temperature, and printing status.
Displays icons indicating WiFi and Moonraker connection statuses.
