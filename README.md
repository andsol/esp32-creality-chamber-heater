# ESP32 Chamber Heater for Creality K1C

This project is a standalone ESP32-based chamber heater installed inside a Creality K1C 3D printer, in the lower-left part of the chamber. Its purpose is to heat the chamber and maintain a temperature of approximately **60 °C for printing ASA filament**.

The device combines a heater with PWM power control, a Duokon brushless DC fan (97 × 97 × 33 mm) for air circulation, and a local DHT20 temperature and humidity sensor. Both the heater and fan are powered from 24 V in the installed setup. It also reads the printer's upper chamber temperature through the Moonraker API over WebSockets.

The two temperature readings represent different locations:

- **Bottom of the chamber:** temperature and humidity measured by the device's DHT20 sensor.
- **Top of the chamber:** temperature measured by the printer's chamber sensor and received through Moonraker.

The web interface allows selecting either temperature source for control, setting the target temperature, and configuring the Moonraker connection. An OLED display provides local monitoring. Approximately 60 °C is the intended operating target, not a verified performance result or the firmware's default setpoint.

## Model Assembly

![Heater Case](https://github.com/andsol/esp32-creality-chamber-heater/blob/main/media/heater-assembly.png)

## Hardware Components

- ESP32 microcontroller with Wi-Fi connectivity.
- DHT20 temperature and humidity sensor in the lower part of the chamber.
- Heater powered from 24 V with PWM power control; its original fan has been removed.
- Duokon Brushless DC Cooling Fan, 97 × 97 × 33 mm, powered from 24 V in the installed setup.
- SSD1306 I2C OLED display.
- Printer chamber temperature sensor in the upper part of the chamber, accessed through Moonraker.

PWM control of both the heater and fan has been confirmed by the owner to work as intended in the installed 24 V setup. The firmware also includes a tachometer input for RPM measurement; RPM measurement requires a compatible tachometer signal.

## Software Components

The project uses PlatformIO with the Arduino framework. The configured environment in `platformio.ini` is `az-delivery-devkit-v4`.

### Libraries

- Arduino/ESP32 components: Wire, WiFi, WebServer, Preferences, and Ticker.
- WiFiManager for Wi-Fi configuration.
- ArduinoJson for JSON messages.
- WebSockets by Markus Sattler (`WebSocketsClient` and `WebSocketsServer`).
- DHT20 by Rob Tillaart.
- PID by Brett Beauregard.
- I2CScanner by Luis Llamas.
- An SSD1306 OLED library providing `SSD1306Wire.h`, as used in `src/main.cpp`. This dependency is not currently declared in `platformio.ini`.

## Functional Description

### Temperature and Humidity Monitoring

The DHT20 sensor measures temperature and humidity at the bottom of the chamber. The upper chamber temperature is received from the printer through Moonraker. Local sensor readings are also shown on the OLED display.

### PWM Heater and Fan Control

The firmware controls heater and fan PWM output based on the selected temperature source and target temperature. It includes a tachometer input for measuring and displaying fan RPM when a compatible signal is connected.

### Web Interface

The web interface provides target temperature settings, Moonraker connection settings (IP address and authentication token), and selection between the local DHT20 and Moonraker temperature sources. It displays live upper and lower chamber temperatures, lower chamber humidity, heater duty, and fan duty, with charts updated over WebSockets.

### WebSocket Integration

The device connects to Moonraker over WebSockets and subscribes to temperature and print state events. The current firmware expects the Moonraker object `temperature_sensor chamber_temp` for the upper chamber temperature. Received messages update the chamber temperature and printing status.

### OLED Display

The display shows temperature readings, target temperature, humidity, fan output, and fan RPM, along with Wi-Fi, Moonraker connection, and printing status indicators.

## NTC Temperature Monitoring

An NTC 100K B3950 thermistor on GPIO34 (IO34 / ADC1) is displayed as
**NTC Temperature** on the web page and as a separate chart line, updated once
per second. It is currently for monitoring only, not heater control or protection.

Wiring: `3V3 -- 47 kOhm resistor -- GPIO34 -- NTC 100K B3950 -- GND`.
Connect a 90-100 nF capacitor between GPIO34 and GND near the ESP32.
Use 3.3 V for the divider, never the 24 V heater supply.

The firmware averages 16 calibrated ADC voltage readings and uses the beta
formula (100 kOhm at 25 C, B = 3950 K). Adjust `NTC_SERIES_OHMS` and
`NTC_SUPPLY_MV` in `src/main.cpp` to the measured resistor and supply values
for calibration. Readings outside 150-2450 mV are reported as `null` over
WebSocket, shown as **Sensor error**, and leave a gap in the chart. This is
approximately an 18-145 C measurement range with the nominal divider;
it is not a permitted operating temperature range for the heater or sensor.
