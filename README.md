# ESP32 Chamber Heater for Creality K1C

This project is a standalone ESP32-based chamber heater installed inside a Creality K1C 3D printer, in the lower-left part of the chamber. Its purpose is to heat the chamber and maintain a temperature of approximately **60 °C for printing ASA filament**.

The device combines a heater with PWM power control, a Duokon brushless DC fan (97 × 97 × 33 mm) for air circulation, and a local DHT20 temperature and humidity sensor. Both the heater and fan are powered from 24 V in the installed setup. It also reads the printer's upper chamber temperature through the Moonraker API over WebSockets.

The three temperature readings represent different locations:

- **Bottom of the chamber:** temperature and humidity measured by the device's DHT20 sensor.
- **Top of the chamber:** temperature measured by the printer's chamber sensor and received through Moonraker.
- **Heat exchanger:** NTC 100K B3950 attached to the metal radiator, connected to GPIO34.

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

PWM control of both the heater and fan has been confirmed by the owner to work as intended in the installed 24 V setup. The tachometer is disconnected and is not read by the controller; fan PWM does not confirm rotation.

## Software Components

The project uses PlatformIO with the Arduino framework. The configured environment in `platformio.ini` is `az-delivery-devkit-v4`.

### Libraries

- Arduino/ESP32 components: Wire, WiFi, WebServer, Preferences.
- WiFiManager for Wi-Fi configuration.
- ArduinoJson for JSON messages.
- WebSockets by Markus Sattler (`WebSocketsClient` and `WebSocketsServer`).
- DHT20 by Rob Tillaart.
- I2CScanner by Luis Llamas.
- An SSD1306 OLED library providing `SSD1306Wire.h`, as used in `src/main.cpp`. This dependency is not currently declared in `platformio.ini`.

## Functional Description

### Temperature and Humidity Monitoring

The DHT20 sensor measures temperature and humidity at the bottom of the chamber. The upper chamber temperature is received from the printer through Moonraker. Local sensor readings are also shown on the OLED display.

### PWM Heater and Fan Control

A chamber PI controller requests heater PWM. Radiator temperature limits heater output and increases fan airflow. Fan speed follows the heating load with a configured minimum and slow deceleration. See commissioning and fault behavior below.

### Web Interface

The web interface provides target temperature settings, Moonraker connection settings (IP address and authentication token), and selection between the local DHT20 and Moonraker temperature sources. It displays live upper and lower chamber temperatures, lower chamber humidity, heater duty, and fan duty, with charts updated over WebSockets.

### WebSocket Integration

The device connects to Moonraker over WebSockets and subscribes to temperature and print state events. The current firmware expects the Moonraker object `temperature_sensor chamber_temp` for the upper chamber temperature. Received messages update the chamber temperature and printing status.

### OLED Display

The display shows temperature readings, target temperature, humidity, fan output, and controller state, along with Wi-Fi, Moonraker connection, and printing status indicators.

## NTC Temperature Monitoring

An NTC 100K B3950 thermistor on GPIO34 (IO34 / ADC1) is displayed as
**NTC Temperature** on the web page and as a separate chart line, updated once
per second. It is also required for radiator temperature limiting and cooldown control.

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

## Thermal Control and Commissioning

The controller is implemented in `include/ThermalControl.h`. The existing
heater/fan PID settings are replaced with one PI controller and a fan curve.
Old PID settings are not migrated: the new gains use percent PWM, not 0-255.

On every boot the target is **0 (off)**. Radiator limits and minimum fan PWM
initially have no commissioned values. The UI shows **Setup required** and
rejects heating commands until valid settings are saved. Heater output is
initialized off before networking; the fan initially runs at 100% until local
measurements establish that cooling is unnecessary. A disconnected or out-of-range
NTC therefore leaves the fan running, even with a zero target. In Setup required, the fan stops only after 30 continuous seconds of confirmed cool readings. A hot or invalid reading restarts this timer, preventing rapid on/off cycling near the cooling threshold.

In **Thermal control**, configure:

| Setting | Meaning / accepted range |
| --- | --- |
| Radiator soft limit | Start reducing heater power; at least 30 C |
| Radiator shutdown limit | Latched fault; at least soft + 5 C, at most 135 C |
| Minimum fan PWM | Tested stable airflow in the assembled duct, 10-100% |
| Normal maximum fan PWM | At least minimum PWM, at most 100% |
| Maximum heater PWM | Greater than 0 and at most 86%; start low for commissioning |
| Kp / Ki | Percent/C and percent/(C*s); initial 2 / 0.02 need hardware tuning |
| Heating verification interval | 60-3600 s; default 600 s |

These accepted ranges are software constraints, **not hardware temperature
ratings or proven safe operating values**. Determine radiator limits from the
assembly, thermistor, wiring and mounting specifications. The 135 C ceiling
leaves measurement headroom below the NTC conversion's upper range. With the
current divider, temperatures below approximately 18 C are outside the accepted
ADC range and inhibit heating; do not bypass a sensor fault to cold-start.

To start: verify all local readings, save limits and fan settings, select the
air-temperature source, then explicitly set a target between 0 and 65 C.
For initial PI tuning, set minimum and normal maximum fan PWM to the same
verified value. Then widen the range after logging heater and radiator behavior.
The normal fan curve is linear between these two PWM settings as heater output
increases from zero to the configured maximum. Top/bottom temperature difference
is displayed for diagnosis; it does not automatically increase fan speed.

### States and limits

- **Fan spin-up:** 100% PWM for two seconds, heater off. This is a timed start,
  not proof of airflow; no tachometer interlock is claimed.
- **Heating:** PI runs every 200 ms using actual elapsed time and conditional
  integration to prevent windup while output is limited. Local sensors update
  once per second. Above the radiator soft limit, the heater ceiling decreases
  linearly to zero at the hard limit. Extra fan cooling starts 10 C below the
  soft limit and reaches 100% at the soft limit. Fan increases are immediate;
  normal decreases are limited to two percentage points/s with a 1% deadband.
- **Cooldown:** zero target disables the heater immediately. The fan runs at
  the configured minimum, or 100% if the radiator is hot or local readings are
  unavailable. It stops after 30 continuous seconds with radiator <= bottom air
  + 5 C, radiator below soft - 2 C, and available air readings below 68 C.
  DHT20 must measure representative return air, outside the hot outlet jet.
- **Fault:** heater off, fan 100% until the same cooling condition holds for
  30 seconds. Fault remains latched. Set target to zero, fix the cause, let the
  assembly cool, and press **Reset fault**. Reset never resumes heating.

Spin-up and protective cooling can exceed the normal fan maximum. Both air
sensors are checked for the existing 70 C chamber limit when valid. Required
local sensor failure, selected Moonraker sensor loss, radiator hard limit,
invalid target, and a control-loop delay over 2.5 s latch a fault. If continuous
heater output >=50% while at least 2 C below target fails to raise the selected
air temperature by 1 C within the verification interval, heating also faults.
This diagnostic needs validation on the real chamber and does not detect every
heater/fan failure (in particular it is inactive below 50% output).

### Sources, scheduling and restart behavior

DHT20 and NTC must have valid local readings no older than three seconds.
Moonraker control additionally requires a connected, valid reading no older
than 15 seconds; there is no automatic substitution of the bottom sensor.
Snapshots are requested every five seconds because change notifications can
omit unchanged temperatures. A query timeout or RPC error invalidates the top
reading. When DHT20 controls temperature, loss of the optional top sensor alone
does not stop heating.

**Web only** ignores Moonraker G-code setpoint messages. **Web + Moonraker** mode
accepts the existing `// Set chamber heater to: ...` messages with the same
validation as the web form. The web Stop button works in both modes. Switching
source, mode or Moonraker connection settings stops heating; a new target is
required. Control settings can only be edited with the target at zero.

Sensor reads, PI, web updates and display run sequentially in `loop()`, avoiding
shared I2C/control state in timer callbacks. A five-second task watchdog resets
a stalled loop; boot starts with heat off. The Wi-Fi setup portal is nonblocking
and uses port **8080** (normally `http://192.168.4.1:8080` in AP mode); the heater
UI remains on port 80. The dashboard and canvas charts are served by the ESP32 and work without internet access. Temperature and PWM use separate plots; the latter stays at 0-100%. The browser retains up to 30 minutes of history while the tab is open, with 5/15/30-minute views, line toggles, cursor values, chart pause, and CSV export. Reloading clears history. Live cards continue updating while charts are paused; a stale connection is indicated and reconnected automatically.

Software and the NTC cannot prove airflow or detect every detached-sensor or
failed-power-switch condition. Use reliable sensor mounting and an independent
appropriately rated thermal cutoff. Hardware pull-downs must keep the heater
off during ESP32 reset before firmware runs.

### Validation

Build with `pio run`. Native scenario tests in `test/test_thermal_control.cpp`
exercise spin-up, delayed updates, timer rollover, missing sensors, thermal
limits, latched faults, cooldown, heating verification and PI anti-windup.
With a C++ compiler: `g++ -std=c++11 test/test_thermal_control.cpp -o thermal_test`
and run the resulting executable. These tests do not replace physical tuning
and validation; firmware is not automatically uploaded.


### NTC filtering and DHT20 calibration

NTC readings retain the 16-sample ADC average and use exponential smoothing
(alpha 0.2 at 1 Hz, approximately 4.5 s time constant). Invalid readings clear
the filter immediately. The calibrated unsmoothed reading also triggers the
radiator hard-limit shutdown, so smoothing cannot delay this protection.
The dashboard and normal thermal control use the filtered, corrected reading.

Open **NTC calibration** in the web interface after stopping heating and letting
the entire assembly reach room temperature. The radiator and DHT20 must be at
the same physical temperature; stable readings alone do not establish this.
The controller requires 60 consecutive valid samples with heat off, with NTC
spread <=4 C and DHT20 spread <=0.5 C. The measurement counter (0/60) and readiness update live. At 60/60, the window
continues updating until the readings are stable. To calibrate,
confirm thermal equilibrium, then select **Calibrate NTC to DHT20**.
The correction is the average DHT20 minus uncorrected filtered NTC, limited to
+/-15 C, and is saved across reboots. Recalibrating replaces the correction;
**Reset NTC correction** restores zero. Calibration is blocked during faults.
This is a single-point offset and does not correct the full thermistor curve.
Cooldown still requires radiator <= lower DHT20 + 5 C for 30 seconds, together
with the existing temperature limits.
