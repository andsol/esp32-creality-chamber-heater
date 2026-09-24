

# Control de Calentador PWM para ESP32 con Interfaz Web e Integración WebSocket

Este proyecto implica un microcontrolador ESP32 que controla un calentador y ventilador PWM, lee la temperatura y humedad de un sensor DHT20 y se integra con una impresora 3D a través de la API de Moonraker usando WebSockets. Incluye una interfaz web para establecer la temperatura objetivo y configurar la conexión con la API de Moonraker, así como una pantalla OLED para monitorización en tiempo real.

# Ensamblaje del modelo

![Heater Case](https://github.com/andsol/esp32-creality-chamber-heater/blob/main/media/heater-assembly.png)

## Componentes de hardware:

Microcontrolador ESP32
Sensor de temperatura y humedad DHT20
Calentador PWM
Ventilador PWM (Noctua NF-A4x10 24V)
Pantalla OLED I2C SSD1306
Tacómetro para medir las RPM del ventilador
Conectividad WiFi
Componentes de software
IDE de Arduino

## Bibliotecas:

Wire
Adafruit_SSD1306
WiFi
WebServer
WiFiManager
Preferences
ArduinoJson
HTTPClient
ArduinoWebsockets por Gil Maimon
DHT20 por Rob Tillart

## Descripción funcional
### Monitoreo de temperatura y humedad:
Utiliza el sensor DHT20 para leer la temperatura y humedad actuales.
Muestra estos valores en la pantalla OLED.
### Control del calentador y ventilador PWM:
Controla un calentador y ventilador PWM en función de la temperatura objetivo.
Mide y muestra las RPM del ventilador usando un tacómetro.
### Interfaz web:
Proporciona una página web para configurar la temperatura objetivo, la dirección IP de la API de Moonraker y el token de autenticación.
Permite cambiar entre el sensor DHT20 y la API de Moonraker para las lecturas de temperatura.
### Integración WebSocket:
Se conecta a la API de Moonraker a través de WebSockets.
Se suscribe a eventos de temperatura y estado de impresión.
Actualiza la temperatura actual y el estado de impresión basándose en los mensajes recibidos.
### Pantalla OLED:
Muestra la temperatura actual, temperatura objetivo, humedad, velocidad del ventilador, RPM del ventilador, temperatura de la cámara y estado de impresión.
Muestra iconos que indican los estados de conexión de WiFi y Moonraker.
