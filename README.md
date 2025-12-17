# 🎙️ ESP32 Voice-Controlled Smart Hub 🏠

[![Platform: ESP32](https://img.shields.io/badge/Platform-ESP32-blue?logo=espressif)](https://www.espressif.com/en/products/socs/esp32)
[![Language: C++](https://img.shields.io/badge/Language-C++-green?logo=c%2B%2B)](https://www.arduino.cc/)
[![Status: Functional](https://img.shields.io/badge/Status-Functional-brightgreen)](#)

This project transforms a standard room into a voice-activated smart space. Using an **ESP32**, it monitors environmental data, controls appliances via **Relays**, and interacts with entertainment systems through **Infrared (IR)**.

---

## ✨ Features

* **🗣️ Voice Commands:** "Turn on the light" triggers the physical relay via voice assistant integration.
* **🌡️ Live Telemetry:** Real-time Temperature & Humidity monitoring via DHT sensor.
* **📺 Universal Remote:** Control your TV, AC, or Fans using the built-in IR transmitter.
* **🔗 IoT Ready:** Seamless Wi-Fi connectivity for remote access and monitoring.

---

## 🛠️ The Hardware Stack

| Component | Role |
| :--- | :--- |
| **ESP32 DevKit V1** | The central brain with Wi-Fi/Bluetooth capabilities. |
| **DHT11/22** | Digital sensor for tracking ambient temperature and humidity. |
| **Relay Module** | The electronic switch for the high-voltage light bulb. |
| **IR Transmitter** | Sends HEX codes to control IR-based appliances. |

---

## 🔌 Wiring & Pins

> [!CAUTION]
> **High Voltage Warning:** Please be extremely careful when wiring the light bulb to the relay. Ensure the main power is disconnected during assembly.

| ESP32 Pin | Component Pin | Function |
| :--- | :--- | :--- |
| **3V3** | VCC (DHT/IR) | Power Supply |
| **GND** | GND (All) | Ground |
| **D4** | Data (DHT) | Temp/Humidity Reading |
| **D5** | IN (Relay) | Light Switch Control |
| **D18** | DAT (IR Send) | Infrared Signal Output |

---

## 🚀 Installation & Setup

### 1. Required Libraries
Install these via the **Arduino Library Manager**:
* **DHT sensor library** (by Adafruit)
* **IRremoteESP8266** (by David Conran)
* **SinricPro** (or your preferred Voice Assistant library)
* **Adafruit Unified Sensor** (Dependency for DHT)

### 2. Configuration
Update the following variables in your main code to match your local network:
* **SSID:** Your Wi-Fi Name
* **Password:** Your Wi-Fi Password
* **Pin 5:** Relay Output
* **Pin 4:** DHT Input

---

## 🎮 How It Works

1. **Boot Up:** The ESP32 initializes the DHT sensor and IR transmitter, then connects to Wi-Fi.
2. **Voice Trigger:** When you say "Turn on the light," the command is processed and sent to the ESP32.
3. **Action:** The ESP32 pulls **GPIO 5** HIGH, closing the relay and lighting the bulb.
4. **Environment:** The DHT sensor logs temperature and humidity data to your serial monitor or IoT dashboard.

