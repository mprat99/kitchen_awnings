# 🏡 Kitchen Awnings Project

Welcome to the Kitchen Awnings repository — a multidisciplinary IoT project that blends mechanics, electronics, 3D printing, battery management, and embedded systems to create a smart, solar-powered awning control system.

## 🚀 Overview

This repository contains the Arduino code for an ESP32-based controller that automates and remotely manages kitchen awnings. Designed for energy efficiency and responsive automation, the system integrates sensorless motor control, weather-based behavior, and a web interface for seamless user interaction.

## 🎯 Features

- **Remote Control**: Operate awnings with `Up`, `Down`, and `Stop` commands via a hosted web interface or API calls.
- **Scheduling**: Set timed operations to raise or lower awnings automatically.
- **Rain Detection**: Awnings automatically lower when rain is detected.
- **Battery Optimization**: Disconnects the solar charger between sunset and sunrise to conserve battery life.
- **Sensorless Homing**: Uses StallGuard4 from TMC2209 stepper drivers for precise, sensor-free positioning.
- **Web Interface**: ESPAsyncWebServer hosts a responsive webpage for control and configuration.
- **File System**: HTML/CSS assets are stored and served using LittleFS.

## 🧰 Technologies & Libraries

| Component              | Purpose                                                                 |
|------------------------|-------------------------------------------------------------------------|
| **ESP32**              | Main microcontroller running the Arduino code                          |
| **TMC2209**            | Stepper motor drivers with StallGuard4 for sensorless homing           |
| **FastAccelStepper**   | Smooth and precise stepper control                                      |
| **WiFi**               | Enables network connectivity                                            |
| **TMCStepper**         | Direct UART control of TMC2209 drivers                                 |
| **ESPAsyncWebServer**  | Hosts the control webpage and handles API requests                      |
| **LittleFS**           | Stores and serves HTML/CSS files from ESP32                            |

## 🛠️ Hardware Highlights

- Custom-designed mechanical awning system
- 3D-printed components for housing and mounts
- Solar panel with smart charging logic
- Battery management circuitry
- Rain sensor integration
- Stepper motors with TMC2209 drivers

## 🌐 Web Interface

The ESP32 serves a lightweight webpage for manual control and scheduling. You can also interact with the system programmatically via RESTful API endpoints.

## 📁 File Structure

```
├── src/
│   └── kitchen_awnings.ino           # Core Arduino logic
├── data/
│   └── index.html         # Web interface
│   └── style.css          # Styling for the interface
├── README.md              # Project documentation
```

## ⚡ Getting Started

1. Flash the `kitchen_awnings.ino` to your ESP32 using the Arduino IDE or PlatformIO.
2. Upload the `data/` folder to LittleFS using the appropriate uploader tool.
3. Connect the ESP32 to your Wi-Fi network.
4. Access the web interface via the device’s IP address.

## 📅 Future Improvements

- Wind sensor integration for storm protection
- Mobile app interface
- OTA firmware updates
