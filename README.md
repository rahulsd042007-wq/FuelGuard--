# 🚛 FuelGuard+ — Smart Fuel Monitoring & Vehicle Theft Detection

### An ESP32-Based Smart Fuel Monitoring and Anti-Theft System

FuelGuard+ is an embedded systems project designed to monitor fuel levels, detect suspicious vehicle activity, and alert users when potential fuel theft or refuelling is detected.

The system uses an ESP32 microcontroller with a fuel-level probe, vibration sensors, GPS, and GSM communication to monitor fuel-related events and provide location information.

---

## 🎯 Problem Statement

Fuel theft and unauthorised fuel removal can cause financial losses for vehicle owners and fleet operators. Conventional fuel indicators may not provide sufficient information to identify suspicious activity or locate a vehicle when an incident occurs.

## 💡 Proposed Solution

FuelGuard+ combines fuel-level monitoring, vehicle-status detection, vibration sensing, GPS positioning, and GSM-based notifications into a single embedded system.

The system analyses fuel-level changes and vehicle activity to help identify suspicious events.

## ⚙️ How It Works

1. **Fuel-Level Monitoring:** The fuel-level probe measures changes in the mock tank's liquid level.
2. **Vehicle Status Detection:** The voltage sensor helps determine whether the vehicle is ON or OFF.
3. **Suspicious Activity Detection:** Vibration sensors help identify possible unauthorised activity when the vehicle is OFF.
4. **Fuel Theft Detection:** A suspicious fuel-level decrease can trigger a potential theft alert.
5. **Refuelling Detection:** A stable increase in fuel level can indicate refuelling.
6. **Location Tracking:** The GPS module provides location information when a valid fix is available.
7. **SMS Notifications:** The GSM module can send alerts to a configured phone number.

*Actual detection performance depends on sensor calibration, hardware integration, and the implemented firmware logic.*

## ✨ Key Features

* Real-time fuel-level monitoring
* Vehicle ON/OFF status detection
* Vibration-based activity monitoring
* Potential fuel theft alerts
* Refuelling event detection
* GPS-based location information
* GSM/SMS notification support
* ESP32-based embedded implementation

## 🔧 Hardware Components

| Component               | Purpose                                    |
| ----------------------- | ------------------------------------------ |
| ESP32-WROOM-32          | Main controller                            |
| Fuel-Level Probe        | Measures the mock tank's liquid level      |
| Voltage Sensor          | Helps determine vehicle status             |
| SW-420 Vibration Sensor | Detects vibration events                   |
| MPU6050                 | Measures acceleration and angular velocity |
| NEO-6M GPS              | Provides geographic location               |
| SIM800L GSM Module      | Supports SMS notifications                 |
| Mock Tank               | Demonstrates fuel-level changes            |

## 🧠 Technology Stack

* **Microcontroller:** ESP32-WROOM-32
* **Programming Environment:** Arduino IDE
* **Language:** Embedded C/C++ using the Arduino framework
* **Communication:** UART, I²C and GPIO, as applicable to the connected modules
* **Positioning:** GPS
* **Wireless Alerts:** GSM/SMS

## 🔄 System Workflow

**Sense → Analyse → Detect → Locate → Alert**

The ESP32 reads the sensors, evaluates vehicle status and fuel-level changes, identifies potential events, and uses the available GPS and GSM modules to provide location information and notifications.

## 🚀 Project Applications

* Commercial vehicle fuel monitoring
* Fleet management prototypes
* Fuel theft detection research
* Embedded systems and IoT demonstrations
* Vehicle security projects

## 📂 Project Files

The repository contains the Arduino firmware and project documentation.

Additional files, such as circuit diagrams, calibration data, and demonstration images, can be added as the project develops.

## 🔮 Future Improvements

* Web-based monitoring dashboard
* Historical fuel-consumption analysis
* Improved fuel-volume calibration
* Enhanced theft-detection algorithms
* Fleet-level monitoring and reporting

## 👨‍💻 Project Status

**Prototype / Academic Project**

FuelGuard+ is being developed and tested as an embedded-system prototype. Detection accuracy and real-world vehicle compatibility require appropriate calibration and validation.

## 📜 License

A suitable open-source license can be added when the project distribution terms are decided.
