# Control Room Environmental Monitoring & Alarm System

## 📌 Project Overview
This project proposes the design and implementation of a control room environmental monitoring system. Industrial control rooms contain critical equipment such as PLCs, computers, and communication devices that must operate within safe environmental conditions. High temperature or excessive humidity can negatively affect the performance of this equipment and may lead to system failures or permanent damage. 

To address this, the system continuously measures temperature and humidity, displays the data in real-time, and provides immediate visual and audible alerts when unsafe conditions occur.

---

## ✨ Features
* **Real-Time Monitoring:** Measures temperature and humidity using a DHT22 sensor and displays the readings on a 16x2 LCD screen.
* **Dynamic Setpoints:** Allows operators to define safe limits for temperature and humidity on-the-fly using a 4x4 matrix keypad.
* **Automated Alarm System:** Activates a 24V buzzer and an LED indicator immediately if measured values exceed the predefined limits.
* **Signal Smoothing:** Implements a digital low-pass filter to clean up sensor noise and ensure reliable triggering.

![WhatsApp Image 2026-04-11 at 15 03 19](https://github.com/user-attachments/assets/bd9614b7-ff9a-4324-8f35-78637947f35d)

## 🛠️ Hardware Components

| Component | Function |
| :--- | :--- |
| **ESP32-WROOM-32** | The core microcontroller processing sensor data and logic. |
| **DHT22 Sensor** | Measures environmental conditions (temperature & humidity). |
| **Keypad 4x4** | Provides user input for defining alarm setpoints. |
| **LCD 16x2 (I2C)** | Displays system information and real-time data. |
| **5V Relay Module** | Safely controls the 24V alarm circuit. |
| **Buzzer (24V) & LED (24V)** | Serves as the audible and visual alarm indicators. |
| **LM2596 Buck Converter** | Converts the 24V power supply down to a stable 5V for electronic components. |

<img width="1636" height="956" alt="image" src="https://github.com/user-attachments/assets/47e4b951-690a-461f-a7e2-0cfbd01d3b32" />


## ⚙️ Core Engineering Concepts

### 1. Signal Filtering (Exponential Moving Average)
To prevent false alarms caused by sensor noise, the software implements an Exponential Moving Average (EMA) digital filter[cite: 62]. This acts as a software low-pass filter with the equation: 
$y(n)=\alpha x(n)+(1-\alpha)y(n-1)$ 

* **Tuning:** A filter parameter of alpha = 0.7 was selected. 
* **Result:** This configuration provides a fast response time (matching the 2-second minimum sampling interval of the DHT22) while effectively blocking unwanted noise.

### 2. Relay Isolation
The system controls an alarm that operates at 24V, while the ESP32 microcontroller operates at 5V.Directly connecting these could damage the board or create safety hazards.A relay module is used to provide strict electrical isolation, allowing a 5V signal from the ESP32 to safely switch the 24V alarm circuit.

### 3. Voltage Drop Mitigation
Calculations were performed to ensure voltage stability over the sensor wiring.Using a 1.50 mm copper wire over a 2-meter length, the calculated voltage drop was determined to be $29.55 \times 10^{-6}$ V. Since the losses are in the micro-scale, they have no negative effect on sensor accuracy.

---

## 🧪 Testing & Integration
Before finalizing the circuit, every hardware component was tested individually to verify proper operation alongside its corresponding code.Components were then integrated on a breadboard to test the entire system logic. The project was initially simulated using Wokwi before moving to final PCB soldering and assembly.

---

## 🚀 Future Improvements
* **IoT Connectivity:** Connecting the system to the internet to allow remote monitoring via a web dashboard or mobile app. This would allow operators to receive out-of-bounds alerts from anywhere.
* **Automatic Cooling Control:** Integrating the system directly with control room ventilation or air conditioning units. If temperatures exceed the setpoint, the ESP32 could automatically activate cooling systems to restore safe environmental conditions.
