# Control Room Environmental Monitoring & Alarm System

## 📌 Project Overview
[cite_start]This project proposes the design and implementation of a control room environmental monitoring system[cite: 33]. [cite_start]Industrial control rooms contain critical equipment such as PLCs, computers, and communication devices that must operate within safe environmental conditions[cite: 19]. [cite_start]High temperature or excessive humidity can negatively affect the performance of this equipment and may lead to system failures or permanent damage[cite: 20]. 

[cite_start]To address this, the system continuously measures temperature and humidity, displays the data in real-time, and provides immediate visual and audible alerts when unsafe conditions occur[cite: 22, 34, 36].

---

## ✨ Features
* [cite_start]**Real-Time Monitoring:** Measures temperature and humidity using a DHT22 sensor and displays the readings on a 16x2 LCD screen[cite: 34, 85].
* [cite_start]**Dynamic Setpoints:** Allows operators to define safe limits for temperature and humidity on-the-fly using a 4x4 matrix keypad[cite: 35, 87].
* [cite_start]**Automated Alarm System:** Activates a 24V buzzer and an LED indicator immediately if measured values exceed the predefined limits[cite: 36, 88].
* [cite_start]**Signal Smoothing:** Implements a digital low-pass filter to clean up sensor noise and ensure reliable triggering[cite: 63, 75].

---

## 🛠️ Hardware Components

| Component | Function |
| :--- | :--- |
| **ESP32-WROOM-32** | [cite_start]The core microcontroller processing sensor data and logic[cite: 86]. |
| **DHT22 Sensor** | [cite_start]Measures environmental conditions (temperature & humidity)[cite: 87]. |
| **Keypad 4x4** | [cite_start]Provides user input for defining alarm setpoints[cite: 87]. |
| **LCD 16x2 (I2C)** | [cite_start]Displays system information and real-time data[cite: 85, 87]. |
| **5V Relay Module** | [cite_start]Safely controls the 24V alarm circuit[cite: 88]. |
| **Buzzer (24V) & LED (5V)** | [cite_start]Serves as the audible and visual alarm indicators[cite: 85]. |
| **LM2596 Buck Converter** | [cite_start]Converts the 24V power supply down to a stable 5V for electronic components[cite: 89]. |

---

## ⚙️ Core Engineering Concepts

### 1. Signal Filtering (Exponential Moving Average)
[cite_start]To prevent false alarms caused by sensor noise, the software implements an Exponential Moving Average (EMA) digital filter[cite: 62]. This acts as a software low-pass filter with the equation: 
[cite_start]$y(n)=\alpha x(n)+(1-\alpha)y(n-1)$ [cite: 65]

* [cite_start]**Tuning:** A filter parameter of alpha = 0.7 was selected[cite: 74]. 
* [cite_start]**Result:** This configuration provides a fast response time (matching the 2-second minimum sampling interval of the DHT22) while effectively blocking unwanted noise[cite: 72, 74].

### 2. Relay Isolation
[cite_start]The system controls an alarm that operates at 24V, while the ESP32 microcontroller operates at 5V[cite: 77]. [cite_start]Directly connecting these could damage the board or create safety hazards[cite: 78]. [cite_start]A relay module is used to provide strict electrical isolation, allowing a 5V signal from the ESP32 to safely switch the 24V alarm circuit[cite: 79, 80].

### 3. Voltage Drop Mitigation
[cite_start]Calculations were performed to ensure voltage stability over the sensor wiring[cite: 41]. [cite_start]Using a 1.50 mm copper wire over a 2-meter length, the calculated voltage drop was determined to be $29.55 \times 10^{-6}$ V[cite: 46, 53]. [cite_start]Since the losses are in the micro-scale, they have no negative effect on sensor accuracy[cite: 53].

---

## 🧪 Testing & Integration
[cite_start]Before finalizing the circuit, every hardware component was tested individually to verify proper operation alongside its corresponding code[cite: 140, 141]. [cite_start]Components were then integrated on a breadboard to test the entire system logic[cite: 142]. [cite_start]The project was initially simulated using Wokwi before moving to final PCB soldering and assembly[cite: 128, 143].

---

## 🚀 Future Improvements
* [cite_start]**IoT Connectivity:** Connecting the system to the internet to allow remote monitoring via a web dashboard or mobile app[cite: 234]. [cite_start]This would allow operators to receive out-of-bounds alerts from anywhere[cite: 235].
* [cite_start]**Automatic Cooling Control:** Integrating the system directly with control room ventilation or air conditioning units[cite: 236]. [cite_start]If temperatures exceed the setpoint, the ESP32 could automatically activate cooling systems to restore safe environmental conditions[cite: 237].
