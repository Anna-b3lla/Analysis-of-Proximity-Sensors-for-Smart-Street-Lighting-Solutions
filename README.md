# Analysis-of-Proximity-Sensors-for-Smart-Street-Lighting-Solutions  

This repository is dedicated to my final-year thesis project titled **"Analysis of Proximity Sensors for Smart Street Lighting Solutions"**, completed at **Savonia University of Applied Sciences**. The project explores the design, development, and evaluation of an IoT-based smart lighting system utilizing proximity sensors and an ESP32 microcontroller. The goal is to improve streetlight automation and energy efficiency, laying the groundwork for scalable smart city infrastructure.  

---

## Project Overview  

Modern cities face growing demands for energy-efficient and intelligent solutions for public utilities. This project demonstrates a **smart street lighting system** that operates based on real-time sensor data, ensuring that streetlights are illuminated only when necessary.  

Using various sensors such as **PIR**, **ultrasonic**, **radar**, and **LaserPing**, the system detects pedestrians and vehicles, dynamically adjusting the lighting. The project also includes the integration of multiple hardware and software components to achieve a reliable, modular, and scalable design.  

The code, located in the main repository, handles sensor data collection, decision-making, and communication with the ESP32. It can be accessed [Project.ino](Project.ino).

---

## Features  

- **Dynamic Lighting Control**: Lights are activated only when proximity sensors detect motion or presence within a specified range.  
- **Multi-Sensor Integration**: Combines the capabilities of PIR, ultrasonic, radar, and LaserPing sensors for robust detection.  
- **Energy Efficiency**: Reduces unnecessary power usage by keeping streetlights off when no activity is detected.  
- **Customizable Thresholds**: Users can adjust sensitivity and range for specific environments, such as highways or residential areas.  
- **Modular Architecture**: Designed for easy scalability, allowing the addition of more sensors or communication protocols.  
- **Real-Time Performance**: Sensor readings are processed and acted upon instantaneously for seamless operation.  

---

## Hardware Used  

- **ESP32 Microcontroller**: The core processing unit, handling sensor data collection and lighting control logic.  
- **PIR Sensor**: Detects human motion for nearby activities.  
- **Ultrasonic Sensor**: Measures distances to objects like vehicles for broader coverage.  
- **LaserPing Sensor**: Provides precise short-range measurements.  
- **RD03 Radar Sensor**: Enables long-range and high-reliability motion detection.  
- **Additional Components**: Breadboard, jumper wires, and a regulated power supply.  

---

