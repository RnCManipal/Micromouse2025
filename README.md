# 🐭 Micromouse 2025

An autonomous maze-solving robot built for the **Micromouse competition**, capable of navigating a maze using **PID-based motion control**, **sensor fusion**, and the **Floodfill algorithm**.  

---

## 📌 Features
- Precise movement using encoders, PID control, and IMU-based orientation.  
- Wall detection and alignment using **ToF sensors**.  
- Pathfinding with **Floodfill algorithm**.  
- Error handling for sensor drift and unexpected conditions.  
- Modular C++ code with clean separation of movement, sensing, and pathfinding.  

---

## 2.1.1 System Overview

The software is divided into **5 C++ files** and **4 header files**:

- **Config.h** → Centralizes pin mappings, sensor/display includes, constants, global variables, and PID parameters for a robotics project using motors, encoders, ToF sensors and IMUs.  
- **Sensors.h** → Declares global sensor objects, encoder counters, an Euler struct, and function prototypes for handling IMU, ToF sensors, encoders, and button input.  
- **Movement.h** → Defines constants, PID parameters, and function prototypes for motor control, PID-based forward/rotational movement, turning and braking in the robot’s motion system.  
- **Floodfill.h** → Declares global variables and function prototypes for maze-solving using the flood fill algorithm, including path storage, wall/arena mapping, path reduction, cost calculation, direction handling, and final run execution.  

**Source Files (.cpp / .ino):**
- **config.cpp** → Defines global variables, constants, and PID parameters declared in `config.h`.  
- **sensors.cpp** → Implements IMU initialization, ToF setup, encoder interrupts, and button handling.  
- **movement.cpp** → Implements motion logic, PID-based forward movement, yaw correction, wall-following, and turning functions.  
- **floodfill.cpp** → Implements maze-solving algorithm, path reduction, cost calculation, and direction handling.  
- **main.ino** → Entry point of the program, initializes sensors/motors/IMU, sets fixed directions, and runs either floodfill or wall-follower based on button input.  

---

## 2.1.2 Modules and Functionalities

- **Movement Module** → Provides forward/rotational movement, turning, braking, and motor speed control. Uses PID for yaw correction and wall distance correction.  
- **Sensor Module** → Handles IMU readings, ToF distance measurements, encoder ticks, and button inputs. Provides fused sensor data to support movement.  
- **Floodfill Module** → Maze-solving logic using the floodfill algorithm. Maintains wall map, assigns costs, reduces path, and generates final optimal path.  
- **Main Control Loop** → Initializes the system, waits for user input, and triggers either floodfill maze-solving or wall-follower navigation.  

---

## 2.2 Development Setup

### 2.2.1 Programming Languages & Tools
- **Languages**: C++ (Arduino framework)  
- **IDE**: Arduino IDE  
- **Microcontroller**: STM32 Blackpill F411CE  
- **Programmer**: DFU (Device Firmware Upgrade via USB)  

### 2.2.2 GitHub Repository & Branches
Repo: [Micromouse2025](https://github.com/RnCManipal/Micromouse2025)  

Branches:  
- **main** → Stable working version  
- **secondary** → Experimental features  

### 2.2.3 Libraries Used
- **Adafruit BNO08x** (v1.2.3) – IMU sensor library  
- **VL53L0X / VL6180X** – Time-of-Flight sensor libraries  
- **Wire.h** – I2C communication  
- **Arduino Core STM32** – Board support package  

---

## 2.3 Core Algorithms

### 2.3.1 PID Control
- Used for straight movement (yaw correction).  
- Wall-following uses PID with ToF sensors.  
- Rotation is stabilized with IMU feedback.  

### 2.3.2 Floodfill Algorithm
- Each maze cell is assigned a cost relative to the goal.  
- The bot moves to the neighbor with the lowest cost.  
- Path reduction is applied to minimize unnecessary moves.  
- Final run executes optimized shortest path.  

---

## 2.4 Testing

### 2.4.1 Testing Environment
- **Hardware**:  
  - STM32 Blackpill F411CE  
  - BNO085 IMU  
  - 3× VL6180X ToF Sensors  
  - Encoders & Motor driver  

- **Arena**: Standard **6×6 or 16×16 Micromouse maze**  

# ⚙️ STM32 BlackPill F411CE Setup & Programming Guide

This guide explains how to set up and program the **STM32 BlackPill F411CE** using the Arduino IDE.

---

## 🔧 Installation & Setup

1. Open **Arduino IDE**.  
2. Go to **File > Preferences** and paste the following URL into *Additional Board Manager URLs*:  
3. Open **Boards Manager** (Tools > Board > Boards Manager) and search for **STM32**.  
- Install the package: **STM32 Cores by STMicroelectronics**.  
4. Install **STM32CubeProgrammer (STM32Prog)** from ST’s official site:  
👉 [Download STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)  
5. Select the board in Arduino IDE:  
- **Tools > Board > STM32 Boards > Generic STM32F4x series**  
- **Board Part Number**: `BlackPill F411CE`  
- **USB Support**: `CDC (Generic 'Serial' supersede U(S)ART)`  
- **Upload Method**: `DFU`  

---

## 🚀 How to Program

1. Connect the board via USB.  
2. Put the board in DFU mode:  
- Press and hold the **BOOT0** button.  
- Press and release the **NRST (reset)** button.  
- Release the **BOOT0** button.  
3. In Arduino IDE, click **Upload**.  
4. Code will be flashed to the STM32.  

---

## ✅ Expected Output

- The board should appear as a **DFU device** when in bootloader mode.  
- After uploading, the STM32 will reset and run the uploaded sketch.  
- Use **Serial Monitor** (set to 115200 baud or project-specific rate) to verify logs/output.  

---


