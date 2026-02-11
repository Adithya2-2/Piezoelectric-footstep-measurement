# Piezoelectric-footstep-measurement

# TapOrWalk-Detector (ESP32 + Piezoelectric + MPU6050)

This project implements a low-cost wearable footstep measurement system using:

- **3 Piezoelectric plates** (2 at heel + 1 at toe)
- **MPU6050 IMU** (3-axis accelerometer + 3-axis gyroscope)
- **ESP32 microcontroller**
- **16×2 I2C LCD**

The system detects and counts steps, distinguishes between **normal walking** and **foot tapping**, and logs **per-step IMU motion partitions** through the Serial Monitor.

---

## 🎯 Objective

To build a wearable prototype that can:

1. Detect and count steps from a single foot
2. Detect rhythmic foot tapping and suppress false step counting
3. Collect IMU motion data for each step and partition it step-wise
4. Display real-time step count + voltage feedback on LCD
5. Output structured diagnostic data through Serial Monitor / Serial Plotter

---

## 🧠 Core Concept (How It Works)

### 1) Piezoelectric Foot Contact Sensing
When the foot hits the ground, each piezoelectric plate generates a voltage spike.

- 2 sensors are placed at the heel
- 1 sensor is placed near the toe / forefoot

These signals represent foot-ground contact intensity.

---

### 2) Signal Conditioning (Safety + Stability)
Piezo sensors output high-voltage spikes and AC signals.  
So the system uses:

- **Rectification** (convert to unipolar signal)
- **Resistor divider** (to protect ESP32 ADC input)

The divider used in this project:

- **R1 = 100 kΩ**
- **R2 = 5.6 kΩ**

This ensures the ADC always stays safe.

---

### 3) Step Detection (State Machine Logic)
A step is detected when the reconstructed voltage exceeds a threshold:

- **stepThresholdVolts = 0.3 V** (can be tuned)
- **debounceMs = 150 ms**
- **releaseHoldMs = 120 ms**

This ensures:
- no double counting
- no false triggers due to noise

---

### 4) IMU Motion Logging (Per-Step Partitioning)
After the first valid step is detected:

- IMU sampling is activated
- IMU samples are taken every **200 ms**
- Samples are stored into a buffer for the current step

A step partition ends when:
- the next step occurs, OR
- 5 seconds passes with no next step

Each partition is printed to Serial as a structured block.

---

## 🧾 What the User Sees

### LCD Output
- Line 1: `Steps: N`
- Line 2: `Vgen: X.XXV`

### Serial Monitor Output
The serial output prints:
- reconstructed piezo voltage (`Vin`)
- total step count
- IMU status (ON/OFF)
- current partition buffer size

---

## ⚙️ Implementation Details (From Code)

The firmware performs:

### Piezo Sampling
- 12-bit ADC sampling (ESP32)
- 16× oversampling for noise reduction

### Voltage Reconstruction
Reconstructed piezo voltage is calculated as:

Vin = Vadc × (R1 + R2) / R2


---

## 🧩 Components Used (With Specifications)

### 1) ESP32 Development Board
- Dual-core microcontroller
- 12-bit ADC
- 3.3V logic
- Built-in Wi-Fi (not required for this implementation)

### 2) Piezoelectric Sensors (3 units)
- Type: Piezoelectric plates/discs
- Placement:
  - 2 × heel
  - 1 × toe/forefoot
- Purpose: Foot contact voltage generation

### 3) MPU6050 IMU (GY-521 Module)
- 6-axis IMU:
  - 3-axis accelerometer
  - 3-axis gyroscope
- I2C interface
- Default address: `0x68`
- Config used:
  - Accel: ±2g
  - Gyro: ±250 dps

### 4) 16×2 I2C LCD Display
- 16 columns × 2 rows
- I2C backpack interface
- Used for step + voltage display

### 5) Resistor Divider (Signal Protection)
- R1 = 100 kΩ
- R2 = 5.6 kΩ
- Used to scale piezo voltage before ADC

### 6) Optional LED / Buzzer Indicator
- GPIO pin: `DOUT_PIN = 4`
- Flashes for each detected step

---

## 🔌 Wiring Summary

### Piezo Input
- Piezo rectified + divided output → ESP32 ADC pin `GPIO 32`

### I2C Devices
- SDA → GPIO 21
- SCL → GPIO 22
- MPU6050 + LCD share the same I2C bus

---


