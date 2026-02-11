
---

## ▶️ How to Run

### 1. Install Arduino IDE
Download and install Arduino IDE.

### 2. Install ESP32 Board Support
- Open Arduino IDE
- Go to: Preferences → Additional Board Manager URLs
- Add ESP32 board URL
- Install **ESP32 by Espressif Systems**

### 3. Install Required Libraries
Install these libraries in Arduino IDE:
- `Wire`
- `hd44780` (for I2C LCD)

### 4. Upload the Code
Open: code/Footstep_IMU_Partition.ino


Select:
- Board: ESP32 Dev Module
- Correct COM Port

Then click **Upload**.

---

## 📌 Notes on Threshold Tuning

If steps are not detected:
- decrease `stepThresholdVolts`

If false steps occur:
- increase `stepThresholdVolts`
- increase `debounceMs`

---



