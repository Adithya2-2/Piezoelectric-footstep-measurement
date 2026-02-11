#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

hd44780_I2Cexp lcd; // I2C LCD auto-detect

// Pins
const int ADC_PIN  = 32; // ADC1_CH4 for piezo divider midpoint
const int DOUT_PIN = 4;  // indicator LED/buzzer (optional)

// Divider: R1=100k (top), R2=5.6k (bottom)
const float R1 = 100000.0f;
const float R2 = 5600.0f;

// ADC characteristics
const float ADC_MAX = 4095.0f;
const float VREF    = 3.3f;

// Oversampling
const int samples = 16;

// Step detection
float stepThresholdVolts    = 0.3f;  // tune 0.3–1.2 V if needed
unsigned long debounceMs    = 150;
unsigned long releaseHoldMs = 120;

unsigned long lastStepTime  = 0;
bool          armedForNext  = true;
unsigned long belowSince    = 0;
volatile unsigned long stepCount = 0;

bool lcd_ok = false;

// -------- MPU6050 (GY-521) --------
const uint8_t MPU_ADDR = 0x68; // default address
bool mpu_ok = false;

// IMU sampling: run continuously at 200 ms AFTER first step
const unsigned long imuIntervalMs = 200;
unsigned long lastImuSampleMs = 0;
bool imuActive = false; // becomes true at first step

// Partitioning per step
const size_t MAX_SAMPLES_PER_STEP = 128;      // per-step cap
const unsigned long PARTITION_FLUSH_TIMEOUT = 5000; // flush if no next step for 5s

struct ImuSample {
  unsigned long t;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

ImuSample stepBuffer[MAX_SAMPLES_PER_STEP];
size_t stepBufLen = 0;
unsigned long currentPartitionStartMs = 0;

// -------------- Helpers --------------
float readAdcVolts() {
  uint32_t sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(ADC_PIN);
    delayMicroseconds(150);
  }
  float avg = (float)sum / samples;
  return (avg / ADC_MAX) * VREF; // voltage at ADC pin (node)
}

float computeInputVoltage(float vAdc) {
  return vAdc * (R1 + R2) / R2;   // reconstruct Vrect+ from divider
}

void flashIndicator() {
  digitalWrite(DOUT_PIN, HIGH);
  delay(30);
  digitalWrite(DOUT_PIN, LOW);
}

// ----- MPU6050 raw read (no DMP) -----
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuReadBytes(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, (int)len, (int)true) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool mpuInit() {
  // Wake up (clear sleep)
  if (!mpuWrite(0x6B, 0x00)) return false; // PWR_MGMT_1 = 0
  delay(50);
  // Set Gyro ±250 dps, Accel ±2g
  if (!mpuWrite(0x1B, 0x00)) return false; // GYRO_CONFIG
  if (!mpuWrite(0x1C, 0x00)) return false; // ACCEL_CONFIG
  // Optional: low-pass filter
  // mpuWrite(0x1A, 0x03);
  return true;
}

bool mpuReadRaw(ImuSample& s) {
  uint8_t data[14];
  if (!mpuReadBytes(0x3B, data, 14)) return false;
  s.ax = (int16_t)((data[0] << 8) | data[1]);
  s.ay = (int16_t)((data[2] << 8) | data[3]);
  s.az = (int16_t)((data[4] << 8) | data[5]);
  // temp at data[6..7] unused
  s.gx = (int16_t)((data[8] << 8) | data[9]);
  s.gy = (int16_t)((data[10] << 8) | data[11]);
  s.gz = (int16_t)((data[12] << 8) | data[13]);
  s.t  = millis();
  return true;
}

// ----- Partition handling -----
void flushCurrentPartition(const char* reason) {
  if (stepBufLen == 0) return;

  // Emit summary to Serial
  Serial.printf("=== Partition (ended %s) for Step #%lu ===\n", reason, stepCount);
  Serial.printf("Samples: %u\n", (unsigned)stepBufLen);
  if (stepBufLen > 0) {
    unsigned long dur = stepBuffer[stepBufLen - 1].t - stepBuffer[0].t;
    Serial.printf("Duration: %lu ms\n", dur);
  }

  // Example: print first and last 3 samples
  size_t nShow = stepBufLen < 6 ? stepBufLen : 3;
  for (size_t i = 0; i < nShow; i++) {
    auto &s = stepBuffer[i];
    Serial.printf("  [%3u] t=%lu ax=%d ay=%d az=%d gx=%d gy=%d gz=%d\n",
                  (unsigned)i, s.t, s.ax, s.ay, s.az, s.gx, s.gy, s.gz);
  }
  if (stepBufLen > 6) {
    Serial.println("  ...");
    for (size_t i = stepBufLen - 3; i < stepBufLen; i++) {
      auto &s = stepBuffer[i];
      Serial.printf("  [%3u] t=%lu ax=%d ay=%d az=%d gx=%d gy=%d gz=%d\n",
                    (unsigned)i, s.t, s.ax, s.ay, s.az, s.gx, s.gy, s.gz);
    }
  }

  // Reset buffer
  stepBufLen = 0;
  currentPartitionStartMs = millis();
}

void beginNewPartition() {
  stepBufLen = 0;
  currentPartitionStartMs = millis();
}

// ----- Step detection state machine -----
void updateStepState(float vIn) {
  unsigned long now = millis();

  if (vIn >= stepThresholdVolts) {
    if (armedForNext && (now - lastStepTime >= debounceMs)) {
      stepCount++;
      lastStepTime = now;
      armedForNext = false;
      flashIndicator();

      if (!imuActive) {
        // First step: start continuous IMU sampling and a new partition
        imuActive = true;
        lastImuSampleMs = 0; // force immediate sample
        beginNewPartition();
      } else {
        // Subsequent steps: close previous partition and start a new one
        flushCurrentPartition("by next step");
        beginNewPartition();
      }
    }
    belowSince = 0;
  } else {
    if (belowSince == 0) belowSince = now;
    if (!armedForNext && (now - belowSince >= releaseHoldMs)) {
      armedForNext = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nBooting...");

  // I2C on default ESP32 pins
  Wire.begin(21, 22);

  // Initialize LCD
  int status = lcd.begin(16, 2); // auto-detect address and map
  if (status) {
    Serial.print("LCD init failed, status=");
    Serial.println(status);
    lcd_ok = false;
  } else {
    lcd_ok = true;
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Foot+IMU Partition");
    lcd.setCursor(0, 1);
    lcd.print("Init...");
    delay(800);
    lcd.clear();
  }

  pinMode(DOUT_PIN, OUTPUT);
  digitalWrite(DOUT_PIN, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN, ADC_11db); // ~0..3.3V

  // Initialize MPU6050
  mpu_ok = mpuInit();
  Serial.printf("MPU6050 init: %s\n", mpu_ok ? "OK" : "FAIL");

  Serial.println("Setup done.");
}

void loop() {
  // Read piezo voltage and detect steps
  float vAdc = readAdcVolts();
  float vIn  = computeInputVoltage(vAdc);
  updateStepState(vIn);

  // Continuous IMU sampling at 200 ms once active
  unsigned long now = millis();
  if (imuActive && mpu_ok && (now - lastImuSampleMs >= imuIntervalMs)) {
    lastImuSampleMs = now;

    ImuSample s;
    if (mpuReadRaw(s)) {
      if (stepBufLen < MAX_SAMPLES_PER_STEP) {
        stepBuffer[stepBufLen++] = s;
      } else {
        // If buffer full, flush and start a new partition to avoid data loss
        flushCurrentPartition("buffer full");
        beginNewPartition();
        // add current sample to new partition
        stepBuffer[stepBufLen++] = s;
      }
    } else {
      Serial.println("MPU read failed");
    }
  }

  // If active but no next step for a while, flush the current partition
  if (imuActive && stepBufLen > 0 && (now - currentPartitionStartMs >= PARTITION_FLUSH_TIMEOUT)) {
    flushCurrentPartition("timeout");
    beginNewPartition();
  }

  // Serial diagnostics (throttled by LCD update)
  Serial.printf("Vin=%.2f V  Steps=%lu  IMU:%s  PartLen=%u\n",
                vIn, stepCount, (imuActive && mpu_ok) ? "ON" : "OFF", (unsigned)stepBufLen);

  // LCD update
  if (lcd_ok) {
    lcd.setCursor(0, 0);
    lcd.print("Steps:");
    char buf1[16];
    snprintf(buf1, sizeof(buf1), "%6lu", stepCount);
    lcd.setCursor(7, 0);
    lcd.print(buf1);

    lcd.setCursor(0, 1);
    lcd.print("Vgen:");
    char buf2[16];
    snprintf(buf2, sizeof(buf2), "%5.2fV", vIn);
    lcd.setCursor(6, 1);
    lcd.print(buf2);
  }

  delay(100);
}
