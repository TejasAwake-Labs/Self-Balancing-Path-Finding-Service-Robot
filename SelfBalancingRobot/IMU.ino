// =============================================================================
// File: IMU.ino
// Project: ESP32 Self-Balancing Robot
// Description: Raw I2C driver for MPU9250 / MPU6500 / MPU6050 6-DOF IMUs.
//              Implements hardware configuration, initial 500-sample gyro Y-bias
//              calibration, and complementary filter sensor fusion for pitch angle.
// =============================================================================

#include <Wire.h>
#include <math.h>

// -------- PIN DEFINITIONS & I2C ADDRESS --------
#define SDA_PIN   21    // ESP32 default I2C SDA pin
#define SCL_PIN   22    // ESP32 default I2C SCL pin
#define MPU_ADDR  0x68  // I2C address of MPU9250 / MPU6050 (AD0 pulled LOW)

// -------- MPU REGISTER ADDRESSES --------
#define REG_CONFIG        0x1A  // Digital Low Pass Filter (DLPF) configuration
#define REG_GYRO_CONFIG   0x1B  // Gyroscope full-scale range configuration
#define REG_ACCEL_CONFIG  0x1C  // Accelerometer full-scale range configuration
#define REG_ACCEL_XOUT_H  0x3B  // First register of 14-byte burst read
                                 // (Accel X, Y, Z, Temp, Gyro X, Y, Z)
#define REG_PWR_MGMT_1    0x6B  // Power management 1 (sleep/wake control)
#define REG_WHO_AM_I      0x75  // Device identification register

// -------- FILTER & CALIBRATION CONSTANTS --------
const float ALPHA               = 0.98f;  // Complementary filter weight
                                           // (98% gyro integration, 2% accelerometer)
const float GYRO_SCALE          = 131.0f; // Sensitivity: 131.0 LSB per deg/s at +/-250 dps
const int   CALIBRATION_SAMPLES = 500;    // Stationary samples for gyro Y zero-rate bias calibration

// -------- EXPORTED SENSOR STATE --------
float angle    = 0.0f;  // Pitch angle estimate in degrees (used by SelfBalancingRobot.ino)
float gyroRate = 0.0f;  // Angular velocity about pitch axis in deg/s

// -------- LOCAL STATE --------
static float gyroBias = 0.0f;  // Calibrated zero-rate offset for Y-axis gyro in deg/s

// -------- LOW-LEVEL I2C HELPERS --------

// Writes a single byte to a sensor register. Returns true on success.
static bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

// Reads multiple contiguous bytes from a sensor register. Returns true on success.
static bool readRegs(uint8_t reg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;  // I2C NACK or bus error
  }
  if (Wire.requestFrom((uint8_t)MPU_ADDR, length, (uint8_t)true) != length) {
    return false;  // Did not receive expected byte count
  }
  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = Wire.read();
  }
  return true;
}

// Reconstructs a signed 16-bit integer from two sequential bytes (big-endian MSB first)
static inline int16_t makeInt16(uint8_t *buf, int index) {
  return (int16_t)((buf[index] << 8) | buf[index + 1]);
}

// -------- IMU INITIALIZATION & CALIBRATION --------

// Configures I2C, wakes the MPU9250, sets measurement ranges and DLPF,
// then performs a 500-sample gyro Y-axis bias calibration.
// Also seeds the complementary filter with the initial accelerometer angle.
void imuSetup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);  // 400 kHz fast I2C for minimum read latency

  // Verify sensor presence via WHO_AM_I register
  uint8_t whoAmI = 0;
  if (!readRegs(REG_WHO_AM_I, &whoAmI, 1)) {
    Serial.println(F("# ERROR: MPU IMU not responding at I2C address 0x68!"));
    while (true) { delay(1000); }
  }
  Serial.printf("# IMU detected! WHO_AM_I = 0x%02X\n", whoAmI);

  // Wake sensor from sleep mode (clear SLEEP bit in PWR_MGMT_1)
  writeReg(REG_PWR_MGMT_1, 0x00);
  delay(100);

  // Gyroscope full-scale range: +/- 250 dps -> sensitivity = 131.0 LSB/(deg/s)
  writeReg(REG_GYRO_CONFIG, 0x00);

  // Accelerometer full-scale range: +/- 2g -> sensitivity = 16384 LSB/g
  writeReg(REG_ACCEL_CONFIG, 0x00);

  // Digital Low Pass Filter: ~41 Hz accelerometer and gyroscope bandwidth
  writeReg(REG_CONFIG, 0x03);

  // ---- Gyroscope Y-axis (Pitch) Bias Calibration ----
  Serial.println(F("# Calibrating Gyro Y bias (500 samples)... KEEP ROBOT COMPLETELY STILL"));
  int64_t gyroYSum  = 0;
  uint8_t rawBuffer[14];

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    if (readRegs(REG_ACCEL_XOUT_H, rawBuffer, 14)) {
      // Byte indices 10-11 = GYRO_YOUT_H and GYRO_YOUT_L
      gyroYSum += makeInt16(rawBuffer, 10);
    }
    delay(2);
  }
  gyroBias = ((float)gyroYSum / (float)CALIBRATION_SAMPLES) / GYRO_SCALE;
  Serial.printf("# Gyro Y bias calibrated: %.3f deg/s\n", gyroBias);

  // ---- Seed initial angle from accelerometer ----
  if (!readRegs(REG_ACCEL_XOUT_H, rawBuffer, 14)) {
    Serial.println(F("# ERROR: Failed initial accelerometer read!"));
    while (true) { delay(1000); }
  }

  // Byte indices 0-1 = Accel X,  Byte indices 4-5 = Accel Z
  // atan2(ax, az) gives pitch angle with respect to gravity
  int16_t ax = makeInt16(rawBuffer, 0);
  int16_t az = makeInt16(rawBuffer, 4);
  angle = atan2f((float)ax, (float)az) * 180.0f / PI;

  Serial.printf("# Initial pitch angle: %.2f deg\n", angle);
}

// -------- IMU SENSOR UPDATE --------

// Reads the full 14-byte sensor burst register, extracts pitch rate and tilt,
// and updates the complementary filter estimate of pitch angle.
// Returns false if the I2C read fails (caller should stop motors).
bool imuRead(float dt) {
  uint8_t rawBuffer[14];

  // Burst-read 14 registers starting from ACCEL_XOUT_H (0x3B)
  if (!readRegs(REG_ACCEL_XOUT_H, rawBuffer, 14)) {
    return false;
  }

  // Extract accelerometer X and Z axes for tilt calculation
  int16_t ax = makeInt16(rawBuffer, 0);   // Bytes 0-1: ACCEL_XOUT
  int16_t az = makeInt16(rawBuffer, 4);   // Bytes 4-5: ACCEL_ZOUT

  // Extract gyroscope Y axis (pitch rate), byte indices 10-11
  int16_t gy = makeInt16(rawBuffer, 10);  // Bytes 10-11: GYRO_YOUT

  // Compute tilt angle from accelerometer (noisy, but drift-free)
  float accAngle = atan2f((float)ax, (float)az) * 180.0f / PI;

  // Convert gyro raw reading to deg/s and subtract calibrated zero-rate bias
  gyroRate = ((float)gy / GYRO_SCALE) - gyroBias;

  // Complementary filter: 98% integrated gyro (fast, low-noise) + 2% accelerometer (slow, drift-free)
  angle = ALPHA * (angle + gyroRate * dt) + (1.0f - ALPHA) * accAngle;

  return true;
}
