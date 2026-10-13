
#include <Wire.h>
#include <math.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_ADDR 0x68

float angle = 0.0f;
float gyroRate = 0.0f;
float gyroBias = 0.0f;

unsigned long lastMicros;
const float alpha = 0.98f;

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t reg, uint8_t *data, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(MPU_ADDR, len, (uint8_t)true) != len)
    return false;

  for (uint8_t i = 0; i < len; i++)
    data[i] = Wire.read();

  return true;
}

int16_t getInt16(uint8_t *data, int i) {
  return (int16_t)((data[i] << 8) | data[i + 1]);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("# ESP32 MPU9250 PID IMU");

  uint8_t id;

  if (!readRegs(0x75, &id, 1)) {
    Serial.println("# ERROR: Sensor not detected at 0x68");
    while (true) delay(1000);
  }

  Serial.print("# WHO_AM_I = 0x");
  Serial.println(id, HEX);

  // Wake sensor
  writeReg(0x6B, 0x00);
  delay(100);

  // Gyroscope +/- 250 degrees/s
  writeReg(0x1B, 0x00);

  // Accelerometer +/- 2g
  writeReg(0x1C, 0x00);

  // Low-pass filter
  writeReg(0x1A, 0x03);

  Serial.println("# Keep sensor still for calibration");

  // Calibrate gyro Y-axis bias
  const int samples = 500;
  long gyroSum = 0;
  uint8_t data[14];

  for (int i = 0; i < samples; i++) {
    if (readRegs(0x3B, data, 14)) {
      gyroSum += getInt16(data, 10);
    }
    delay(3);
  }

  gyroBias = (float)gyroSum / samples / 131.0f;

  if (!readRegs(0x3B, data, 14)) {
    Serial.println("# ERROR: Cannot read IMU");
    while (true) delay(1000);
  }

  int16_t ax = getInt16(data, 0);
  int16_t az = getInt16(data, 4);

  angle = atan2f((float)ax, (float)az) * 180.0f / PI;

  lastMicros = micros();

  Serial.println("time_ms,angle_deg,gyro_rate_dps");
}

void loop() {
  uint8_t data[14];

  if (!readRegs(0x3B, data, 14)) return;

  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0f;
  lastMicros = now;

  if (dt <= 0 || dt > 0.05f) return;

  int16_t ax = getInt16(data, 0);
  int16_t az = getInt16(data, 4);
  int16_t gy = getInt16(data, 10);

  float accAngle =
      atan2f((float)ax, (float)az) * 180.0f / PI;

  gyroRate = gy / 131.0f - gyroBias;

  angle = alpha * (angle + gyroRate * dt)
        + (1.0f - alpha) * accAngle;

  Serial.print(millis());
  Serial.print(",");
  Serial.print(angle, 3);
  Serial.print(",");
  Serial.println(gyroRate, 3);

  delay(2000);
}
