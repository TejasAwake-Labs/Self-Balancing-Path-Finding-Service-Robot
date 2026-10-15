// Self-balancing robot: ESP32 + MPU9250/6500 + 2x TB6600 + 2x NEMA17
// Pins/IMU setup taken from Stepper.ino and IMU.ino.
//
// Serial commands (115200, newline terminated) for live tuning:
//   p<val>  Kp      i<val>  Ki      d<val>  Kd
//   s<val>  balance setpoint (deg)
//   z       set setpoint = current angle (hold robot upright first!)
//   f       flip control direction
//   e       enable/disable motors
//   ?       print parameters
// Example: "p300" sets Kp = 300

#include <Wire.h>
#include <math.h>
#include "driver/gpio.h"
#include "soc/gpio_struct.h"

// ---------------- Pins ----------------
#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_ADDR 0x68
const int PUL1 = 18, DIR1 = 19;
const int PUL2 = 25, DIR2 = 26;

// ---------------- Tunables ----------------
float Kp = 120.0f;   // steps/s per degree
float Ki = 0.0f;     // steps/s per degree*second
float Kd = 4.0f;     // steps/s per deg/s (uses gyro directly)
float setpoint = 0.0f;       // deg; trim until robot stands still
int   dirSign = -1;          // flip with 'f' if robot falls faster

const float ALPHA = 0.98f;             // complementary filter
const float FALL_ANGLE = 30.0f;        // deg: motors cut beyond this
const float MAX_SPEED = 3000.0f;       // steps/s (keep within TB6600 + torque limits)
const float MAX_ACCEL = 12000.0f;      // steps/s^2 slew limit
const float I_LIMIT = 3000.0f;         // integral clamp (steps/s)
const float LOOP_HZ = 200.0f;
const uint32_t LOOP_US = (uint32_t)(1000000.0f / LOOP_HZ);

// Step generator
const uint32_t TICK_HZ = 20000;        // ISR rate, 50us pulse width

// ---------------- State ----------------
float angle = 0, gyroRate = 0, gyroBias = 0;
float integral = 0, speedCmd = 0;
bool enabled = true, active = false;
uint32_t lastLoop = 0;

volatile uint32_t stepRate = 0;        // steps/s (magnitude)
volatile bool pulseHigh = false;
uint32_t phase = 0;
hw_timer_t *timer = nullptr;

// ---------------- Step ISR ----------------
void IRAM_ATTR onTick() {
  if (pulseHigh) {
    GPIO.out_w1tc = (1UL << PUL1) | (1UL << PUL2);
    pulseHigh = false;
    return;
  }
  phase += stepRate;
  if (phase >= TICK_HZ) {
    phase -= TICK_HZ;
    GPIO.out_w1ts = (1UL << PUL1) | (1UL << PUL2);
    pulseHigh = true;
  }
}

void startTimer() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTick);
  timerAlarm(timer, 1000000 / TICK_HZ, true, 0);
#else
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTick, true);
  timerAlarmWrite(timer, 1000000 / TICK_HZ, true);
  timerAlarmEnable(timer);
#endif
}

// ---------------- IMU ----------------
bool writeReg(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(v);
  return Wire.endTransmission() == 0;
}
bool readRegs(uint8_t reg, uint8_t *d, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, len, (uint8_t)true) != len) return false;
  for (uint8_t i = 0; i < len; i++) d[i] = Wire.read();
  return true;
}
int16_t getInt16(uint8_t *d, int i) { return (int16_t)((d[i] << 8) | d[i + 1]); }

// ---------------- Motors ----------------
// Opposite DIR levels = same physical direction (from Stepper.ino)
void setDirection(bool forward) {
  digitalWrite(DIR1, forward ? HIGH : LOW);
  digitalWrite(DIR2, forward ? LOW : HIGH);
}

void setSpeed(float s) {
  static bool lastFwd = true;
  bool fwd = s >= 0;
  if (fwd != lastFwd) {
    stepRate = 0;           // stop pulses briefly around direction change
    delayMicroseconds(10);  // TB6600 DIR setup time
    setDirection(fwd);
    delayMicroseconds(10);
    lastFwd = fwd;
  }
  stepRate = (uint32_t)fabsf(s);
}

void stopMotors() {
  stepRate = 0;
  speedCmd = 0;
  integral = 0;
}

// ---------------- Serial tuning ----------------
void printParams() {
  Serial.printf("# Kp=%.2f Ki=%.3f Kd=%.2f setpoint=%.2f dirSign=%d enabled=%d\n",
                Kp, Ki, Kd, setpoint, dirSign, enabled);
}

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (!line.length()) return;
  char c = line[0];
  float v = line.substring(1).toFloat();
  switch (c) {
    case 'p': Kp = v; break;
    case 'i': Ki = v; integral = 0; break;
    case 'd': Kd = v; break;
    case 's': setpoint = v; break;
    case 'z': setpoint = angle; integral = 0; break;
    case 'f': dirSign = -dirSign; break;
    case 'e': enabled = !enabled; if (!enabled) stopMotors(); break;
    default: break;
  }
  printParams();
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PUL1, OUTPUT); pinMode(DIR1, OUTPUT);
  pinMode(PUL2, OUTPUT); pinMode(DIR2, OUTPUT);
  digitalWrite(PUL1, LOW); digitalWrite(PUL2, LOW);
  setDirection(true);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  uint8_t id;
  if (!readRegs(0x75, &id, 1)) {
    Serial.println("# ERROR: IMU not found at 0x68");
    while (true) delay(1000);
  }
  writeReg(0x6B, 0x00); delay(100);
  writeReg(0x1B, 0x00);   // gyro  +/-250 dps
  writeReg(0x1C, 0x00);   // accel +/-2 g
  writeReg(0x1A, 0x03);   // DLPF ~41 Hz

  Serial.println("# Keep robot still: calibrating gyro...");
  const int N = 500;
  long sum = 0;
  uint8_t d[14];
  for (int i = 0; i < N; i++) {
    if (readRegs(0x3B, d, 14)) sum += getInt16(d, 10);
    delay(3);
  }
  gyroBias = (float)sum / N / 131.0f;

  readRegs(0x3B, d, 14);
  angle = atan2f((float)getInt16(d, 0), (float)getInt16(d, 4)) * 180.0f / PI;

  startTimer();
  lastLoop = micros();
  Serial.println("# Ready. Hold robot upright; it activates near setpoint.");
  printParams();
}

// ---------------- Loop ----------------
void loop() {
  handleSerial();

  uint32_t now = micros();
  if ((uint32_t)(now - lastLoop) < LOOP_US) return;
  float dt = (now - lastLoop) / 1e6f;
  lastLoop = now;
  if (dt > 0.05f) return;

  uint8_t d[14];
  if (!readRegs(0x3B, d, 14)) { stopMotors(); return; }

  float accAngle = atan2f((float)getInt16(d, 0), (float)getInt16(d, 4)) * 180.0f / PI;
  gyroRate = getInt16(d, 10) / 131.0f - gyroBias;
  angle = ALPHA * (angle + gyroRate * dt) + (1.0f - ALPHA) * accAngle;

  float err = angle - setpoint;

  // Safety: engage only when upright, cut out when fallen
  if (fabsf(err) > FALL_ANGLE || !enabled) {
    active = false;
    stopMotors();
  } else if (!active && fabsf(err) < 2.0f && enabled) {
    active = true;
    integral = 0;
  }

  if (active) {
    integral += Ki * err * dt;
    integral = constrain(integral, -I_LIMIT, I_LIMIT);

    float target = dirSign * (Kp * err + integral + Kd * gyroRate);
    target = constrain(target, -MAX_SPEED, MAX_SPEED);

    // Slew-rate limit
    float maxStep = MAX_ACCEL * dt;
    speedCmd += constrain(target - speedCmd, -maxStep, maxStep);
    setSpeed(speedCmd);
  }

  // Telemetry at ~10 Hz
  static uint8_t cnt = 0;
  if (++cnt >= 20) {
    cnt = 0;
    Serial.printf("%lu,%.2f,%.2f,%.0f,%d\n", millis(), angle, gyroRate, speedCmd, active);
  }
}
