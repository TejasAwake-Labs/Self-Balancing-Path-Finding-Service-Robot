// =============================================================================
// File: SelfBalancingRobot.ino
// Project: ESP32 Self-Balancing Robot with Path-Finding Stub
// Description: Main entry point for the self-balancing robot. Implements the
//              200 Hz control loop, 10 Hz telemetry streaming, and interactive
//              Serial command interface.
// =============================================================================

#include <Wire.h>
#include <math.h>
#include <Preferences.h>
#include "driver/gpio.h"
#include "soc/gpio_struct.h"

// -------- TIMING & CONTROL CONSTANTS --------
const float    LOOP_HZ = 200.0f;                             // Main control loop frequency (Hz)
const uint32_t LOOP_US = (uint32_t)(1000000.0f / LOOP_HZ);  // Main control loop period in microseconds (5000 us)

// -------- FORWARD DECLARATIONS & SHARED GLOBALS --------

// Exported by IMU.ino
extern float angle;        // Current pitch angle (degrees)
extern float gyroRate;     // Current pitch angular rate (deg/s)
void imuSetup();           // Initializes MPU9250 and calibrates gyro bias
bool imuRead(float dt);    // Reads raw IMU registers and updates angle/gyroRate

// Exported by Stepper.ino
extern volatile uint32_t stepRate;  // Current step frequency sent to ISR (steps/s)
extern float speedCmd;              // Slew-rate-limited speed command (steps/s)
extern int   dirSign;               // Direction sign multiplier (+1 or -1)
extern const float MAX_SPEED;       // Maximum motor speed (steps/s)
extern const float MAX_ACCEL;       // Maximum motor acceleration (steps/s^2)
void stepperSetup();                // Configures GPIOs and hardware timer ISR
void setSpeed(float stepsPerSec);   // Updates motor step rate and direction
void stopMotors();                  // Immediately stops pulse generation and resets speed

// Exported by PID.ino
extern float Kp;                    // Proportional gain (steps/s per degree)
extern float Ki;                    // Integral gain (steps/s per degree*second)
extern float Kd;                    // Derivative gain (steps/s per deg/s)
extern float setpoint;              // Desired balance angle trim (degrees)
extern float integral;              // Accumulated PID integral term
extern bool  active;                // True when robot balance controller is actively driving
extern bool  enabled;               // Master software enable flag for motors
extern bool  tuning;                // True when Ziegler-Nichols auto-tune is running
extern const float FALL_ANGLE;      // Tilt cutoff threshold in degrees (30.0 deg)
void pidLoadPrefs();                // Loads PID parameters and setpoint from NVS
void pidSavePrefs();                // Saves current PID parameters and setpoint to NVS
void pidResetPrefs();               // Resets PID parameters to default values and saves to NVS
float pidUpdate(float error, float gRate, float dt);  // Calculates target speed command
void  startAutoTune();              // Begins relay-feedback auto-tuning
void  stopAutoTune();               // Finalizes auto-tune, calculates ZN gains, and saves to NVS
float autoTuneUpdate(float error, float dt);          // Computes relay control output during tuning

// -------- PATH FINDING STUB --------
// Replace these stub functions with your actual navigation commands.
// setpointOffset shifts the balance point to lean the robot forward/backward,
// causing the motors to drive the robot in that direction.
float setpointOffset = 0.0f;  // degrees added to setpoint for forward/reverse lean

void driveForward(float speed_pct) {
  // Lean forward by offsetting the balance setpoint (positive = forward lean)
  setpointOffset = map(speed_pct, 0, 100, 0, 3);  // up to 3 deg lean at full speed
}
void driveReverse(float speed_pct) {
  // Lean backward by negating the offset
  setpointOffset = -map(speed_pct, 0, 100, 0, 3);
}
void turnLeft()  { /* TODO: differential speed between motors */ }
void turnRight() { /* TODO: differential speed between motors */ }
void stopDrive() { setpointOffset = 0.0f; }

// -------- LOCAL STATE VARIABLES --------
static uint32_t lastLoopMicros = 0;  // Timestamp of previous control loop cycle (micros)

// -------- SERIAL UTILITIES --------

// Prints all current runtime parameters and configuration states to Serial
void printParams() {
  Serial.printf("# PARAMS: Kp=%.2f Ki=%.3f Kd=%.2f setpoint=%.2f dirSign=%d enabled=%d tuning=%d\n",
                Kp, Ki, Kd, setpoint, dirSign, enabled ? 1 : 0, tuning ? 1 : 0);
}

// Parses and executes single-character commands received over Serial at 115200 baud.
// Commands:
//   p<val>  - Set Kp and save       i<val> - Set Ki and save
//   d<val>  - Set Kd and save       s<val> - Set setpoint and save
//   z       - Zero setpoint to current angle
//   f       - Flip motor direction sign
//   e       - Toggle motor enable/disable
//   t       - Toggle Ziegler-Nichols auto-tune
//   r       - Reset PID to factory defaults
//   ?       - Print current parameters
void handleSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char  cmd = line[0];
  float val = line.substring(1).toFloat();

  switch (cmd) {
    case 'z':  // Zero trim: calibrate setpoint to current resting angle
      setpoint = angle;
      integral = 0.0f;
      pidSavePrefs();
      Serial.printf("# Zeroed setpoint to current angle: %.2f deg\n", setpoint);
      printParams();
      break;

    case 'f':  // Flip motor direction sign
      dirSign = -dirSign;
      // IMPORTANT: Reset motion state so the slew-limiter doesn't fight
      // the new direction — without this, a large speedCmd in the old
      // direction takes ~250 ms to reverse (robot falls before it can correct).
      stopMotors();   // zeros stepRate, speedCmd, integral
      active = false; // force clean re-engagement from the hysteresis window
      Serial.printf("# Direction sign flipped to %d. Hold robot upright to re-engage.\n", dirSign);
      printParams();
      break;

    case 'e':  // Toggle motor enable / disable
      enabled = !enabled;
      if (!enabled) {
        stopMotors();
        active = false;
        Serial.println(F("# Motors DISABLED"));
      } else {
        Serial.println(F("# Motors ENABLED (hold upright to engage)"));
      }
      printParams();
      break;

    case 't':  // Toggle Ziegler-Nichols auto-tune mode
      if (tuning) {
        Serial.println(F("# User requested manual stop of auto-tune"));
        stopAutoTune();
      } else {
        startAutoTune();
      }
      break;

    case 'r':  // Reset PID parameters to factory defaults
      pidResetPrefs();
      printParams();
      break;

    case '?':  // Query current parameters
      printParams();
      break;

    case 'p':  // Set proportional gain Kp and persist to NVS
      Kp = val;
      pidSavePrefs();
      printParams();
      break;

    case 'i':  // Set integral gain Ki, clear accumulator, and persist to NVS
      Ki = val;
      integral = 0.0f;
      pidSavePrefs();
      printParams();
      break;

    case 'd':  // Set derivative gain Kd and persist to NVS
      Kd = val;
      pidSavePrefs();
      printParams();
      break;

    case 's':  // Set balance setpoint angle and persist to NVS
      setpoint = val;
      integral = 0.0f;
      pidSavePrefs();
      printParams();
      break;

    default:
      Serial.printf("# Unknown command: '%c'. Valid: p i d s z f e t r ?\n", cmd);
      break;
  }
}

// -------- ARDUINO SETUP --------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n# ================================================"));
  Serial.println(F("# ESP32 Self-Balancing Robot Firmware Initializing"));
  Serial.println(F("# ================================================"));

  // Load tuned PID parameters and setpoint from NVS (survives reboot)
  pidLoadPrefs();

  // Initialize hardware timer and TB6600 stepper driver GPIO pins
  stepperSetup();

  // Initialize I2C bus and MPU9250 sensor (includes 500-sample gyro bias calibration)
  imuSetup();

  printParams();
  Serial.println(F("# System ready. Hold robot upright near setpoint to engage balance."));
  lastLoopMicros = micros();
}

// -------- ARDUINO MAIN LOOP --------
void loop() {
  // Process any incoming serial tuning commands
  handleSerial();

  // Enforce 200 Hz loop timing using micros()
  uint32_t nowMicros = micros();
  if ((uint32_t)(nowMicros - lastLoopMicros) < LOOP_US) {
    return;
  }

  // Calculate actual elapsed delta time in seconds
  float dt = (float)(nowMicros - lastLoopMicros) * 1e-6f;
  lastLoopMicros = nowMicros;

  // Guard against timing anomalies (e.g., following long serial pauses)
  if (dt <= 0.0f || dt > 0.05f) {
    return;
  }

  // Update sensor fusion: complementary filter angle + gyro rate
  if (!imuRead(dt)) {
    stopMotors();
    return;
  }

  // Calculate tilt error including navigation setpoint offset
  float error = angle - (setpoint + setpointOffset);

  // ---- CONTROL MODE: Auto-Tune OR Normal Balancing ----
  if (tuning) {
    // Relay feedback drives motors to induce controlled oscillation
    float targetSpeed = autoTuneUpdate(error, dt);
    float maxStep     = MAX_ACCEL * dt;
    speedCmd += constrain(targetSpeed - speedCmd, -maxStep, maxStep);
    setSpeed(speedCmd);

  } else {
    // Normal PID balance mode

    // Safety: cut motors if tilt exceeds FALL_ANGLE or motors are disabled
    if (fabsf(error) > FALL_ANGLE || !enabled) {
      active = false;
      stopMotors();
    }
    // Hysteresis activation: engage only when near-vertical (|error| < 2.0 deg)
    else if (!active && fabsf(error) < 2.0f && enabled) {
      active   = true;
      integral = 0.0f;
      speedCmd = 0.0f;
    }

    // Run PID and apply slew-rate limit when active
    if (active) {
      float targetSpeed = pidUpdate(error, gyroRate, dt);
      float maxStep     = MAX_ACCEL * dt;
      speedCmd += constrain(targetSpeed - speedCmd, -maxStep, maxStep);
      setSpeed(speedCmd);
    }
  }

  // Telemetry output at ~10 Hz (every 20 iterations at 200 Hz)
  // Format: millis,angle_deg,gyroRate_dps,speedCmd_steps_s,active,tuneMode
  static uint8_t telDiv = 0;
  if (++telDiv >= 20) {
    telDiv = 0;
    Serial.printf("%lu,%.2f,%.2f,%.0f,%d,%d\n",
                  millis(), angle, gyroRate, speedCmd,
                  active ? 1 : 0, tuning ? 1 : 0);
  }
}
