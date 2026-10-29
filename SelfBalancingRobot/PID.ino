// =============================================================================
// File: PID.ino
// Project: ESP32 Self-Balancing Robot
// Description: PID balance controller, NVS persistence via Preferences library,
//              and Ziegler-Nichols Relay-Feedback Auto-Tuner.
// =============================================================================

#include <math.h>
#include <Preferences.h>

// -------- FACTORY DEFAULT PID GAINS --------
const float DEFAULT_KP       = 120.0f;  // Default proportional gain (steps/s per degree)
const float DEFAULT_KI       = 0.0f;    // Default integral gain (steps/s per degree*second)
const float DEFAULT_KD       = 4.0f;    // Default derivative gain (steps/s per deg/s)
const float DEFAULT_SETPOINT = 0.0f;    // Default balance trim angle (degrees)

// -------- CONTROLLER TUNING LIMITS --------
extern const float FALL_ANGLE = 30.0f;  // Tilt cutoff: motors cut if |error| > 30 deg
const float I_LIMIT           = 3000.0f; // Maximum integral accumulator value (steps/s)

// -------- AUTO-TUNE CONSTANTS --------
const float RELAY_AMP       = 800.0f; // Relay excitation amplitude (steps/s)
const int   MAX_TUNE_CYCLES = 20;     // Target oscillation cycles before auto-tune completes

// -------- EXPORTED CONTROLLER STATE --------
float Kp       = DEFAULT_KP;       // Active proportional gain
float Ki       = DEFAULT_KI;       // Active integral gain
float Kd       = DEFAULT_KD;       // Active derivative gain
float setpoint = DEFAULT_SETPOINT; // Active balance setpoint angle (degrees)
float integral = 0.0f;             // Active PID integral accumulator (steps/s)
bool  active   = false;            // True when balance loop is engaged and driving motors
bool  enabled  = true;             // Master software motor enable flag
bool  tuning   = false;            // True while Ziegler-Nichols auto-tune is running

// -------- EXTERNAL REFERENCES --------
extern int         dirSign;    // Direction sign multiplier from Stepper.ino
extern const float MAX_SPEED;  // Maximum step rate from Stepper.ino
extern float       angle;      // Current pitch angle from IMU.ino
void stopMotors();              // Motor stop function from Stepper.ino

// -------- AUTO-TUNE INTERNAL STATE --------
static int      tuneCycleCount   = 0;     // Total oscillation cycles observed
static int      validCycleCount  = 0;     // Stable cycles used in average (skip first 2)
static float    sumKu            = 0.0f;  // Running sum of per-cycle ultimate gain estimates
static float    sumTu            = 0.0f;  // Running sum of per-cycle oscillation period estimates
static float    tunePeakPos      = 0.0f;  // Maximum positive error in current cycle
static float    tunePeakNeg      = 0.0f;  // Maximum negative error in current cycle
static int      halfCycleCount   = 0;     // Zero-crossing counter (2 per full cycle)
static int      lastSign         = 0;     // Sign of error in previous control step
static uint32_t cycleStartMicros = 0;     // Timestamp at start of current oscillation cycle

// ============================================================
//  NVS PERSISTENCE (Preferences)
// ============================================================

// Loads Kp, Ki, Kd, and setpoint from NVS namespace "pid".
// Falls back to factory defaults if namespace is new or keys are missing.
void pidLoadPrefs() {
  Preferences prefs;
  if (prefs.begin("pid", true)) {
    Kp       = prefs.getFloat("kp",       DEFAULT_KP);
    Ki       = prefs.getFloat("ki",       DEFAULT_KI);
    Kd       = prefs.getFloat("kd",       DEFAULT_KD);
    setpoint = prefs.getFloat("setpoint", DEFAULT_SETPOINT);
    prefs.end();
    Serial.printf("# Loaded PID from NVS: Kp=%.2f Ki=%.3f Kd=%.2f setpoint=%.2f\n",
                  Kp, Ki, Kd, setpoint);
  } else {
    Kp = DEFAULT_KP; Ki = DEFAULT_KI; Kd = DEFAULT_KD; setpoint = DEFAULT_SETPOINT;
    Serial.println(F("# NVS 'pid' namespace not found. Using factory defaults."));
  }
}

// Persists current Kp, Ki, Kd, and setpoint to NVS namespace "pid".
// Values survive power cycles and reboots automatically.
void pidSavePrefs() {
  Preferences prefs;
  if (prefs.begin("pid", false)) {
    prefs.putFloat("kp",       Kp);
    prefs.putFloat("ki",       Ki);
    prefs.putFloat("kd",       Kd);
    prefs.putFloat("setpoint", setpoint);
    prefs.end();
    Serial.printf("# Saved PID to NVS: Kp=%.2f Ki=%.3f Kd=%.2f setpoint=%.2f\n",
                  Kp, Ki, Kd, setpoint);
  } else {
    Serial.println(F("# ERROR: Failed to open NVS 'pid' namespace for writing!"));
  }
}

// Resets all PID parameters and setpoint to factory defaults, then saves to NVS.
void pidResetPrefs() {
  Kp = DEFAULT_KP; Ki = DEFAULT_KI; Kd = DEFAULT_KD; setpoint = DEFAULT_SETPOINT;
  integral = 0.0f;
  pidSavePrefs();
  Serial.println(F("# Reset to factory defaults: Kp=120 Ki=0 Kd=4 setpoint=0"));
}

// ============================================================
//  PID CONTROLLER
// ============================================================

// Computes desired motor speed command from tilt error and angular rate.
//   P: drives motors proportional to current tilt displacement
//   I: eliminates steady-state lean (wind-up clamped to I_LIMIT)
//   D: uses raw gyro rate (D-on-measurement) to damp oscillation without kick
float pidUpdate(float error, float gyroRate, float dt) {
  float pTerm  = Kp * error;

  integral += Ki * error * dt;
  integral   = constrain(integral, -I_LIMIT, I_LIMIT);

  float dTerm = Kd * gyroRate;

  float target = (float)dirSign * (pTerm + integral + dTerm);
  return constrain(target, -MAX_SPEED, MAX_SPEED);
}

// ============================================================
//  ZIEGLER-NICHOLS AUTO-TUNE (Astrom-Hagglund Relay Method)
// ============================================================

// HOW TO USE:
//   1. Place robot on a stand or hold it near the balance point.
//   2. Send 't' over Serial Monitor (115200 baud, newline-terminated).
//   3. The robot oscillates — observe "AutoTune cycle N" lines.
//   4. After 20 cycles (or send 't' again to stop early).
//   5. Ku and Tu are computed, ZN gains applied and SAVED to NVS.
//   6. Fine-tune Kp/Ki/Kd manually from Serial if needed.

// Initialises relay-feedback auto-tuning state and starts oscillation.
void startAutoTune() {
  stopMotors();
  integral = 0.0f;

  float initialError = angle - setpoint;
  if (fabsf(initialError) > 10.0f) {
    Serial.println(F("# WARNING: Hold robot near balance point before auto-tune!"));
  }

  tuning           = true;
  active           = false;
  tuneCycleCount   = 0;
  validCycleCount  = 0;
  sumKu            = 0.0f;
  sumTu            = 0.0f;
  tunePeakPos      = 0.0f;
  tunePeakNeg      = 0.0f;
  halfCycleCount   = 0;
  lastSign         = (initialError >= 0.0f) ? 1 : -1;
  cycleStartMicros = micros();

  Serial.println(F("# AutoTune STARTED — Relay Feedback Oscillation Method"));
  Serial.printf("# Relay amplitude: +/-%.0f steps/s | Target cycles: %d\n", RELAY_AMP, MAX_TUNE_CYCLES);
  Serial.println(F("# Send 't' again to stop early."));
}

// Finalises auto-tune: averages Ku and Tu, applies ZN formulas, saves to NVS.
void stopAutoTune() {
  if (!tuning) return;

  tuning = false;
  stopMotors();

  if (validCycleCount > 0) {
    float Ku = sumKu / (float)validCycleCount;
    float Tu = sumTu / (float)validCycleCount;

    // Classic Ziegler-Nichols closed-loop rules:
    //   Kp = 0.6 * Ku
    //   Ki = 1.2 * Ku / Tu   (= 2*Kp / Tu)
    //   Kd = 0.075 * Ku * Tu (= Kp * Tu / 8)
    Kp = 0.6f   * Ku;
    Ki = 1.2f   * Ku / Tu;
    Kd = 0.075f * Ku * Tu;

    pidSavePrefs();

    Serial.println(F("# ================================================="));
    Serial.println(F("# ------  ZIEGLER-NICHOLS AUTO-TUNE RESULTS  ------"));
    Serial.printf( "# Ultimate Gain (Ku)   : %.3f steps/s/deg\n", Ku);
    Serial.printf( "# Ultimate Period (Tu) : %.4f s\n", Tu);
    Serial.printf( "# Stable cycles used   : %d\n", validCycleCount);
    Serial.println(F("# -------------------------------------------------"));
    Serial.printf( "# Tuned Kp : %.2f\n", Kp);
    Serial.printf( "# Tuned Ki : %.3f\n", Ki);
    Serial.printf( "# Tuned Kd : %.2f\n", Kd);
    Serial.println(F("# Parameters SAVED to NVS (survive reboot)."));
    Serial.println(F("# ================================================="));
    Serial.println(F("# TIP: Hold robot upright — balance mode re-activates within 2 deg of setpoint."));
  } else {
    Serial.println(F("# AutoTune ended without stable cycles. Previous PID gains retained."));
  }
}

// Called every control loop iteration while tuning==true.
// Applies relay bang-bang output, detects zero-crossings, estimates Ku/Tu per cycle.
float autoTuneUpdate(float error, float dt) {
  if (!tuning) return 0.0f;

  // Safety abort if robot falls beyond limit
  if (fabsf(error) > FALL_ANGLE) {
    Serial.println(F("# AutoTune ABORTED: Robot fell beyond FALL_ANGLE!"));
    stopAutoTune();
    return 0.0f;
  }

  // Relay output: opposite signs drive oscillation
  float relayOut = (error > 0.0f)
    ? ((float)dirSign *  RELAY_AMP)
    : ((float)dirSign * -RELAY_AMP);

  // Track peak positive and negative error within current cycle
  if (error > tunePeakPos) tunePeakPos = error;
  if (error < tunePeakNeg) tunePeakNeg = error;

  // Zero-crossing detection
  int currentSign = (error >= 0.0f) ? 1 : -1;
  if (currentSign != lastSign) {
    lastSign = currentSign;
    halfCycleCount++;

    if (halfCycleCount >= 2) {
      halfCycleCount = 0;
      tuneCycleCount++;

      uint32_t nowUs       = micros();
      float    cyclePeriod = (float)(nowUs - cycleStartMicros) * 1e-6f;
      cycleStartMicros     = nowUs;

      // Peak-to-peak semi-amplitude in degrees
      float amplitude = (tunePeakPos - tunePeakNeg) * 0.5f;
      tunePeakPos = 0.0f;
      tunePeakNeg = 0.0f;

      if (amplitude > 0.05f && cyclePeriod > 0.02f) {
        // Astrom-Hagglund formula: Ku = 4*d / (pi*a)
        float kuEst = (4.0f * RELAY_AMP) / (PI * amplitude);
        float tuEst = cyclePeriod;

        // Skip first 2 transient cycles before including in average
        if (tuneCycleCount > 2) {
          sumKu += kuEst;
          sumTu += tuEst;
          validCycleCount++;
        }

        Serial.printf("# AutoTune cycle %2d: Ku_est=%7.2f  Tu_est=%.3fs  amp=%.3fdeg\n",
                      tuneCycleCount, kuEst, tuEst, amplitude);
      } else {
        Serial.printf("# AutoTune cycle %2d: weak oscillation (amp=%.3fdeg) — skipped\n",
                      tuneCycleCount, amplitude);
      }

      // Auto-complete after target cycle count
      if (tuneCycleCount >= MAX_TUNE_CYCLES) {
        stopAutoTune();
        return 0.0f;
      }
    }
  }

  return relayOut;
}
