// =============================================================================
// File: Stepper.ino
// Project: ESP32 Self-Balancing Robot
// Description: Stepper motor pulse generator using hardware timer interrupts.
//              Drives two TB6600 stepper drivers in common-anode configuration
//              with mirrored mount direction control and fast register GPIO pulses.
//
// NOTE: TB6600 ENA pins are NOT software-controlled. Wire ENA pins to GND
//       (or leave floating on common-anode boards) so drivers are always enabled.
// =============================================================================

#include <math.h>
#include "driver/gpio.h"
#include "soc/gpio_struct.h"

// -------- MOTOR PIN DEFINITIONS --------
// Motor 1 (right stepper)
const int PUL1 = 18;  // Step pulse output pin for Motor 1
const int DIR1 = 19;  // Direction output pin for Motor 1

// Motor 2 (left stepper - mirrored physical mount)
const int PUL2 = 25;  // Step pulse output pin for Motor 2
const int DIR2 = 26;  // Direction output pin for Motor 2

// GPIO bitmask for fast direct-register pulse toggling.
// PUL1=18 and PUL2=25 are both in ESP32 GPIO bank 0 (pins 0-31).
const uint32_t PUL_MASK = (1UL << PUL1) | (1UL << PUL2);

// -------- STEPPER TIMING & SPEED CONSTANTS --------
const uint32_t TICK_HZ       = 20000;    // ISR frequency: 20 kHz (50 us per tick)
extern const float MAX_SPEED = 3000.0f;  // Maximum allowed step rate (steps/s)
extern const float MAX_ACCEL = 12000.0f; // Maximum acceleration slew rate (steps/s^2)

// -------- EXPORTED MOTOR STATE --------
volatile uint32_t stepRate = 0;    // Desired step rate in steps/s (shared with ISR; volatile)
float             speedCmd = 0.0f; // Slew-rate-limited motor speed command (steps/s)
int               dirSign  = -1;   // Direction multiplier (+1 or -1), flipped with 'f' command

// -------- EXTERNAL REFERENCES --------
extern float integral;  // PID integral from PID.ino — reset when motors stop

// -------- PRIVATE ISR STATE --------
static volatile bool     pulseHigh = false;  // True while step pulse output is HIGH
static volatile uint32_t phase     = 0;      // DDS phase accumulator for step timing
static hw_timer_t       *stepTimer = nullptr; // ESP32 hardware timer handle

// -------- STEP GENERATOR ISR --------

// Hardware timer ISR running at TICK_HZ (20 kHz = 50 us intervals).
// Uses Direct Digital Synthesis (DDS): phase accumulates by stepRate each tick.
// When phase overflows TICK_HZ, a step pulse is issued.
// This produces jitter-free, rate-accurate pulses for smooth microstepping.
void IRAM_ATTR onStepTimerTick() {
  if (pulseHigh) {
    // End of active pulse: pull PUL1 and PUL2 LOW via direct register write
    GPIO.out_w1tc = PUL_MASK;
    pulseHigh = false;
    return;
  }

  // Accumulate DDS phase
  phase += stepRate;
  if (phase >= TICK_HZ) {
    phase -= TICK_HZ;
    // Start of active pulse: drive PUL1 and PUL2 HIGH via direct register write
    GPIO.out_w1ts = PUL_MASK;
    pulseHigh = true;
  }
}

// -------- DIRECTION CONTROL --------

// Sets physical travel direction for both mirrored motors.
// Due to opposing physical orientations, opposite DIR signals produce the same chassis direction.
static void setDirection(bool forward) {
  digitalWrite(DIR1, forward ? HIGH : LOW);
  digitalWrite(DIR2, forward ? LOW  : HIGH);  // Opposite: mirrored motor mount
}

// -------- STEPPER HARDWARE INITIALIZATION --------

// Configures GPIO output pins and starts the hardware timer ISR.
// Supports both ESP32 Arduino Core v2.x and v3.x timer APIs via compile-time guard.
void stepperSetup() {
  // Configure pulse and direction GPIOs as outputs
  pinMode(PUL1, OUTPUT);  pinMode(DIR1, OUTPUT);
  pinMode(PUL2, OUTPUT);  pinMode(DIR2, OUTPUT);

  // Drive to known-safe initial states
  digitalWrite(PUL1, LOW);
  digitalWrite(PUL2, LOW);
  setDirection(true);  // Initial direction: forward

  // ---- Start hardware timer at TICK_HZ (20 kHz) ----
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  // ESP32 Arduino Core v3.x API
  stepTimer = timerBegin(1000000);                      // 1 MHz base clock
  timerAttachInterrupt(stepTimer, &onStepTimerTick);
  timerAlarm(stepTimer, 1000000 / TICK_HZ, true, 0);   // 50 us period, auto-reload
#else
  // ESP32 Arduino Core v2.x API
  stepTimer = timerBegin(0, 80, true);                  // Timer 0, prescaler 80 (80MHz/80=1MHz)
  timerAttachInterrupt(stepTimer, &onStepTimerTick, true);
  timerAlarmWrite(stepTimer, 1000000 / TICK_HZ, true); // 50 us alarm, auto-reload
  timerAlarmEnable(stepTimer);
#endif

  Serial.println(F("# Stepper drivers initialized (20 kHz DDS hardware timer ISR active)"));
}

// -------- SPEED COMMAND --------

// Sets motor step rate and manages direction transitions safely.
// Implements a 10 us DIR setup/hold pause when reversing, as required by TB6600 spec.
void setSpeed(float stepsPerSec) {
  static bool lastFwd = true;
  bool fwd = (stepsPerSec >= 0.0f);

  if (fwd != lastFwd) {
    // Pause step pulses before changing direction
    stepRate = 0;
    delayMicroseconds(10);   // DIR hold time: ensure driver has seen previous direction
    setDirection(fwd);
    delayMicroseconds(10);   // DIR setup time: ensure driver sees new direction before first pulse
    lastFwd = fwd;
  }

  // Clamp to maximum allowed step rate
  float mag = fabsf(stepsPerSec);
  if (mag > MAX_SPEED) mag = MAX_SPEED;

  stepRate = (uint32_t)mag;
}

// -------- MOTOR STOP --------

// Halts step generation and resets all motion-related state.
// Called on fall detection, disable command, or I2C fault.
void stopMotors() {
  stepRate  = 0;
  speedCmd  = 0.0f;
  integral  = 0.0f;         // Reset PID integrator to avoid wind-up from stop event
  pulseHigh = false;
  GPIO.out_w1tc = PUL_MASK; // Force PUL lines LOW immediately
}
