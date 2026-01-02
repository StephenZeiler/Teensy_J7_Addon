/*  Teensy Bulb Ram Controller (NON-BLOCKING step timing using micros())

  What changed vs your delayMicroseconds version?
  - NO delayMicroseconds() for step timing anymore.
  - Step pulses are generated with a "stopwatch" (micros) so code can scale to more motors later.

  NOTE:
  - This is still "single-motor" today, but the stepping is non-blocking and ready for multi-motor expansion.

  PIN LAYOUT
  Motor driver:
    PUL (step)  -> Teensy pin 2
    DIR         -> Teensy pin 3
    ENA         -> Teensy pin 4

  Sensors:
    HOME_SENSOR    -> pin 8  (active LOW by default)
    OVERRUN_SENSOR -> pin 9  (active LOW by default)

  Arduino signals:
    INJECT_CMD     -> pin 10 (Arduino -> Teensy)
    HOME_READY     -> pin 11 (Teensy -> Arduino)
    OVERRUN_ALARM  -> pin 12 (Teensy -> Arduino)

  Direction note:
    Swap DIR_CW / DIR_CCW if motor direction is reversed.
*/

#include <Arduino.h>

// -------------------- PINS --------------------
constexpr uint8_t PIN_PUL = 2;
constexpr uint8_t PIN_DIR = 3;
constexpr uint8_t PIN_ENA = 4;

constexpr uint8_t PIN_HOME_SENSOR    = 8;
constexpr uint8_t PIN_OVERRUN_SENSOR = 9;

constexpr uint8_t PIN_INJECT_CMD    = 10;
constexpr uint8_t PIN_HOME_READY    = 11;
constexpr uint8_t PIN_OVERRUN_ALARM = 12;

// -------------------- DIRECTION --------------------
// These are flipped from your earlier code (keep whatever matches your mechanics)
constexpr bool DIR_CW  = LOW;
constexpr bool DIR_CCW = HIGH;

// -------------------- MOTION CONSTANTS --------------------
constexpr int STEPS_INJECT       = 480;
constexpr int STEPS_EXTRA_SEARCH = 128;                 // used for sanity check and "missed home window"
constexpr int MAX_RETURN_STEPS   = STEPS_INJECT + STEPS_EXTRA_SEARCH; // 608

// Keep speed EXACTLY the same as your old code:
// stepOnce(6, 300) => HIGH 6us, then LOW 300us between steps.
constexpr uint16_t STEP_PULSE_US_FAST = 6;
constexpr uint16_t STEP_GAP_US_FAST   = 300;

// Slow creep used in homing "re-enter home precisely"
constexpr uint16_t STEP_PULSE_US_SLOW = 6;
constexpr uint16_t STEP_GAP_US_SLOW   = 900;

// -------------------- LOGIC POLARITY --------------------
constexpr bool HOME_ACTIVE_LOW    = true;
constexpr bool OVERRUN_ACTIVE_LOW = true;
constexpr bool ENA_ACTIVE_LOW     = true;

// -------------------- SAFETIES --------------------
constexpr int HOMING_MAX_STEPS_CW  = 4000;
constexpr int HOMING_MAX_STEPS_CCW = 4000;

constexpr uint16_t OVERRUN_ALARM_PULSE_MS = 1000;

// Startup delays you requested earlier:
constexpr uint16_t DELAY_BEFORE_SETUP_MS = 1000;
constexpr uint16_t DELAY_AFTER_SETUP_MS  = 3000;

// -------------------- OPTIONAL TESTING --------------------
// If you want to test without Arduino, set to true to simulate pin10 HIGH forever.
constexpr bool TEST_SIMULATE_PIN10_HIGH = false;

// If true: level-triggered (pin10 HIGH = keep cycling injections when home)
constexpr bool LEVEL_TRIGGERED = true;
// If true: edge-triggered (one injection per LOW->HIGH pulse on pin10)
constexpr bool EDGE_TRIGGERED  = false;

// Small delay at HOME between cycles when level-triggered
constexpr uint16_t CONTINUOUS_CYCLE_DELAY_MS = 150;

// -------------------- HELPERS --------------------
inline bool isHomeActive() {
  bool v = digitalRead(PIN_HOME_SENSOR);
  return HOME_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

inline bool isOverrunActive() {
  bool v = digitalRead(PIN_OVERRUN_SENSOR);
  return OVERRUN_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

inline void setHomeReady(bool ready) {
  digitalWrite(PIN_HOME_READY, ready ? HIGH : LOW);
}

inline void motorEnable(bool en) {
  if (ENA_ACTIVE_LOW) digitalWrite(PIN_ENA, en ? LOW : HIGH);
  else                digitalWrite(PIN_ENA, en ? HIGH : LOW);
}

inline void setDir(bool dir) {
  digitalWrite(PIN_DIR, dir);
}

[[noreturn]] void faultOverrun(const char* reason) {
  (void)reason;

  setHomeReady(false);
  motorEnable(false);

  digitalWrite(PIN_OVERRUN_ALARM, HIGH);
  delay(OVERRUN_ALARM_PULSE_MS);
  digitalWrite(PIN_OVERRUN_ALARM, LOW);

  while (true) delay(100);
}

// =========================================================
// NON-BLOCKING STEP GENERATOR (micros stopwatch)
// =========================================================
struct StepperPulseGen {
  uint8_t pinStep = 0;
  uint8_t pinDir  = 0;

  // timing
  uint16_t pulseHighUs = 6;
  uint16_t gapLowUs    = 300;

  // motion state
  bool active = false;
  bool dir = LOW;
  long stepsRemaining = 0;     // for fixed-step moves
  long stepsDone = 0;          // counts completed steps (LOW-> completed)
  bool stepLevelHigh = false;  // current STEP pin level in the pulse sequence

  uint32_t nextMicros = 0;

  void begin(uint8_t stepPin, uint8_t dirPin) {
    pinStep = stepPin;
    pinDir  = dirPin;
    pinMode(pinStep, OUTPUT);
    pinMode(pinDir, OUTPUT);
    digitalWrite(pinStep, LOW);
  }

  void setSpeed(uint16_t highUs, uint16_t lowUs) {
    pulseHighUs = highUs;
    gapLowUs    = lowUs;
  }

  void startFixedMove(bool direction, long steps) {
    dir = direction;
    digitalWrite(pinDir, dir);

    stepsRemaining = steps;
    stepsDone = 0;
    active = (stepsRemaining > 0);

    stepLevelHigh = false;
    digitalWrite(pinStep, LOW);

    nextMicros = micros(); // start asap
  }

  void stop() {
    active = false;
    stepLevelHigh = false;
    digitalWrite(pinStep, LOW);
  }

  // Returns true if it toggled something / made progress
  // Returns false if it did nothing this call
  bool tick() {
    if (!active) return false;

    uint32_t now = micros();
    // handle micros wrap safely with signed comparison
    if ((int32_t)(now - nextMicros) < 0) return false;

    if (!stepLevelHigh) {
      // Start the STEP HIGH pulse
      digitalWrite(pinStep, HIGH);
      stepLevelHigh = true;
      nextMicros = now + pulseHighUs;
      return true;
    } else {
      // End pulse -> STEP LOW. Count ONE FULL STEP here.
      digitalWrite(pinStep, LOW);
      stepLevelHigh = false;

      stepsDone++;
      stepsRemaining--;

      if (stepsRemaining <= 0) {
        active = false;
        return true;
      }

      nextMicros = now + gapLowUs;
      return true;
    }
  }
};

StepperPulseGen motor;

// =========================================================
// HIGH-LEVEL MOVE STATE MACHINE
// This replaces your blocking while/for loops.
// =========================================================
enum class MoveMode {
  NONE,
  FIXED_STEPS,        // move N steps
  UNTIL_HOME_ACTIVE,  // move until HOME becomes active (LOW) with max steps
  UNTIL_HOME_INACTIVE // move until HOME becomes inactive (HIGH) with max steps
};

struct MotionRequest {
  MoveMode mode = MoveMode::NONE;
  bool dir = LOW;
  long maxSteps = 0;       // for safety
  uint16_t pulseUs = 6;
  uint16_t gapUs   = 300;

  // internal
  long startedSteps = 0;
  bool running = false;
};

MotionRequest motion;

void startFixedSteps(bool dir, long steps, uint16_t pulseUs, uint16_t gapUs) {
  motion = {};
  motion.mode = MoveMode::FIXED_STEPS;
  motion.dir = dir;
  motion.maxSteps = steps;
  motion.pulseUs = pulseUs;
  motion.gapUs = gapUs;
  motion.running = true;

  motor.setSpeed(pulseUs, gapUs);
  motor.startFixedMove(dir, steps);
}

void startUntilHomeActive(bool dir, long maxSteps, uint16_t pulseUs, uint16_t gapUs) {
  motion = {};
  motion.mode = MoveMode::UNTIL_HOME_ACTIVE;
  motion.dir = dir;
  motion.maxSteps = maxSteps;
  motion.pulseUs = pulseUs;
  motion.gapUs = gapUs;
  motion.running = true;

  motor.setSpeed(pulseUs, gapUs);
  motor.startFixedMove(dir, maxSteps); // we will stop early when condition met
}

void startUntilHomeInactive(bool dir, long maxSteps, uint16_t pulseUs, uint16_t gapUs) {
  motion = {};
  motion.mode = MoveMode::UNTIL_HOME_INACTIVE;
  motion.dir = dir;
  motion.maxSteps = maxSteps;
  motion.pulseUs = pulseUs;
  motion.gapUs = gapUs;
  motion.running = true;

  motor.setSpeed(pulseUs, gapUs);
  motor.startFixedMove(dir, maxSteps);
}

// Call often. Returns true when the requested motion completes (success),
// or it will fault if overrun / timeout.
bool tickMotion() {
  if (!motion.running) return true;

  // Overrun safety ALWAYS
  if (isOverrunActive()) faultOverrun("OVERRUN during motion");

  // Advance stepper pulse generator
  motor.tick();

  // Evaluate stop conditions (for until-home modes)
  if (motion.mode == MoveMode::UNTIL_HOME_ACTIVE) {
    if (isHomeActive()) {
      motor.stop();
      motion.running = false;
      return true;
    }
  } else if (motion.mode == MoveMode::UNTIL_HOME_INACTIVE) {
    if (!isHomeActive()) {
      motor.stop();
      motion.running = false;
      return true;
    }
  } else if (motion.mode == MoveMode::FIXED_STEPS) {
    if (!motor.active) {
      motion.running = false;
      return true;
    }
  }

  // If we started a fixed move of maxSteps and it ended without meeting condition,
  // that means timeout (for until-home modes).
  if ((motion.mode == MoveMode::UNTIL_HOME_ACTIVE || motion.mode == MoveMode::UNTIL_HOME_INACTIVE) && !motor.active) {
    faultOverrun("Timeout: HOME condition not reached within max steps");
  }

  return false; // still running
}

// =========================================================
// TOP-LEVEL MACHINE STATE MACHINE
// =========================================================
enum class MainState {
  BOOT_DELAY,
  HOMING_CASE_A_LEAVE_HOME,   // CCW until HOME inactive
  HOMING_CASE_A_REENTER_HOME, // CW creep until HOME active
  HOMING_CASE_B_TO_HOME,      // CW until HOME active (fault if overrun/timeout)
  HOMING_SANITY_CW_128,       // CW 128 steps
  HOMING_SANITY_BACK_128,     // CCW 128 steps
  AFTER_SETUP_DELAY,

  READY_IDLE,
  INJECT_OUT_480,
  INJECT_RETURN_UNTIL_HOME,   // CW until HOME active, max 608
  HOME_CYCLE_PAUSE,

  FAULT
};

MainState st = MainState::BOOT_DELAY;

uint32_t bootDelayStartMs = 0;
uint32_t afterSetupStartMs = 0;
uint32_t homePauseStartMs = 0;

bool lastInjectCmd = false;

// =========================================================
// SETUP / LOOP
// =========================================================
void setup() {
  delay(DELAY_BEFORE_SETUP_MS);

  pinMode(PIN_ENA, OUTPUT);
  motorEnable(true);

  motor.begin(PIN_PUL, PIN_DIR);

  pinMode(PIN_HOME_SENSOR, INPUT_PULLUP);
  pinMode(PIN_OVERRUN_SENSOR, INPUT_PULLUP);

  pinMode(PIN_INJECT_CMD, INPUT);

  pinMode(PIN_HOME_READY, OUTPUT);
  pinMode(PIN_OVERRUN_ALARM, OUTPUT);
  digitalWrite(PIN_OVERRUN_ALARM, LOW);

  setHomeReady(false);

  bootDelayStartMs = millis();
  st = MainState::BOOT_DELAY;
}

void loop() {
  // Global overrun guard
  if (isOverrunActive()) faultOverrun("OVERRUN active (global)");

  switch (st) {
    case MainState::BOOT_DELAY: {
      // optional: could add extra power stabilization here if needed
      // (we already did delay() in setup)
      // Decide homing path based on current HOME level:
      if (isHomeActive()) {
        // CASE A: HOME is LOW -> move CCW until HOME goes HIGH
        startUntilHomeInactive(DIR_CCW, HOMING_MAX_STEPS_CCW, STEP_PULSE_US_FAST, STEP_GAP_US_FAST);
        st = MainState::HOMING_CASE_A_LEAVE_HOME;
      } else {
        // CASE B: HOME is HIGH -> move CW until HOME goes LOW
        startUntilHomeActive(DIR_CW, HOMING_MAX_STEPS_CW, STEP_PULSE_US_FAST, STEP_GAP_US_FAST);
        st = MainState::HOMING_CASE_B_TO_HOME;
      }
      break;
    }

    // ---- HOMING CASE A ----
    case MainState::HOMING_CASE_A_LEAVE_HOME: {
      if (tickMotion()) {
        // Now creep CW until HOME becomes LOW again
        startUntilHomeActive(DIR_CW, HOMING_MAX_STEPS_CW, STEP_PULSE_US_SLOW, STEP_GAP_US_SLOW);
        st = MainState::HOMING_CASE_A_REENTER_HOME;
      }
      break;
    }

    case MainState::HOMING_CASE_A_REENTER_HOME: {
      if (tickMotion()) {
        // At HOME precisely
        startFixedSteps(DIR_CW, STEPS_EXTRA_SEARCH, STEP_PULSE_US_FAST, STEP_GAP_US_FAST);
        st = MainState::HOMING_SANITY_CW_128;
      }
      break;
    }

    // ---- HOMING CASE B ----
    case MainState::HOMING_CASE_B_TO_HOME: {
      if (tickMotion()) {
        // Found HOME
        startFixedSteps(DIR_CW, STEPS_EXTRA_SEARCH, STEP_PULSE_US_FAST, STEP_GAP_US_FAST);
        st = MainState::HOMING_SANITY_CW_128;
      }
      break;
    }

    // ---- SANITY CHECK ----
    case MainState::HOMING_SANITY_CW_128: {
      if (tickMotion()) {
        // after moving CW 128, OVERRUN should NOT be active
        if (isOverrunActive()) faultOverrun("Overrun active during sanity check (expected inactive)");

        startFixedSteps(DIR_CCW, STEPS_EXTRA_SEARCH, STEP_PULSE_US_FAST, STEP_GAP_US_FAST);
        st = MainState::HOMING_SANITY_BACK_128;
      }
      break;
    }

    case MainState::HOMING_SANITY_BACK_128: {
      if (tickMotion()) {
        if (!isHomeActive()) faultOverrun("Failed to return to HOME after sanity check");

        setHomeReady(true);
        afterSetupStartMs = millis();
        st = MainState::AFTER_SETUP_DELAY;
      }
      break;
    }

    case MainState::AFTER_SETUP_DELAY: {
      if ((millis() - afterSetupStartMs) >= DELAY_AFTER_SETUP_MS) {
        st = MainState::READY_IDLE;
      }
      break;
    }

    // ---- READY / INJECT ----
    case MainState::READY_IDLE: {
      setHomeReady(isHomeActive());

      bool injectCmd = TEST_SIMULATE_PIN10_HIGH ? true : (digitalRead(PIN_INJECT_CMD) == HIGH);

      if (LEVEL_TRIGGERED) {
        // pin10 HIGH means "keep cycling", but only start when we are at HOME
        if (injectCmd && isHomeActive()) {
          // small pause at home to look realistic and avoid instant re-trigger
          homePauseStartMs = millis();
          st = MainState::HOME_CYCLE_PAUSE;
        }
      } else if (EDGE_TRIGGERED) {
        // uncomment-style behavior but implemented here:
        if (injectCmd && !lastInjectCmd) {
          homePauseStartMs = millis();
          st = MainState::HOME_CYCLE_PAUSE;
        }
        lastInjectCmd = injectCmd;
      }

      break;
    }

    case MainState::HOME_CYCLE_PAUSE: {
      if ((millis() - homePauseStartMs) >= CONTINUOUS_CYCLE_DELAY_MS) {
        // Inject out 480 steps CCW
        setHomeReady(false);
        startFixedSteps(DIR_CCW, STEPS_INJECT, STEP_PULSE_US_FAST, STEP_GAP_US_FAST);
        st = MainState::INJECT_OUT_480;
      }
      break;
    }

    case MainState::INJECT_OUT_480: {
      if (tickMotion()) {
        // Return CW until HOME is LOW again, but max 608 steps
        startUntilHomeActive(DIR_CW, MAX_RETURN_STEPS, STEP_PULSE_US_FAST, STEP_GAP_US_FAST);
        st = MainState::INJECT_RETURN_UNTIL_HOME;
      }
      break;
    }

    case MainState::INJECT_RETURN_UNTIL_HOME: {
      if (tickMotion()) {
        // We are home
        setHomeReady(true);
        st = MainState::READY_IDLE;
      }
      break;
    }

    case MainState::FAULT:
    default:
      motorEnable(false);
      setHomeReady(false);
      break;
  }
}
