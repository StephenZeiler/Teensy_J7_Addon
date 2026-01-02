/*  Teensy Bulb Ram Controller (Stepper + Home + Overrun + Arduino handshake)

  MODES IN THIS FILE
  ------------------
  TEST MODE (no Arduino yet):
    - After homing + 3s delay, continuously inject:
        HOME -> inject -> return HOME -> repeat
    - Simulates "Arduino keeps pin10 HIGH".

  PRODUCTION MODE (Arduino connected):
    - Teensy waits for Arduino INJECT_CMD on pin10 (level-triggered by default):
        If pin10 HIGH: each time we get HOME, we do one inject cycle.

  IMPORTANT PRODUCTION CHANGE (your latest request)
  -------------------------------------------------
  After the ram travels 480 steps (inject stroke), DO NOT return a fixed 480.
  Instead return CW UNTIL HOME is active (LOW), but do NOT travel more than 608 steps.
  (608 = 480 + 128 safety window). If exceeded, fault.

  ------------------------------------------------------------
  PIN LAYOUT (suggested)
  Motor driver:
    PUL (step)  -> Teensy pin 2
    DIR         -> Teensy pin 3
    ENA         -> Teensy pin 4

  Sensors (to Teensy inputs):
    HOME_SENSOR    -> Teensy pin 8
    OVERRUN_SENSOR -> Teensy pin 9

  Arduino <-> Teensy signals:
    INJECT_CMD     (Arduino -> Teensy input)  -> Teensy pin 10
    HOME_READY     (Teensy -> Arduino output) -> Teensy pin 11
    OVERRUN_ALARM  (Teensy -> Arduino output) -> Teensy pin 12

  ------------------------------------------------------------
  SENSOR LOGIC
  Default assumes INPUT_PULLUP and ACTIVE_LOW = true:
    - HOME reads LOW when "at home"
    - OVERRUN reads LOW when "hit overrun"
*/

#include <Arduino.h>

// -------------------- PIN CONFIG --------------------
constexpr uint8_t PIN_PUL = 2;   // step
constexpr uint8_t PIN_DIR = 3;   // direction
constexpr uint8_t PIN_ENA = 4;   // enable

constexpr uint8_t PIN_HOME_SENSOR    = 8;
constexpr uint8_t PIN_OVERRUN_SENSOR = 9;

constexpr uint8_t PIN_INJECT_CMD    = 10; // Arduino -> Teensy
constexpr uint8_t PIN_HOME_READY    = 11; // Teensy  -> Arduino
constexpr uint8_t PIN_OVERRUN_ALARM = 12; // Teensy  -> Arduino

// -------------------- DIRECTION DEFINITIONS --------------------
// To flip motor direction, swap these two constants.
constexpr bool DIR_CW  = HIGH; // clockwise
constexpr bool DIR_CCW = LOW;  // counter-clockwise

// -------------------- MOTION SETTINGS --------------------
constexpr int STEPS_INJECT = 480; // inject stroke steps
constexpr int MAX_RETURN_STEPS = 608; // 480 + 128 safety window (do not exceed)

// Speed tuning (microseconds)
constexpr uint16_t STEP_PULSE_US_FAST = 6;
constexpr uint16_t STEP_DELAY_US_FAST = 300;

constexpr uint16_t STEP_PULSE_US_SLOW = 6;
constexpr uint16_t STEP_DELAY_US_SLOW = 900;

// -------------------- POLARITY SETTINGS --------------------
constexpr bool HOME_ACTIVE_LOW    = true;
constexpr bool OVERRUN_ACTIVE_LOW = true;

constexpr bool ENA_ACTIVE_LOW = true;

// -------------------- HOMING SAFETIES --------------------
constexpr int HOMING_MAX_STEPS_CW  = 4000;
constexpr int HOMING_MAX_STEPS_CCW = 4000;

// -------------------- ALARM --------------------
constexpr uint16_t OVERRUN_ALARM_PULSE_MS = 1000;

// -------------------- STARTUP DELAYS (your request) --------------------
constexpr uint16_t DELAY_BEFORE_SETUP_MS = 1000; // 1 second before anything happens
constexpr uint16_t DELAY_AFTER_SETUP_MS  = 3000; // 3 seconds after homing complete

// -------------------- TEST / PRODUCTION SWITCHES --------------------
/*
  TEST_SIMULATE_PIN10_HIGH:
    true  = behave like Arduino holds pin10 HIGH forever (continuous injections)
    false = read the actual pin10 input from Arduino
*/
constexpr bool TEST_SIMULATE_PIN10_HIGH = true; // TEST: true.  PRODUCTION: false.

// Trigger behavior (for production)
constexpr bool LEVEL_TRIGGERED = true;  // pin10 HIGH -> keep cycling injections when home
constexpr bool EDGE_TRIGGERED  = false; // (optional) pulse HIGH once per injection

// Delay at home between cycles (helps realism & prevents hammering)
constexpr uint16_t CONTINUOUS_CYCLE_DELAY_MS = 150;

// -------------------- STATE --------------------
enum class State {
  BOOT_HOMING,
  READY_IDLE,
  INJECTING,
  FAULT
};

State state = State::BOOT_HOMING;
bool lastInjectCmd = false; // used only for EDGE_TRIGGERED

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

inline void stepOnce(uint16_t pulseHighUs, uint16_t stepDelayUs) {
  digitalWrite(PIN_PUL, HIGH);
  delayMicroseconds(pulseHighUs);
  digitalWrite(PIN_PUL, LOW);
  delayMicroseconds(stepDelayUs);
}

[[noreturn]] void faultOverrun(const char* reason) {
  (void)reason;

  // Immediately disable motion + clear ready
  setHomeReady(false);
  motorEnable(false);

  // Signal Arduino for 1 second
  digitalWrite(PIN_OVERRUN_ALARM, HIGH);
  delay(OVERRUN_ALARM_PULSE_MS);
  digitalWrite(PIN_OVERRUN_ALARM, LOW);

  state = State::FAULT;
  while (true) delay(100);
}

void moveStepsWithOverrunWatch(bool dir, int steps, uint16_t pulseUs, uint16_t delayUs) {
  setDir(dir);
  for (int i = 0; i < steps; i++) {
    if (isOverrunActive()) faultOverrun("OVERRUN during moveStepsWithOverrunWatch()");
    stepOnce(pulseUs, delayUs);
  }
}

// -------------------- HOMING --------------------
void doHoming() {
  setHomeReady(false);
  motorEnable(true);

  // CASE A: HOME is LOW at boot (on/near sensor)
  if (isHomeActive()) {
    // Move CCW until HOME becomes HIGH (leave sensor)
    setDir(DIR_CCW);
    int steps = 0;
    while (isHomeActive()) {
      if (steps++ > HOMING_MAX_STEPS_CCW) faultOverrun("Homing timeout leaving HOME (CCW)");
      if (isOverrunActive()) faultOverrun("OVERRUN during homing CCW");
      stepOnce(STEP_PULSE_US_FAST, STEP_DELAY_US_FAST);
    }

    // Creep CW until HOME becomes LOW again (re-enter precisely)
    setDir(DIR_CW);
    steps = 0;
    while (!isHomeActive()) {
      if (steps++ > HOMING_MAX_STEPS_CW) faultOverrun("Homing timeout re-enter HOME (CW)");
      if (isOverrunActive()) faultOverrun("OVERRUN during homing CW creep");
      stepOnce(STEP_PULSE_US_SLOW, STEP_DELAY_US_SLOW);
    }
  }
  // CASE B: HOME is HIGH at boot (not at home)
  else {
    // Move CW until HOME goes LOW; if OVERRUN hits first -> fault
    setDir(DIR_CW);
    int steps = 0;
    while (!isHomeActive()) {
      if (steps++ > HOMING_MAX_STEPS_CW) faultOverrun("Homing timeout searching HOME (CW)");
      if (isOverrunActive()) faultOverrun("OVERRUN hit before HOME during homing (CW)");
      stepOnce(STEP_PULSE_US_FAST, STEP_DELAY_US_FAST);
    }
  }

  // Overrun sensor sanity check:
  // Move CW 128, ensure OVERRUN is NOT active, then move back CCW 128
  moveStepsWithOverrunWatch(DIR_CW, 128, STEP_PULSE_US_FAST, STEP_DELAY_US_FAST);
  if (isOverrunActive()) faultOverrun("Overrun active during sanity check (expected inactive)");
  moveStepsWithOverrunWatch(DIR_CCW, 128, STEP_PULSE_US_FAST, STEP_DELAY_US_FAST);

  // Confirm we're at HOME at end
  if (!isHomeActive()) faultOverrun("Not at HOME after sanity-check return");

  setHomeReady(true);
}

// -------------------- INJECTION CYCLE --------------------
void doInjectCycle() {
  setHomeReady(false);
  motorEnable(true);

  // Safety: should never inject if already in overrun
  if (isOverrunActive()) faultOverrun("OVERRUN active at start of inject");

  // 1) Inject stroke: from HOME go CCW 480 steps
  moveStepsWithOverrunWatch(DIR_CCW, STEPS_INJECT, STEP_PULSE_US_FAST, STEP_DELAY_US_FAST);

  // 2) Return: go CW until HOME is active (LOW) again,
  //    but do NOT exceed 608 steps (or you'd pass the overrun region).
  setDir(DIR_CW);
  for (int i = 0; i < MAX_RETURN_STEPS; i++) {
    if (isOverrunActive()) faultOverrun("OVERRUN during return-to-home");

    if (isHomeActive()) {
      setHomeReady(true);
      return;
    }

    stepOnce(STEP_PULSE_US_FAST, STEP_DELAY_US_FAST);
  }

  // If we got here: travelled 608 CW and never found HOME
  faultOverrun("Missed HOME on return (exceeded 608 steps)");
}

// -------------------- SETUP / LOOP --------------------
void setup() {
  delay(DELAY_BEFORE_SETUP_MS); // 1 second before setup (requested)

  pinMode(PIN_PUL, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);

  pinMode(PIN_HOME_SENSOR, INPUT_PULLUP);
  pinMode(PIN_OVERRUN_SENSOR, INPUT_PULLUP);

  // Arduino command input:
  // If your pin might float in production, and your Teensy supports it, use INPUT_PULLDOWN:
  // pinMode(PIN_INJECT_CMD, INPUT_PULLDOWN);   // <-- uncomment if needed
  pinMode(PIN_INJECT_CMD, INPUT);

  pinMode(PIN_HOME_READY, OUTPUT);
  pinMode(PIN_OVERRUN_ALARM, OUTPUT);

  digitalWrite(PIN_OVERRUN_ALARM, LOW);
  setHomeReady(false);

  digitalWrite(PIN_PUL, LOW);
  setDir(DIR_CW);
  motorEnable(true);

  state = State::BOOT_HOMING;
}

void loop() {
  switch (state) {

    case State::BOOT_HOMING: {
      if (isOverrunActive()) faultOverrun("OVERRUN active at boot");
      doHoming();

      delay(DELAY_AFTER_SETUP_MS); // 3 seconds after setup (requested)

      state = State::READY_IDLE;
      break;
    }

    case State::READY_IDLE: {
      if (isOverrunActive()) faultOverrun("OVERRUN during READY_IDLE");

      // Only claim ready when truly at home
      setHomeReady(isHomeActive());

      // Source of inject command:
      // TEST: simulate Arduino holding HIGH
      // PROD: read actual pin
      bool injectCmd = TEST_SIMULATE_PIN10_HIGH ? true : (digitalRead(PIN_INJECT_CMD) == HIGH);

      // ----------------------------
      // PRODUCTION / TEST TRIGGERS
      // ----------------------------

      // LEVEL TRIGGERED:
      // If pin10 is HIGH, inject whenever we are home (repeat cycle).
      if (LEVEL_TRIGGERED) {
        if (injectCmd && isHomeActive()) {
          delay(CONTINUOUS_CYCLE_DELAY_MS);
          state = State::INJECTING;
        }
        break;
      }

      // EDGE TRIGGERED (optional):
      // One injection per rising edge pulse.
      // To use:
      //   - set LEVEL_TRIGGERED=false, EDGE_TRIGGERED=true
      //   - uncomment below
      //
      // if (EDGE_TRIGGERED) {
      //   if (injectCmd && !lastInjectCmd) state = State::INJECTING;
      //   lastInjectCmd = injectCmd;
      //   break;
      // }

      break;
    }

    case State::INJECTING: {
      doInjectCycle();
      state = State::READY_IDLE;
      break;
    }

    case State::FAULT:
    default: {
      motorEnable(false);
      setHomeReady(false);
      break;
    }
  }
}

/* -----------------------------------------------------------------------
   PRODUCTION CHECKLIST (what to change when Arduino is connected)
   -----------------------------------------------------------------------
   1) Disable test simulation:
        constexpr bool TEST_SIMULATE_PIN10_HIGH = false;

   2) Choose trigger style:
      - If Arduino holds HIGH to keep cycling: LEVEL_TRIGGERED=true (recommended)
      - If Arduino pulses one bulb at a time: set EDGE_TRIGGERED=true and uncomment edge block.

   3) If pin10 may float:
      - Use INPUT_PULLDOWN if supported, or add a physical pulldown resistor.

   4) If direction is backwards:
      - swap DIR_CW and DIR_CCW.
-------------------------------------------------------------------------*/
