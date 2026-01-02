/*  Teensy Bulb Ram Controller (Stepper + Home + Overrun + Arduino handshake)

  WHAT THIS DOES
  1) On power-up, find a precise HOME position using HOME sensor (+ OVERRUN safety).
  2) After homing, do a quick "overrun sensor sanity check" by moving CW 128 steps and back.
  3) In production:
      - Teensy asserts HOME_READY HIGH when ram is home and ready
      - Arduino pulses INJECT_CMD HIGH to request one bulb injection
      - Teensy performs:
          CCW 480 steps (inject)
          then CW until HOME is found (max 480+128 steps)
      - If OVERRUN triggers at any time -> alarm Arduino and stop/disable motor.

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

  IMPORTANT: Sensor logic (active LOW vs active HIGH)
    This sketch defaults to INPUT_PULLUP and ACTIVE_LOW = true,
    meaning:
      - HOME sensor reads LOW when "at home"
      - OVERRUN sensor reads LOW when "overrun hit"
    If your sensors are opposite, flip the *_ACTIVE_LOW constants below.

  IMPORTANT: ENA polarity
    Many stepper drivers use ENA = LOW to enable. This sketch assumes that.
    If yours is opposite, flip ENA_ACTIVE_LOW.
*/

#include <Arduino.h>

// -------------------- USER CONFIG --------------------
// Motor pins (as you requested)
constexpr uint8_t PIN_PUL = 2;
constexpr uint8_t PIN_DIR = 3;
constexpr uint8_t PIN_ENA = 4;

// Sensor pins
constexpr uint8_t PIN_HOME_SENSOR    = 8;
constexpr uint8_t PIN_OVERRUN_SENSOR = 9;

// Arduino handshake pins
constexpr uint8_t PIN_INJECT_CMD    = 10; // Arduino -> Teensy (input)
constexpr uint8_t PIN_HOME_READY    = 11; // Teensy  -> Arduino (output)
constexpr uint8_t PIN_OVERRUN_ALARM = 12; // Teensy  -> Arduino (output)

// Direction definitions (change if your mechanics are flipped)
constexpr bool DIR_CW  = LOW;   // flipped
constexpr bool DIR_CCW = HIGH;  // flipped

// Step counts
constexpr int STEPS_INJECT       = 480; // from home to inject
constexpr int STEPS_EXTRA_SEARCH = 128; // extra allowance to find home on return + overrun check travel

// Speed tuning (microseconds). Increase for slower/safer.
constexpr uint16_t STEP_PULSE_US_FAST  = 6;   // step HIGH time
constexpr uint16_t STEP_DELAY_US_FAST  = 300; // time between steps (FAST)
constexpr uint16_t STEP_PULSE_US_SLOW  = 6;
constexpr uint16_t STEP_DELAY_US_SLOW  = 900; // (SLOW) used for final "creep" onto home

// Logic polarity
constexpr bool HOME_ACTIVE_LOW    = true; // HOME is "triggered" when LOW
constexpr bool OVERRUN_ACTIVE_LOW = true; // OVERRUN is "triggered" when LOW
constexpr bool ENA_ACTIVE_LOW     = true; // ENA LOW enables driver

// Safety timeouts to avoid infinite motion if a sensor fails (in steps)
constexpr int HOMING_MAX_STEPS_CW  = 4000;
constexpr int HOMING_MAX_STEPS_CCW = 4000;

// Alarm behavior
constexpr uint16_t OVERRUN_ALARM_PULSE_MS = 1000;

// -----------------------------------------------------

enum class State {
  BOOT_HOMING,
  READY_IDLE,
  INJECTING,
  RETURNING,
  FAULT
};

State state = State::BOOT_HOMING;
bool lastInjectCmd = false;

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
  // ENA polarity handling
  if (ENA_ACTIVE_LOW) digitalWrite(PIN_ENA, en ? LOW : HIGH);
  else                digitalWrite(PIN_ENA, en ? HIGH : LOW);
}

inline void setDir(bool dir) {
  digitalWrite(PIN_DIR, dir);
}

// Single step pulse
inline void stepOnce(uint16_t pulseHighUs, uint16_t stepDelayUs) {
  digitalWrite(PIN_PUL, HIGH);
  delayMicroseconds(pulseHighUs);
  digitalWrite(PIN_PUL, LOW);
  delayMicroseconds(stepDelayUs);
}

// Emergency stop + notify Arduino
[[noreturn]] void faultOverrun(const char* reason) {
  // Stop outputs / disable motor
  setHomeReady(false);
  motorEnable(false);

  // Pulse alarm to Arduino for 1 second
  digitalWrite(PIN_OVERRUN_ALARM, HIGH);
  delay(OVERRUN_ALARM_PULSE_MS);
  digitalWrite(PIN_OVERRUN_ALARM, LOW);

  // If you want serial debugging, uncomment:
  // Serial.println(reason);

  state = State::FAULT;
  while (true) {
    // Hard halt. Power cycle to reset, or add a reset input if desired.
    delay(100);
  }
}

// Move N steps in a direction, watching OVERRUN the whole time
void moveStepsWithOverrunWatch(bool dir, int steps, uint16_t pulseUs, uint16_t delayUs) {
  setDir(dir);
  for (int i = 0; i < steps; i++) {
    if (isOverrunActive()) faultOverrun("OVERRUN during moveStepsWithOverrunWatch()");
    stepOnce(pulseUs, delayUs);
  }
}

// Homing routine described in your message
void doHoming() {
  setHomeReady(false);
  motorEnable(true);

  // --- CASE A: On power-up, HOME is LOW (we are on/near the home sensor) ---
  if (isHomeActive()) {
    // Move CCW until HOME goes HIGH (leave the sensor)
    setDir(DIR_CCW);
    int steps = 0;
    while (isHomeActive()) {
      if (steps++ > HOMING_MAX_STEPS_CCW) faultOverrun("Homing CCW timeout leaving HOME");
      if (isOverrunActive()) faultOverrun("OVERRUN during homing (CCW leave HOME)");
      stepOnce(STEP_PULSE_US_FAST, STEP_DELAY_US_FAST);
    }

    // Now creep CW slowly until HOME becomes LOW again (re-enter home precisely)
    setDir(DIR_CW);
    steps = 0;
    while (!isHomeActive()) {
      if (steps++ > HOMING_MAX_STEPS_CW) faultOverrun("Homing CW timeout re-entering HOME");
      if (isOverrunActive()) faultOverrun("OVERRUN during homing (CW re-enter HOME)");
      stepOnce(STEP_PULSE_US_SLOW, STEP_DELAY_US_SLOW);
    }
    // We are now at HOME (LOW) precisely.
  }
  // --- CASE B: On power-up, HOME is HIGH (not at home) ---
  else {
    // Move CW until HOME goes LOW. If OVERRUN hits first, fault.
    setDir(DIR_CW);
    int steps = 0;
    while (!isHomeActive()) {
      if (steps++ > HOMING_MAX_STEPS_CW) faultOverrun("Homing CW timeout searching HOME");
      if (isOverrunActive()) faultOverrun("OVERRUN hit before HOME during homing (CW)");
      stepOnce(STEP_PULSE_US_FAST, STEP_DELAY_US_FAST);
    }
    // HOME is now LOW -> exactly at home per your spec.
  }

  // --- Overrun sensor sanity check (move CW 128, verify overrun not triggered, return) ---
  // NOTE: You wrote "it should give a LOW signal" after moving CW 128.
  // Most setups expect "overrun NOT triggered" here. This code enforces that: overrun must be INACTIVE.
  moveStepsWithOverrunWatch(DIR_CW, STEPS_EXTRA_SEARCH, STEP_PULSE_US_FAST, STEP_DELAY_US_FAST);

  if (isOverrunActive()) {
    // If your wiring truly expects LOW here, flip OVERRUN_ACTIVE_LOW or change this check.
    faultOverrun("Overrun sensor active during sanity check (expected inactive)");
  }

  // Go back to HOME by moving CCW 128 steps.
  moveStepsWithOverrunWatch(DIR_CCW, STEPS_EXTRA_SEARCH, STEP_PULSE_US_FAST, STEP_DELAY_US_FAST);

  // Confirm we are still at home (HOME should be active/LOW)
  if (!isHomeActive()) {
    // Not necessarily "overrun", but it's a hard fault in practice
    faultOverrun("Failed to return to HOME after sanity check");
  }

  // Ready for production
  setHomeReady(true);
}

// Do one injection cycle: CCW 480 (inject) then CW back to HOME (max 480+128)
void doInjectCycle() {
  setHomeReady(false);
  motorEnable(true);

  // Inject: from HOME go CCW 480 steps
  moveStepsWithOverrunWatch(DIR_CCW, STEPS_INJECT, STEP_PULSE_US_FAST, STEP_DELAY_US_FAST);

  // Return: go CW until HOME found, but do NOT exceed 480+128 steps.
  setDir(DIR_CW);
  const int maxReturnSteps = STEPS_INJECT + STEPS_EXTRA_SEARCH;
  for (int i = 0; i < maxReturnSteps; i++) {
    if (isOverrunActive()) faultOverrun("OVERRUN during return-to-home");
    if (isHomeActive()) {
      // Back at home
      setHomeReady(true);
      return;
    }
    stepOnce(STEP_PULSE_US_FAST, STEP_DELAY_US_FAST);
  }

  // If we got here: we moved 480+128 CW and never saw HOME -> missed both sensors region
  faultOverrun("Missed HOME on return (exceeded 480+128 steps)");
}

void setup() {
  // Optional debugging:
  // Serial.begin(115200);

  pinMode(PIN_PUL, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);

  // Sensors: default to pullups (common for mechanical switches / NPN sensors with open collector)
  pinMode(PIN_HOME_SENSOR, INPUT_PULLUP);
  pinMode(PIN_OVERRUN_SENSOR, INPUT_PULLUP);

  // Arduino command input:
  // If Arduino drives a strong HIGH/LOW, plain INPUT is fine.
  // If you need a default LOW, you can use INPUT_PULLDOWN on many Teensy boards.
  pinMode(PIN_INJECT_CMD, INPUT);

  // Outputs to Arduino
  pinMode(PIN_HOME_READY, OUTPUT);
  pinMode(PIN_OVERRUN_ALARM, OUTPUT);

  digitalWrite(PIN_OVERRUN_ALARM, LOW);
  setHomeReady(false);

  // Motor initial
  digitalWrite(PIN_PUL, LOW);
  setDir(DIR_CW);
  motorEnable(true);

  // Start homing immediately
  state = State::BOOT_HOMING;
}

void loop() {
  switch (state) {
    case State::BOOT_HOMING: {
      if (isOverrunActive()) faultOverrun("OVERRUN active at boot");
      doHoming();
      state = State::READY_IDLE;
      break;
    }

    case State::READY_IDLE: {
      // In production, continuously verify overrun isn't hit
      if (isOverrunActive()) faultOverrun("OVERRUN during READY_IDLE");

      // Keep HOME_READY asserted when we are truly home
      // (If you want it strictly from logic, you can also do: setHomeReady(isHomeActive()); )
      setHomeReady(true);

      // Rising-edge detect on INJECT_CMD
      bool injectCmd = digitalRead(PIN_INJECT_CMD) == HIGH;
      if (injectCmd && !lastInjectCmd) {
        state = State::INJECTING;
      }
      lastInjectCmd = injectCmd;

      break;
    }

    case State::INJECTING: {
      doInjectCycle();
      // After cycle, we should be home and HOME_READY already set HIGH by doInjectCycle()
      state = State::READY_IDLE;
      break;
    }

    case State::RETURNING:
      // Not used (kept for future expansion)
      state = State::READY_IDLE;
      break;

    case State::FAULT:
    default:
      // We should never reach here because faultOverrun() halts.
      motorEnable(false);
      setHomeReady(false);
      break;
  }
}
