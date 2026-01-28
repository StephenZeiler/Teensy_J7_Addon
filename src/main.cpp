#include <Arduino.h>
#include <IntervalTimer.h>

constexpr uint8_t PIN_PUL = 2;
constexpr uint8_t PIN_DIR = 3;
constexpr uint8_t PIN_ENA = 4;

constexpr uint8_t PIN_HOME_SENSOR = 29;
constexpr uint8_t PIN_OVERRUN_SENSOR = 30;

constexpr uint8_t PIN_INJECT_CMD    = 10;
constexpr uint8_t PIN_HOME_READY    = 11;
constexpr uint8_t PIN_OVERRUN_ALARM = 12;

constexpr bool DIR_CW  = HIGH;
constexpr bool DIR_CCW = LOW;

constexpr int STEPS_INJECT       = 480;
constexpr int STEPS_EXTRA_SEARCH = 128;

constexpr uint16_t STEP_PULSE_US_FAST  = 6;
constexpr uint16_t STEP_DELAY_US_FAST  = 300;
constexpr uint16_t STEP_PULSE_US_SLOW  = 6;
constexpr uint16_t STEP_DELAY_US_SLOW  = 900;

constexpr bool HOME_ACTIVE_HIGH    = true;
constexpr bool OVERRUN_ACTIVE_HIGH = true;

constexpr int HOMING_MAX_STEPS_CW  = 4000;
constexpr int HOMING_MAX_STEPS_CCW = 4000;

constexpr uint16_t OVERRUN_ALARM_PULSE_MS = 1000;

enum class State {
  BOOT_HOMING,
  READY_IDLE,
  INJECTING,
  RETURNING,
  FAULT
};

State state = State::BOOT_HOMING;

inline bool isHomeActive() {
  bool v = digitalRead(PIN_HOME_SENSOR);
  return HOME_ACTIVE_HIGH ? (v == HIGH) : (v == LOW);
}

inline bool isOverrunActive() {
  bool v = digitalRead(PIN_OVERRUN_SENSOR);
  return OVERRUN_ACTIVE_HIGH ? (v == HIGH) : (v == LOW);
}

inline void setHomeReady(bool ready) {
  digitalWrite(PIN_HOME_READY, ready ? HIGH : LOW);
}

inline void setDir(bool dir) {
  digitalWrite(PIN_DIR, dir);
}

IntervalTimer stepTimer;

static volatile uint8_t  stepState = 0;
static volatile uint16_t highUsReq = 0;
static volatile uint16_t lowUsReq  = 0;
static volatile uint16_t ticksLeft = 0;

void stepperISR() {
  if (stepState == 0) return;

  if (ticksLeft > 0) {
    ticksLeft--;
    return;
  }

  if (stepState == 1) {
    digitalWriteFast(PIN_PUL, LOW);
    stepState = 2;
    ticksLeft = (lowUsReq > 0) ? (uint16_t)(lowUsReq - 1) : 0;
    return;
  }

  if (stepState == 2) {
    stepState = 0;
    return;
  }
}

inline void stepOnce(uint16_t pulseHighUs, uint16_t stepDelayUs) {
  noInterrupts();
  highUsReq = pulseHighUs;
  lowUsReq  = stepDelayUs;
  stepState = 1;
  ticksLeft = (highUsReq > 0) ? (uint16_t)(highUsReq - 1) : 0;
  digitalWriteFast(PIN_PUL, HIGH);
  interrupts();

  while (stepState != 0) {
  }
}

[[noreturn]] void faultOverrun(const char* reason) {
  setHomeReady(false);

  digitalWrite(PIN_OVERRUN_ALARM, HIGH);
  delay(OVERRUN_ALARM_PULSE_MS);
  digitalWrite(PIN_OVERRUN_ALARM, LOW);

  state = State::FAULT;
  while (true) {
    delay(100);
  }
}

void moveStepsWithOverrunWatch(bool dir, int steps, uint16_t pulseUs, uint16_t delayUs) {
  setDir(dir);
  for (int i = 0; i < steps; i++) {
    if (isOverrunActive()) faultOverrun("OVERRUN during moveStepsWithOverrunWatch()");
    stepOnce(pulseUs, delayUs);
  }
}

void doHoming() {
  setHomeReady(false);

  if (isHomeActive()) {
    setDir(DIR_CCW);
    int steps = 0;
    while (isHomeActive()) {
      if (steps++ > HOMING_MAX_STEPS_CCW) faultOverrun("Homing CCW timeout leaving HOME");
      if (isOverrunActive()) faultOverrun("OVERRUN during homing (CCW leave HOME)");
      stepOnce(STEP_PULSE_US_FAST, STEP_DELAY_US_FAST);
    }

    setDir(DIR_CW);
    steps = 0;
    while (!isHomeActive()) {
      if (steps++ > HOMING_MAX_STEPS_CW) faultOverrun("Homing CW timeout re-entering HOME");
      if (isOverrunActive()) faultOverrun("OVERRUN during homing (CW re-enter HOME)");
      stepOnce(STEP_PULSE_US_SLOW, STEP_DELAY_US_SLOW);
    }

  } else {

    setDir(DIR_CW);
    int steps = 0;
    while (!isHomeActive()) {
      if (steps++ > HOMING_MAX_STEPS_CW) faultOverrun("Homing CW timeout searching HOME");
      if (isOverrunActive()) faultOverrun("OVERRUN hit before HOME during homing (CW)");
      stepOnce(STEP_PULSE_US_FAST, STEP_DELAY_US_FAST);
    }

  }

  moveStepsWithOverrunWatch(DIR_CW, STEPS_EXTRA_SEARCH, STEP_PULSE_US_FAST, STEP_DELAY_US_FAST);

  if (isOverrunActive()) {
    faultOverrun("Overrun sensor active during sanity check (expected inactive)");
  }

  moveStepsWithOverrunWatch(DIR_CCW, STEPS_EXTRA_SEARCH, STEP_PULSE_US_FAST, STEP_DELAY_US_FAST);

  if (!isHomeActive()) {
    faultOverrun("Failed to return to HOME after sanity check");
  }

  setHomeReady(true);
}

void doInjectCycle() {
  setHomeReady(false);

  moveStepsWithOverrunWatch(DIR_CCW, STEPS_INJECT, STEP_PULSE_US_FAST, STEP_DELAY_US_FAST);

  setDir(DIR_CW);
  const int maxReturnSteps = STEPS_INJECT + STEPS_EXTRA_SEARCH;
  for (int i = 0; i < maxReturnSteps; i++) {
    if (isOverrunActive()) faultOverrun("OVERRUN during return-to-home");
    if (isHomeActive()) {
      setHomeReady(true);
      return;
    }
    stepOnce(STEP_PULSE_US_FAST, STEP_DELAY_US_FAST);
  }

  faultOverrun("Missed HOME on return (exceeded 480+128 steps)");
}

void setup() {
  delay(1000);
  pinMode(PIN_PUL, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);

  pinMode(PIN_HOME_SENSOR, INPUT);
  pinMode(PIN_OVERRUN_SENSOR, INPUT);

  pinMode(PIN_INJECT_CMD, INPUT_PULLUP);

  pinMode(PIN_HOME_READY, OUTPUT);
  pinMode(PIN_OVERRUN_ALARM, OUTPUT);

  digitalWrite(PIN_ENA, LOW);
  digitalWrite(PIN_OVERRUN_ALARM, LOW);
  setHomeReady(false);

  digitalWrite(PIN_PUL, LOW);
  setDir(DIR_CW);

  stepTimer.begin(stepperISR, 1);

  state = State::BOOT_HOMING;
}

// constexpr uint16_t STEP_PULSE_US = 5;
// constexpr uint16_t STEP_DELAY_US = 1500;
// constexpr bool DIR_A = HIGH;
// constexpr bool DIR_B = LOW;
// void stepOnce() {
//   digitalWrite(PIN_PUL, HIGH);
//   delayMicroseconds(STEP_PULSE_US);
//   digitalWrite(PIN_PUL, LOW);
//   delayMicroseconds(STEP_DELAY_US);
// }

// void moveSteps(bool dir, int steps) {
//   digitalWrite(PIN_DIR, dir);
//   for (int i = 0; i < steps; i++) {
//     stepOnce();
//   }
// }

void loop() {
  switch (state) {
    case State::BOOT_HOMING: {
      if (isOverrunActive()) faultOverrun("OVERRUN active at boot");
      doHoming();
      state = State::READY_IDLE;
      break;
    }

    case State::READY_IDLE: {
      if (isOverrunActive()) faultOverrun("OVERRUN during READY_IDLE");

      setHomeReady(true);

      bool injectCmd = digitalRead(PIN_INJECT_CMD) == HIGH;
      if (injectCmd && isHomeActive()) {
        delay(150);
        state = State::INJECTING;
      }
      break;
    }

    case State::INJECTING: {
      doInjectCycle();
      state = State::READY_IDLE;
      break;
    }

    case State::RETURNING:
      state = State::READY_IDLE;
      break;

    case State::FAULT:
    default:
      setHomeReady(false);
      break;
  }

  // bool homeActive = (digitalRead(PIN_HOME_SENSOR) == HIGH);

  // if (homeActive) {
  //   moveSteps(DIR_A, 20);
  //   delay(100);
  //   moveSteps(DIR_B, 20);
  //   delay(100);
  // }
  // else {
  //   delay(50);
  // }
}
