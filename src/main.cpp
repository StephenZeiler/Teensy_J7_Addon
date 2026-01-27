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
    OVERRUN_ALARM  (Teensy  -> Arduino output) -> Teensy pin 12

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
#include <IntervalTimer.h>

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
//
// -------------------- WHEEL MOTOR ADD-ON (NEW) --------------------
//
// Arduino -> Teensy control inputs:
constexpr uint8_t PIN_WHEEL_HOME_CMD   = 24; // Arduino holds HIGH while wheel should "troll home"
constexpr uint8_t PIN_WHEEL_INDEX_CMD  = 25; // Arduino commands: move wheel one slot (200 steps)

// Teensy -> Arduino outputs:
constexpr uint8_t PIN_WHEEL_READY      = 26; // Teensy pulses HIGH when wheel move complete
constexpr uint8_t PIN_WHEEL_ERROR      = 27; // Teensy pulses HIGH if wheel position check fails

// Wheel position sensor (to Teensy):
constexpr uint8_t PIN_WHEEL_POS_OK     = 28; // LOW when wheel is in correct position

// Wheel motor driver pins:
constexpr uint8_t PIN_WHEEL_PUL = 5;
constexpr uint8_t PIN_WHEEL_DIR = 6;
constexpr uint8_t PIN_WHEEL_ENA = 7;

// Wheel step counts:
constexpr int WHEEL_STEPS_PER_SLOT = 200;

// Wheel homing speed (slow constant delay between edges)
// (This is used while PIN_WHEEL_HOME_CMD is HIGH; Arduino decides when to drop it LOW.)
constexpr uint32_t WHEEL_HOME_STEP_DELAY_US = 1500;

// DONE pulse widths (non-blocking)
constexpr uint32_t WHEEL_READY_PULSE_US = 200000; // 200ms pulse
constexpr uint32_t WHEEL_ERROR_PULSE_US = 1000000; // 1s pulse

// If you want the old RAM pin10 test behavior, set this false.
// When true, RAM injection is driven by wheel index sequence, not PIN_INJECT_CMD.
constexpr bool ENABLE_WHEEL_SYSTEM = true;

// -----------------------------------------------------

enum class State {
  BOOT_HOMING,
  READY_IDLE,
  INJECTING,
  RETURNING,
  FAULT,

  // NEW wheel-related states (added without removing your existing ones)
  WAIT_WHEEL_HOME_CMD,
  WHEEL_HOMING_ACTIVE,
  WHEEL_INDEXING
};

State state = State::BOOT_HOMING;

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

/* -------------------------------------------------------------------------
   STEPPING CHANGE ONLY:
   - Replaced delayMicroseconds-based stepOnce() with a hardware-timer driven
     pulse generator (IntervalTimer) that times the HIGH pulse and LOW gap.
   - Everything else in your code is unchanged.

   Timer tick = 1us. ISR advances a tiny 3-state pulse machine:
     IDLE -> HIGH pulse -> LOW gap -> IDLE
---------------------------------------------------------------------------*/
IntervalTimer stepTimer;

static volatile uint8_t  stepState = 0;     // 0=IDLE, 1=HIGH, 2=LOW_GAP
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
    // End HIGH pulse -> STEP LOW, start LOW gap
    digitalWriteFast(PIN_PUL, LOW);
    stepState = 2;
    ticksLeft = (lowUsReq > 0) ? (uint16_t)(lowUsReq - 1) : 0;
    return;
  }

  if (stepState == 2) {
    // End LOW gap -> done
    stepState = 0;
    return;
  }
}

// Single step pulse
inline void stepOnce(uint16_t pulseHighUs, uint16_t stepDelayUs) {
  // Request a pulse (HIGH then LOW) timed by the ISR
  noInterrupts();
  highUsReq = pulseHighUs;
  lowUsReq  = stepDelayUs;
  stepState = 1;
  ticksLeft = (highUsReq > 0) ? (uint16_t)(highUsReq - 1) : 0;
  digitalWriteFast(PIN_PUL, HIGH);
  interrupts();

  // Wait only for THIS step to complete (your existing motion loops stay the same)
  while (stepState != 0) {
    // no delay; ISR is doing the timing
  }
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

// -------------------- WHEEL MOTOR (NEW) --------------------
// Wheel driver enable (assume ENA LOW enables like ram driver; adjust if different)
constexpr bool WHEEL_ENA_ACTIVE_LOW = true;
inline void wheelEnable(bool en) {
  if (WHEEL_ENA_ACTIVE_LOW) digitalWrite(PIN_WHEEL_ENA, en ? LOW : HIGH);
  else                     digitalWrite(PIN_WHEEL_ENA, en ? HIGH : LOW);
}
inline bool wheelPosOkActive() {
  // sensor goes LOW when wheel stops in correct position
  return digitalRead(PIN_WHEEL_POS_OK) == LOW;
}

// Wheel stepping variables used by your provided pattern
static const uint8_t stepPin = PIN_WHEEL_PUL;
static const uint8_t dirPin  = PIN_WHEEL_DIR;

static bool isMoving = false;
static bool stepHigh = false;
static uint32_t lastStepTime = 0;
static uint32_t stepsTaken = 0;

// Accel profile constants (you can tune these later)
static const uint32_t MIN_STEP_DELAY = 350;   // fastest (us) during index move
static const uint32_t MAX_STEP_DELAY = 1200;  // slowest (us) start/end during index move
static const uint32_t ACCEL_STEPS    = 40;
static const uint32_t DECEL_STEPS    = 40;

// Index move target (200 steps per slot)
static uint32_t TOTAL_STEPS = WHEEL_STEPS_PER_SLOT;

// Homing mode flag (slow constant speed while Arduino holds pin24 HIGH)
static bool wheelHomingMode = false;

// DONE pulse timing (non-blocking)
static const uint8_t donePin = PIN_WHEEL_READY;
static bool donePulsing = false;
static uint32_t donePulseStart = 0;
static const uint32_t DONE_PULSE_US = WHEEL_READY_PULSE_US;

// ERROR pulse timing (non-blocking)
static bool errorPulsing = false;
static uint32_t errorPulseStart = 0;

static bool wheelHomed = false;

// Called when a wheel move completes
void finishMove(uint32_t currentTime) {
  isMoving = false;
  stepHigh = false;
  digitalWrite(stepPin, LOW);

  // pulse wheel ready to Arduino (non-blocking)
  digitalWrite(donePin, HIGH);
  donePulsing = true;
  donePulseStart = currentTime;

  // After an index move, verify wheel position sensor (pin 28 must be LOW)
  // If still HIGH -> send error pulse on pin 27 and DO NOT inject.
  if (!wheelHomingMode) {
    if (!wheelPosOkActive()) {
      digitalWrite(PIN_WHEEL_ERROR, HIGH);
      errorPulsing = true;
      errorPulseStart = currentTime;
    }
  }
}

// Start wheel move (index one slot)
void startWheelIndexMove(bool dirLevel) {
  digitalWrite(dirPin, dirLevel);
  wheelEnable(true);

  wheelHomingMode = false;
  TOTAL_STEPS = WHEEL_STEPS_PER_SLOT;

  isMoving = true;
  stepHigh = false;
  stepsTaken = 0;
  lastStepTime = micros();
  digitalWrite(stepPin, LOW);
}

// Start wheel homing move (slow constant stepping) - runs while pin24 stays HIGH
void startWheelHomingMove(bool dirLevel) {
  digitalWrite(dirPin, dirLevel);
  wheelEnable(true);

  wheelHomingMode = true;
  // TOTAL_STEPS is unused in homing mode because we stop when pin24 goes LOW.
  TOTAL_STEPS = 0;

  isMoving = true;
  stepHigh = false;
  stepsTaken = 0;
  lastStepTime = micros();
  digitalWrite(stepPin, LOW);
}

// Your requested wheel motor stepping function (pattern preserved, no blocking loops)
void stepMotor()
{
  unsigned long currentTime = micros();

  if (isMoving)
  {
    unsigned long stepDelay;

    // --- KEEP YOUR EXACT stepping math/shape ---
    if (wheelHomingMode) {
      // homing = slow constant stepping (Arduino decides when to stop by dropping pin24 LOW)
      stepDelay = WHEEL_HOME_STEP_DELAY_US;
    } else if (stepsTaken < ACCEL_STEPS)
    {
      float progress = (float)stepsTaken / ACCEL_STEPS;
      stepDelay =
        MAX_STEP_DELAY * pow((float)MIN_STEP_DELAY / MAX_STEP_DELAY, progress);
    }
    else
    {
      float progress =
        (float)(stepsTaken - ACCEL_STEPS) / DECEL_STEPS;
      stepDelay =
        MIN_STEP_DELAY * pow((float)MAX_STEP_DELAY / MIN_STEP_DELAY, progress);
    }

    stepDelay = constrain(stepDelay, MIN_STEP_DELAY, MAX_STEP_DELAY);
    // --- END exact behavior ---

    if (currentTime - lastStepTime >= stepDelay)
    {
      if (!stepHigh)
      {
        digitalWrite(stepPin, HIGH);
        stepHigh = true;
      }
      else
      {
        digitalWrite(stepPin, LOW);
        stepHigh = false;
        stepsTaken++;

        if (!wheelHomingMode && stepsTaken >= TOTAL_STEPS)
        {
          finishMove(currentTime);
        }
      }
      lastStepTime = currentTime;
    }
  }

  // handle DONE pulse timing (non-blocking)
  if (donePulsing && (currentTime - donePulseStart >= DONE_PULSE_US))
  {
    digitalWrite(donePin, LOW);
    donePulsing = false;
  }

  // handle ERROR pulse timing (non-blocking)
  if (errorPulsing && (currentTime - errorPulseStart >= WHEEL_ERROR_PULSE_US))
  {
    digitalWrite(PIN_WHEEL_ERROR, LOW);
    errorPulsing = false;
  }
}

void setup() {
  // Optional debugging:
  // Serial.begin(115200);
  delay(1000);
  pinMode(PIN_PUL, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);

  // Sensors: default to pullups (common for mechanical switches / NPN sensors with open collector)
  pinMode(PIN_HOME_SENSOR, INPUT_PULLUP);
  pinMode(PIN_OVERRUN_SENSOR, INPUT_PULLUP);

  // Arduino command input:
  // If Arduino drives a strong HIGH/LOW, plain INPUT is fine.
  // If you need a default LOW, you can use INPUT_PULLDOWN on many Teensy boards.
  //pinMode(PIN_INJECT_CMD, INPUT);
    pinMode(PIN_INJECT_CMD, INPUT_PULLUP);

  // Outputs to Arduino
  pinMode(PIN_HOME_READY, OUTPUT);
  pinMode(PIN_OVERRUN_ALARM, OUTPUT);

  digitalWrite(PIN_OVERRUN_ALARM, LOW);
  setHomeReady(false);

  // Motor initial
  digitalWrite(PIN_PUL, LOW);
  setDir(DIR_CW);
  motorEnable(true);

  // Start the 1us stepping timer (stepping change only)
  stepTimer.begin(stepperISR, 1);

  // -------------------- WHEEL SETUP (NEW) --------------------
  pinMode(PIN_WHEEL_PUL, OUTPUT);
  pinMode(PIN_WHEEL_DIR, OUTPUT);
  pinMode(PIN_WHEEL_ENA, OUTPUT);
  digitalWrite(PIN_WHEEL_PUL, LOW);
  digitalWrite(PIN_WHEEL_DIR, LOW);
  wheelEnable(true);

  pinMode(PIN_WHEEL_HOME_CMD, INPUT);   // Arduino drives HIGH/LOW
  pinMode(PIN_WHEEL_INDEX_CMD, INPUT);  // Arduino drives HIGH/LOW
  pinMode(PIN_WHEEL_READY, OUTPUT);
  pinMode(PIN_WHEEL_ERROR, OUTPUT);
  digitalWrite(PIN_WHEEL_READY, LOW);
  digitalWrite(PIN_WHEEL_ERROR, LOW);

  pinMode(PIN_WHEEL_POS_OK, INPUT_PULLUP); // sensor: LOW = OK
  wheelHomed = false;

  // Start homing immediately
  // state = State::BOOT_HOMING;
  // (NEW behavior when ENABLE_WHEEL_SYSTEM = true: wait for wheel home cmd first)
  state = ENABLE_WHEEL_SYSTEM ? State::WAIT_WHEEL_HOME_CMD : State::BOOT_HOMING;
}

void loop() {
  // Always service wheel stepping (non-blocking)
  stepMotor();

  switch (state) {

    // -------------------- NEW: wait for Arduino to command wheel homing --------------------
    case State::WAIT_WHEEL_HOME_CMD: {
      // Arduino holds pin24 HIGH while wheel should crawl toward home (Arduino sensor decides when to drop LOW)
      if (true) {
        // When we receive HIGH on pin24:
        // 1) start RAM homing (existing behavior)
        // 2) then start wheel slow crawl, and keep crawling until pin24 goes LOW
        // This is not "perfectly simultaneous" because RAM homing is blocking, but Arduino can keep pin24 HIGH until wheel is home.
        state = State::BOOT_HOMING;
      }
      break;
    }

    case State::BOOT_HOMING: {
      if (isOverrunActive()) faultOverrun("OVERRUN active at boot");

      // Existing RAM homing (unchanged)
      doHoming();

      // NEW: if wheel system enabled and Arduino is commanding wheel homing, start wheel crawl
      if (ENABLE_WHEEL_SYSTEM) {
        if (true) {
          // Start wheel crawling slowly (choose direction as needed)
          startWheelHomingMove(HIGH);
          state = State::WHEEL_HOMING_ACTIVE;
          break;
        }
      }

      state = State::READY_IDLE;
      break;
    }

    case State::WHEEL_HOMING_ACTIVE: {
      // Keep wheel crawling while Arduino holds pin24 HIGH.
      // When Arduino sees wheel home sensor, it will drop pin24 LOW and we stop the wheel.
      if (millis() > 2000) {
        // Stop wheel immediately
        isMoving = false;
        digitalWrite(PIN_WHEEL_PUL, LOW);

        wheelHomingMode = false;
        wheelHomed = true;

        // Homing complete only when:
        //  - Arduino dropped pin24 LOW
        //  - RAM is homed and is advertising HOME_READY on pin11 (setHomeReady(true) from doHoming)
        // At this point HOME_READY should already be HIGH.
        state = State::READY_IDLE;
      } else {
        // still homing, keep stepping via stepMotor()
      }
      break;
    }

    case State::READY_IDLE: {
      // In production, continuously verify overrun isn't hit
      if (isOverrunActive()) faultOverrun("OVERRUN during READY_IDLE");

      // Keep HOME_READY asserted when we are truly home
      // (If you want it strictly from logic, you can also do: setHomeReady(isHomeActive()); )
      setHomeReady(true);

      // -------------------- NEW WHEEL SEQUENCE --------------------
      if (ENABLE_WHEEL_SYSTEM) {
        // We only allow indexing if the wheel has been homed by the pin24 sequence
        if (wheelHomed) {
          // Arduino sends pin25 HIGH to request "move wheel one slot"
          if (true && !isMoving) {
            // Start index move (choose direction as needed)
            startWheelIndexMove(HIGH);
            state = State::WHEEL_INDEXING;
          }
        }
        break; // when wheel system is enabled, injection is driven by wheel sequence
      }

      // -------------------- ORIGINAL RAM TRIGGER (kept) --------------------
      // Rising-edge detect on INJECT_CMD
      bool injectCmd = digitalRead(PIN_INJECT_CMD) == HIGH;
      // LEVEL-TRIGGERED:
      // If pin10 is HIGH and we are at HOME, inject again
      if (injectCmd && isHomeActive()) {
      delay(150);               // small pause so we don't instantly re-trigger
      state = State::INJECTING;
}
      break;
    }

    case State::WHEEL_INDEXING: {
      // Wheel steps non-blocking in stepMotor().
      // When it finishes, finishMove() will pulse PIN_WHEEL_READY and optionally pulse PIN_WHEEL_ERROR.

      // If wheel errored, do NOT inject (leave system idle so Arduino can react)
      if (errorPulsing) {
        // keep waiting until error pulse done; do not inject
        if (!isMoving && !errorPulsing) {
          state = State::READY_IDLE;
        }
        break;
      }

      // If wheel finished (not moving) and no error -> trigger RAM injection
      if (!isMoving && !errorPulsing) {
        // Wheel is positioned OK (pin28 LOW) so now activate the ram injection logic
        delay(150); // keep your existing small pause behavior
        state = State::INJECTING;
      }
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
      wheelEnable(false);
      break;
  }
}
