#include <Arduino.h>
#include <math.h>

/*
 * Bulb Ram + Wheel Motor Controller - Teensy Code
 * UPDATED FOR 1600 STEPS/REV (doubled from 800)
 *
 * PIN LAYOUT:
 *
 * RAM Motor Driver:
 *   DIR = 3
 *   PUL = 2
 *   ENA = 4
 *
 * Wheel Motor Driver:
 *   DIR  = 6
 *   STEP = 5
 *   ENA  = 7
 *
 * Sensors:
 *   HOME_SENSOR          = 32 (HIGH when active)
 *   OVERRUN_SENSOR       = 30 (HIGH when active)
 *   WHEEL_POSITION_SENSOR= 31 (LOW when correct)
 *
 * Arduino Communication:
 *   HOME_NOTIFICATION    = 11 (Teensy -> Arduino)
 *   INJECT_COMMAND       = 10 (Arduino -> Teensy)
 *   OVERRUN_ALARM        = 12 (Teensy -> Arduino)
 *   TROLL_HOME_COMMAND   = 24 (Arduino -> Teensy)
 *   MOVE_WHEEL_COMMAND   = 25 (Arduino -> Teensy)
 *   WHEEL_READY_NOTIFICATION = 26 (Teensy -> Arduino)
 *   WHEEL_POSITION_ALARM = 27 (Teensy -> Arduino)
 */

// =====================
// RAM MOTOR PINS
// =====================
const int DIR_PIN = 3;
const int PUL_PIN = 2;
const int ENA_PIN = 4;

// =====================
// WHEEL MOTOR PINS
// =====================
const int WHEEL_STEP_PIN   = 5;
const int WHEEL_DIR_PIN    = 6;
const int WHEEL_ENABLE_PIN = 7;

// =====================
// SENSOR PINS
// =====================
const int HOME_SENSOR           = 32;
const int OVERRUN_SENSOR        = 30;
const int WHEEL_POSITION_SENSOR = 31;

// =====================
// ARDUINO COMMUNICATION PINS
// =====================
const int HOME_NOTIFICATION       = 11; // Output to Arduino: Ram home
const int INJECT_COMMAND          = 10; // Input from Arduino: Ram inject (stays down while HIGH)
const int OVERRUN_ALARM           = 12; // Output to Arduino: Error/alarm
const int TROLL_HOME_COMMAND      = 24; // Input from Arduino: Troll wheel to home
const int MOVE_WHEEL_COMMAND      = 25; // Input from Arduino: Pulse to move wheel one slot
const int WHEEL_READY_NOTIFICATION= 26; // Output to Arduino: Wheel ready
const int WHEEL_POSITION_ALARM    = 27; // Output to Arduino: Wheel position error

// =====================
// RAM MOTOR PARAMETERS (doubled for 1600 steps/rev)
// =====================
const int STEP_DELAY          = 70;   // production speed (15% faster)
const int HOME_STEP_DELAY     = 105;  // homing fast (15% faster)
const int SLOW_STEP_DELAY     = 140;  // homing creep (15% faster)
const int INJECT_STEPS        = 853;  // Current setting is at 80mm travel...150mm is the circuference of the pulley. 1600 steps/rev.  1600/150mm = 10.6666 steps /mm desired in travel.. So if i want to travel 65mm INJECT_STEPS would be 10.6666*65 == 692. 
const int OVERRUN_CHECK_STEPS = 256;  // 
const int SAFETY_MARGIN_STEPS = 256;  // 

// =====================
// WHEEL MOTOR PARAMETERS (doubled for 1600 steps/rev)
// =====================
const int WHEEL_STEPS_PER_SLOT = 400;  // (200 * 2)
const int WHEEL_TROLL_SPEED    = 1000; // homing speed (already 50% faster)

// 15% faster wheel motion (accel + cruise + decel)
const int MIN_STEP_DELAY = 119;  // was 140
const int MAX_STEP_DELAY = 957;  // was 1126
const int ACCEL_STEPS    = 92;   // (46 * 2)
const int DECEL_STEPS    = 92;   // (46 * 2)

// Test mode
const bool TEST_MODE = false;

// Direction definitions
const bool CLOCKWISE         = LOW;
const bool COUNTER_CLOCKWISE = HIGH;

// =====================
// STATE VARIABLES
// =====================
bool ramIsHomed = false;
bool wheelIsHomed = false;
bool readyForProduction = false;

int lastInjectCommand     = LOW;
int lastTrollHomeCommand  = LOW;
int lastMoveWheelCommand  = LOW;

// =====================
// RAM MOTOR LOW-LEVEL
// =====================
void setDirection(bool dir) {
  digitalWrite(DIR_PIN, dir);
  delayMicroseconds(5);
}

void stepMotor(int delayTime) {
  digitalWrite(PUL_PIN, HIGH);
  delayMicroseconds(delayTime);
  digitalWrite(PUL_PIN, LOW);
  delayMicroseconds(delayTime);
}

// Active-HIGH sensors (as you stated)
inline bool isHomeActive()    { return (digitalRead(HOME_SENSOR) == HIGH); }
inline bool isOverrunActive() { return (digitalRead(OVERRUN_SENSOR) == HIGH); }

// =====================
// ALARMS
// =====================
void triggerOverrunAlarm() {
  Serial.println("\n!!! OVERRUN ALARM TRIGGERED !!!");
  Serial.println("Sending alarm signal to Arduino...");

  digitalWrite(HOME_NOTIFICATION, LOW);
  digitalWrite(WHEEL_READY_NOTIFICATION, LOW);

  digitalWrite(OVERRUN_ALARM, HIGH);
  delay(1000);
  digitalWrite(OVERRUN_ALARM, LOW);

  Serial.println("Machine stopped - manual intervention required");
  Serial.println("Please reset the system after correcting the issue\n");

  while (1) { delay(1000); }
}

void triggerWheelPositionAlarm() {
  Serial.println("\n!!! WHEEL POSITION ALARM TRIGGERED !!!");
  Serial.println("Wheel position sensor did not detect correct position!");
  Serial.println("Sending alarm signal to Arduino...");

  digitalWrite(HOME_NOTIFICATION, LOW);
  digitalWrite(WHEEL_READY_NOTIFICATION, LOW);

  digitalWrite(WHEEL_POSITION_ALARM, HIGH);
  delay(1000);
  digitalWrite(WHEEL_POSITION_ALARM, LOW);

  Serial.println("Machine stopped - manual intervention required");
  Serial.println("Please reset the system after correcting the issue\n");

  while (1) { delay(1000); }
}

// =====================
// HOMING HELPERS (EDGE-BASED HOME)
// =====================
bool stepWithOverrunCheck(int delayTime) {
  if (isOverrunActive()) {
    Serial.println("ERROR: Overrun sensor triggered!");
    triggerOverrunAlarm();
    return false;
  }
  stepMotor(delayTime);
  return true;
}

bool moveUntil(bool (*condition)(), int delayTime, long maxSteps, const char* failMsg) {
  long steps = 0;
  while (!condition()) {
    if (!stepWithOverrunCheck(delayTime)) return false;
    steps++;
    if (steps >= maxSteps) {
      Serial.println(failMsg);
      triggerOverrunAlarm();
      return false;
    }
  }
  return true;
}

bool moveUntilNot(bool (*condition)(), int delayTime, long maxSteps, const char* failMsg) {
  long steps = 0;
  while (condition()) {
    if (!stepWithOverrunCheck(delayTime)) return false;
    steps++;
    if (steps >= maxSteps) {
      Serial.println(failMsg);
      triggerOverrunAlarm();
      return false;
    }
  }
  return true;
}

// Clear HOME zone CW, then creep CCW until HOME becomes active (repeatable edge)
bool homeByClearingAndCreepingIn(long maxClearSteps, long maxCreepSteps) {
  Serial.println("Edge-homing: clearing HOME zone CLOCKWISE...");
  setDirection(CLOCKWISE);

  // was STEP_DELAY -> now HOME_STEP_DELAY
  if (!moveUntilNot(isHomeActive, HOME_STEP_DELAY, maxClearSteps,
                    "ERROR: Could not clear HOME zone (CW) within max steps!")) {
    return false;
  }

  Serial.println("Edge-homing: creeping back into HOME zone COUNTER-CLOCKWISE...");
  setDirection(COUNTER_CLOCKWISE);

  // keep creep slow
  if (!moveUntil(isHomeActive, SLOW_STEP_DELAY, maxCreepSteps,
                 "ERROR: Could not re-enter HOME zone (CCW) within max steps!")) {
    return false;
  }

  Serial.println("HOME edge captured (repeatable).");
  return true;
}
// Escape OVERRUN by moving CLOCKWISE until OVERRUN clears.
// We intentionally DO NOT treat OVERRUN=HIGH as a fault during this escape.
bool escapeOverrunClockwise(long maxSteps) {
  Serial.println("Escaping OVERRUN: moving CLOCKWISE until OVERRUN clears...");
  setDirection(CLOCKWISE);

  long steps = 0;
  while (isOverrunActive()) {
    stepMotor(HOME_STEP_DELAY);   // <-- no overrun check here on purpose
    steps++;
    if (steps >= maxSteps) {
      Serial.println("ERROR: OVERRUN did not clear while moving CW (stuck sensor or wrong direction)!");
      triggerOverrunAlarm();
      return false;
    }
  }

  Serial.println("OVERRUN cleared.");
  return true;
}

// =====================
// UPDATED RAM HOMING SEQUENCE
// =====================
void performRamHomingSequence() {
  Serial.println("--- RAM HOMING SEQUENCE START (EDGE-BASED) ---");

  // Tune these if your travel range is bigger/smaller
  const long MAX_SEARCH_STEPS = 12000;
  const long MAX_CLEAR_STEPS  = 6000;
  const long MAX_CREEP_STEPS  = 6000;
  const long MAX_CW_TO_HOME   = 12000;

  bool home0 = isHomeActive();
  bool over0 = isOverrunActive();

  Serial.print("Initial sensors: HOME=");
  Serial.print(home0 ? "HIGH" : "LOW");
  Serial.print(" OVERRUN=");
  Serial.println(over0 ? "HIGH" : "LOW");

  // CASE A: Already on OVERRUN at start
  if (over0) {
  Serial.println("Start condition: OVERRUN active.");

  // 1) Escape OVERRUN first (CW), ignoring OVERRUN=HIGH during this escape
  if (!escapeOverrunClockwise(4000)) return;   // tune max if needed

  // 2) Now OVERRUN is cleared, continue CW until HOME is found
  Serial.println("Moving CLOCKWISE to find HOME...");
  setDirection(CLOCKWISE);

  // IMPORTANT: now that OVERRUN is cleared, it's safe to use the normal guarded move
  if (!moveUntil(isHomeActive, STEP_DELAY, MAX_CW_TO_HOME,
                 "ERROR: HOME never found while moving CW from OVERRUN!")) {
    return;
  }

  Serial.println("HOME zone found while moving CW.");

  // 3) Continue CW until HOME clears, then creep CCW until HOME active again (repeatable edge)
  if (!homeByClearingAndCreepingIn(MAX_CLEAR_STEPS, MAX_CREEP_STEPS)) return;
}
  // CASE B: HOME active at start (wide zone) -> must clear then creep back
  else if (home0) {
    Serial.println("Start condition: HOME active (wide zone). Clearing then creeping back...");
    if (!homeByClearingAndCreepingIn(MAX_CLEAR_STEPS, MAX_CREEP_STEPS)) return;
  }
  // CASE C: Neither active -> unknown position
  else {
    Serial.println("Start condition: Neither HOME nor OVERRUN active. Searching CCW for HOME or OVERRUN...");

    setDirection(COUNTER_CLOCKWISE);
    long steps = 0;
    while (!isHomeActive() && !isOverrunActive()) {
      stepMotor(STEP_DELAY);
      steps++;
      if (steps >= MAX_SEARCH_STEPS) {
        Serial.println("ERROR: Neither HOME nor OVERRUN found during search!");
        triggerOverrunAlarm();
        return;
      }
    }

    if (isOverrunActive()) {
      Serial.println("OVERRUN found during search. Moving CW to HOME then edge-homing...");

      setDirection(CLOCKWISE);
      if (!moveUntil(isHomeActive, STEP_DELAY, MAX_CW_TO_HOME,
                     "ERROR: HOME never found while moving CW from OVERRUN!")) {
        return;
      }

      if (!homeByClearingAndCreepingIn(MAX_CLEAR_STEPS, MAX_CREEP_STEPS)) return;
    } else {
      Serial.println("HOME found during search. Edge-homing (clear CW, creep CCW)...");
      if (!homeByClearingAndCreepingIn(MAX_CLEAR_STEPS, MAX_CREEP_STEPS)) return;
    }
  }

  // =====================
  // Verify OVERRUN sensor works (move CCW into it, then return CW)
  // =====================
  Serial.println("\nVerifying OVERRUN sensor...");
  Serial.print("Moving COUNTER-CLOCKWISE ");
  Serial.print(OVERRUN_CHECK_STEPS);
  Serial.println(" steps to check OVERRUN");

  setDirection(COUNTER_CLOCKWISE);
  for (int i = 0; i < OVERRUN_CHECK_STEPS; i++) {
    stepMotor(STEP_DELAY);
  }

  if (!isOverrunActive()) {
    Serial.println("ERROR: Overrun sensor did NOT trigger during verification!");
    triggerOverrunAlarm();
    return;
  }
  Serial.println("OVERRUN verified - reading HIGH");

  Serial.print("Returning CLOCKWISE ");
  Serial.print(OVERRUN_CHECK_STEPS);
  Serial.println(" steps back toward HOME...");

  setDirection(CLOCKWISE);
  for (int i = 0; i < OVERRUN_CHECK_STEPS; i++) {
    stepMotor(STEP_DELAY);
  }

  // Re-capture the HOME edge so "home" is identical every time
  Serial.println("Re-capturing HOME edge after OVERRUN verification...");
  if (!homeByClearingAndCreepingIn(MAX_CLEAR_STEPS, MAX_CREEP_STEPS)) return;

  Serial.println("\n=================================");
  Serial.println("RAM HOMING COMPLETE (EDGE-BASED)");
  Serial.println("=================================\n");

  ramIsHomed = true;
  digitalWrite(HOME_NOTIFICATION, HIGH);
}

// =====================
// INJECTION CYCLE
// =====================
void performInjectionCycle() {
  unsigned long injectionStartTime = millis();

  // Inject down
  setDirection(CLOCKWISE);
  for (int i = 0; i < INJECT_STEPS; i++) {
    stepMotor(STEP_DELAY);
  }

  // Wait for Arduino to drop command LOW
  while (digitalRead(INJECT_COMMAND) == HIGH) {
    delay(10);
  }

  // Return up
  setDirection(COUNTER_CLOCKWISE);
  bool reachedHome = false;

  for (int i = 0; i < INJECT_STEPS; i++) {
    if (isOverrunActive()) {
      Serial.println("ERROR: Overrun sensor triggered!");
      triggerOverrunAlarm();
      readyForProduction = false;
      return;
    }
    stepMotor(STEP_DELAY);
    if (isHomeActive() && !reachedHome) reachedHome = true;
  }

  // Safety margin search for home
  if (!isHomeActive()) {
    for (int i = 0; i < SAFETY_MARGIN_STEPS; i++) {
      if (isHomeActive()) { reachedHome = true; break; }
      if (isOverrunActive()) {
        Serial.println("ERROR: Hit overrun!");
        triggerOverrunAlarm();
        readyForProduction = false;
        return;
      }
      stepMotor(STEP_DELAY);
    }

    if (!isHomeActive()) {
      Serial.println("ERROR: Missed home!");
      triggerOverrunAlarm();
      readyForProduction = false;
      return;
    }
  }

  unsigned long totalCycleTime = millis() - injectionStartTime;
  Serial.print("Ram: ");
  Serial.print(totalCycleTime);
  Serial.println("ms");

  digitalWrite(HOME_NOTIFICATION, HIGH);
}

// =====================
// WHEEL MOTOR FUNCTIONS
// =====================
void stepWheelMotor(int delayTime) {
  digitalWrite(WHEEL_STEP_PIN, HIGH);
  delayMicroseconds(delayTime);
  digitalWrite(WHEEL_STEP_PIN, LOW);
  delayMicroseconds(delayTime);
}

void trollWheelToHome() {
  Serial.println("--- WHEEL TROLLING TO HOME ---");
  Serial.println("Moving wheel slowly until Arduino sensor detects home...");

  digitalWrite(WHEEL_DIR_PIN, LOW);

  while (digitalRead(TROLL_HOME_COMMAND) == HIGH) {
    stepWheelMotor(WHEEL_TROLL_SPEED);
  }

  Serial.println("Arduino home sensor detected - wheel is home");
  wheelIsHomed = true;
}

void moveWheelOneSlot() {
  unsigned long wheelStartTime = millis();

  digitalWrite(WHEEL_DIR_PIN, LOW);

  unsigned long lastStepTime = 0;
  unsigned long currentTime;
  bool stepHigh = false;
  int stepsTaken = 0;

  while (stepsTaken < WHEEL_STEPS_PER_SLOT) {
    currentTime = micros();
    unsigned long stepDelay;

    if (stepsTaken < ACCEL_STEPS) {
      float progress = (float)stepsTaken / ACCEL_STEPS;
      stepDelay = MAX_STEP_DELAY * pow((float)MIN_STEP_DELAY / MAX_STEP_DELAY, progress);
    } else if (stepsTaken < (WHEEL_STEPS_PER_SLOT - DECEL_STEPS)) {
      stepDelay = MIN_STEP_DELAY;
    } else {
      int stepsIntoDecel = stepsTaken - (WHEEL_STEPS_PER_SLOT - DECEL_STEPS);
      float progress = (float)stepsIntoDecel / DECEL_STEPS;
      stepDelay = MIN_STEP_DELAY * pow((float)MAX_STEP_DELAY / MIN_STEP_DELAY, progress);
    }

    stepDelay = constrain(stepDelay, MIN_STEP_DELAY, MAX_STEP_DELAY);

    if (currentTime - lastStepTime >= stepDelay) {
      if (!stepHigh) {
        digitalWrite(WHEEL_STEP_PIN, HIGH);
        stepHigh = true;
      } else {
        digitalWrite(WHEEL_STEP_PIN, LOW);
        stepHigh = false;
        stepsTaken++;
      }
      lastStepTime = currentTime;
    }
  }

  unsigned long wheelDuration = millis() - wheelStartTime;
  Serial.print("Wheel: ");
  Serial.print(wheelDuration);
  Serial.println("ms");

  delay(20); // Let wheel settle before checking position sensor
  if (digitalRead(WHEEL_POSITION_SENSOR) == LOW) {
    digitalWrite(WHEEL_READY_NOTIFICATION, HIGH);
  } else {
    Serial.println("ERROR: Wheel position sensor HIGH!");
    triggerWheelPositionAlarm();
  }
}

// =====================
// SETUP
// =====================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=================================");
  Serial.println("Bulb Ram + Wheel Controller Starting...");
  Serial.println("=================================");

  // Configure ram motor pins
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PUL_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);

  // Configure wheel motor pins
  pinMode(WHEEL_STEP_PIN, OUTPUT);
  pinMode(WHEEL_DIR_PIN, OUTPUT);
  pinMode(WHEEL_ENABLE_PIN, OUTPUT);

  // Configure sensor pins
  pinMode(HOME_SENSOR, INPUT);
  pinMode(OVERRUN_SENSOR, INPUT);
  pinMode(WHEEL_POSITION_SENSOR, INPUT);

  // Configure communication pins
  pinMode(HOME_NOTIFICATION, OUTPUT);
  pinMode(INJECT_COMMAND, INPUT_PULLDOWN);  // Pull-down to prevent floating/noise
  pinMode(OVERRUN_ALARM, OUTPUT);
  pinMode(TROLL_HOME_COMMAND, INPUT_PULLDOWN);  // Pull-down to prevent floating/noise
  pinMode(MOVE_WHEEL_COMMAND, INPUT_PULLDOWN);  // Pull-down to prevent floating/noise
  pinMode(WHEEL_READY_NOTIFICATION, OUTPUT);
  pinMode(WHEEL_POSITION_ALARM, OUTPUT);

  // Initialize outputs to LOW
  digitalWrite(HOME_NOTIFICATION, LOW);
  digitalWrite(OVERRUN_ALARM, LOW);
  digitalWrite(WHEEL_READY_NOTIFICATION, LOW);
  digitalWrite(WHEEL_POSITION_ALARM, LOW);

  // Enable motor drivers (active low)
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(WHEEL_ENABLE_PIN, LOW);

  Serial.println("Hardware initialized");
  Serial.println("Motor drivers enabled");
  Serial.println("Waiting for homing command (pin 24) from Arduino...\n");
}

// =====================
// MAIN LOOP
// =====================
void loop() {
  // Check for troll home command from Arduino
  int trollHomeSignal = digitalRead(TROLL_HOME_COMMAND);

  // Rising edge of troll home command
  if (trollHomeSignal == HIGH && lastTrollHomeCommand == LOW) {
    Serial.println("\n>>> TROLL HOME COMMAND RECEIVED (pin 24) <<<");

    digitalWrite(HOME_NOTIFICATION, LOW);
    digitalWrite(WHEEL_READY_NOTIFICATION, LOW);

    // Start both homing sequences
    trollWheelToHome();
    performRamHomingSequence();

    // Both are now homed
    if (ramIsHomed && wheelIsHomed) {
      Serial.println("\n=================================");
      Serial.println("BOTH MOTORS HOMED - READY FOR PRODUCTION");
      Serial.println("=================================\n");

      readyForProduction = true;
      delay(100);

      Serial.println("Checking wheel position after homing...");
      if (digitalRead(WHEEL_POSITION_SENSOR) == LOW) {
        Serial.println("Wheel position sensor: CORRECT (LOW)");
        digitalWrite(WHEEL_READY_NOTIFICATION, HIGH);
      } else {
        Serial.println("ERROR: Wheel position sensor HIGH after homing!");
        triggerWheelPositionAlarm();
      }
    }
  }

  lastTrollHomeCommand = trollHomeSignal;

  // Check for move wheel command (only if ready for production)
  if (readyForProduction) {
    int moveWheelSignal = digitalRead(MOVE_WHEEL_COMMAND);

    // Rising edge of move wheel command
    if (moveWheelSignal == HIGH && lastMoveWheelCommand == LOW) {
      Serial.println("\n>>> MOVE WHEEL COMMAND RECEIVED (pin 25) <<<");
      digitalWrite(WHEEL_READY_NOTIFICATION, LOW);
      moveWheelOneSlot();
    }
    lastMoveWheelCommand = moveWheelSignal;

    // Check for inject command
    int injectSignal = digitalRead(INJECT_COMMAND);

    // Rising edge of inject command
    if (injectSignal == HIGH && lastInjectCommand == LOW) {
      Serial.println("\n>>> INJECT COMMAND RECEIVED (pin 10) <<<");
      digitalWrite(HOME_NOTIFICATION, LOW);
      performInjectionCycle();
    }
    lastInjectCommand = injectSignal;
  }

  delay(10);
}
