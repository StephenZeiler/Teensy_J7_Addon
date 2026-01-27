#include <Arduino.h>
#include <math.h>

// =====================
// PINS (Teensy 4.1)
// =====================
constexpr uint8_t PIN_WHEEL_PUL = 5;
constexpr uint8_t PIN_WHEEL_DIR = 6;
constexpr uint8_t PIN_WHEEL_ENA = 7;

// Uncomment later when you re-enable Arduino trigger
// const int triggerPin = 5;  // Arduino output -> Teensy input

// Optional Teensy -> Arduino "done" pulse (unused in this test, but kept)
const int donePin = 6;

// =====================
// MOVE PARAMETERS
// =====================
const int TOTAL_STEPS = 200;

// --- 2× faster (half the delays) ---
int MIN_STEP_DELAY = 21;   // 31 * (2/3) = 20.666 -> 21
int MAX_STEP_DELAY = 411;  // 616 * (2/3) = 410.666 -> 411
int ACCEL_STEPS    = 46;   // unchanged
int DECEL_STEPS    = 15;   // unchanged

// =====================
// TEST PAUSE BETWEEN MOVES
// =====================
const unsigned long TEST_PAUSE_US = 200000UL; // 0.2 seconds
unsigned long testPauseStart = 0;
bool waitingBetweenMoves = false;

// =====================
// MOTOR STATE
// =====================
unsigned long lastStepTime = 0;
int stepsTaken = 0;
bool isMoving = false;
bool stepHigh = false;

// Optional done pulse settings
const unsigned long DONE_PULSE_US = 2000;
bool donePulsing = false;
unsigned long donePulseStart = 0;

void startMove()
{
  digitalWrite(enablePin, LOW); // change if your driver enable is active-high
  digitalWrite(dirPin, LOW);    // fixed direction; change if needed

  isMoving = true;
  stepHigh = false;
  stepsTaken = 0;
  lastStepTime = micros();
}

void finishMove(unsigned long currentTime)
{
  isMoving = false;
  stepsTaken = 0;
  stepHigh = false;

  digitalWrite(stepPin, LOW);

  // Optional: DONE pulse (not required for this test)
  digitalWrite(donePin, HIGH);
  donePulsing = true;
  donePulseStart = currentTime;

  // Begin the 0.2s pause before restarting
  waitingBetweenMoves = true;
  testPauseStart = currentTime;
}

void stepMotor()
{
  unsigned long currentTime = micros();

  if (isMoving)
  {
    unsigned long stepDelay;

    // --- KEEP YOUR EXACT stepping math/shape ---
    if (stepsTaken < ACCEL_STEPS)
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

        if (stepsTaken >= TOTAL_STEPS)
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
}

void setup()
{
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // pinMode(triggerPin, INPUT_PULLDOWN); // later when you re-enable Arduino trigger
  pinMode(donePin, OUTPUT);

  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, LOW);
  digitalWrite(donePin, LOW);

  digitalWrite(enablePin, LOW); // enable driver (adjust if needed)

  // Start immediately for testing
  startMove();
}

void loop()
{
  unsigned long now = micros();

  // ============================
  // TEST MODE: force trigger HIGH
  // (Arduino trigger logic commented out)
  // ============================

  // If we finished a move, wait 0.2s, then go again
  if (!isMoving && waitingBetweenMoves)
  {
    if (now - testPauseStart >= TEST_PAUSE_US)
    {
      waitingBetweenMoves = false;
      startMove();
    }
  }

  // Run motor service as fast as possible
  stepMotor();
}