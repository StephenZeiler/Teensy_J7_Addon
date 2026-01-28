/*  Teensy Bulb Ram Controller (Stepper + HOME + OVERRUN + Arduino Handshake)

  SUMMARY (what this sketch does)
  - On boot, it "homes" the ram using HOME + OVERRUN sensors.
    Case A: HOME is HIGH at boot:
      1) Jog CW until HOME goes LOW (leave the switch)
      2) Then creep CCW until HOME goes HIGH (re-find exact home edge)
    Case B: HOME is LOW at boot:
      1) Move CCW until HOME goes HIGH (find home)
  - After homing:
      Move CCW 128 steps and confirm OVERRUN becomes HIGH (sanity check).
      Then move CW 128 steps back to HOME.
      If OVERRUN sanity check fails -> alarm Arduino on pin 12 for 1s and disable.
  - Production:
      Teensy asserts HOME_READY (pin 11) HIGH when at home and ready.
      Arduino asserts INJECT_CMD (pin 10) HIGH to request one injection.
      Teensy performs:
        CW 480 steps (inject)
        then CCW back toward home:
           - It expects to hit HOME within 480 steps
           - It allows up to +128 extra steps “grace”
           - If HOME not found by 480+128, or OVERRUN triggers unexpectedly -> alarm and disable.
  - OVERRUN alarm: Teensy pulses pin 12 HIGH for 1 second, then stops/disable.

  ASSUMPTIONS (change if your wiring is different)
  - HOME sensor reads HIGH when the ram is at/near home.
  - OVERRUN sensor reads HIGH when the ram has hit the overrun limit.
  - Arduino commands are simple HIGH level on pin 10; we treat rising-edge as “one request”.
  - Stepper driver: STEP pulses on PUL pin, DIR pin sets direction, ENA enables the driver.
    Many drivers are ENABLE=LOW to enable; set ENA_ACTIVE_LOW accordingly.

  PIN LAYOUT (as requested)
  Stepper driver:
    PUL/STEP -> Teensy pin 2
    DIR      -> Teensy pin 3
    ENA      -> Teensy pin 4

  Sensors:
    HOME     -> Teensy pin 28
    OVERRUN  -> Teensy pin 30

  Arduino handshake:
    HOME_READY (Teensy -> Arduino) -> pin 11
    INJECT_CMD (Arduino -> Teensy) -> pin 10
    OVERRUN_ALARM (Teensy -> Arduino) -> pin 12
*/

#include <Arduino.h>

// ----------------------- Pins -----------------------
static const uint8_t PIN_STEP          = 2;   // PUL
static const uint8_t PIN_DIR           = 3;   // DIR
static const uint8_t PIN_ENA           = 4;   // ENA

static const uint8_t PIN_HOME_SENSOR   = 28;
static const uint8_t PIN_OVERRUN_SENSOR= 30;

static const uint8_t PIN_HOME_READY    = 11;  // Teensy -> Arduino
static const uint8_t PIN_INJECT_CMD    = 10;  // Arduino -> Teensy
static const uint8_t PIN_OVERRUN_ALARM = 12;  // Teensy -> Arduino

// -------------------- Behavior knobs --------------------
static const bool ENA_ACTIVE_LOW = true;  // many stepper drivers: LOW=enable, HIGH=disable

static const uint16_t INJECT_STEPS = 480;
static const uint16_t OVERRUN_CHECK_STEPS = 128;
static const uint16_t RETURN_GRACE_STEPS = 128;   // extra travel allowed when returning home

// Pulse timing (microseconds). Smaller = faster.
static const uint16_t STEP_PULSE_US_FAST = 3;     // STEP HIGH duration
static const uint16_t STEP_DELAY_US_FAST = 700;   // delay between pulses (speed)

static const uint16_t STEP_PULSE_US_SLOW = 3;
static const uint16_t STEP_DELAY_US_SLOW = 1400;  // slow creep for edge finding

// Basic debounce/read stability (ms)
static const uint16_t SENSOR_STABLE_MS = 5;

// ----------------------- Helpers -----------------------
enum Dir : uint8_t { DIR_CW = 0, DIR_CCW = 1 };

static inline void driverEnable(bool enabled)
{
  if (ENA_ACTIVE_LOW) digitalWriteFast(PIN_ENA, enabled ? LOW : HIGH);
  else               digitalWriteFast(PIN_ENA, enabled ? HIGH : LOW);
}

static inline void setDir(Dir d)
{
  digitalWriteFast(PIN_DIR, (d == DIR_CW) ? HIGH : LOW);
}

static inline bool homeRaw()     { return digitalReadFast(PIN_HOME_SENSOR); }
static inline bool overrunRaw()  { return digitalReadFast(PIN_OVERRUN_SENSOR); }

static bool readStable(bool (*fn)())
{
  // Simple "two-sample + small delay" stability check
  bool a = fn();
  delay(SENSOR_STABLE_MS);
  bool b = fn();
  return (a && b) || (!a && !b) ? b : b; // if noisy, we still return latest; you can improve if needed
}

static inline bool home()    { return readStable(homeRaw); }
static inline bool overrun() { return readStable(overrunRaw); }

static void stepOnceFast()
{
  digitalWriteFast(PIN_STEP, HIGH);
  delayMicroseconds(STEP_PULSE_US_FAST);
  digitalWriteFast(PIN_STEP, LOW);
  delayMicroseconds(STEP_DELAY_US_FAST);
}

static void stepOnceSlow()
{
  digitalWriteFast(PIN_STEP, HIGH);
  delayMicroseconds(STEP_PULSE_US_SLOW);
  digitalWriteFast(PIN_STEP, LOW);
  delayMicroseconds(STEP_DELAY_US_SLOW);
}

static void pulseOverrunAlarm1s()
{
  Serial.println("[ALARM] OVERRUN alarm -> Arduino (pin 12) HIGH for 1s");
  digitalWriteFast(PIN_OVERRUN_ALARM, HIGH);
  delay(1000);
  digitalWriteFast(PIN_OVERRUN_ALARM, LOW);
}

static void stopAndDisable(const char* reason)
{
  Serial.print("[STOP] ");
  Serial.println(reason);

  // Tell Arduino not ready
  digitalWriteFast(PIN_HOME_READY, LOW);

  // Alarm
  pulseOverrunAlarm1s();

  // Disable driver
  driverEnable(false);
  Serial.println("[STOP] Driver disabled. Halting.");
  while (true) { delay(100); }
}

// Move a fixed number of steps. Optional: abort if OVERRUN triggers.
static void moveSteps(Dir d, uint32_t steps, bool fast, bool abortOnOverrun, const char* label)
{
  Serial.print("[MOVE] ");
  Serial.print(label);
  Serial.print(" dir=");
  Serial.print((d == DIR_CW) ? "CW" : "CCW");
  Serial.print(" steps=");
  Serial.println(steps);

  setDir(d);

  for (uint32_t i = 0; i < steps; i++)
  {
    if (abortOnOverrun && overrun())
    {
      stopAndDisable("OVERRUN triggered unexpectedly during move");
    }
    if (fast) stepOnceFast();
    else      stepOnceSlow();
  }
}

// Move until HOME becomes target state or until OVERRUN triggers or until maxSteps exceeded.
// Returns true if target achieved.
static bool moveUntilHomeState(Dir d, bool targetHomeHigh, uint32_t maxSteps, bool slow, bool abortOnOverrun, const char* label)
{
  Serial.print("[SEEK] ");
  Serial.print(label);
  Serial.print(" dir=");
  Serial.print((d == DIR_CW) ? "CW" : "CCW");
  Serial.print(" targetHome=");
  Serial.print(targetHomeHigh ? "HIGH" : "LOW");
  Serial.print(" maxSteps=");
  Serial.println(maxSteps);

  setDir(d);

  for (uint32_t i = 0; i < maxSteps; i++)
  {
    if (abortOnOverrun && overrun())
    {
      Serial.println("[SEEK] OVERRUN triggered while seeking HOME");
      return false;
    }

    bool h = home();
    if (h == targetHomeHigh)
    {
      Serial.print("[SEEK] HOME reached at step ");
      Serial.println(i);
      return true;
    }

    if (slow) stepOnceSlow();
    else      stepOnceFast();
  }

  Serial.println("[SEEK] HOME target not reached within maxSteps");
  return false;
}

// For production return: move CCW until HOME becomes HIGH, with step limits and OVERRUN monitoring.
// Returns true if HOME found.
static bool returnHomeWithLimits()
{
  const uint32_t maxSteps = (uint32_t)INJECT_STEPS + (uint32_t)RETURN_GRACE_STEPS;

  Serial.print("[RETURN] Seeking HOME within ");
  Serial.print(maxSteps);
  Serial.println(" steps (CCW)");

  setDir(DIR_CCW);

  for (uint32_t i = 0; i < maxSteps; i++)
  {
    if (overrun())
    {
      Serial.println("[RETURN] OVERRUN triggered while returning home");
      return false;
    }

    if (home())
    {
      Serial.print("[RETURN] HOME found at step ");
      Serial.println(i);
      return true;
    }

    stepOnceFast();
  }

  Serial.println("[RETURN] HOME not found within limit (missed sensors?)");
  return false;
}

// ----------------------- Homing logic -----------------------
static void doHoming()
{
  Serial.println("========== BOOT: HOMING START ==========");
  driverEnable(true);

  bool homeAtBoot = home();
  bool overAtBoot = overrun();

  Serial.print("[BOOT] HOME=");
  Serial.print(homeAtBoot ? "HIGH" : "LOW");
  Serial.print(" OVERRUN=");
  Serial.println(overAtBoot ? "HIGH" : "LOW");

  // Safety: if OVERRUN already active at boot, something is wrong
  if (overAtBoot)
  {
    stopAndDisable("OVERRUN is already HIGH at boot");
  }

  // NOTE: We need some max travel to prevent running forever.
  // Set this to something comfortably larger than your full travel span.
  const uint32_t MAX_HOME_SEEK_STEPS = 8000;

  if (homeAtBoot)
  {
    // Case A: HOME HIGH => move CW until HOME goes LOW, then creep CCW until HOME HIGH.
    Serial.println("[HOME] HOME is HIGH at boot: leaving switch, then re-approach slowly.");

    bool leftHome = moveUntilHomeState(DIR_CW, /*targetHomeHigh=*/false, MAX_HOME_SEEK_STEPS,
                                       /*slow=*/false, /*abortOnOverrun=*/true,
                                       "Leave HOME (CW until HOME LOW)");
    if (!leftHome)
      stopAndDisable("Failed to leave HOME before OVERRUN or max steps");

    bool foundHome = moveUntilHomeState(DIR_CCW, /*targetHomeHigh=*/true, MAX_HOME_SEEK_STEPS,
                                        /*slow=*/true, /*abortOnOverrun=*/true,
                                        "Re-find HOME slowly (CCW until HOME HIGH)");
    if (!foundHome)
      stopAndDisable("Failed to re-find HOME before OVERRUN or max steps");
  }
  else
  {
    // Case B: HOME LOW => move CCW until HOME goes HIGH
    Serial.println("[HOME] HOME is LOW at boot: seeking HOME CCW.");

    bool foundHome = moveUntilHomeState(DIR_CCW, /*targetHomeHigh=*/true, MAX_HOME_SEEK_STEPS,
                                        /*slow=*/false, /*abortOnOverrun=*/true,
                                        "Find HOME (CCW until HOME HIGH)");
    if (!foundHome)
    {
      // If we hit OVERRUN while seeking HOME, that means we never saw HOME
      stopAndDisable("HOME never went HIGH before OVERRUN (or max steps) during homing");
    }
  }

  // At this point we believe we are at HOME
  Serial.println("[HOME] HOME established. Running OVERRUN sanity check...");

  // Move CCW 128 steps; OVERRUN should become HIGH at/within this window (per your spec).
  setDir(DIR_CCW);
  bool sawOverrun = false;
  for (uint16_t i = 0; i < OVERRUN_CHECK_STEPS; i++)
  {
    stepOnceFast();
    if (overrun())
    {
      sawOverrun = true;
      Serial.print("[CHECK] OVERRUN became HIGH at step ");
      Serial.println(i);
      break;
    }
  }

  if (!sawOverrun)
  {
    stopAndDisable("OVERRUN did NOT go HIGH during 128-step sanity check");
  }

  // Return CW same number of steps (ensure we’re back at HOME)
  Serial.println("[CHECK] Returning to HOME (CW 128 steps)...");
  moveSteps(DIR_CW, OVERRUN_CHECK_STEPS, /*fast=*/true, /*abortOnOverrun=*/false, "Return from OVERRUN check");

  // Optional: verify HOME is HIGH again
  if (!home())
  {
    stopAndDisable("After OVERRUN check return, HOME is not HIGH");
  }

  Serial.println("[HOME] Homing complete. Ready for production.");
  Serial.println("========== BOOT: HOMING DONE ==========");
}

// ----------------------- Production logic -----------------------
static bool lastInjectCmd = false;

static bool injectCommandRisingEdge()
{
  bool now = digitalReadFast(PIN_INJECT_CMD);
  bool rising = (now && !lastInjectCmd);
  lastInjectCmd = now;
  return rising;
}

static void doOneInjectionCycle()
{
  // Safety: should only be called when at home/ready
  if (overrun())
  {
    stopAndDisable("OVERRUN HIGH while idle/ready");
  }

  Serial.println("[CYCLE] Inject command received.");

  // Clear ready while moving
  digitalWriteFast(PIN_HOME_READY, LOW);

  // Inject: CW 480 steps
  moveSteps(DIR_CW, INJECT_STEPS, /*fast=*/true, /*abortOnOverrun=*/true, "Inject (CW 480)");

  // Return: seek HOME CCW within 480+128 steps
  if (!returnHomeWithLimits())
  {
    stopAndDisable("Failed to return HOME within 480+128 steps or OVERRUN triggered");
  }

  // Confirm HOME is HIGH at end
  if (!home())
  {
    stopAndDisable("Return routine ended but HOME is not HIGH");
  }

  Serial.println("[CYCLE] Back at HOME. Ready.");
  digitalWriteFast(PIN_HOME_READY, HIGH);
}

// ----------------------- Setup / Loop -----------------------
void setup()
{
  Serial.begin(115200);
  delay(200);
  Serial.println("\n\nTeensy Bulb Ram Controller starting...");

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);

  pinMode(PIN_HOME_SENSOR, INPUT_PULLDOWN);     // Change to INPUT_PULLUP if your sensor is active-low
  pinMode(PIN_OVERRUN_SENSOR, INPUT_PULLDOWN);  // Change to INPUT_PULLUP if your sensor is active-low

  pinMode(PIN_HOME_READY, OUTPUT);
  pinMode(PIN_OVERRUN_ALARM, OUTPUT);
  pinMode(PIN_INJECT_CMD, INPUT_PULLDOWN);      // If Arduino drives it strongly, pull can be optional

  digitalWriteFast(PIN_HOME_READY, LOW);
  digitalWriteFast(PIN_OVERRUN_ALARM, LOW);

  // Default safe state: driver disabled until we begin homing
  driverEnable(false);

  // Initialize inject edge tracker
  lastInjectCmd = digitalReadFast(PIN_INJECT_CMD);

  // Home and sanity checks
  doHoming();

  // If homing succeeded, signal ready
  digitalWriteFast(PIN_HOME_READY, HIGH);
  Serial.println("[READY] HOME_READY=HIGH (pin 11). Waiting for inject commands...");
}

void loop()
{
  // Always monitor OVERRUN
  if (overrun())
  {
    stopAndDisable("OVERRUN triggered in main loop");
  }

  // If we somehow lost home while "ready", stop (optional but recommended)
  if (digitalReadFast(PIN_HOME_READY) && !home())
  {
    stopAndDisable("HOME_READY was HIGH but HOME sensor is LOW (unexpected position drift)");
  }

  // Command from Arduino: rising edge on pin 10 triggers one cycle
  if (injectCommandRisingEdge())
  {
    doOneInjectionCycle();
  }

  // Light idle delay (keeps loop responsive but not crazy fast)
  delay(1);
}
