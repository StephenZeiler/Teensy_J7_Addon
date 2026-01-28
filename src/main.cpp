#include <Arduino.h>
#include <math.h>
/*
 * Bulb Ram + Wheel Motor Controller - Teensy Code
 * 
 * PIN LAYOUT:
 * 
 * RAM Motor Driver:
 *   DIR = 3  (Direction control)
 *   PUL = 2  (Pulse/Step control)
 *   ENA = 4  (Enable)
 * 
 * Wheel Motor Driver:
 *   DIR = 6  (Direction control)
 *   STEP = 5 (Pulse/Step control)
 *   ENA = 7  (Enable)
 * 
 * Sensors:
 *   HOME_SENSOR = 32 (Detects ram home position)
 *   OVERRUN_SENSOR = 30 (Detects ram overrun/limit)
 *   WHEEL_POSITION_SENSOR = 31 (Detects correct wheel position - LOW when correct)
 * 
 * Arduino Communication:
 *   HOME_NOTIFICATION = 11 (Teensy -> Arduino: Ram is home and ready)
 *   INJECT_COMMAND = 10 (Arduino -> Teensy: Ram go down, wait for LOW to return)
 *   OVERRUN_ALARM = 12 (Teensy -> Arduino: Error/alarm signal)
 *   TROLL_HOME_COMMAND = 24 (Arduino -> Teensy: Move wheel to home)
 *   MOVE_WHEEL_COMMAND = 25 (Arduino -> Teensy: Pulse to move wheel one slot)
 *   WHEEL_READY_NOTIFICATION = 26 (Teensy -> Arduino: Wheel is ready)
 *   WHEEL_POSITION_ALARM = 27 (Teensy -> Arduino: Wheel position sensor error)
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
const int WHEEL_STEP_PIN = 5;
const int WHEEL_DIR_PIN = 6;
const int WHEEL_ENABLE_PIN = 7;

// =====================
// SENSOR PINS
// =====================
const int HOME_SENSOR = 32;
const int OVERRUN_SENSOR = 30;
const int WHEEL_POSITION_SENSOR = 31;

// =====================
// ARDUINO COMMUNICATION PINS
// =====================
const int HOME_NOTIFICATION = 11;        // Output to Arduino: Ram home
const int INJECT_COMMAND = 10;           // Input from Arduino: Ram inject (stays down while HIGH)
const int OVERRUN_ALARM = 12;            // Output to Arduino: Error
const int TROLL_HOME_COMMAND = 24;       // Input from Arduino: Troll wheel to home
const int MOVE_WHEEL_COMMAND = 25;       // Input from Arduino: Pulse to move wheel one slot
const int WHEEL_READY_NOTIFICATION = 26; // Output to Arduino: Wheel ready
const int WHEEL_POSITION_ALARM = 27;     // Output to Arduino: Wheel position error

// =====================
// RAM MOTOR PARAMETERS
// =====================
const int STEP_DELAY = 500;        // Microseconds between steps (adjust for speed)
const int SLOW_STEP_DELAY = 1000;  // Slower speed for precise homing
const int INJECT_STEPS = 346;      // Steps to inject one bulb
const int OVERRUN_CHECK_STEPS = 128; // Steps to verify overrun sensor
const int SAFETY_MARGIN_STEPS = 128; // Additional steps before declaring error

// =====================
// WHEEL MOTOR PARAMETERS
// =====================
const int WHEEL_STEPS_PER_SLOT = 200;  // Steps to move one wheel slot
const int WHEEL_TROLL_SPEED = 1000;    // Microseconds between steps when trolling home

// Test mode - set to true for automatic continuous injection, false for normal Arduino control
const bool TEST_MODE = false;

// Direction definitions
const bool CLOCKWISE = LOW;
const bool COUNTER_CLOCKWISE = HIGH;

// =====================
// STATE VARIABLES
// =====================
bool ramIsHomed = false;
bool wheelIsHomed = false;
bool readyForProduction = false;
int lastInjectCommand = LOW;
int lastTrollHomeCommand = LOW;
int lastMoveWheelCommand = LOW;

// =====================
// RAM MOTOR FUNCTIONS
// =====================
void setDirection(bool dir) {
  digitalWrite(DIR_PIN, dir);
  delayMicroseconds(5); // Small delay for direction change
}

void stepMotor(int delayTime) {
  digitalWrite(PUL_PIN, HIGH);
  delayMicroseconds(delayTime);
  digitalWrite(PUL_PIN, LOW);
  delayMicroseconds(delayTime);
}

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
  
  // Disable motors
  digitalWrite(ENA_PIN, HIGH);
  digitalWrite(WHEEL_ENABLE_PIN, HIGH);
  
  // Halt execution
  while(1) {
    delay(1000);
  }
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
  
  // Disable motors
  digitalWrite(ENA_PIN, HIGH);
  digitalWrite(WHEEL_ENABLE_PIN, HIGH);
  
  // Halt execution
  while(1) {
    delay(1000);
  }
}

void performRamHomingSequence() {
  Serial.println("--- RAM HOMING SEQUENCE START ---");
  
  // Check initial home sensor state
  bool homeState = digitalRead(HOME_SENSOR);
  
  if (homeState == HIGH) {
    Serial.println("Home sensor is HIGH - ram near home position");
    Serial.println("Moving CLOCKWISE until home sensor goes LOW...");
    
    // Move clockwise until home sensor goes LOW
    setDirection(CLOCKWISE);
    while (digitalRead(HOME_SENSOR) == HIGH) {
      if (digitalRead(OVERRUN_SENSOR) == HIGH) {
        Serial.println("ERROR: Hit overrun sensor while clearing home!");
        triggerOverrunAlarm();
        return;
      }
      stepMotor(STEP_DELAY);
    }
    
    Serial.println("Home sensor is now LOW");
    Serial.println("Moving COUNTER-CLOCKWISE slowly back to home...");
    
    // Slowly move counter-clockwise back to home
    setDirection(COUNTER_CLOCKWISE);
    while (digitalRead(HOME_SENSOR) == LOW) {
      if (digitalRead(OVERRUN_SENSOR) == HIGH) {
        Serial.println("ERROR: Hit overrun sensor while returning to home!");
        triggerOverrunAlarm();
        return;
      }
      stepMotor(SLOW_STEP_DELAY);
    }
    
    Serial.println("Home sensor is HIGH - at home position");
    
  } else {
    Serial.println("Home sensor is LOW - ram away from home");
    Serial.println("Moving COUNTER-CLOCKWISE to find home...");
    
    // Move counter-clockwise until home sensor goes HIGH
    setDirection(COUNTER_CLOCKWISE);
    int stepCount = 0;
    while (digitalRead(HOME_SENSOR) == LOW) {
      if (digitalRead(OVERRUN_SENSOR) == HIGH) {
        Serial.println("ERROR: Hit overrun sensor before finding home!");
        triggerOverrunAlarm();
        return;
      }
      stepMotor(STEP_DELAY);
      stepCount++;
      
      // Safety check - prevent infinite loop
      if (stepCount > 2000) {
        Serial.println("ERROR: Home sensor never found!");
        triggerOverrunAlarm();
        return;
      }
    }
    
    Serial.println("Home sensor is HIGH - at home position");
  }
  
  // Now verify overrun sensor is working
  Serial.println("\nVerifying overrun sensor...");
  Serial.print("Moving COUNTER-CLOCKWISE ");
  Serial.print(OVERRUN_CHECK_STEPS);
  Serial.println(" steps to check overrun sensor");
  
  setDirection(COUNTER_CLOCKWISE);
  for (int i = 0; i < OVERRUN_CHECK_STEPS; i++) {
    stepMotor(STEP_DELAY);
  }
  
  // Check if overrun sensor is triggered
  if (digitalRead(OVERRUN_SENSOR) != HIGH) {
    Serial.println("ERROR: Overrun sensor did NOT trigger!");
    Serial.println("Overrun sensor may be faulty or disconnected");
    triggerOverrunAlarm();
    return;
  }
  
  Serial.println("Overrun sensor verified - reading HIGH");
  Serial.print("Returning CLOCKWISE ");
  Serial.print(OVERRUN_CHECK_STEPS);
  Serial.println(" steps to home");
  
  // Return to home
  setDirection(CLOCKWISE);
  for (int i = 0; i < OVERRUN_CHECK_STEPS; i++) {
    stepMotor(STEP_DELAY);
  }
  
  // Verify we're back at home
  if (digitalRead(HOME_SENSOR) != HIGH) {
    Serial.println("WARNING: Not at home sensor after return");
    // Fine tune position
    while (digitalRead(HOME_SENSOR) == LOW) {
      stepMotor(SLOW_STEP_DELAY);
    }
  }
  
  Serial.println("\n=================================");
  Serial.println("RAM HOMING COMPLETE");
  Serial.println("=================================\n");
  
  ramIsHomed = true;
  
  // Signal Arduino that ram is home
  digitalWrite(HOME_NOTIFICATION, HIGH);
}

void performInjectionCycle() {
  Serial.println("Starting injection cycle...");
  
  // Move clockwise to inject bulb
  Serial.print("Injecting: Moving CLOCKWISE ");
  Serial.print(INJECT_STEPS);
  Serial.println(" steps");
  
  setDirection(CLOCKWISE);
  for (int i = 0; i < INJECT_STEPS; i++) {
    stepMotor(STEP_DELAY);
  }
  
  Serial.println("Injection position reached - ram is DOWN");
  Serial.println("Waiting for inject command (pin 10) to go LOW before returning...");
  
  // Wait for inject command to go LOW
  while (digitalRead(INJECT_COMMAND) == HIGH) {
    delay(10); // Small delay while waiting
  }
  
  Serial.println("Inject command is now LOW - returning to home");
  
  // Return counter-clockwise to home
  Serial.print("Returning: Moving COUNTER-CLOCKWISE ");
  Serial.print(INJECT_STEPS);
  Serial.println(" steps");
  
  setDirection(COUNTER_CLOCKWISE);
  int stepCount = 0;
  bool reachedHome = false;
  
  for (int i = 0; i < INJECT_STEPS; i++) {
    // Check for overrun during return
    if (digitalRead(OVERRUN_SENSOR) == HIGH) {
      Serial.println("ERROR: Overrun sensor triggered during return!");
      triggerOverrunAlarm();
      readyForProduction = false;
      return;
    }
    
    stepMotor(STEP_DELAY);
    
    // Check if we reached home
    if (digitalRead(HOME_SENSOR) == HIGH && !reachedHome) {
      Serial.println("Home sensor detected during return");
      reachedHome = true;
    }
  }
  
  // Verify we're at home
  if (digitalRead(HOME_SENSOR) != HIGH) {
    Serial.println("WARNING: Not at home after return steps");
    Serial.println("Attempting to find home...");
    
    // Try additional steps
    for (int i = 0; i < SAFETY_MARGIN_STEPS; i++) {
      if (digitalRead(HOME_SENSOR) == HIGH) {
        Serial.println("Home found within safety margin");
        reachedHome = true;
        break;
      }
      
      if (digitalRead(OVERRUN_SENSOR) == HIGH) {
        Serial.println("ERROR: Hit overrun, missed home sensor!");
        triggerOverrunAlarm();
        readyForProduction = false;
        return;
      }
      
      stepMotor(STEP_DELAY);
    }
    
    if (digitalRead(HOME_SENSOR) != HIGH) {
      Serial.println("ERROR: Missed both home and overrun sensors!");
      triggerOverrunAlarm();
      readyForProduction = false;
      return;
    }
  }
  
  Serial.println("Successfully returned to home");
  Serial.println("Ready for next cycle\n");
  
  // Signal Arduino that ram is ready again
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
  
  digitalWrite(WHEEL_ENABLE_PIN, LOW); // Enable wheel motor
  digitalWrite(WHEEL_DIR_PIN, LOW);    // Set direction
  
  // Move slowly while Arduino signal is HIGH
  while (digitalRead(TROLL_HOME_COMMAND) == HIGH) {
    stepWheelMotor(WHEEL_TROLL_SPEED);
  }
  
  Serial.println("Arduino home sensor detected - wheel is home");
  wheelIsHomed = true;
}

void moveWheelOneSlot() {
  Serial.println("--- MOVING WHEEL ONE SLOT ---");
  Serial.print("Moving ");
  Serial.print(WHEEL_STEPS_PER_SLOT);
  Serial.println(" steps");
  
  digitalWrite(WHEEL_ENABLE_PIN, LOW); // Enable wheel motor
  digitalWrite(WHEEL_DIR_PIN, LOW);    // Set direction
  
  // Move the wheel the specified number of steps
  for (int i = 0; i < WHEEL_STEPS_PER_SLOT; i++) {
    stepWheelMotor(WHEEL_TROLL_SPEED);
  }
  
  Serial.println("Wheel movement complete");
  
  // Check wheel position sensor
  Serial.println("Checking wheel position sensor (pin 31)...");
  if (digitalRead(WHEEL_POSITION_SENSOR) == LOW) {
    Serial.println("Wheel position sensor: CORRECT (LOW)");
    Serial.println("Wheel is ready for injection\n");
    
    // Signal Arduino that wheel is ready
    digitalWrite(WHEEL_READY_NOTIFICATION, HIGH);
  } else {
    Serial.println("ERROR: Wheel position sensor still HIGH!");
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
  pinMode(INJECT_COMMAND, INPUT);
  pinMode(OVERRUN_ALARM, OUTPUT);
  pinMode(TROLL_HOME_COMMAND, INPUT);
  pinMode(MOVE_WHEEL_COMMAND, INPUT);
  pinMode(WHEEL_READY_NOTIFICATION, OUTPUT);
  pinMode(WHEEL_POSITION_ALARM, OUTPUT);
  
  // Initialize outputs to LOW
  digitalWrite(HOME_NOTIFICATION, LOW);
  digitalWrite(OVERRUN_ALARM, LOW);
  digitalWrite(WHEEL_READY_NOTIFICATION, LOW);
  digitalWrite(WHEEL_POSITION_ALARM, LOW);
  digitalWrite(ENA_PIN, LOW); // Enable ram motor (active low for most drivers)
  digitalWrite(WHEEL_ENABLE_PIN, HIGH); // Disable wheel motor initially
  
  Serial.println("Hardware initialized");
  Serial.println("Waiting for homing command (pin 24) from Arduino...\n");
}

// =====================
// MAIN LOOP
// =====================
void loop() {
  // Check for troll home command from Arduino
  int trollHomeSignal = digitalRead(TROLL_HOME_COMMAND);
  
  // Detect rising edge of troll home command
  if (trollHomeSignal == HIGH && lastTrollHomeCommand == LOW) {
    Serial.println("\n>>> TROLL HOME COMMAND RECEIVED (pin 24) <<<");
    digitalWrite(HOME_NOTIFICATION, LOW);
    digitalWrite(WHEEL_READY_NOTIFICATION, LOW);
    
    // Start both homing sequences
    // Wheel will troll until Arduino sensor detects home
    trollWheelToHome();
    
    // Ram will find its home
    performRamHomingSequence();
    
    // Both are now homed
    if (ramIsHomed && wheelIsHomed) {
      Serial.println("\n=================================");
      Serial.println("BOTH MOTORS HOMED - READY FOR PRODUCTION");
      Serial.println("=================================\n");
      readyForProduction = true;
    }
  }
  
  lastTrollHomeCommand = trollHomeSignal;
  
  // Check for move wheel command (only if ready for production)
  if (readyForProduction) {
    int moveWheelSignal = digitalRead(MOVE_WHEEL_COMMAND);
    
    // Detect rising edge of move wheel command (short pulse on pin 25)
    if (moveWheelSignal == HIGH && lastMoveWheelCommand == LOW) {
      Serial.println("\n>>> MOVE WHEEL COMMAND RECEIVED (pin 25) <<<");
      digitalWrite(WHEEL_READY_NOTIFICATION, LOW);
      
      // Move wheel one slot
      moveWheelOneSlot();
      // Wheel ready signal (pin 26) is set inside moveWheelOneSlot if position is correct
    }
    
    lastMoveWheelCommand = moveWheelSignal;
    
    // Check for inject command
    int injectSignal = digitalRead(INJECT_COMMAND);
    
    // Detect rising edge of inject command (pin 10)
    if (injectSignal == HIGH && lastInjectCommand == LOW) {
      Serial.println("\n>>> INJECT COMMAND RECEIVED (pin 10) <<<");
      digitalWrite(HOME_NOTIFICATION, LOW);
      
      // Perform injection cycle (will wait for pin 10 to go LOW before returning)
      performInjectionCycle();
    }
    
    lastInjectCommand = injectSignal;
  }
  
  delay(10); // Small delay to prevent excessive polling
}