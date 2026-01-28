/*
 * Bulb Ram Controller - Teensy Code
 * 
 * PIN LAYOUT:
 * Motor Driver:
 *   DIR = 3  (Direction control)
 *   PUL = 2  (Pulse/Step control)
 *   ENA = 4  (Enable)
 * 
 * Sensors:
 *   HOME_SENSOR = 28 (Detects home position)
 *   OVERRUN_SENSOR = 30 (Detects overrun/limit)
 * 
 * Arduino Communication:
 *   HOME_NOTIFICATION = 11 (Teensy -> Arduino: Ready signal)
 *   INJECT_COMMAND = 10 (Arduino -> Teensy: Inject one bulb)
 *   OVERRUN_ALARM = 12 (Teensy -> Arduino: Error/alarm signal)
 */

// Motor pins
const int DIR_PIN = 3;
const int PUL_PIN = 2;
const int ENA_PIN = 4;

// Sensor pins
const int HOME_SENSOR = 28;
const int OVERRUN_SENSOR = 30;

// Arduino communication pins
const int HOME_NOTIFICATION = 11;  // Output to Arduino
const int INJECT_COMMAND = 10;     // Input from Arduino
const int OVERRUN_ALARM = 12;      // Output to Arduino

// Motor parameters
const int STEP_DELAY = 500;        // Microseconds between steps (adjust for speed)
const int SLOW_STEP_DELAY = 1000;  // Slower speed for precise homing
const int INJECT_STEPS = 480;      // Steps to inject one bulb
const int OVERRUN_CHECK_STEPS = 128; // Steps to verify overrun sensor
const int SAFETY_MARGIN_STEPS = 128; // Additional steps before declaring error

// Direction definitions
const bool CLOCKWISE = HIGH;
const bool COUNTER_CLOCKWISE = LOW;

// State variables
bool isHomed = false;
bool readyForProduction = false;
int lastInjectCommand = LOW;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=================================");
  Serial.println("Bulb Ram Controller Starting...");
  Serial.println("=================================");
  
  // Configure motor pins
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PUL_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  
  // Configure sensor pins
  pinMode(HOME_SENSOR, INPUT);
  pinMode(OVERRUN_SENSOR, INPUT);
  
  // Configure communication pins
  pinMode(HOME_NOTIFICATION, OUTPUT);
  pinMode(INJECT_COMMAND, INPUT);
  pinMode(OVERRUN_ALARM, OUTPUT);
  
  // Initialize outputs to LOW
  digitalWrite(HOME_NOTIFICATION, LOW);
  digitalWrite(OVERRUN_ALARM, LOW);
  digitalWrite(ENA_PIN, LOW); // Enable motor (active low for most drivers)
  
  Serial.println("Hardware initialized");
  Serial.println("Beginning homing sequence...\n");
  
  // Perform initial homing sequence
  performHomingSequence();
}

void loop() {
  if (readyForProduction) {
    // Check for inject command from Arduino
    int injectSignal = digitalRead(INJECT_COMMAND);
    
    // Detect rising edge of inject command
    if (injectSignal == HIGH && lastInjectCommand == LOW) {
      Serial.println("\n>>> INJECT COMMAND RECEIVED <<<");
      digitalWrite(HOME_NOTIFICATION, LOW); // Clear ready signal
      
      // Perform injection cycle
      performInjectionCycle();
    }
    
    lastInjectCommand = injectSignal;
  }
  
  delay(10); // Small delay to prevent excessive polling
}

void performHomingSequence() {
  Serial.println("--- HOMING SEQUENCE START ---");
  
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
  Serial.println("HOMING COMPLETE - READY FOR PRODUCTION");
  Serial.println("=================================\n");
  
  isHomed = true;
  readyForProduction = true;
  
  // Signal Arduino that we're ready
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
  
  Serial.println("Injection complete - returning to home");
  
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
  
  // Signal Arduino that we're ready again
  digitalWrite(HOME_NOTIFICATION, HIGH);
}

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
  digitalWrite(OVERRUN_ALARM, HIGH);
  delay(1000);
  digitalWrite(OVERRUN_ALARM, LOW);
  
  Serial.println("Machine stopped - manual intervention required");
  Serial.println("Please reset the system after correcting the issue\n");
  
  // Disable motor
  digitalWrite(ENA_PIN, HIGH);
  
  // Halt execution
  while(1) {
    delay(1000);
  }
}
