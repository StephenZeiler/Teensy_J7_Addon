#include "motor_control.h"
#include <Arduino.h>

// Motor control pins
#define stepPinM1 22
#define dirPinM1 23
#define enPinM1 24
 // Example pin, adjust as needed
 unsigned long previousM1Micros = 0;
 unsigned long stepInterval = 1000;
 bool moving = false;
 int currentStep = 0;
 unsigned long cyclePauseStart = 0;
 const int TOTAL_STEPS = 100;
 const int MAX_SPEED = 1000;
 const int MIN_SPEED = 100;
 const unsigned long CYCLE_DELAY = 1000000;  // 1 second delay between cycles

void runMotorM1() {
  pinMode(stepPinM1, OUTPUT);
  pinMode(dirPinM1, OUTPUT);
  pinMode(enPinM1, OUTPUT);
  digitalWrite(enPinM1, LOW);
  digitalWrite(dirPinM1, LOW);  // Set direction

unsigned long currentMicros = micros();

    if (moving) {
        // Check if it's time to step
        if ((currentMicros - previousM1Micros) >= stepInterval) {
            // Make a step
            if (currentStep < TOTAL_STEPS) {
                if (currentStep % 2 == 0) {
                    digitalWrite(stepPinM1, HIGH);
                } else {
                    digitalWrite(stepPinM1, LOW);
                    currentStep++; // Count steps
                }
            }

            // Acceleration & Deceleration using exponential curve
            float progress = (float)currentStep / TOTAL_STEPS;  // Progress from 0.0 to 1.0

            // Exponential acceleration for the first 70% of the steps
            if (progress < 0.7) {
                float accelProgress = exp(progress * 3);  // Exponential acceleration, adjust "3" for curve steepness
                stepInterval = MAX_SPEED - (MAX_SPEED - MIN_SPEED) * accelProgress;
            } 
            // Exponential deceleration for the last 30% of the steps
            else {
                float decelProgress = exp((progress - 0.7) * 3);  // Exponential deceleration
                stepInterval = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * (1 - decelProgress);
            }

            previousM1Micros = currentMicros;

            // Check if cycle is complete
            if (currentStep >= TOTAL_STEPS) {
                moving = false;
                cyclePauseStart = micros();  // Record the pause start time
                currentStep = 0; // Reset step counter
                stepInterval = MAX_SPEED; // Reset speed for next cycle
            }
        }
    } else {
        // 1-second pause before restarting motion
        if (currentMicros - cyclePauseStart >= CYCLE_DELAY) {
            moving = true;
        }
    }
}