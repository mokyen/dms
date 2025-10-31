#pragma once
#include "adapters/MotorDriver.h"
#include "adapters/EncoderReader.h"
#include "Config.h"
#include "app/MotorControlPid.h"
#include "app/MotionController.h"
#include "app/MotorSequencer.h"
#include <Arduino.h>

namespace SpiderPatterns {

inline long inchesToCounts(float inches) {
  return (long)(inches * COUNTS_PER_IN);
}

inline float countsToInches(long counts) {
  return counts * INV_COUNTS_PER_IN;
}

// Simple gentle move
inline void gentleMoveTo(MotorControlPid& controller, float targetInches) {
  long targetCounts = inchesToCounts(targetInches);
  controller.moveToCounts(targetCounts, 55);  // 55% max speed
  
  // Wait for arrival
  while (controller.isMoving()) {
    controller.update();
    delay(10);
  }
}

inline void pattern_stalker(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("Pattern: The Stalker"));
  
  gentleMoveTo(controller, 30.0f);  // Rise to 30 inches
  delay(1000);
  
  gentleMoveTo(controller, 50.0f);  // Continue to 50 inches
  delay(1500);
  
  gentleMoveTo(controller, 0.0f);   // Drop
}

inline void pattern_pounce(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("Pattern: The Pounce"));
  
  gentleMoveTo(controller, MAX_TRAVEL_IN_FLOAT * 0.5f);  // Rise to mid
  delay(500);
  
  gentleMoveTo(controller, MAX_TRAVEL_IN_FLOAT);         // Go to top
  delay(800);
  
  gentleMoveTo(controller, 0.0f);                        // Drop
}

inline void pattern_patrol(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("Pattern: The Patrol"));
  
  gentleMoveTo(controller, 30.0f);
  delay(1200);
  
  gentleMoveTo(controller, 50.0f);
  delay(1200);
  
  gentleMoveTo(controller, 65.0f);
  delay(1200);
  
  gentleMoveTo(controller, 0.0f);
}

inline void pattern_lurker(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("Pattern: The Lurker"));
  
  gentleMoveTo(controller, MAX_TRAVEL_IN_FLOAT);  // Slow rise to top
  delay(3000);  // Long pause - lurking
  
  gentleMoveTo(controller, MAX_TRAVEL_IN_FLOAT * 0.4f);  // Drop partway
  delay(800);
  
  gentleMoveTo(controller, 0.0f);  // Return to bottom
}

inline void pattern_zero_encoder(MotorControlPid& controller, EncoderReader& encoder, MotorDriver& motor) {
  Serial.println(F("=== HOMING ROUTINE ==="));
  Serial.println(F("Finding mechanical zero using current sensing..."));
  
  // Step 1: Move up to clear any bottom contact
  Serial.println(F("Step 1: Moving up to clear bottom"));
  gentleMoveTo(controller, 10.0f);
  delay(1000);
  
  // Step 2: Slowly move down while monitoring current
  Serial.println(F("Step 2: Searching for bottom (monitoring current)"));
  
  const float SEARCH_SPEED = -0.15f;
  const float CURRENT_SPIKE_THRESHOLD = 2.5f;
  const unsigned long SAMPLE_INTERVAL_MS = 20;
  
  controller.stop();
  delay(100);
  
  bool foundBottom = false;
  unsigned long lastSampleMs = millis();
  float maxCurrent = 0.0f;
  int stableHighCount = 0;
  
  motor.setSpeed(SEARCH_SPEED);
  
  unsigned long searchStartMs = millis();
  const unsigned long SEARCH_TIMEOUT_MS = 30000;
  
  while (!foundBottom && (millis() - searchStartMs < SEARCH_TIMEOUT_MS)) {
    unsigned long now = millis();
    
    if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
      lastSampleMs = now;
      
      float current = motor.readCurrent();
      long position = encoder.getPositionCounts();
      
      if (current > maxCurrent) maxCurrent = current;
      
      if (current > CURRENT_SPIKE_THRESHOLD) {
        stableHighCount++;
        if (stableHighCount >= 3) {
          foundBottom = true;
          Serial.print(F("Bottom detected! Current: "));
          Serial.print(current, 3);
          Serial.print(F("A, Position: "));
          Serial.println(position);
        }
      } else {
        stableHighCount = 0;
      }
      
      static unsigned long lastDebugMs = 0;
      if (now - lastDebugMs > 500) {
        Serial.print(F("Searching... pos="));
        Serial.print(position);
        Serial.print(F(" current="));
        Serial.print(current, 3);
        Serial.print(F("A maxCurrent="));
        Serial.println(maxCurrent, 3);
        lastDebugMs = now;
      }
    }
    
    delay(10);
  }
  
  motor.stop();
  
  if (!foundBottom) {
    Serial.println(F("ERROR: Homing timeout!"));
    return;
  }
  
  delay(500);
  
  // Step 3: Back off slightly
  Serial.println(F("Step 3: Backing off from mechanical stop"));
  long currentPos = encoder.getPositionCounts();
  long backoffCounts = (long)(0.5f * COUNTS_PER_IN);
  
  controller.moveToCounts(currentPos + backoffCounts, 30);
  while (controller.isMoving()) {
    controller.update();
    delay(10);
  }
  
  // Step 4: Zero
  Serial.println(F("Step 4: Setting encoder zero"));
  encoder.zero();
  
  Serial.println(F("=== HOMING COMPLETE ==="));
  Serial.print(F("Max current seen: "));
  Serial.print(maxCurrent, 3);
  Serial.println(F("A"));
  
  delay(1000);
}

inline void runPattern(int patternNum, MotorControlPid& controller, EncoderReader& encoder) {
  switch (patternNum) {
    case 1: pattern_stalker(controller, encoder); break;
    case 2: pattern_pounce(controller, encoder); break;
    case 3: pattern_patrol(controller, encoder); break;
    case 4: pattern_lurker(controller, encoder); break;
    default: 
      Serial.println(F("Invalid pattern number"));
      break;
  }
}

inline void hauntingMode(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("=== HAUNTING MODE ACTIVATED ==="));
  int patternIndex = 1;
  
  while (true) {
    runPattern(patternIndex, controller, encoder);
    
    Serial.print(F("Resting for 3 seconds..."));
    delay(3000);
    
    // Cycle through patterns: 1, 2, 3, 4, 1, 2, 3, 4...
    patternIndex++;
    if (patternIndex > 4) patternIndex = 1;
  }
}

} // namespace SpiderPatterns