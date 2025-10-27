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

// Simple gentle move - just use your working moveToCounts
inline void gentleMoveTo(MotorControlPid& controller, float targetInches) {
  long targetCounts = inchesToCounts(targetInches);
  controller.moveToCounts(targetCounts, 50);  // 65% max speed like your 'u' command
  
  // Wait for arrival
  while (controller.isMoving()) {
    controller.update();
    delay(10);
  }
}

// Simple gentle move - just use your working moveToCounts
inline void veryGentleMoveTo(MotorControlPid& controller, float targetInches) {
  long targetCounts = inchesToCounts(targetInches);
  controller.moveToCounts(targetCounts, 45);
  
  // Wait for arrival
  while (controller.isMoving()) {
    controller.update();
    delay(10);
  }
}

inline void pattern_stalker(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("Pattern: The Stalker"));
  
  // Slow creep up
  float height1 = 20.0f + random(0, 16);  // 20-35 inches
  gentleMoveTo(controller, height1);
  delay(800 + random(0, 801));  // Pause
  
  // Continue creeping
  float height2 = height1 + 15.0f + random(0, 11);  // Add 15-25 inches
  if (height2 > MAX_TRAVEL_IN_FLOAT) height2 = MAX_TRAVEL_IN_FLOAT;
  gentleMoveTo(controller, height2);
  delay(1200 + random(0, 1201));  // Long pause at top
  
  // Drop back down
  gentleMoveTo(controller, 0.0f);
}

inline void pattern_pounce(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("Pattern: The Pounce"));
  
  // Rise slowly to mid height
  float midHeight = MAX_TRAVEL_IN_FLOAT * 0.5f;
  gentleMoveTo(controller, midHeight);
  delay(400 + random(0, 401));
  
  // Go to top
  gentleMoveTo(controller, MAX_TRAVEL_IN_FLOAT);
  delay(500 + random(0, 501));  // Pause at top
  
  // Drop
  gentleMoveTo(controller, 0.0f);
}

inline void pattern_patrol(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("Pattern: The Patrol"));
  
  // Three gentle heights
  float point1 = 25.0f + random(0, 21);  // 25-45 inches
  float point2 = 40.0f + random(0, 21);  // 40-60 inches
  float point3 = 55.0f + random(0, 21);  // 55-75 inches
  
  gentleMoveTo(controller, point1);
  delay(1000 + random(0, 1001));
  
  gentleMoveTo(controller, point2);
  delay(1000 + random(0, 1001));
  
  gentleMoveTo(controller, point3);
  delay(1000 + random(0, 1001));
  
  gentleMoveTo(controller, 0.0f);
}

inline void pattern_twitch(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("Pattern: The Twitch"));
  
  // Start at a moderate height
  float baseHeight = 30.0f + random(0, 16);  // 30-45 inches
  gentleMoveTo(controller, baseHeight);
  delay(300);
  
  // Small twitchy movements (all gentle)
  gentleMoveTo(controller, baseHeight - 8.0f);
  delay(200);
  
  gentleMoveTo(controller, baseHeight + 6.0f);
  delay(200);
  
  gentleMoveTo(controller, baseHeight - 5.0f);
  delay(200);
  
  gentleMoveTo(controller, baseHeight + 10.0f);
  delay(400);
  
  // Return to bottom
  gentleMoveTo(controller, 0.0f);
}

inline void pattern_lurker(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("Pattern: The Lurker"));
  
  // Very slow rise to top
  gentleMoveTo(controller, MAX_TRAVEL_IN_FLOAT);
  delay(2500 + random(0, 2501));  // Long pause - lurking
  
  // Drop partway
  float midHeight = MAX_TRAVEL_IN_FLOAT * 0.4f;
  gentleMoveTo(controller, midHeight);
  delay(600 + random(0, 601));
  
  // Return to bottom
  gentleMoveTo(controller, 0.0f);
}

inline void pattern_zero_encoder(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("Haunting mode: resetting encoder to prevent drift."));
  veryGentleMoveTo(controller, MAX_TRAVEL_IN_FLOAT);
  delay(2000);
  veryGentleMoveTo(controller, 0.0f);
  delay(2000);
  encoder.zero();
}

inline void runRandomPattern(MotorControlPid& controller, EncoderReader& encoder) {
  int pattern = random(1, 6);
  switch (pattern) {
    case 1: pattern_stalker(controller, encoder); break;
    case 2: pattern_pounce(controller, encoder); break;
    case 3: pattern_patrol(controller, encoder); break;
    case 4: pattern_twitch(controller, encoder); break;
    case 5: pattern_lurker(controller, encoder); break;
  }
}

inline void hauntingMode(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("=== HAUNTING MODE ACTIVATED ==="));

  static int countSinceReset = 0;

  while (true) {
    runRandomPattern(controller, encoder);
    unsigned long pauseMs = 2000 + random(0, 4001);  // 2-6 second rest
    Serial.print(F("Resting for "));
    Serial.print(pauseMs / 1000.0f);
    Serial.println(F(" seconds..."));
    delay(pauseMs);

    if (++countSinceReset >= 3) {
      pattern_zero_encoder(controller, encoder);
      countSinceReset = 0;
      delay(pauseMs);
    }
  }
}

} // namespace SpiderPatterns