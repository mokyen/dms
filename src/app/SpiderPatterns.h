#pragma once
#include "adapters/MotorDriver.h"
#include "adapters/EncoderReader.h"
#include "Config.h"
#include "app/MotorControlPid.h"
#include "app/MotionController.h"
#include "app/MotorSequencer.h"
#include <Arduino.h>

namespace SpiderPatterns {

// --- Percentage Helper ---

/**
 * @brief Converts a percentage (0.0f to 100.0f) into an absolute height in inches
 * based on the globally defined MAX_TRAVEL_IN_FLOAT.
 * @param percent The target height as a percentage (e.g., 50.0f for 50%).
 * @return The corresponding height in inches.
 */
inline float percentToInches(float percent) {
  // Clamp the percentage to ensure it's within the 0-100% range
  if (percent < 0.0f) percent = 0.0f;
  if (percent > 100.0f) percent = 100.0f;
  
  return (percent / 100.0f) * MAX_TRAVEL_IN_FLOAT;
}

// --- Conversion Functions (Unchanged) ---

inline long inchesToCounts(float inches) {
  return (long)(inches * COUNTS_PER_IN);
}

inline float countsToInches(long counts) {
  return counts * INV_COUNTS_PER_IN;
}

// --- Movement Functions (Unchanged Interface) ---

// Simple gentle move - takes inches, remains the low-level interface
inline void gentleMoveTo(MotorControlPid& controller, float targetInches) {
  long targetCounts = inchesToCounts(targetInches);
  controller.moveToCounts(targetCounts, 50);  // 65% max speed like your 'u' command
  
  // Wait for arrival
  while (controller.isMoving()) {
    controller.update();
    delay(10);
  }
}

// Simple gentle move - takes inches, remains the low-level interface
inline void veryGentleMoveTo(MotorControlPid& controller, float targetInches) {
  long targetCounts = inchesToCounts(targetInches);
  controller.moveToCounts(targetCounts, 45);
  
  // Wait for arrival
  while (controller.isMoving()) {
    controller.update();
    delay(10);
  }
}

// --- Percentage-Based Patterns ---

inline void pattern_stalker(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("Pattern: The Stalker (Percentage-Based)"));
  
  // Slow creep up to a moderate height (25% to 45% of max travel)
  // This replaces 20-35 inches
  float height1_perc = 25.0f + random(0, 21);
  gentleMoveTo(controller, percentToInches(height1_perc));
  delay(800 + random(0, 801));  // Pause
  
  // Continue creeping upwards (add 20-30% of max travel)
  // This replaces adding 15-25 inches
  float height2_perc = height1_perc + 20.0f + random(0, 11);
  
  // Clamp to 100% (MAX_TRAVEL)
  if (height2_perc > 100.0f) height2_perc = 100.0f; 
  
  gentleMoveTo(controller, percentToInches(height2_perc));
  delay(1200 + random(0, 1201));  // Long pause at top
  
  // Drop back down to 0%
  gentleMoveTo(controller, percentToInches(0.0f));
}

inline void pattern_pounce(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("Pattern: The Pounce (Percentage-Based)"));
  
  // Rise slowly to mid height (50%)
  float midHeight_perc = 50.0f;
  gentleMoveTo(controller, percentToInches(midHeight_perc));
  delay(400 + random(0, 401));
  
  // Go to top (100%)
  gentleMoveTo(controller, percentToInches(100.0f));
  delay(500 + random(0, 501));  // Pause at top
  
  // Drop to 0%
  gentleMoveTo(controller, percentToInches(0.0f));
}

inline void pattern_patrol(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("Pattern: The Patrol (Percentage-Based)"));
  
  // Three gentle heights (30-50%, 50-70%, 70-90%)
  // Replaces 25-45in, 40-60in, 55-75in
  float point1_perc = 30.0f + random(0, 21);  
  float point2_perc = 50.0f + random(0, 21); 
  float point3_perc = 70.0f + random(0, 21); 
  
  gentleMoveTo(controller, percentToInches(point1_perc));
  delay(1000 + random(0, 1001));
  
  gentleMoveTo(controller, percentToInches(point2_perc));
  delay(1000 + random(0, 1001));
  
  gentleMoveTo(controller, percentToInches(point3_perc));
  delay(1000 + random(0, 1001));
  
  gentleMoveTo(controller, percentToInches(0.0f));
}

inline void pattern_twitch(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("Pattern: The Twitch (Percentage-Based)"));
  
  // Start at a moderate height (40-55% of max travel)
  // Replaces 30-45 inches
  float baseHeight_perc = 40.0f + random(0, 16); 
  gentleMoveTo(controller, percentToInches(baseHeight_perc));
  delay(300);
  
  // Small twitchy movements, now based on percentage of max travel (~10% range)
  // Replaces offsets of 8in, 6in, 5in, 10in
  gentleMoveTo(controller, percentToInches(baseHeight_perc - 10.0f));
  delay(200);
  
  gentleMoveTo(controller, percentToInches(baseHeight_perc + 8.0f));
  delay(200);
  
  gentleMoveTo(controller, percentToInches(baseHeight_perc - 7.0f));
  delay(200);
  
  gentleMoveTo(controller, percentToInches(baseHeight_perc + 12.0f));
  delay(400);
  
  // Return to bottom (0%)
  gentleMoveTo(controller, percentToInches(0.0f));
}

inline void pattern_lurker(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("Pattern: The Lurker (Percentage-Based)"));
  
  // Very slow rise to top (100%)
  gentleMoveTo(controller, percentToInches(100.0f));
  delay(2500 + random(0, 2501));  // Long pause - lurking
  
  // Drop partway (40%)
  float midHeight_perc = 40.0f;
  gentleMoveTo(controller, percentToInches(midHeight_perc));
  delay(600 + random(0, 601));
  
  // Return to bottom (0%)
  gentleMoveTo(controller, percentToInches(0.0f));
}

inline void pattern_zero_encoder(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("Haunting mode: resetting encoder to prevent drift."));
  // These movements are already relative (100% and 0%)
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
