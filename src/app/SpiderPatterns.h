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

inline void cruiseToPosition(MotorControlPid& controller, EncoderReader& encoder, 
                             float targetInches, float velocityInchesPerSec, 
                             int updateRateHz = 50) {
  float currentInches = encoder.getPositionInches();
  float direction = (targetInches > currentInches) ? 1.0f : -1.0f;
  float distance = abs(targetInches - currentInches);
  
  int stepsNeeded = (int)((distance / abs(velocityInchesPerSec)) * updateRateHz);
  if (stepsNeeded < 1) stepsNeeded = 1;
  static constexpr int MAX_STEPS = 12;
  if (stepsNeeded > MAX_STEPS) stepsNeeded = MAX_STEPS;
  
  float positionIncrement = distance / stepsNeeded * direction;
  
  for (int step = 0; step < stepsNeeded; step++) {
    currentInches += positionIncrement;
    controller.moveToCounts(inchesToCounts(currentInches), 100);
    runUpdateUntil(controller, timeout(1000 / updateRateHz));
  }
  
  controller.moveToCounts(inchesToCounts(targetInches), 100);
  runUpdateUntilArrived(controller);
}

inline void pattern_stalker(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("Pattern: The Stalker"));
  float burstHeight = 30.0f + random(0, 21);
  cruiseToPosition(controller, encoder, burstHeight, 80.0f, 25);
  runUpdateUntil(controller, timeout(500 + random(0, 501)));
  float creepDistance = 20.0f + random(0, 11);
  float creepTarget = burstHeight + creepDistance;
  if (creepTarget > MAX_TRAVEL_IN_FLOAT) creepTarget = MAX_TRAVEL_IN_FLOAT;
  cruiseToPosition(controller, encoder, creepTarget, 8.0f);
  runUpdateUntil(controller, timeout(1000 + random(0, 1001)));
  cruiseToPosition(controller, encoder, 0.0f, 50.0f);
}

inline void pattern_pounce(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("Pattern: The Pounce"));
  float midHeight = MAX_TRAVEL_IN_FLOAT * 0.6f;
  cruiseToPosition(controller, encoder, midHeight, 10.0f);
  cruiseToPosition(controller, encoder, MAX_TRAVEL_IN_FLOAT, 60.0f);
  runUpdateUntil(controller, timeout(300 + random(0, 201)));
  cruiseToPosition(controller, encoder, 0.0f, 55.0f);
}

inline void pattern_patrol(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("Pattern: The Patrol"));
  float point1 = 80.0f + random(0, 51);
  float point2 = 80.0f + random(0, 51);
  float point3 = 80.0f + random(0, 51);
  cruiseToPosition(controller, encoder, point1, 25.0f);
  runUpdateUntil(controller, timeout(1000 + random(0, 1001)));
  cruiseToPosition(controller, encoder, point2, 25.0f);
  runUpdateUntil(controller, timeout(1000 + random(0, 1001)));
  cruiseToPosition(controller, encoder, point3, 25.0f);
  runUpdateUntil(controller, timeout(1000 + random(0, 1001)));
  cruiseToPosition(controller, encoder, 0.0f, 35.0f);
}

inline void pattern_twitch(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("Pattern: The Twitch"));
  float startHeight = 40.0f + random(0, 21);
  cruiseToPosition(controller, encoder, startHeight, 35.0f);
  float currentPos = startHeight;
  currentPos -= 10.0f;
  cruiseToPosition(controller, encoder, currentPos, 30.0f);
  currentPos += 15.0f;
  cruiseToPosition(controller, encoder, currentPos, 30.0f);
  currentPos -= 8.0f;
  cruiseToPosition(controller, encoder, currentPos, 30.0f);
  currentPos += 12.0f;
  cruiseToPosition(controller, encoder, currentPos, 30.0f);
  runUpdateUntil(controller, timeout(500));
  if (random(0, 2) == 0) {
    cruiseToPosition(controller, encoder, 0.0f, 45.0f);
  } else {
    cruiseToPosition(controller, encoder, MAX_TRAVEL_IN_FLOAT, 40.0f);
  }
}

inline void pattern_lurker(MotorControlPid& controller, EncoderReader& encoder) {
  Serial.println(F("Pattern: The Lurker"));
  cruiseToPosition(controller, encoder, MAX_TRAVEL_IN_FLOAT, 6.0f);
  runUpdateUntil(controller, timeout(2000 + random(0, 2001)));
  float midHeight = MAX_TRAVEL_IN_FLOAT * 0.4f;
  cruiseToPosition(controller, encoder, midHeight, 20.0f);
  runUpdateUntil(controller, timeout(500 + random(0, 501)));
  cruiseToPosition(controller, encoder, 0.0f, 50.0f);
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
  while (true) {
    runRandomPattern(controller, encoder);
    unsigned long pauseMs = 1000 + random(0, 4001);
    Serial.print(F("Resting for "));
    Serial.print(pauseMs / 1000.0f);
    Serial.println(F(" seconds..."));
    runUpdateUntil(controller, timeout(pauseMs));
  }
}

} // namespace SpiderPatterns
