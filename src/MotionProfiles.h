#pragma once
#include <Arduino.h>
#include "MotorDriver.h"
#include "EncoderReader.h"
#include "Config.h"

namespace MotionProfiles {

constexpr float INV_MAX_TRAVEL = 1.0f / MAX_TRAVEL_IN;
constexpr float POSITION_TO_PHASE = PI * INV_MAX_TRAVEL;

// Linear move to a specific position
inline void moveToPosition(MotorDriver& motor, EncoderReader& encoder, float targetInches, float speedFraction) {
  if (targetInches < 0) targetInches = 0;
  if (targetInches > MAX_TRAVEL_IN) targetInches = MAX_TRAVEL_IN;

  float error = targetInches - encoder.getPositionInches();
  if (fabs(error) < POSITION_TOLERANCE_IN) {
    motor.brake();
    return;
  }
  float direction = (error > 0) ? 1.0f : -1.0f;
  motor.setSpeed(speedFraction * direction);
}

// Stop when near bottom
inline void stopAtBottom(MotorDriver& motor, EncoderReader& encoder) {
  if (encoder.getPositionInches() <= POSITION_TOLERANCE_IN)
    motor.stop();
}

// Stop when near top
inline void stopAtTop(MotorDriver& motor, EncoderReader& encoder) {
  if (encoder.getPositionInches() >= MAX_TRAVEL_IN - POSITION_TOLERANCE_IN)
    motor.stop();
}

} // namespace MotionProfiles