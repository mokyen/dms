#pragma once
#include <Arduino.h>
#include "Config.h"
#include "MotorDriver.h"
#include "EncoderReader.h"

constexpr float INV_MAX_TRAVEL = 1.0f / MAX_TRAVEL_IN;
constexpr float POSITION_TO_PHASE = PI * INV_MAX_TRAVEL;

namespace MotionProfiles {

// How close we consider “at target” before stopping (in inches)
constexpr float POSITION_TOLERANCE_IN = 0.25f;

// A basic linear move to a target position (inches)
inline void moveToPosition(MotorDriver& motor,
                           EncoderReader& encoder,
                           float targetPosIn,
                           float speedFraction)
{
  // Clamp target position to travel range
  if (targetPosIn < 0) targetPosIn = 0;
  if (targetPosIn > MAX_TRAVEL_IN) targetPosIn = MAX_TRAVEL_IN;

  const float currentPos = encoder.getPositionInches();
  const float error = targetPosIn - currentPos;

  if (fabs(error) < POSITION_TOLERANCE_IN) {
    motor.brake();
    return;
  }

  // Decide direction based on sign of error
  const MotorDriver::Direction dir =
      (error > 0) ? MotorDriver::Direction::Up
                  : MotorDriver::Direction::Down;

  motor.setDirection(dir);
  motor.setSpeed(fabs(speedFraction));
  motor.run();
}

// Stop when bottom limit reached
inline void stopAtBottom(MotorDriver& motor,
                         EncoderReader& encoder)
{
  const float pos = encoder.getPositionInches();
  if (pos <= 0.0f + POSITION_TOLERANCE_IN) {
    motor.coast();  // Let it roll to a gentle stop
  }
}

// Stop when top limit reached
inline void stopAtTop(MotorDriver& motor,
                      EncoderReader& encoder)
{
  const float pos = encoder.getPositionInches();
  if (pos >= MAX_TRAVEL_IN - POSITION_TOLERANCE_IN) {
    motor.brake();  // Hard stop to avoid overrun
  }
}

} // namespace MotionProfiles


// Optional class for procedural motion (sinusoidal example)
class MotionProfile {
public:
  MotionProfile() = default;

  float computeVelocity(float positionInches, int lastDirection) {
    if (positionInches >= MAX_TRAVEL_IN) {
      currentDir = -1; // reached top
    } else if (positionInches <= 0) {
      currentDir = 1; // reached bottom
    }

    // Example profile: smooth sinusoidal up/down motion
    float phase = positionInches * POSITION_TO_PHASE;
    float speed = sin(phase) * currentDir;

    return speed; // normalized [-1, 1]
  }

private:
  int currentDir = 1;
};
