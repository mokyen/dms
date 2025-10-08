#include "MotorControl.h"
#include "EncoderReader.h"
#include "Config.h"

MotorControl::MotorControl(MotorDriver& driver, EncoderReader& encoder)
  : motor(driver), this->encoder(encoder), position(MotorPosition::Unknown), target(MotorPosition::Unknown),
    lastUpdateMs(0) {}

void MotorControl::begin() {
  motor.begin();
  position = MotorPosition::Unknown;
  target = MotorPosition::Unknown;
}

void MotorControl::moveToPosition(MotorPosition targetPos) {
  target = targetPos;
  lastUpdateMs = millis();

  if (target == MotorPosition::Top) {
    motor.setSpeed(1.0f); // full speed up
    position = MotorPosition::Moving;
  } else if (target == MotorPosition::Bottom) {
    motor.setSpeed(-1.0f); // full speed down
    position = MotorPosition::Moving;
  }
}


  void MotorControl::moveToPositionPercent(float targetPercent) {
    // Clamp targetPercent to [0.0, 100.0]
    if (targetPercent < 0.0f) {targetPercent = 0.0f;}
    if (targetPercent > 100.0f) {targetPercent = 100.0f;}

    constexpr float TARGET_PERCENT_TO_TARGET_INCHES = MAX_TRAVEL_IN / 100.0f;
    float targetInches = targetPercent * TARGET_PERCENT_TO_TARGET_INCHES;
    float currentInches = encoder.getPositionInches();

    float error = targetInches - currentInches;

    if (fabs(error) < 0.5f) { // within half an inch
      motor.stop();
      position = MotorPosition::Unknown;
      target = MotorPosition::Unknown;
      return;
    }

    if (error > 0) {
      motor.setSpeed(1.0f); // Move up
      position = MotorPosition::Moving;
      target = MotorPosition::Top; // Set target to Top for logic purposes
    } else {
      motor.setSpeed(-1.0f); // Move down
      position = MotorPosition::Moving;
      target = MotorPosition::Bottom; // Set target to Bottom for logic purposes
    }

    lastUpdateMs = millis();
  }

void MotorControl::stopAtTop() {
  motor.stop();
  position = MotorPosition::Top;
  target = MotorPosition::Unknown;
}

void MotorControl::stopAtBottom() {
  motor.stop();
  position = MotorPosition::Bottom;
  target = MotorPosition::Unknown;
}

void MotorControl::update() {
  if (position != MotorPosition::Moving)
    return;

  unsigned long now = millis();

  // Example placeholder logic:
  // In the future, you could add limit switch or current-sense logic here
  // For now, just time out after MOVE_TIMEOUT_MS for safety.
  if (now - lastUpdateMs > MOVE_TIMEOUT_MS) {
    motor.stop();
    position = MotorPosition::Unknown;
    target = MotorPosition::Unknown;
  }
}
