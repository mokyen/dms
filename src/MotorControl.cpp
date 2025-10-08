#include "MotorControl.h"
#include "MotionProfiles.h"
#include "Config.h"

MotorControl::MotorControl(MotorDriver& driver, EncoderReader& enc)
  : motor(driver), encoder(enc), position(MotorPosition::Unknown),
    target(MotorPosition::Unknown), lastUpdateMs(0) {}

void MotorControl::begin() {
  motor.begin();
  encoder.begin();
  EncoderReader::attachInstance(&encoder);
  position = MotorPosition::Unknown;
}

void MotorControl::moveToPosition(MotorPosition targetPos) {
  target = targetPos;
  lastUpdateMs = millis();

  float targetInches = (target == MotorPosition::Top) ? MAX_TRAVEL_IN : 0.0f;
  MotionProfiles::moveToPosition(motor, encoder, targetInches, 1.0f);
  position = MotorPosition::Moving;
}

void MotorControl::moveToPositionPercent(float targetPercent) {
  targetPercent = constrain(targetPercent, 0.0f, 100.0f);
  float targetInches = (targetPercent / 100.0f) * MAX_TRAVEL_IN;
  MotionProfiles::moveToPosition(motor, encoder, targetInches, 1.0f);
  position = MotorPosition::Moving;
  lastUpdateMs = millis();
}

void MotorControl::stopAtTop()    { MotionProfiles::stopAtTop(motor, encoder); }
void MotorControl::stopAtBottom() { MotionProfiles::stopAtBottom(motor, encoder); }

void MotorControl::update() {
  if (position != MotorPosition::Moving) return;

  unsigned long now = millis();
  if (now - lastUpdateMs > MOVE_TIMEOUT_MS) {
    motor.stop();
    position = MotorPosition::Unknown;
    target = MotorPosition::Unknown;
  }

  if (target == MotorPosition::Top)
    stopAtTop();
  else if (target == MotorPosition::Bottom)
    stopAtBottom();
}