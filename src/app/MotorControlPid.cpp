#include "MotorControlPid.h"
#include <Arduino.h>

MotorControlPid::MotorControlPid(MotorDriver& driver, EncoderReader& encoder)
  : motor_(driver), encoder_(encoder), targetCounts_(0), maxSpeedFraction_(1.0f),
    moving_(false), arrived_(false), moveStartMs_(0),
    pidInput_(0), pidOutput_(0), pidSetpoint_(0), pid_(nullptr), activeProfile_(PidProfile::Gentle)
{
  pid_ = new PID(&pidInput_, &pidOutput_, &pidSetpoint_, 0.0004, 0.0005, 0.00008, DIRECT);
  pid_->SetMode(AUTOMATIC);
  pid_->SetOutputLimits(-1.0, 1.0);
  pid_->SetSampleTime(10);
}

void MotorControlPid::begin() {
  motor_.begin();
  encoder_.begin();
  EncoderReader::attachInstance(&encoder_);
}

void MotorControlPid::moveToCounts(long counts, int maxSpeedPercent) {
  targetCounts_ = constrain(counts, 0L, MAX_TRAVEL_COUNTS);
  maxSpeedPercent = constrain(maxSpeedPercent, 1, 100);
  maxSpeedFraction_ = maxSpeedPercent / 100.0f;
  pidSetpoint_ = (double)targetCounts_;
  pid_->SetOutputLimits(-1.0, 1.0); // TODO: Use maxSpeedFraction_
  moving_ = true;
  arrived_ = false;
  moveStartMs_ = millis();
}

void MotorControlPid::stop() {
  motor_.stop();
  moving_ = false;
}

void MotorControlPid::emergencyStop() {
  motor_.brake();
  moving_ = false;
  arrived_ = false;
}

void MotorControlPid::update() {
  // TODO: Implement PID update logic
  // For now, just stub
}