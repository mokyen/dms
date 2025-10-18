#include "adapters/MotorDriver.h"
#include "adapters/EncoderReader.h"
#include "Config.h"
#include "app/MotorControlPid.h"
#include "app/MotionController.h"
#include "adapters/SerialCommandAdapter.h"
#include "SpiderPatterns.h"
#include "MotorSequencer.h"
#include <Arduino.h>

MotionController::MotionController(MotorControlPid& pid, EncoderReader& encoder, MotorDriver& motor)
  : pid_(pid), encoder_(encoder), motor_(motor) {}

void MotionController::handleCommand(char cmd, float value) {
  switch (cmd) {
    case 'u': moveToTop(); break;
    case 'd': moveToBottom(); break;
    case 'p': moveToPercent(value); break;
    case 's': stopMotor(); break;
    case 'r': printPosition(); break;
    case 'z': zeroEncoder(); break;
    case 'm': rampUpSpeed(); break;
    case 'j': jogMode(); break;
    case 'c': continuousMonitor(); break;
    case 'l': currentLimitTest(); break;
    case 'f': manualDutyMode(); break;
    case 'R': resistanceMeasurement(); break;
    case 'S': stepResponse(); break;
    case 'K': backEMFTest(); break;
    case 'E': rlsElectricalTest(); break;
    case 'X': continuousRLSTest(); break;
    case 'h': printHelp(); break;
    case '?': printStatus(); break;
    case '1': SpiderPatterns::pattern_stalker(pid_, encoder_); break;
    case '2': SpiderPatterns::pattern_pounce(pid_, encoder_); break;
    case '3': SpiderPatterns::pattern_patrol(pid_, encoder_); break;
    case '4': SpiderPatterns::pattern_twitch(pid_, encoder_); break;
    case '5': SpiderPatterns::pattern_lurker(pid_, encoder_); break;
    case 'A': SpiderPatterns::hauntingMode(pid_, encoder_); break;
    case 'T': SpiderPatterns::runRandomPattern(pid_, encoder_); break;
    default:
      Serial.print(F("Unknown command: "));
      Serial.println(cmd);
      Serial.println(F("Type 'h' for help."));
      break;
  }
}

void MotionController::update() {
  pid_.update();
  updateContinuousRLS();
}

void MotionController::moveToTop() {
  Serial.println(F("Moving to TOP (45% speed)..."));
  pid_.moveToCounts(MAX_TRAVEL_COUNTS, 65);
}

void MotionController::moveToBottom() {
  Serial.println(F("Moving to BOTTOM (45% speed)..."));
  pid_.moveToCounts(0L, 45);
}

void MotionController::moveToPercent(float percent) {
  percent = constrain(percent, 0.0f, 100.0f);
  long targetCounts = (long)((percent / 100.0f) * MAX_TRAVEL_COUNTS);
  Serial.print(F("Moving to "));
  Serial.print(percent, 1);
  Serial.println(F("%"));
  pid_.moveToCounts(targetCounts, 100);
}

void MotionController::stopMotor() {
  Serial.println(F("Stopping motor."));
  motor_.stop();
}

void MotionController::printPosition() {
  const float pos = encoder_.getPositionInches();
  const long counts = encoder_.getPositionCounts();
  const float current = motor_.readCurrent();
  Serial.print(F("Position: "));
  Serial.print(pos, 2);
  Serial.print(F(" in ("));
  Serial.print(counts);
  Serial.print(F(" counts) | Current: "));
  Serial.print(current, 3);
  Serial.println(F(" A"));
}

void MotionController::zeroEncoder() {
  encoder_.zero();
  Serial.println(F("Encoder zeroed. Position set to 0."));
}

void MotionController::rampUpSpeed() {}
void MotionController::jogMode() {}
void MotionController::continuousMonitor() {}
void MotionController::currentLimitTest() {}
void MotionController::manualDutyMode() {}
void MotionController::resistanceMeasurement() {}
void MotionController::stepResponse() {}
void MotionController::backEMFTest() {}
void MotionController::rlsElectricalTest() {}
void MotionController::continuousRLSTest() {}
void MotionController::printHelp() {}
void MotionController::printStatus() {}
void MotionController::updateContinuousRLS() {}

// ... Implement all other test_* and pattern methods by moving logic from dms.ino ...
// For brevity, only a subset is shown here. The rest should be migrated similarly.
