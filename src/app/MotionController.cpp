#include "MotionController.h"
#include "MotorSequencer.h"
#include "../app/SpiderPatterns.h"

MotionController::MotionController(MotorControlPid& pid, EncoderReader& encoder, MotorDriver& motor)
  : pid_(pid), encoder_(encoder), motor_(motor) {}

// Single place to map command letters to actions
void MotionController::handleCommand(char cmd, float value) {
  switch (cmd) {
    // Motion commands
    case 'u': case 'U': moveToTop(); break;
    case 'd': case 'D': moveToBottom(); break;
    case 'p': case 'P': moveToPercent(value); break;
    case 's': case 'S': stopMotor(); break;
    case 'z': case 'Z': zeroEncoder(); break;

    // Diagnostics
    case 'r': case 'R': printPosition(); break;
    case 'j': case 'J': jogMode(); break;
    case 'c': case 'C': continuousMonitor(); break;
    case 'm': case 'M': rampUpSpeed(); break;
    case 'l': case 'L': currentLimitTest(); break;
    case 'f': case 'F': manualDutyMode(); break;

    // System ID
    case '1': runPattern(1); break;
    case '2': runPattern(2); break;
    case '3': runPattern(3); break;
    case '4': runPattern(4); break;
    case '5': runPattern(5); break;
    case 'T': runRandomPattern(); break;
    case 'A': hauntingMode(); break;

    // Tuning / configuration:
    // 'F' with numeric arg sets feed-forward (handled here as single float for convenience)
    // But 'f' (lowercase) is taken for manual duty mode above
    case 'Q': {
      // value is interpreted as integer code: 1,2,3
      setPidProfile(static_cast<int>(value));
      break;
    }

    case 'G': {
      // Feed-forward setter (G for Gain). Choose 'G' to avoid confusion with 'f' manual duty.
      // value is the feed-forward decimal (0.0 - 0.5)
      setFeedForward(value);
      break;
    }

    default:
      Serial.print(F("Unknown command: "));
      Serial.println(cmd);
      Serial.println(F("Type 'h' for help."));
      break;
  }
}

void MotionController::update() {
  // Always update PID controller
  pid_.update();
  // If you have continuous RLS or background tasks, call them (no-op unless implemented)
  pid_.update(); // Safe: pid_.update() is idempotent when not moving; kept explicit
}

void MotionController::moveToTop() {
  pid_.moveToCounts(MAX_TRAVEL_COUNTS, 65);
}
void MotionController::moveToBottom() {
  pid_.moveToCounts(0L, 45);
}
void MotionController::moveToPercent(float percent) {
  pid_.moveToCounts((long)((constrain(percent, 0.0f, 100.0f) / 100.0f) * MAX_TRAVEL_COUNTS), 100);
}
void MotionController::stopMotor() {
  pid_.stop();
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

// Diagnostics: delegate to the free functions or inline logic
void MotionController::rampUpSpeed() {
  // reuse your old helper which manipulated motor pins directly
  // For brevity, call into the dms.ino helper if you keep it; otherwise reimplement here.
  Serial.println(F("Ramp up speed: delegated to manual test (not reimplemented here)."));
}
void MotionController::jogMode() {
  Serial.println(F("JOG MODE: delegated to manual test (not reimplemented here)."));
}
void MotionController::continuousMonitor() {
  Serial.println(F("Continuous monitor: delegated."));
}
void MotionController::currentLimitTest() {
  Serial.println(F("Current limit test: delegated."));
}
void MotionController::manualDutyMode() {
  Serial.println(F("Manual duty mode: delegated."));
}

// System ID placeholders; actual implementations can be migrated from dms.ino if desired
void MotionController::resistanceMeasurement() { Serial.println(F("Resistance test (delegated).")); }
void MotionController::stepResponse() { Serial.println(F("Step response (delegated).")); }
void MotionController::backEMFTest() { Serial.println(F("Back EMF test (delegated).")); }
void MotionController::rlsElectricalTest() { Serial.println(F("RLS electrical (delegated).")); }
void MotionController::continuousRLSTest() { Serial.println(F("Continuous RLS (delegated).")); }

// Patterns: delegate to SpiderPatterns helpers
void MotionController::runPattern(int id) {
  switch (id) {
    case 1: SpiderPatterns::pattern_stalker(pid_, encoder_); break;
    case 2: SpiderPatterns::pattern_pounce(pid_, encoder_); break;
    case 3: SpiderPatterns::pattern_patrol(pid_, encoder_); break;
    case 4: SpiderPatterns::pattern_twitch(pid_, encoder_); break;
    case 5: SpiderPatterns::pattern_lurker(pid_, encoder_); break;
  }
}
void MotionController::hauntingMode() {
  SpiderPatterns::hauntingMode(pid_, encoder_);
}
void MotionController::runRandomPattern() {
  SpiderPatterns::runRandomPattern(pid_, encoder_);
}

// Configuration
void MotionController::setFeedForward(float ff) {
  // Apply same up/down (symmetric) for quick changes
  pid_.setFeedForward(ff, ff);
  Serial.print(F("Feed-forward set to "));
  Serial.println(ff, 3);
}
void MotionController::setFeedForward(float up, float down) {
  pid_.setFeedForward(up, down);
  Serial.print(F("Feed-forward up/down set to "));
  Serial.print(up, 3);
  Serial.print(F(" / "));
  Serial.println(down, 3);
}

void MotionController::setPidProfile(int idx) {
  if (idx == 2) pid_.setPidProfile(PidProfile::Balanced);
  else if (idx == 3) pid_.setPidProfile(PidProfile::Responsive);
  else pid_.setPidProfile(PidProfile::Gentle);
}
