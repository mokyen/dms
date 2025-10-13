#include "MotorControl.h"
#include "MotionProfiles.h"
#include "Config.h"

MotorControl::MotorControl(MotorDriver& driver, EncoderReader& enc)
  : motor(driver), encoder(enc), position(MotorPosition::Unknown),
    target(MotorPosition::Unknown), targetCounts(0), lastUpdateMs(0) {}

void MotorControl::begin() {
  motor.begin();
  encoder.begin();
  EncoderReader::attachInstance(&encoder);
  position = MotorPosition::Unknown;
}

// --------------------------------------------------------------------------
// PRIVATE HELPER: Central function to initiate any move
// --------------------------------------------------------------------------

static long s_previousTargetCounts = 0; 
void MotorControl::setTargetAndStartMove(long counts, MotorPosition state) {
  
  // Clamp counts to safe travel limits
  if (counts < 0L) counts = 0L;
  if (counts > MAX_TRAVEL_COUNTS) counts = MAX_TRAVEL_COUNTS;
  
  targetCounts = counts;
  target = state; // Store the state (Top, Bottom, or Unknown) for later status reporting
  lastUpdateMs = millis();
  
  // Debug print logic
  auto delta = labs(targetCounts - s_previousTargetCounts); 
  if (delta > POSITION_TOLERANCE_COUNTS) {
    Serial.print(F("Move target set to: "));
    Serial.print(targetCounts);
    Serial.println(F(" counts"));
    s_previousTargetCounts = targetCounts;
  }

  // Set initial speed/direction. The continuous 'update' handles the rest.
  MotionProfiles::moveToPositionCounts(motor, encoder, targetCounts, 1.0f); 
  position = MotorPosition::Moving;
}

// --------------------------------------------------------------------------
// PUBLIC API: Simplified to only accept counts
// --------------------------------------------------------------------------

void MotorControl::moveToPositionCounts(long counts) {
  // Determine the target state for internal tracking/arrival logic
  MotorPosition targetState = MotorPosition::Unknown;
  
  // Check if the target is functionally Top or Bottom for state tracking
  if (labs(counts - MAX_TRAVEL_COUNTS) <= POSITION_TOLERANCE_COUNTS) {
      targetState = MotorPosition::Top;
  } else if (labs(counts) <= POSITION_TOLERANCE_COUNTS) {
      targetState = MotorPosition::Bottom;
  } 

  // Delegate to the single initiation function
  setTargetAndStartMove(counts, targetState);
}


void MotorControl::stopAtTop()    { MotionProfiles::stopAtTop(motor, encoder); }
void MotorControl::stopAtBottom() { MotionProfiles::stopAtBottom(motor, encoder); }

// --------------------------------------------------------------------------
// UPDATE LOOP: Stays focused on counts
// --------------------------------------------------------------------------

void MotorControl::update() {
  if (position != MotorPosition::Moving) return;

  unsigned long now = millis();
  if (now - lastUpdateMs > MOVE_TIMEOUT_MS) {
    motor.stop();
    position = MotorPosition::Unknown;
    target = MotorPosition::Unknown;
    Serial.println(F("ERROR: Move timed out. Stopping."));
    return;
  }
  
  // P-Controller Loop: Continuously drive the motor toward the targetCounts
  MotionProfiles::moveToPositionCounts(motor, encoder, targetCounts, 1.0f);

  // Check if the motor has successfully reached the target
  long currentError = labs(targetCounts - encoder.getPositionCounts());
  if (currentError <= POSITION_TOLERANCE_COUNTS) {
      motor.brake();
      
      // Update the position state based on the tracked 'target'
      if (target == MotorPosition::Top)
          position = MotorPosition::Top;
      else if (target == MotorPosition::Bottom)
          position = MotorPosition::Bottom;
      else 
          // Covers any other count-based movements
          position = MotorPosition::Unknown; 
          
      target = MotorPosition::Unknown; // Clear target after arrival
  }
}