#include "src/Config.h"
#include "src/MotorDriver.h"
#include "src/MotorControl.h"
#include "src/EncoderReader.h"
#include "src/MotionProfile.h"

MotorDriver driver(PIN_PWM, PIN_INA, PIN_INB, PIN_CS);
EncoderReader encoder(PIN_ENCODER_A, PIN_ENCODER_B);
MotorControl control(driver, encoder);

MotionProfile profile;

void motion_slowUpDown() {
    control.moveToPosition(168);  // up slowly
    delay(3000);
    control.moveToPosition(0);    // down slowly
}

void motion_fastUpSlowDown() {
    control.moveToPosition(168);
    delay(2000);
    control.moveToPosition(0);
}

void motion_stepPattern() {
    control.moveToPosition(50);
    delay(1000);
    control.moveToPosition(25);
    delay(1000);
    control.moveToPosition(100);
}

void setup() {
  Serial.begin(115200);
  encoder.begin();
  driver.begin();

  Serial.println("Spider controller started.");
}

void loop() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

  if (now - lastUpdate >= LOOP_INTERVAL_MS) {
    lastUpdate = now;

    float positionInches = encoder.getPositionInches();
    float velocityCmd = profile.computeVelocity(positionInches, encoder.getDirection());
    driver.setSpeed(velocityCmd);

    // Optional current limit check
    float current = driver.readCurrent();
    if (current > MAX_CURRENT_A) {
      Serial.println("⚠️ Current limit exceeded — stopping driver.");
      driver.brake();
    }
  }
}
