#include "SerialCommandAdapter.h"
#include "../app/MotionController.h"
#include <Arduino.h>

SerialCommandAdapter::SerialCommandAdapter(MotionController& controller)
  : controller_(controller) {}

void SerialCommandAdapter::processSerial() {
  if (Serial.available()) {
    char cmd = Serial.read();
    float value = 0;
    if (Serial.available() >= 4) {
      Serial.readBytes((char*)&value, sizeof(float));
    }
    handleInput(cmd, value);
  }
}

void SerialCommandAdapter::handleInput(char cmd, float value) {
  controller_.handleCommand(cmd, value);
}
