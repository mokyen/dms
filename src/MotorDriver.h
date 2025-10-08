#pragma once
#include <Arduino.h>
#include "Config.h"

class MotorDriver {
public:
  MotorDriver(uint8_t pwm, uint8_t ina, uint8_t inb, uint8_t cs)
    : pwmPin(pwm), inAPin(ina), inBPin(inb), csPin(cs) {}

  void begin() {
    pinMode(pwmPin, OUTPUT);
    pinMode(inAPin, OUTPUT);
    pinMode(inBPin, OUTPUT);
    pinMode(csPin, INPUT);
    stop();
  }

  void setSpeed(float command) {
    command = constrain(command, -1.0f, 1.0f);
    float magnitude = fabs(command);
    magnitude = MIN_DUTY_CYCLE + magnitude * (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE);

    const uint8_t pwmValue = static_cast<uint8_t>(255.0f * magnitude);
    if (command > 0) {
      digitalWrite(inAPin, HIGH);
      digitalWrite(inBPin, LOW);
    } else if (command < 0) {
      digitalWrite(inAPin, LOW);
      digitalWrite(inBPin, HIGH);
    } else {
      stop();
      return;
    }
    analogWrite(pwmPin, pwmValue);
  }

  void stop() {
    digitalWrite(inAPin, LOW);
    digitalWrite(inBPin, LOW);
    analogWrite(pwmPin, 0);
  }

  void brake() {
    digitalWrite(inAPin, HIGH);
    digitalWrite(inBPin, HIGH);
    analogWrite(pwmPin, 0);
  }

  float readCurrent() const {
    int adcValue = analogRead(csPin);
    return adcValue * ADC_TO_CURRENT;
  }

private:
  uint8_t pwmPin, inAPin, inBPin, csPin;
};