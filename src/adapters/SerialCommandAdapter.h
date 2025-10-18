#pragma once

class MotionController; // Forward declaration

class SerialCommandAdapter {
public:
  SerialCommandAdapter(MotionController& controller);
  void processSerial(); // To be called in loop()
private:
  MotionController& controller_;
  void handleInput(char cmd, float value);
};
