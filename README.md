# dms
My decoration motor system.

A modular Arduino project to control a Halloween spider lift using a Pololu 1451 motor driver, encoder feedback, and motion profiles.

## Features
- Modular, constexpr-based configuration
- Automatic up/down motion with direction reversal
- Optional current limiting
- Clean interface for later PWM tuning or advanced control

## Hardware
- Arduino Uno or Mega
- Pololu 1451 (VNH5019) motor driver
- 12 V DC motor (Pololu 37D gearmotor, 30:1, 64 CPR encoder)

## File Layout
```
src/
DMS.ino
MotorDriver.cpp/.h
EncoderReader.cpp/.h
MotionProfile.cpp/.h
Config.h
```

## Operation
On startup, the spider begins at the bottom, moves upward until reaching top position (~4 rotations), reverses, and repeats.