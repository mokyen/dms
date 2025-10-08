#pragma once
#include <Arduino.h>

// -------------------- Hardware pins --------------------
constexpr uint8_t PIN_PWM        = 5;
constexpr uint8_t PIN_INA        = 6;
constexpr uint8_t PIN_INB        = 7;
constexpr uint8_t PIN_CS         = A0;
constexpr uint8_t PIN_ENCODER_A  = 4;
constexpr uint8_t PIN_ENCODER_B  = 3;

// -------------------- Motor parameters --------------------
constexpr float SUPPLY_VOLTAGE_V  = 12.0f;
constexpr float MIN_DUTY_CYCLE    = 0.45f;
constexpr float MAX_DUTY_CYCLE    = 0.80f;
constexpr float MAX_CURRENT_A     = 2.5f;  // conservative default

// Encoder + mechanics
constexpr float ENCODER_CPR       = 64.0f;
constexpr float GEAR_RATIO        = 30.0f;
constexpr float COUNTS_PER_REV    = ENCODER_CPR * GEAR_RATIO;
constexpr float TRAVEL_PER_REV_IN = 55.0f;
constexpr float COUNTS_PER_IN     = COUNTS_PER_REV / TRAVEL_PER_REV_IN;
constexpr float DISTANCE_TO_SYSTEM_FT = 15.0f; // vertical distance from motor to DMS system
constexpr float BUFFER_DISTANCE_IN = 12.0f; // extra travel to avoid hard stops
constexpr float MAX_TRAVEL_IN     = DISTANCE_TO_SYSTEM_FT * 12.0f - BUFFER_DISTANCE_IN;    // ~4 revs × 55 in
constexpr float MAX_POSITION_CNT  = MAX_TRAVEL_IN * COUNTS_PER_IN;
constexpr float INV_COUNTS_PER_IN = 1.0f / COUNTS_PER_IN;

// Loop timing
constexpr unsigned long LOOP_INTERVAL_MS = 50;  // ~20 Hz control update
