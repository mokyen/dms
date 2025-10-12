#pragma once
#include <Arduino.h>

// ==============================================================================
// BOARD DETECTION AND CONFIGURATION
// ==============================================================================

// Detect which board we're compiling for
#if defined(NUCLEO_F411RE) || defined(STM32F411xE)
  #define BOARD_STM32_NUCLEO
  #define BOARD_NAME "STM32 NUCLEO-F411RE"
#elif defined(ARDUINO_AVR_UNO) || defined(__AVR_ATmega328P__)
  #define BOARD_ARDUINO_UNO
  #define BOARD_NAME "Arduino Uno"
#else
  #warning "Unknown board - defaulting to Arduino Uno configuration"
  #define BOARD_ARDUINO_UNO
  #define BOARD_NAME "Unknown (Uno defaults)"
#endif

// ==============================================================================
// BOARD-SPECIFIC ADC CONFIGURATION
// ==============================================================================

#ifdef BOARD_STM32_NUCLEO
  // STM32F411RE: 12-bit ADC, 3.3V reference, 100MHz CPU
  constexpr float ADC_RESOLUTION_BITS = 12;
  constexpr float ADC_MAX_VALUE = 4095.0f;
  constexpr float ADC_REFERENCE_VOLTAGE = 3.3f;
  constexpr uint32_t CPU_FREQUENCY_HZ = 100000000UL;
  
  // Set ADC resolution at runtime for STM32
  inline void initADC() {
    analogReadResolution(12);
  }
  
#else // BOARD_ARDUINO_UNO
  // Arduino Uno: 10-bit ADC, 5V reference, 16MHz CPU
  constexpr float ADC_RESOLUTION_BITS = 10;
  constexpr float ADC_MAX_VALUE = 1023.0f;
  constexpr float ADC_REFERENCE_VOLTAGE = 5.0f;
  constexpr uint32_t CPU_FREQUENCY_HZ = 16000000UL;
  
  // No ADC init needed for Uno
  inline void initADC() {
    // Nothing to do on Arduino Uno
  }
#endif

// Precomputed ADC constants
constexpr float ADC_VOLTS_PER_COUNT = ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE;
constexpr float ADC_COUNTS_PER_VOLT = ADC_MAX_VALUE / ADC_REFERENCE_VOLTAGE;

// ==============================================================================
// HARDWARE PIN ASSIGNMENTS
// ==============================================================================

// These Arduino pin numbers work on both boards thanks to Arduino pin mapping
constexpr uint8_t MOTOR_PWM_PIN = 5;
constexpr uint8_t MOTOR_INB_PIN = 6;
constexpr uint8_t MOTOR_INA_PIN = 7;
constexpr uint8_t MOTOR_CS_PIN  = A0;

constexpr uint8_t ENCODER_A_PIN = 3;
constexpr uint8_t ENCODER_B_PIN = 4;

// ==============================================================================
// SYSTEM PHYSICAL CONSTANTS
// ==============================================================================

constexpr float DISTANCE_TO_SYSTEM_FT = 18.0f;
constexpr float DISTANCE_TO_SYSTEM_IN = DISTANCE_TO_SYSTEM_FT * 12.0f;
constexpr float PULLEY_DIAMETER_IN = 16.5f;  // Diameter of pulley attached to motor shaft
constexpr float PULLEY_CIRCUMFERENCE_IN = PULLEY_DIAMETER_IN * PI;

constexpr float MAX_ROTATIONS = DISTANCE_TO_SYSTEM_IN / PULLEY_CIRCUMFERENCE_IN; // ~13.75 revolutions

constexpr float SAFETY_MARGIN_IN = 6.0f;  // Extra length to avoid bottoming out
constexpr float MAX_TRAVEL_IN = DISTANCE_TO_SYSTEM_IN - SAFETY_MARGIN_IN; // ~210 inches

// 64 CPR at motor shaft, 30:1 gearbox, 4x decoding (all edges)
constexpr float ENCODER_CPR_MOTOR = 64.0f;
constexpr float GEAR_RATIO = 30.0f;
constexpr float COUNTS_PER_REV = ENCODER_CPR_MOTOR * GEAR_RATIO; // 1920
constexpr float TRAVEL_PER_REV_IN = PULLEY_CIRCUMFERENCE_IN;
constexpr float COUNTS_PER_IN = COUNTS_PER_REV / TRAVEL_PER_REV_IN;
constexpr float INV_COUNTS_PER_IN = 1.0f / COUNTS_PER_IN;

// ==============================================================================
// MOTION CONTROL TUNING
// ==============================================================================

constexpr float MIN_DUTY_CYCLE = 0.2f;
constexpr float MAX_DUTY_CYCLE = 0.95f;

constexpr float POSITION_TOLERANCE_IN = 0.25f;   // stop within 1/4"
constexpr unsigned long MOVE_TIMEOUT_MS = 5000;  // safety stop

// ==============================================================================
// MOTOR DRIVER CURRENT SENSING (VNH5019)
// ==============================================================================

// Pololu VNH5019 current sense characteristics
constexpr float CURRENT_SENSE_V_PER_A = 0.14f;  // 140 mV/A from datasheet
constexpr float CURRENT_SENSE_A_PER_V = 1.0f / CURRENT_SENSE_V_PER_A;

// Board-specific current sensing calibration
#ifdef BOARD_STM32_NUCLEO
  // STM32 may need different calibration due to 3.3V reference
  // and different ADC characteristics
  constexpr float CURRENT_SENSE_CALIBRATION = 1.0f;  // Adjust after testing
#else
  // Arduino Uno baseline calibration
  constexpr float CURRENT_SENSE_CALIBRATION = 1.0f;
#endif

// Combined current sensing constant
constexpr float ADC_TO_CURRENT = ADC_VOLTS_PER_COUNT * CURRENT_SENSE_A_PER_V * CURRENT_SENSE_CALIBRATION;

// ==============================================================================
// SERIAL COMMUNICATION
// ==============================================================================

#ifdef BOARD_STM32_NUCLEO
  constexpr uint32_t SERIAL_BAUD_RATE = 115200;  // STM32 can handle higher baud rates
#else
  constexpr uint32_t SERIAL_BAUD_RATE = 9600;    // Conservative for Arduino Uno
#endif

// ==============================================================================
// PWM CONFIGURATION
// ==============================================================================

#ifdef BOARD_STM32_NUCLEO
  // STM32 can do much higher PWM frequencies for smoother motor control
  constexpr uint32_t PWM_FREQUENCY_HZ = 20000;  // 20kHz - above audible range
  
  // Note: Actual PWM frequency configuration depends on timer setup
  // The Arduino framework default is ~1kHz, but you can adjust it
  inline void initPWM() {
    // TODO: For precise PWM control on STM32, you may need to use
    // HAL timer functions. The Arduino framework uses default ~1kHz.
    // For now, we rely on Arduino's analogWrite()
  }
  
#else // BOARD_ARDUINO_UNO
  // Arduino Uno PWM frequencies (hardware dependent)
  // Pins 5, 6: ~980Hz (Timer 0)
  // Pins 9, 10: ~490Hz (Timer 1)
  // Pins 3, 11: ~490Hz (Timer 2)
  constexpr uint32_t PWM_FREQUENCY_HZ = 980;  // Pin 5 default
  
  inline void initPWM() {
    // Optional: Adjust Timer 0 for higher frequency on pins 5 & 6
    // TCCR0B = (TCCR0B & 0b11111000) | 0x01; // Set prescaler to 1 for ~62kHz
    // Warning: This affects delay() and millis()!
  }
#endif

// ==============================================================================
// PERFORMANCE/OPTIMIZATION FLAGS
// ==============================================================================

#ifdef BOARD_STM32_NUCLEO
  // STM32 has plenty of processing power
  #define ENABLE_ADVANCED_CONTROL    // Enable more sophisticated algorithms
  #define ENABLE_FLOAT_MATH          // Use floating point freely
  constexpr bool USE_LOOKUP_TABLES = false;  // No need, CPU is fast enough
  
#else // BOARD_ARDUINO_UNO
  // Arduino Uno needs optimization
  // #define ENABLE_ADVANCED_CONTROL // Uncomment if you have CPU headroom
  #define AVOID_FLOAT_DIVISION       // Use multiplication by inverse instead
  constexpr bool USE_LOOKUP_TABLES = true;   // Consider lookup tables for trig
#endif

// ==============================================================================
// DEBUG AND DIAGNOSTICS
// ==============================================================================

// Enable verbose debugging on faster board
#ifdef BOARD_STM32_NUCLEO
  #define DEBUG_VERBOSE
  constexpr bool ENABLE_PERFORMANCE_MONITORING = true;
#else
  // Keep debug light on Arduino Uno
  constexpr bool ENABLE_PERFORMANCE_MONITORING = false;
#endif

#ifdef DEBUG_VERBOSE
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// ==============================================================================
// SAFETY LIMITS
// ==============================================================================

constexpr float MAX_CURRENT_AMPS = 5.0f;  // Maximum safe current
constexpr float WARNING_CURRENT_AMPS = 4.0f;  // Warning threshold

// ==============================================================================
// INITIALIZATION HELPER
// ==============================================================================

inline void initBoardSpecificHardware() {
  initADC();
  initPWM();
  
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial && millis() < 3000) {
    ; // Wait for serial on boards that need it (like STM32), timeout after 3s
  }
  
  DEBUG_PRINT(F("Board: "));
  DEBUG_PRINTLN(F(BOARD_NAME));
  DEBUG_PRINT(F("ADC: "));
  DEBUG_PRINT(ADC_RESOLUTION_BITS);
  DEBUG_PRINT(F("-bit, "));
  DEBUG_PRINT(ADC_REFERENCE_VOLTAGE);
  DEBUG_PRINTLN(F("V ref"));
  DEBUG_PRINT(F("CPU: "));
  DEBUG_PRINT(CPU_FREQUENCY_HZ / 1000000UL);
  DEBUG_PRINTLN(F(" MHz"));
  DEBUG_PRINT(F("Serial: "));
  DEBUG_PRINT(SERIAL_BAUD_RATE);
  DEBUG_PRINTLN(F(" baud"));
}

// ==============================================================================
// MOTION PROFILE CONSTANTS (BOARD-INDEPENDENT)
// ==============================================================================

constexpr float INV_MAX_TRAVEL = 1.0f / MAX_TRAVEL_IN;
constexpr float POSITION_TO_PHASE = PI * INV_MAX_TRAVEL;