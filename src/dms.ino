#include "Config.h"
#include "MotorDriver.h"
#include "EncoderReader.h"
#include "MotorControl.h"
#include "MotionProfiles.h"

// Create hardware instances for runtime initialization
MotorDriver motor(MOTOR_PWM_PIN, MOTOR_INA_PIN, MOTOR_INB_PIN, MOTOR_CS_PIN);
EncoderReader encoder(ENCODER_A_PIN, ENCODER_B_PIN);

// Controller needs to be non-const because it maintains state
MotorControl controller(motor, encoder);

// =======================================================
// ================ Test Function Definitions ============
// =======================================================

void test_moveToTop() {
  Serial.println(F("Moving to TOP (min speed)..."));
  // NEW LOGIC: Convert TOP to MAX_TRAVEL_COUNTS
  controller.moveToPositionCounts(MAX_TRAVEL_COUNTS);
}

void test_moveToBottom() {
  Serial.println(F("Moving to BOTTOM (min speed)..."));
  // NEW LOGIC: Convert BOTTOM to 0L counts
  controller.moveToPositionCounts(0L);
}

void test_moveToPercent(float percent) {
  percent = constrain(percent, 0.0f, 100.0f);
  
  // NEW LOGIC: Conversion from float percent to long counts
  long targetCounts = (long)((percent / 100.0f) * MAX_TRAVEL_COUNTS);
  
  Serial.print(F("Moving to "));
  Serial.print(percent, 1);
  Serial.print(F("% of travel ("));
  Serial.print(targetCounts);
  Serial.println(F(" counts)..."));
  
  controller.moveToPositionCounts(targetCounts);
}

void test_stopMotor() {
  Serial.println(F("Stopping motor."));
  motor.stop();
}

void test_printPosition() {
  const float pos = encoder.getPositionInches();
  const long counts = encoder.getPositionCounts();
  const float current = motor.readCurrent();
  
  Serial.print(F("Position: "));
  Serial.print(pos, 2);
  Serial.print(F(" in ("));
  Serial.print(counts);
  Serial.print(F(" counts) | Current: "));
  Serial.print(current, 3);
  Serial.println(F(" A"));
}

void test_zeroEncoder() {
  encoder.zero();
  Serial.println(F("Encoder zeroed. Position set to 0."));
}

void test_rampUpSpeed() {
  static bool direction = true; // Alternate direction each call
  
  Serial.print(F("Ramping motor speed ("));
  Serial.print(direction ? F("FORWARD") : F("REVERSE"));
  Serial.println(F("): MIN → MAX → MIN"));

  // Set direction
  digitalWrite(MOTOR_INA_PIN, direction ? HIGH : LOW);
  digitalWrite(MOTOR_INB_PIN, direction ? LOW : HIGH);

  // Convert duty cycle percentages to PWM values
  constexpr uint8_t MIN_PWM = static_cast<uint8_t>(MIN_DUTY_CYCLE * 255.0f);
  constexpr uint8_t MAX_PWM = static_cast<uint8_t>(MAX_DUTY_CYCLE * 255.0f);
  constexpr uint8_t STEP = 13; // ~5% of 255

  // Ramp up
  for (uint8_t pwm = MIN_PWM; pwm <= MAX_PWM; pwm += STEP) {
    analogWrite(MOTOR_PWM_PIN, pwm);
    const float duty = (pwm / 255.0f) * 100.0f;
    Serial.print(F("Duty ↑: "));
    Serial.print(duty, 1);
    Serial.println(F("%"));
    delay(300);
  }

  // Ramp down
  for (uint8_t pwm = MAX_PWM; pwm >= MIN_PWM; pwm -= STEP) {
    analogWrite(MOTOR_PWM_PIN, pwm);
    const float duty = (pwm / 255.0f) * 100.0f;
    Serial.print(F("Duty ↓: "));
    Serial.print(duty, 1);
    Serial.println(F("%"));
    delay(300);
    if (pwm < STEP) break; // Prevent underflow
  }

  Serial.println(F("Ramp complete. Stopping motor."));
  motor.stop();
  
  // Alternate direction for next call
  direction = !direction;
}

void test_jogMode() {
  constexpr float JOG_DISTANCE_IN = 0.5f; // Move 0.5 inches per jog
  
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║          JOG MODE ACTIVE           ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  Serial.println(F("+ or w - Jog UP (forward)"));
  Serial.println(F("- or s - Jog DOWN (reverse)"));
  Serial.println(F("0      - Stop motor"));
  Serial.println(F("r      - Read position"));
  Serial.println(F("x      - EXIT jog mode"));
  Serial.println();
  
  // Start with the current position in counts
  long jogTargetCounts = encoder.getPositionCounts();
  // Pre-calculate the jog distance in counts
  const long JOG_DISTANCE_COUNTS = (long)(JOG_DISTANCE_IN * COUNTS_PER_IN);
  
  bool jogActive = true;
  while (jogActive) {
    if (Serial.available()) {
      const char cmd = Serial.read();
      
      switch (cmd) {
        case '+':
        case 'w':
        case 'W': {
          jogTargetCounts += JOG_DISTANCE_COUNTS;
          if (jogTargetCounts > MAX_TRAVEL_COUNTS) jogTargetCounts = MAX_TRAVEL_COUNTS;
          
          Serial.print(F("Jog UP to "));
          Serial.print(jogTargetCounts);
          Serial.println(F(" counts"));
          
          controller.moveToPositionCounts(jogTargetCounts);
          break;
        }
        
        case '-':
        case 's':
        case 'S': {
          jogTargetCounts -= JOG_DISTANCE_COUNTS;
          if (jogTargetCounts < 0L) jogTargetCounts = 0L;
          
          Serial.print(F("Jog DOWN to "));
          Serial.print(jogTargetCounts);
          Serial.println(F(" counts"));
          
          controller.moveToPositionCounts(jogTargetCounts);
          break;
        }
        
        case '0':
          motor.stop();
          Serial.println(F("Motor stopped"));
          break;
          
        case 'r':
        case 'R':
          test_printPosition();
          break;
          
        case 'x':
        case 'X':
          motor.stop();
          Serial.println(F("Exiting jog mode"));
          jogActive = false;
          break;
          
        case '\n':
        case '\r':
          break;
          
        default:
          Serial.println(F("Unknown jog command"));
          break;
      }
      
      // Clear buffer
      while (Serial.available()) {
        Serial.read();
      }
    }
    
    // Update controller during jog mode
    controller.update();
    delay(10);
  }
  
  Serial.println(F("Returned to main menu\n"));
}

void test_continuousMonitor() {
  constexpr unsigned long UPDATE_INTERVAL_MS = 500; // Update every 500ms
  
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║      CONTINUOUS MONITOR MODE       ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  Serial.println(F("Press any key to exit\n"));
  
  // Print header
  Serial.println(F("Position(in) | Counts  | Current(A) | State"));
  Serial.println(F("─────────────┼─────────┼────────────┼───────────"));
  
  unsigned long lastUpdateMs = 0;
  bool monitorActive = true;
  
  while (monitorActive) {
    const unsigned long now = millis();
    
    // Check for exit command
    if (Serial.available()) {
      Serial.read(); // Read and discard character
      // Clear buffer
      while (Serial.available()) {
        Serial.read();
      }
      monitorActive = false;
      break;
    }
    
    // Update display at regular intervals
    if (now - lastUpdateMs >= UPDATE_INTERVAL_MS) {
      lastUpdateMs = now;
      
      // Get current values
      const float pos = encoder.getPositionInches();
      const long counts = encoder.getPositionCounts();
      const float current = motor.readCurrent();
      
      // Print formatted data
      Serial.print(F("  "));
      if (pos < 10.0f) Serial.print(F(" "));
      Serial.print(pos, 2);
      Serial.print(F("      | "));
      
      if (counts < 10000) Serial.print(F(" "));
      if (counts < 1000) Serial.print(F(" "));
      if (counts < 100) Serial.print(F(" "));
      if (counts < 10) Serial.print(F(" "));
      Serial.print(counts);
      Serial.print(F("   | "));
      
      if (current < 1.0f) Serial.print(F(" "));
      Serial.print(current, 3);
      Serial.print(F("     | "));
      
      switch (controller.currentPosition()) {
        case MotorPosition::Top:
          Serial.println(F("TOP      "));
          break;
        case MotorPosition::Bottom:
          Serial.println(F("BOTTOM   "));
          break;
        case MotorPosition::Moving:
          Serial.println(F("MOVING   "));
          break;
        default:
          Serial.println(F("UNKNOWN  "));
          break;
      }
    }
    
    // Update controller
    controller.update();
    delay(10);
  }
  
  Serial.println(F("\nExiting monitor mode\n"));
}

void test_currentLimit() {
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║       CURRENT LIMIT TEST           ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  Serial.println(F("Testing current sensing and limits"));
  Serial.println(F("Press any key to stop\n"));
  
  constexpr float TEST_SPEED = 0.3f; // 30% speed
  constexpr unsigned long SAMPLE_INTERVAL_MS = 50; // 20Hz sampling
  
  float maxCurrent = 0.0f;
  float avgCurrent = 0.0f;
  unsigned long sampleCount = 0;
  unsigned long lastSampleMs = 0;
  
  // Start motor at test speed
  motor.setSpeed(TEST_SPEED);
  Serial.println(F("Motor running at 30% speed..."));
  Serial.println(F("Time(s) | Current(A) | Max(A) | Avg(A)"));
  Serial.println(F("────────┼────────────┼────────┼────────"));
  
  const unsigned long startMs = millis();
  bool testActive = true;
  
  while (testActive) {
    const unsigned long now = millis();
    
    // Check for exit
    if (Serial.available()) {
      Serial.read();
      while (Serial.available()) Serial.read();
      testActive = false;
      break;
    }
    
    // Sample current
    if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
      lastSampleMs = now;
      
      const float current = motor.readCurrent();
      sampleCount++;
      
      // Update statistics
      avgCurrent = avgCurrent + (current - avgCurrent) / sampleCount;
      if (current > maxCurrent) maxCurrent = current;
      
      // Print every 500ms
      if (sampleCount % 10 == 0) {
        const float elapsed = (now - startMs) / 1000.0f;
        Serial.print(elapsed, 2);
        Serial.print(F("    | "));
        Serial.print(current, 3);
        Serial.print(F("      | "));
        Serial.print(maxCurrent, 3);
        Serial.print(F("  | "));
        Serial.println(avgCurrent, 3);
      }
      
      // Safety check
      if (current > MAX_CURRENT_AMPS) {
        Serial.println(F("\n*** CURRENT LIMIT EXCEEDED ***"));
        testActive = false;
      }
    }
    
    delay(10);
  }
  
  motor.stop();
  
  Serial.println(F("\n=== Test Complete ==="));
  Serial.print(F("Samples: "));
  Serial.println(sampleCount);
  Serial.print(F("Max current: "));
  Serial.print(maxCurrent, 3);
  Serial.println(F(" A"));
  Serial.print(F("Avg current: "));
  Serial.print(avgCurrent, 3);
  Serial.println(F(" A"));
  Serial.println();
}

// =======================================================
// ============ System Identification Tests ==============
// =======================================================

void test_resistanceMeasurement() {
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║    RESISTANCE MEASUREMENT TEST     ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  Serial.println(F("Real-time linear regression: V vs I"));
  Serial.println(F("Measuring armature resistance R...\n"));
  
  constexpr float TEST_VOLTAGE_FRACTION = 0.5f; // 50% PWM
  constexpr unsigned long SETTLING_TIME_MS = 500;
  constexpr unsigned long TEST_DURATION_MS = 2000;
  constexpr unsigned long SAMPLE_INTERVAL_MS = 10; // 100Hz
  
  // Online linear regression accumulators
  float sum_i = 0.0f, sum_v = 0.0f;
  float sum_ii = 0.0f, sum_vi = 0.0f;
  int n = 0;
  
  // Apply test voltage
  motor.setSpeed(TEST_VOLTAGE_FRACTION);
  const float testVoltage = TEST_VOLTAGE_FRACTION * ADC_REFERENCE_VOLTAGE;
  
  Serial.print(F("Applied voltage: "));
  Serial.print(testVoltage, 2);
  Serial.println(F(" V"));
  Serial.print(F("Settling for "));
  Serial.print(SETTLING_TIME_MS);
  Serial.println(F(" ms...\n"));
  
  delay(SETTLING_TIME_MS);
  
  Serial.println(F("Sampling... (R estimate updates every 200ms)"));
  const unsigned long startMs = millis();
  unsigned long lastPrintMs = startMs;
  unsigned long lastSampleMs = startMs;
  
  while (millis() - startMs < TEST_DURATION_MS) {
    const unsigned long now = millis();
    
    if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
      lastSampleMs = now;
      
      const float current = motor.readCurrent();
      const float voltage = testVoltage; // Could measure actual PWM voltage
      
      // Accumulate for linear regression
      n++;
      sum_i += current;
      sum_v += voltage;
      sum_ii += current * current;
      sum_vi += voltage * current;
      
      // Print progress every 200ms
      if (now - lastPrintMs >= 200) {
        lastPrintMs = now;
        
        // Compute current R estimate
        const float i_mean = sum_i / n;
        const float v_mean = sum_v / n;
        const float R_est = (sum_vi - n * v_mean * i_mean) / 
                           (sum_ii - n * i_mean * i_mean);
        
        Serial.print(F("n="));
        Serial.print(n);
        Serial.print(F(", I="));
        Serial.print(current, 3);
        Serial.print(F("A, R≈"));
        Serial.print(R_est, 2);
        Serial.println(F("Ω"));
      }
    }
  }
  
  motor.stop();
  
  // Final calculation
  const float i_mean = sum_i / n;
  const float v_mean = sum_v / n;
  const float R_final = (sum_vi - n * v_mean * i_mean) / 
                       (sum_ii - n * i_mean * i_mean);
  
  Serial.println(F("\n=== Results ==="));
  Serial.print(F("Samples: "));
  Serial.println(n);
  Serial.print(F("Mean current: "));
  Serial.print(i_mean, 3);
  Serial.println(F(" A"));
  Serial.print(F("Armature resistance R = "));
  Serial.print(R_final, 2);
  Serial.println(F(" Ω"));
  Serial.println();
}

void test_stepResponse() {
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║        STEP RESPONSE TEST          ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  Serial.println(F("Applies voltage step and logs response"));
  Serial.println(F("For offline time constant analysis\n"));
  
  constexpr float STEP_VOLTAGE_FRACTION = 0.5f;
  constexpr unsigned long PRE_STEP_MS = 500;
  constexpr unsigned long POST_STEP_MS = 3000;
  constexpr unsigned long SAMPLE_INTERVAL_MS = 2; // 500Hz
  
  Serial.println(F("Format: time_ms, voltage, current, velocity"));
  Serial.println(F("Starting in 1 second..."));
  delay(1000);
  
  const unsigned long startMs = millis();
  unsigned long lastSampleMs = startMs;
  bool stepApplied = false;
  
  Serial.println(F("DATA_START"));
  
  while (millis() - startMs < PRE_STEP_MS + POST_STEP_MS) {
    const unsigned long now = millis();
    const unsigned long elapsed = now - startMs;
    
    // Apply step at PRE_STEP_MS
    if (!stepApplied && elapsed >= PRE_STEP_MS) {
      motor.setSpeed(STEP_VOLTAGE_FRACTION);
      stepApplied = true;
    }
    
    // Sample and log
    if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
      lastSampleMs = now;
      
      const float current = motor.readCurrent();
      const float velocity = encoder.getPositionInches(); // Simple position for now
      const float voltage = stepApplied ? 
        (STEP_VOLTAGE_FRACTION * ADC_REFERENCE_VOLTAGE) : 0.0f;
      
      Serial.print(elapsed);
      Serial.print(F(", "));
      Serial.print(voltage, 2);
      Serial.print(F(", "));
      Serial.print(current, 4);
      Serial.print(F(", "));
      Serial.println(velocity, 4);
    }
  }
  
  Serial.println(F("DATA_END"));
  motor.stop();
  
  Serial.println(F("\nTest complete. Copy data for offline analysis."));
  Serial.println(F("Fit current to: i(t) = i_ss*(1-exp(-t/tau_e))"));
  Serial.println(F("Electrical time constant tau_e = L/R\n"));
}

void test_backEMF() {
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║      BACK-EMF CONSTANT TEST        ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  Serial.println(F("Measures back-EMF constant (Kt) and resistance (R)"));
  Serial.println(F("by back-driving motor with external force\n"));
  
  Serial.println(F("SETUP INSTRUCTIONS:"));
  Serial.println(F("1. Connect 10kΩ resistor across motor terminals"));
  Serial.println(F("2. Motor driver will be disabled"));
  Serial.println(F("3. Manually spin the Ferris wheel at various speeds"));
  Serial.println(F("4. System will measure voltage and encoder velocity"));
  Serial.println(F("5. Press any key when done\n"));
  
  Serial.print(F("Ready? Press any key to start..."));
  while (!Serial.available()) delay(100);
  Serial.read();
  while (Serial.available()) Serial.read();
  Serial.println(F("\n"));
  
  // Disable motor driver (high-impedance mode)
  motor.stop();
  digitalWrite(MOTOR_INA_PIN, LOW);
  digitalWrite(MOTOR_INB_PIN, LOW);
  
  Serial.println(F("Motor driver DISABLED. You may now spin the wheel."));
  Serial.println(F("Collecting data... (updates every 100ms)"));
  Serial.println(F("Press any key to finish and compute results.\n"));
  
  // Online linear regression accumulators
  float sum_v = 0.0f, sum_w = 0.0f;
  float sum_vv = 0.0f, sum_wv = 0.0f;
  int n = 0;
  
  constexpr unsigned long SAMPLE_INTERVAL_MS = 100;
  unsigned long lastSampleMs = millis();
  unsigned long lastPrintMs = millis();
  
  // For velocity calculation
  float lastPosition = encoder.getPositionInches();
  unsigned long lastVelocityMs = millis();
  
  bool testActive = true;
  while (testActive) {
    const unsigned long now = millis();
    
    // Check for exit
    if (Serial.available()) {
      Serial.read();
      while (Serial.available()) Serial.read();
      testActive = false;
      break;
    }
    
    if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
      lastSampleMs = now;
      
      // Calculate velocity from encoder
      const float currentPosition = encoder.getPositionInches();
      const unsigned long dt = now - lastVelocityMs;
      const float velocity_counts_per_sec = 
        (currentPosition - lastPosition) * 1000.0f / dt;
      const float velocity_rad_per_sec = 
        velocity_counts_per_sec * (2.0f * PI / COUNTS_PER_REV);
      
      lastPosition = currentPosition;
      lastVelocityMs = now;
      
      // Read voltage across motor (with 10k resistor, this is mostly back-EMF)
      // TODO: Implement readVoltage() in MotorDriver if needed
      const float voltage = 0.0f; // Placeholder, no readVoltage() method
      
      // Only accumulate if wheel is actually spinning (avoid noise at zero)
      if (fabs(velocity_rad_per_sec) > 0.1f) {
        n++;
        sum_v += voltage;
        sum_w += velocity_rad_per_sec;
        sum_vv += voltage * voltage;
        sum_wv += velocity_rad_per_sec * voltage;
        
        // Print progress every second
        if (now - lastPrintMs >= 1000) {
          lastPrintMs = now;
          
          if (n > 5) {
            const float w_mean = sum_w / n;
            const float v_mean = sum_v / n;
            const float Kt_est = (sum_wv - n * w_mean * v_mean) / 
                                 (sum_w * sum_w / n - w_mean * w_mean);
            
            Serial.print(F("n="));
            Serial.print(n);
            Serial.print(F(", ω="));
            Serial.print(velocity_rad_per_sec, 2);
            Serial.print(F("rad/s, V="));
            Serial.print(voltage, 3);
            Serial.print(F("V, Kt≈"));
            Serial.print(Kt_est, 4);
            Serial.println(F(" V·s/rad"));
          } else {
            Serial.print(F("Waiting for motion... ω="));
            Serial.print(velocity_rad_per_sec, 2);
            Serial.println(F(" rad/s"));
          }
        }
      }
    }
    
    delay(10);
  }
  
  Serial.println(F("\n=== Results ==="));
  
  if (n > 10) {
    // Compute final Kt using linear regression
    const float w_mean = sum_w / n;
    const float v_mean = sum_v / n;
    const float Kt_final = (sum_wv - n * w_mean * v_mean) / 
                          (sum_w * sum_w / n - w_mean * w_mean);
    
    Serial.print(F("Valid samples: "));
    Serial.println(n);
    Serial.print(F("Mean velocity: "));
    Serial.print(w_mean, 2);
    Serial.println(F(" rad/s"));
    Serial.print(F("Mean voltage: "));
    Serial.print(v_mean, 3);
    Serial.println(F(" V"));
    Serial.print(F("\nBack-EMF constant Kt = "));
    Serial.print(Kt_final, 4);
    Serial.println(F(" V·s/rad"));
    Serial.print(F("Torque constant Kt = "));
    Serial.print(Kt_final, 4);
    Serial.println(F(" N·m/A"));
  } else {
    Serial.println(F("Insufficient data - spin wheel faster/longer!"));
  }
  
  Serial.println();
}

void test_rlsElectrical() {
  Serial.println(F("RLS Electrical test not implemented on Uno."));
}

void test_continuousRLS() {
  Serial.println(F("Continuous RLS not implemented on Uno."));
}

void updateContinuousRLS() {
  // No-op for Uno
}

// =======================================================
// ================= Serial Menu System ==================
// =======================================================

void printHelp() {
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║    DMS Manual Control Menu         ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  Serial.println(F("=== Motion Commands ==="));
  Serial.println(F("u       - Move to TOP (min speed)"));
  Serial.println(F("d       - Move to BOTTOM (min speed)"));
  Serial.println(F("p <num> - Move to % of travel (0-100)"));
  Serial.println(F("s       - STOP motor immediately"));
  Serial.println(F("z       - Zero encoder position (set current as 0)"));
  Serial.println(F(""));
  Serial.println(F("=== Diagnostic Modes ==="));
  Serial.println(F("r       - READ encoder position & current"));
  Serial.println(F("j       - JOG MODE (incremental movement)"));
  Serial.println(F("c       - CONTINUOUS MONITOR (live data)"));
  Serial.println(F(""));
  Serial.println(F("=== Testing Modes ==="));
  Serial.println(F("m       - RAMP speed test (alternates dir)"));
  Serial.println(F("l       - CURRENT LIMIT test"));
  Serial.println(F(""));
  Serial.println(F("=== System ID Tests ==="));
  Serial.println(F("R       - RESISTANCE measurement (R)"));
  Serial.println(F("K       - BACK-EMF constant test (Kt)"));
  Serial.println(F("S       - STEP RESPONSE test (L, R, J, b)"));
  Serial.println(F("E       - RLS electrical params (L, R, Kt)"));
  Serial.println(F("X       - TOGGLE continuous RLS (background)"));
  Serial.println(F(""));
  Serial.println(F("h       - Show this help menu"));
  Serial.println(F("?       - Show system status"));
  Serial.println();
}

void printStatus() {
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║         System Status              ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  
  Serial.print(F("Board: "));
  Serial.println(F(BOARD_NAME));
  
  Serial.print(F("CPU: "));
  Serial.print(CPU_FREQUENCY_HZ / 1000000UL);
  Serial.println(F(" MHz"));
  
  Serial.print(F("ADC: "));
  Serial.print(static_cast<int>(ADC_RESOLUTION_BITS));
  Serial.print(F("-bit, "));
  Serial.print(ADC_REFERENCE_VOLTAGE);
  Serial.println(F("V ref"));
  
  Serial.print(F("Position: "));
  Serial.print(encoder.getPositionInches(), 2);
  Serial.print(F(" in ("));
  Serial.print((encoder.getPositionInches() / MAX_TRAVEL_IN_FLOAT) * 100.0f, 1);
  Serial.println(F("%)"));
  
  Serial.print(F("Encoder: "));
  Serial.print(encoder.getPositionCounts());
  Serial.println(F(" counts"));
  
  Serial.print(F("Current: "));
  Serial.print(motor.readCurrent(), 3);
  Serial.println(F(" A"));
  
  Serial.print(F("State: "));
  switch (controller.currentPosition()) {
    case MotorPosition::Top: Serial.println(F("TOP")); break;
    case MotorPosition::Bottom: Serial.println(F("BOTTOM")); break;
    case MotorPosition::Moving: Serial.println(F("MOVING")); break;
    default: Serial.println(F("UNKNOWN")); break;
  }
  
  Serial.println();
}

void setup() {
  // Initialize board-specific hardware (ADC, PWM, Serial)
  initBoardSpecificHardware();
  
  // Initialize motor control system
  controller.begin();
  
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║  Decoration Motor System (DMS)     ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  
  // Print board info
  Serial.print(F("\nBoard: "));
  Serial.println(F(BOARD_NAME));
  Serial.print(F("CPU: "));
  Serial.print(CPU_FREQUENCY_HZ / 1000000UL);
  Serial.println(F(" MHz"));
  Serial.print(F("ADC: "));
  Serial.print(static_cast<int>(ADC_RESOLUTION_BITS));
  Serial.print(F("-bit, "));
  Serial.print(ADC_REFERENCE_VOLTAGE);
  Serial.println(F("V ref"));
  
  // Show compile-time constants
  Serial.println(F("\nSystem Configuration:"));
  Serial.print(F("  Max travel: "));
  Serial.print(MAX_TRAVEL_IN_FLOAT);
  Serial.println(F(" inches"));
  Serial.print(F("  Encoder: "));
  Serial.print(COUNTS_PER_REV, 0);
  Serial.println(F(" counts/rev"));
  Serial.print(F("  Resolution: "));
  Serial.print(COUNTS_PER_IN);
  Serial.println(F(" counts/inch"));
  Serial.print(F("  Duty cycle: "));
  Serial.print(MIN_DUTY_CYCLE * 100.0f, 0);
  Serial.print(F("% - "));
  Serial.print(MAX_DUTY_CYCLE * 100.0f, 0);
  Serial.println(F("%"));
  
#ifdef ENABLE_ADVANCED_CONTROL
  Serial.println(F("  Advanced control: ENABLED"));
#else
  Serial.println(F("  Advanced control: DISABLED"));
#endif
  
  // Show available commands
  printHelp();
  
  // Optional: Automatic homing on startup
  // Uncomment the next line if you want to home to bottom on power-up
  // controller.homeToBottom();

  Serial.print(F("POSITION_TOLERANCE_COUNTS = "));
  Serial.println(POSITION_TOLERANCE_COUNTS);
}

void loop() {
  // Process serial commands
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0)
      return;

    // Parse command
    char cmd = input.charAt(0);
    
    switch (cmd) {
      case 'u':
      case 'U':
        test_moveToTop();
        break;
        
      case 'd':
      case 'D':
        test_moveToBottom();
        break;
        
      case 'p':
      case 'P': {
        // Extract number after 'p'
        String numStr = input.substring(1);
        numStr.trim();
        float percent = numStr.toFloat();
        test_moveToPercent(percent);
        break;
      }
      
      case 's':
        test_stopMotor();
        break;
      case 'S': // Capital S for step response
        test_stepResponse();
        break;
      case 'r':
        test_printPosition();
        break;
      case 'R': // Capital R for resistance test
        test_resistanceMeasurement();
        break;
        
      case 'm':
      case 'M':
        test_rampUpSpeed();
        break;
        
      case 'j':
      case 'J':
        test_jogMode();
        break;
        
      case 'c':
      case 'C':
        test_continuousMonitor();
        break;
        
      case 'l':
      case 'L':
        test_currentLimit();
        break;
        
      case 'K': // Capital K for back-EMF constant
        test_backEMF();
        break;
      case 'E': // Capital E for RLS electrical
        test_rlsElectrical();
        break;
      case 'X': // Capital X for continuous RLS toggle
        test_continuousRLS();
        break;
        
      case 'z':
      case 'Z':
        test_zeroEncoder();
        break;
        
      case 'h':
      case 'H':
        printHelp();
        break;
        
      case '?':
        printStatus();
        break;
        
      default:
        Serial.print(F("Unknown command: "));
        Serial.println(cmd);
        Serial.println(F("Type 'h' for help."));
        break;
    }
  }
  
  // Update motor control (handles motion profiles, safety checks)
  controller.update();
  
  // Update continuous RLS if enabled
  updateContinuousRLS();
  
#ifdef ENABLE_PERFORMANCE_MONITORING
  static unsigned long loopCount = 0;
  static unsigned long lastPrintMs = 0;
  
  loopCount++;
  const unsigned long now = millis();
  
  if (now - lastPrintMs >= 5000) { // Print every 5 seconds
    const unsigned long elapsed = now - lastPrintMs;
    const float loopsPerSec = (loopCount * 1000.0f) / static_cast<float>(elapsed);
    
    Serial.print(F("Performance: "));
    Serial.print(loopsPerSec, 0);
    Serial.println(F(" Hz"));
    
    loopCount = 0;
    lastPrintMs = now;
  }
#endif
  
  // Small delay to prevent overwhelming the serial port
  // On STM32, this can be very small or even removed
  #ifdef BOARD_ARDUINO_UNO
    delay(10);  // 100Hz update rate on Uno
  #else
    delay(1);   // 1000Hz update rate on STM32
  #endif
}