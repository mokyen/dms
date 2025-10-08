#include "src/Config.h"
#include "src/MotorDriver.h"
#include "src/EncoderReader.h"
#include "src/MotorControl.h"
#include "src/MotionProfiles.h"

// Create hardware instances using constexpr for compile-time initialization
constexpr MotorDriver motor(MOTOR_PWM_PIN, MOTOR_INA_PIN, MOTOR_INB_PIN, MOTOR_CS_PIN);
constexpr EncoderReader encoder(ENCODER_A_PIN, ENCODER_B_PIN);

// Controller needs to be non-const because it maintains state
MotorControl controller(const_cast<MotorDriver&>(motor), 
                        const_cast<EncoderReader&>(encoder));

// =======================================================
// ================ Test Function Definitions ============
// =======================================================

void test_moveToTop() {
  Serial.println(F("Moving to TOP (min speed)..."));
  MotionProfiles::moveToPosition(motor, encoder, MAX_TRAVEL_IN, MIN_DUTY_CYCLE);
}

void test_moveToBottom() {
  Serial.println(F("Moving to BOTTOM (min speed)..."));
  MotionProfiles::moveToPosition(motor, encoder, 0.0f, MIN_DUTY_CYCLE);
}

void test_moveToPercent(float percent) {
  percent = constrain(percent, 0.0f, 100.0f);
  Serial.print(F("Moving to "));
  Serial.print(percent);
  Serial.println(F("% of travel..."));
  
  // Calculate target position
  const float targetInches = (percent * 0.01f) * MAX_TRAVEL_IN;
  MotionProfiles::moveToPosition(motor, encoder, targetInches, MIN_DUTY_CYCLE);
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
  constexpr float JOG_SPEED = MIN_DUTY_CYCLE;
  
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║          JOG MODE ACTIVE           ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  Serial.println(F("+ or w - Jog UP (forward)"));
  Serial.println(F("- or s - Jog DOWN (reverse)"));
  Serial.println(F("0      - Stop motor"));
  Serial.println(F("r      - Read position"));
  Serial.println(F("x      - EXIT jog mode"));
  Serial.println();
  
  bool jogActive = true;
  while (jogActive) {
    if (Serial.available()) {
      const char cmd = Serial.read();
      
      switch (cmd) {
        case '+':
        case 'w':
        case 'W': {
          const float currentPos = encoder.getPositionInches();
          const float targetPos = currentPos + JOG_DISTANCE_IN;
          Serial.print(F("Jog UP to "));
          Serial.print(targetPos, 2);
          Serial.println(F(" in"));
          MotionProfiles::moveToPosition(motor, encoder, targetPos, JOG_SPEED);
          break;
        }
        
        case '-':
        case 's':
        case 'S': {
          const float currentPos = encoder.getPositionInches();
          const float targetPos = currentPos - JOG_DISTANCE_IN;
          Serial.print(F("Jog DOWN to "));
          Serial.print(targetPos, 2);
          Serial.println(F(" in"));
          MotionProfiles::moveToPosition(motor, encoder, targetPos, JOG_SPEED);
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
  long lastPosition = encoder.getPositionCounts();
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
      const long currentPosition = encoder.getPositionCounts();
      const unsigned long dt = now - lastVelocityMs;
      const float velocity_counts_per_sec = 
        (currentPosition - lastPosition) * 1000.0f / dt;
      const float velocity_rad_per_sec = 
        velocity_counts_per_sec * (2.0f * PI / COUNTS_PER_REV);
      
      lastPosition = currentPosition;
      lastVelocityMs = now;
      
      // Read voltage across motor (with 10k resistor, this is mostly back-EMF)
      const float voltage = motor.readVoltage(); // Read voltage from current sense
      
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

// =======================================================
// ========== Recursive Least Squares (RLS) =============
// =======================================================

// RLS state for electrical subsystem parameters
struct RLS_Electrical {
  // Parameters: [Δt/L, Δt·R/L, Δt·Kt/L]
  float theta[3] = {0.1f, 0.5f, 0.01f}; // Initial guesses
  float P[3][3];  // Covariance matrix
  float lambda = 0.98f; // Forgetting factor
  bool initialized = false;
  
  void init() {
    // Initialize covariance with large uncertainty
    constexpr float sigma_sq = 1000.0f;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        P[i][j] = (i == j) ? sigma_sq : 0.0f;
      }
    }
    initialized = true;
  }
  
  void update(float voltage, float current, float current_prev, 
              float velocity, float dt) {
    // Measurement: y = i[k+1] - i[k]
    const float y = current - current_prev;
    
    // Regressor: φ = [V[k], -i[k], -ω[k]]
    float phi[3] = {voltage, -current_prev, -velocity};
    
    // Prediction: ŷ = φᵀ·θ
    float y_pred = 0.0f;
    for (int i = 0; i < 3; i++) {
      y_pred += phi[i] * theta[i];
    }
    
    // Innovation: ε = y - ŷ
    const float epsilon = y - y_pred;
    
    // Compute P·φ
    float P_phi[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        P_phi[i] += P[i][j] * phi[j];
      }
    }
    
    // Compute φᵀ·P·φ
    float phi_P_phi = 0.0f;
    for (int i = 0; i < 3; i++) {
      phi_P_phi += phi[i] * P_phi[i];
    }
    
    // Kalman gain: K = P·φ / (λ + φᵀ·P·φ)
    float K[3];
    const float denom = lambda + phi_P_phi;
    for (int i = 0; i < 3; i++) {
      K[i] = P_phi[i] / denom;
    }
    
    // Update parameters: θ = θ + K·ε
    for (int i = 0; i < 3; i++) {
      theta[i] += K[i] * epsilon;
    }
    
    // Update covariance: P = (P - K·φᵀ·P) / λ
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        P[i][j] = (P[i][j] - K[i] * P_phi[j]) / lambda;
      }
    }
  }
  
  void getParameters(float dt, float &L, float &R, float &Kt) const {
    // Extract physical parameters from θ
    // θ = [Δt/L, Δt·R/L, Δt·Kt/L]
    L = dt / theta[0];
    R = theta[1] * L / dt;
    Kt = theta[2] * L / dt;
  }
};

RLS_Electrical rls_elec;

// Continuous RLS mode flag
bool continuousRLS = false;
float rls_L = 0.0f, rls_R = 0.0f, rls_Kt = 0.0f;
unsigned long lastRLSUpdate = 0;
float rls_current_prev = 0.0f;

void test_rlsElectrical() {
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║  RECURSIVE LEAST SQUARES (RLS)     ║"));
  Serial.println(F("║  Electrical Parameter Estimation   ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  Serial.println(F("Real-time parameter adaptation"));
  Serial.println(F("Identifies: L (inductance), R (resistance), Kt (back-EMF)\n"));
  
  if (!rls_elec.initialized) {
    rls_elec.init();
    Serial.println(F("RLS initialized with default parameters"));
  }
  
  Serial.println(F("Applying excitation signal (PRBS-like)..."));
  Serial.println(F("Press any key to stop\n"));
  
  constexpr unsigned long TEST_DURATION_MS = 10000; // 10 seconds
  constexpr float SAMPLE_RATE_HZ = 100.0f;
  constexpr float DT = 1.0f / SAMPLE_RATE_HZ;
  constexpr unsigned long SAMPLE_INTERVAL_MS = 
    static_cast<unsigned long>(DT * 1000.0f);
  
  const unsigned long startMs = millis();
  unsigned long lastSampleMs = startMs;
  unsigned long lastPrintMs = startMs;
  
  float voltage = 0.0f;
  float current_prev = motor.readCurrent();
  int excitation_state = 0;
  
  Serial.println(F("Time(s) | L(mH)  | R(Ω)   | Kt(V·s/rad)"));
  Serial.println(F("────────┼────────┼────────┼────────────"));
  
  bool testActive = true;
  while (testActive && (millis() - startMs < TEST_DURATION_MS)) {
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
      
      // Generate pseudo-random excitation (simple PRBS-like)
      excitation_state = (excitation_state + 1) % 20;
      if (excitation_state < 5) voltage = 0.3f;
      else if (excitation_state < 10) voltage = 0.6f;
      else if (excitation_state < 15) voltage = -0.3f;
      else voltage = 0.0f;
      
      motor.setSpeed(voltage);
      
      // Read measurements
      const float current = motor.readCurrent();
      const float velocity = encoder.getPositionInches(); // Simplified
      
      // Update RLS
      rls_elec.update(voltage * ADC_REFERENCE_VOLTAGE, 
                     current, current_prev, velocity, DT);
      current_prev = current;
      
      // Print progress every 500ms
      if (now - lastPrintMs >= 500) {
        lastPrintMs = now;
        
        float L, R, Kt;
        rls_elec.getParameters(DT, L, R, Kt);
        
        const float elapsed = (now - startMs) / 1000.0f;
        Serial.print(elapsed, 1);
        Serial.print(F("     | "));
        Serial.print(L * 1000.0f, 2); // Convert to mH
        Serial.print(F("   | "));
        Serial.print(R, 2);
        Serial.print(F("    | "));
        Serial.println(Kt, 4);
      }
    }
  }
  
  motor.stop();
  
  Serial.println(F("\n=== Final Parameters ==="));
  float L, R, Kt;
  rls_elec.getParameters(DT, L, R, Kt);
  
  Serial.print(F("Inductance L = "));
  Serial.print(L * 1000.0f, 2);
  Serial.println(F(" mH"));
  Serial.print(F("Resistance R = "));
  Serial.print(R, 2);
  Serial.println(F(" Ω"));
  Serial.print(F("Back-EMF Kt = "));
  Serial.print(Kt, 4);
  Serial.println(F(" V·s/rad"));
  Serial.print(F("Torque Kt = "));
  Serial.print(Kt, 4);
  Serial.println(F(" N·m/A\n"));
}

void test_continuousRLS() {
  if (!continuousRLS) {
    Serial.println(F("\n╔════════════════════════════════════╗"));
    Serial.println(F("║   CONTINUOUS RLS MODE ENABLED      ║"));
    Serial.println(F("╚════════════════════════════════════╝"));
    Serial.println(F("RLS now runs in background during all operations"));
    Serial.println(F("Parameter estimates logged every 5 seconds"));
    Serial.println(F("Use 'X' command to disable\n"));
    
    if (!rls_elec.initialized) {
      rls_elec.init();
      Serial.println(F("RLS initialized"));
    }
    
    continuousRLS = true;
    rls_current_prev = motor.readCurrent();
    lastRLSUpdate = millis();
    
  } else {
    Serial.println(F("\n╔════════════════════════════════════╗"));
    Serial.println(F("║   CONTINUOUS RLS MODE DISABLED     ║"));
    Serial.println(F("╚════════════════════════════════════╝"));
    
    Serial.println(F("Final parameter estimates:"));
    Serial.print(F("  L = "));
    Serial.print(rls_L * 1000.0f, 2);
    Serial.println(F(" mH"));
    Serial.print(F("  R = "));
    Serial.print(rls_R, 2);
    Serial.println(F(" Ω"));
    Serial.print(F("  Kt = "));
    Serial.print(rls_Kt, 4);
    Serial.println(F(" V·s/rad\n"));
    
    continuousRLS = false;
  }
}

void updateContinuousRLS() {
  if (!continuousRLS) return;
  
  constexpr float RLS_SAMPLE_RATE_HZ = 50.0f; // 50Hz background rate
  constexpr float RLS_DT = 1.0f / RLS_SAMPLE_RATE_HZ;
  constexpr unsigned long RLS_INTERVAL_MS = 
    static_cast<unsigned long>(RLS_DT * 1000.0f);
  
  const unsigned long now = millis();
  
  if (now - lastRLSUpdate >= RLS_INTERVAL_MS) {
    lastRLSUpdate = now;
    
    // Get current measurements
    const float current = motor.readCurrent();
    const float velocity = encoder.getPositionInches(); // Simplified
    const float voltage = 6.0f; // Approximate - would need actual PWM duty
    
    // Update RLS
    rls_elec.update(voltage, current, rls_current_prev, velocity, RLS_DT);
    rls_current_prev = current;
    
    // Extract parameters
    rls_elec.getParameters(RLS_DT, rls_L, rls_R, rls_Kt);
    
    // Log every 5 seconds
    static unsigned long lastLog = 0;
    if (now - lastLog >= 5000) {
      lastLog = now;
      
      DEBUG_PRINT(F("RLS: L="));
      DEBUG_PRINT(rls_L * 1000.0f, 2);
      DEBUG_PRINT(F("mH, R="));
      DEBUG_PRINT(rls_R, 2);
      DEBUG_PRINT(F("Ω, Kt="));
      DEBUG_PRINTLN(rls_Kt, 4);
    }
  }
}

// =======================================================
// ============ Falling Weight Test =====================
// =======================================================

void test_fallingWeight() {
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║      FALLING WEIGHT TEST           ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  Serial.println(F("Measures moment of inertia (J) and friction (b)"));
  Serial.println(F("by observing falling mass dynamics\n"));
  
  Serial.println(F("SETUP INSTRUCTIONS:"));
  Serial.println(F("1. Wrap string around Ferris wheel at known radius"));
  Serial.println(F("2. Attach known mass to string"));
  Serial.println(F("3. Hold mass at starting position"));
  Serial.println(F("4. Motor driver will be disabled"));
  Serial.println(F("5. Release mass when prompted\n"));
  
  // Get test parameters
  Serial.print(F("Enter mass in grams (e.g., 100): "));
  while (!Serial.available()) delay(100);
  const float mass_g = Serial.parseFloat();
  while (Serial.available()) Serial.read();
  Serial.println(mass_g, 1);
  
  Serial.print(F("Enter string radius in cm (e.g., 15): "));
  while (!Serial.available()) delay(100);
  const float radius_cm = Serial.parseFloat();
  while (Serial.available()) Serial.read();
  Serial.println(radius_cm, 1);
  
  const float mass_kg = mass_g / 1000.0f;
  const float radius_m = radius_cm / 100.0f;
  constexpr float g = 9.81f; // m/s²
  
  Serial.println(F("\nTest configuration:"));
  Serial.print(F("  Mass: "));
  Serial.print(mass_kg, 4);
  Serial.println(F(" kg"));
  Serial.print(F("  Radius: "));
  Serial.print(radius_m, 3);
  Serial.println(F(" m"));
  Serial.print(F("  Expected torque: "));
  Serial.print(mass_kg * g * radius_m, 4);
  Serial.println(F(" N·m\n"));
  
  Serial.println(F("Ready to start test."));
  Serial.print(F("Press any key, then RELEASE the mass..."));
  while (!Serial.available()) delay(100);
  Serial.read();
  while (Serial.available()) Serial.read();
  Serial.println(F("\n"));
  
  // Disable motor driver
  motor.stop();
  digitalWrite(MOTOR_INA_PIN, LOW);
  digitalWrite(MOTOR_INB_PIN, LOW);
  
  Serial.println(F("Motor disabled. Waiting for motion to start..."));
  
  // Wait for motion to start (velocity threshold)
  constexpr float START_THRESHOLD_COUNTS_PER_SEC = 10.0f;
  long lastPos = encoder.getPositionCounts();
  unsigned long lastTime = millis();
  bool motionDetected = false;
  
  // Wait up to 5 seconds for motion
  const unsigned long waitStart = millis();
  while (!motionDetected && (millis() - waitStart < 5000)) {
    delay(50);
    const long currentPos = encoder.getPositionCounts();
    const unsigned long currentTime = millis();
    const float dt = (currentTime - lastTime) / 1000.0f;
    const float velocity = (currentPos - lastPos) / dt;
    
    if (fabs(velocity) > START_THRESHOLD_COUNTS_PER_SEC) {
      motionDetected = true;
      Serial.println(F("Motion detected! Recording data..."));
    }
    
    lastPos = currentPos;
    lastTime = currentTime;
  }
  
  if (!motionDetected) {
    Serial.println(F("No motion detected. Test aborted."));
    return;
  }
  
  // Record falling motion
  constexpr int MAX_SAMPLES = 500;
  constexpr unsigned long SAMPLE_INTERVAL_MS = 10; // 100Hz
  
  float time_data[MAX_SAMPLES];
  long position_data[MAX_SAMPLES];
  int sampleCount = 0;
  
  const unsigned long testStart = millis();
  unsigned long lastSample = testStart;
  long startPosition = encoder.getPositionCounts();
  
  // Record until motion stops or buffer full
  while (sampleCount < MAX_SAMPLES) {
    const unsigned long now = millis();
    
    if (now - lastSample >= SAMPLE_INTERVAL_MS) {
      lastSample = now;
      
      time_data[sampleCount] = (now - testStart) / 1000.0f;
      position_data[sampleCount] = encoder.getPositionCounts() - startPosition;
      sampleCount++;
      
      // Check if motion stopped (looking at last few samples)
      if (sampleCount > 20) {
        long recentMotion = abs(position_data[sampleCount-1] - 
                               position_data[sampleCount-10]);
        if (recentMotion < 5) {  // Less than 5 counts in 100ms
          Serial.println(F("Motion stopped."));
          break;
        }
      }
    }
    
    delay(1);
  }
  
  Serial.println(F("\n=== Data Collection Complete ==="));
  Serial.print(F("Samples collected: "));
  Serial.println(sampleCount);
  Serial.print(F("Test duration: "));
  Serial.print(time_data[sampleCount-1], 2);
  Serial.println(F(" seconds"));
  Serial.print(F("Total rotation: "));
  Serial.print(position_data[sampleCount-1]);
  Serial.println(F(" counts\n"));
  
  // Simple analysis: estimate terminal velocity and acceleration
  // For more accurate results, use offline curve fitting
  
  // Find terminal velocity (average of last 20% of samples)
  int terminalStart = sampleCount * 4 / 5;
  float terminalVelocity = 0.0f;
  int terminalCount = 0;
  
  for (int i = terminalStart; i < sampleCount - 1; i++) {
    float dt = time_data[i+1] - time_data[i];
    float dpos = (position_data[i+1] - position_data[i]) / COUNTS_PER_IN;
    terminalVelocity += dpos / dt;
    terminalCount++;
  }
  
  if (terminalCount > 0) {
    terminalVelocity /= terminalCount;
    
    // Convert to rad/s
    const float omega_terminal = terminalVelocity * (2.0f * PI / TRAVEL_PER_REV_IN);
    
    // At terminal velocity: b·ω = m·g·r
    const float b_estimated = (mass_kg * g * radius_m) / omega_terminal;
    
    Serial.println(F("=== Preliminary Results ==="));
    Serial.print(F("Terminal velocity: "));
    Serial.print(omega_terminal, 3);
    Serial.println(F(" rad/s"));
    Serial.print(F("Friction coefficient b ≈ "));
    Serial.print(b_estimated, 6);
    Serial.println(F(" N·m·s/rad\n"));
    
    Serial.println(F("For accurate J estimation, export data:"));
    Serial.println(F("Format: time(s), position(counts)"));
    Serial.print(F("Export data? (y/n): "));
    while (!Serial.available()) delay(100);
    char response = Serial.read();
    while (Serial.available()) Serial.read();
    Serial.println(response);
    
    if (response == 'y' || response == 'Y') {
      Serial.println(F("\nDATA_START"));
      for (int i = 0; i < sampleCount; i++) {
        Serial.print(time_data[i], 4);
        Serial.print(F(", "));
        Serial.println(position_data[i]);
      }
      Serial.println(F("DATA_END\n"));
      
      Serial.println(F("Fit in Python to: y(t) = A*(1-exp(-t/tau)) + v0*t"));
      Serial.println(F("where tau = (m + J/r²)/(b/r²)"));
      Serial.println(F("Solve for J using known b from above."));
    }
  } else {
    Serial.println(F("Could not determine terminal velocity."));
    Serial.println(F("Mass may be too light or fall distance too short."));
  }
  
  Serial.println();
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
  Serial.print((encoder.getPositionInches() / MAX_TRAVEL_IN) * 100.0f, 1);
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
      case MotionMode::Proportional:
#ifdef ENABLE_ADVANCED_CONTROL
        currentMode = MotionMode::Trapezoidal;
        break;
      case MotionMode::Trapezoidal:
#endif
        currentMode = MotionMode::Linear;
        break;
    }
    Serial.print(F("Motion mode changed to: "));
    printMotionMode();
  }
  
  void printMotionMode() const {
    switch (currentMode) {
      case MotionMode::Linear:
        Serial.println(F("Linear (constant speed)"));
        break;
      case MotionMode::Proportional:
        Serial.println(F("Proportional (slows near target)"));
        break;
#ifdef ENABLE_ADVANCED_CONTROL
      case MotionMode::Trapezoidal:
        Serial.println(F("Trapezoidal (smooth accel/decel)"));
        break;
#endif
    }
  }
};

CommandParser parser;

// Performance monitoring (STM32 only)
#ifdef ENABLE_PERFORMANCE_MONITORING
class PerformanceMonitor {
public:
  void update() {
    loopCount++;
    const unsigned long now = millis();
    
    if (now - lastPrintMs >= 1000) {
      const unsigned long elapsed = now - lastPrintMs;
      const float loopsPerSec = (loopCount * 1000.0f) / static_cast<float>(elapsed);
      
      Serial.print(F("Performance: "));
      Serial.print(loopsPerSec, 0);
      Serial.println(F(" Hz"));
      
      loopCount = 0;
      lastPrintMs = now;
    }
  }
  
private:
  unsigned long loopCount = 0;
  unsigned long lastPrintMs = 0;
};

PerformanceMonitor perfMon;
#endif

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
  Serial.print(MAX_TRAVEL_IN);
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
      case 'S':
        test_stopMotor();
        break;
        
      case 'r':
      case 'R':
        test_printPosition();
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
        
      case 'R': // Capital R for resistance test
        test_resistanceMeasurement();
        break;
        
      case 'K': // Capital K for back-EMF constant
        test_backEMF();
        break;
        
      case 'S': // Capital S for step response
        test_stepResponse();
        break;
        
      case 'E': // Capital E for RLS electrical
        test_rlsElectrical();
        break;
        
      case 'X': // Capital X for continuous RLS toggle
        test_continuousRLS();
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
    
    DEBUG_PRINT(F("Performance: "));
    DEBUG_PRINT(loopsPerSec, 0);
    DEBUG_PRINTLN(F(" Hz"));
    
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