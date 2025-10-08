# Advanced System Identification Guide

## Motor Model Parameters

Your DC motor system is characterized by 5 key parameters:

### Electrical Parameters
1. **R** - Armature resistance (Ω)
   - Measured by: Resistance test (R), RLS (E)
   - Typical range: 5-20Ω for small DC motors
   - Affects: Electrical time constant, power dissipation

2. **L** - Armature inductance (H)
   - Measured by: Step response (S), RLS (E)
   - Typical range: 1-10 mH for small DC motors
   - Affects: Current rise time, high-frequency dynamics

3. **Kt** - Torque/Back-EMF constant (N·m/A or V·s/rad)
   - Measured by: Back-EMF test (K), RLS (E)
   - Typical range: 0.01-0.1 for small motors
   - Affects: Torque production, back-EMF voltage
   - **Note:** Kt (torque) = Ke (back-EMF) in SI units

### Mechanical Parameters
4. **J** - Total moment of inertia (kg·m²)
   - Measured by: Step response (S), Falling weight test
   - Includes: Motor rotor + gearbox + Ferris wheel
   - Dominant contribution: Ferris wheel
   - Affects: Acceleration response, mechanical time constant

5. **b** - Viscous friction coefficient (N·m·s/rad)
   - Measured by: Step response (S), Coast-down test
   - Includes: Motor friction + gearbox friction + bearing friction
   - Affects: Steady-state velocity, coast-down behavior

## State-Space Model

The complete motor dynamics are:

```
Electrical equation:
L·(di/dt) = V - R·i - Kt·ω

Mechanical equation:
J·(dω/dt) = Kt·i - b·ω - τ_load

Position:
dθ/dt = ω
```

State vector: **x** = [θ, ω, i]ᵀ

Input: **u** = V (applied voltage)

### Discrete-Time Model

For control implementation at sample time Δt:

```
x[k+1] = A·x[k] + B·u[k]
y[k] = C·x[k]

where:
A = [1    Δt      0          ]
    [0    1-b·Δt/J  Kt·Δt/J  ]
    [0   -Kt·Δt/L   1-R·Δt/L ]

B = [0        ]
    [0        ]
    [Δt/L     ]

C = [1  0  0]  (position output)
    [0  0  1]  (current output)
```

## RLS Theory and Implementation

### Why RLS?

Traditional batch least squares requires:
- Storing all historical data
- Recomputing entire fit when new data arrives
- Not suitable for real-time embedded systems

**RLS advantages:**
- Processes data one sample at a time
- Constant memory (only stores parameters and covariance)
- Constant computation time per update
- Can track time-varying parameters with forgetting factor

### Mathematical Foundation

**Problem:** Identify parameters θ in linear model:
```
y[k] = φ[k]ᵀ·θ + noise
```

**Objective:** Minimize weighted sum of squared errors:
```
J = Σ λ^(N-k) · (y[k] - φ[k]ᵀ·θ)²
```

where λ is forgetting factor (0 < λ ≤ 1)

**Recursive solution:**
```
1. Prediction error:
   ε[k] = y[k] - φ[k]ᵀ·θ[k-1]

2. Kalman gain:
   K[k] = P[k-1]·φ[k] / (λ + φ[k]ᵀ·P[k-1]·φ[k])

3. Parameter update:
   θ[k] = θ[k-1] + K[k]·ε[k]

4. Covariance update:
   P[k] = (1/λ)·(P[k-1] - K[k]·φ[k]ᵀ·P[k-1])
```

### Electrical Subsystem RLS

**Motor voltage equation:**
```
L·Δi = Δt·(V - R·i - Kt·ω)
```

Rearranging:
```
Δi = (Δt/L)·V - (Δt·R/L)·i - (Δt·Kt/L)·ω
```

**Linear regression form:**
```
y = Δi (current change)
φ = [V, -i, -ω]ᵀ (regressor)
θ = [Δt/L, Δt·R/L, Δt·Kt/L]ᵀ (parameters)
```

**Extract physical parameters:**
```
L = Δt / θ₁
R = θ₂ · L / Δt
Kt = θ₃ · L / Δt
```

### Forgetting Factor Selection

**λ = 1.0:** Standard RLS
- All data weighted equally
- Good for constant parameters
- Slow adaptation to changes

**λ = 0.95-0.99:** Exponential forgetting
- Recent data weighted more heavily
- Tracks slowly time-varying parameters
- Trade-off: faster adaptation vs. more noise sensitivity

**Effective memory:**
```
N_eff ≈ 1 / (1 - λ)

λ = 0.95 → N_eff ≈ 20 samples
λ = 0.98 → N_eff ≈ 50 samples
λ = 0.99 → N_eff ≈ 100 samples
```

### Persistent Excitation

**Critical requirement:** Input signal must be "sufficiently rich" to excite all system modes.

**For 3-parameter identification:** Need input with at least 3 distinct frequencies

**Good excitation signals:**
1. **PRBS (Pseudo-Random Binary Sequence)**
   - Switches between voltage levels pseudo-randomly
   - Flat power spectrum over bandwidth
   - Easy to generate

2. **Multi-sine**
   - Sum of sinusoids at different frequencies
   - V(t) = A₁sin(ω₁t) + A₂sin(ω₂t) + A₃sin(ω₃t)
   - Precisely controlled frequency content

3. **Chirp**
   - Frequency sweep
   - V(t) = A·sin(2π·f(t)·t) where f(t) increases linearly

**Poor excitation:**
- Constant voltage (identifies nothing)
- Single frequency sine wave (underdetermined system)
- Step input (limited frequency content)

### Numerical Stability

**Covariance conditioning:**
- Matrix P can become ill-conditioned over long runs
- Symptoms: Parameters stop updating or diverge
- Solutions:
  1. **Covariance resetting:** Periodically reinitialize P
  2. **Upper/lower bounds:** Limit diagonal elements of P
  3. **Square-root filtering:** More complex but numerically robust

**Implementation tip:**
```cpp
// Monitor condition number
float P_trace = P[0][0] + P[1][1] + P[2][2];
if (P_trace < 0.01 || P_trace > 10000.0) {
  // Reset covariance
  reinitialize_P();
}
```

## Mechanical Subsystem Identification

### Falling Weight Test (Most Accurate)

**Setup:**
1. Wrap string around wheel at radius r
2. Attach known mass m to string
3. Release and record falling mass position y(t)

**Physics:**
```
Mass equation: m·ÿ = m·g - T (tension)
Motor equation: J·α = T·r - b·ω
Constraint: ÿ = r·α (no slip)
```

**Combined dynamics:**
```
ÿ = (m·g - b·ω/r) / (m + J/r²)
```

**At terminal velocity (ÿ = 0):**
```
b·ω_terminal = m·g·r
→ b = (m·g·r) / ω_terminal
```

**From transient (fitting y(t)):**
```
y(t) = A·(1 - exp(-t/τ)) + v₀·t

where: τ = (m + J/r²) / (b/r²)
```

With known b from terminal velocity, solve for J.

**Measurement techniques:**
- High-speed video (120+ fps)
- Tracker Video Analysis software
- Measure y(t) with ruler in frame
- Expected fall time: 2-5 seconds for good measurement

### Coast-Down Test

**Setup:**
1. Spin motor to moderate speed
2. **Disable H-bridge** (high-impedance mode)
   - **Critical:** Use enable pins, NOT zero PWM
   - Zero PWM causes dynamic braking
3. Record velocity decay ω(t)

**Dynamics:**
```
J·(dω/dt) = -b·ω

Solution: ω(t) = ω₀·exp(-t/τ_m)

where: τ_m = J/b (mechanical time constant)
```

**Parameter extraction:**
- Fit exponential to ω(t) → extract τ_m = J/b
- Need independent estimate of b or J to separate
- Repeat with known added inertia to create system of equations

### Progressive Testing Strategy

**Test sequence for separating motor and load:**

1. **Test Ferris wheel alone** (motor disconnected)
   - Falling weight test → J_wheel, b_wheel
   
2. **Test motor + Ferris wheel together**
   - Falling weight test → J_total, b_total
   
3. **Subtract to get motor contribution:**
   - J_motor_reflected = J_total - J_wheel
   - b_motor_reflected = b_total - b_wheel

**Expected results:**
- J_motor_reflected ≈ negligible (due to gear ratio²)
- b_motor_reflected ≈ significant (gearbox friction dominates)

## Model Validation

### One-Step-Ahead Prediction

Use identified model to predict next state:
```
x̂[k+1] = A·x[k] + B·u[k]
```

Compare to actual measurement x[k+1]

**Error metric:**
```
RMSE = sqrt(mean((x[k+1] - x̂[k+1])²))
```

**Variance Accounted For (VAF):**
```
VAF = 100·(1 - var(error) / var(signal))

VAF > 90% → Excellent model
VAF > 75% → Good model
VAF < 50% → Poor model
```

### Multi-Step-Ahead Prediction

More challenging test: predict multiple steps ahead using only initial condition and input sequence.

**Accumulates errors** - tests long-term model accuracy

### Frequency Response Validation

1. Apply sine wave input at frequency ω
2. Measure steady-state output amplitude and phase
3. Compute gain and phase from model
4. Compare across multiple frequencies (0.1-10 Hz)

**Bode plot:** Model vs. measured gain and phase

## Adaptive Control Applications

### Gain Scheduling

Update LQR gains based on identified parameters:
```
Q, R unchanged (cost function)
A, B updated from RLS parameters
Solve: Pₖ = Qₖ + AᵀPₖ₊₁A - AᵀPₖ₊₁B(R + BᵀPₖ₊₁B)⁻¹BᵀPₖ₊₁A
Controller: u = -K·x where K = (R + BᵀPB)⁻¹BᵀPA
```

Recompute K whenever parameters update significantly.

### Model Reference Adaptive Control (MRAC)

Force system to track reference model:
```
Reference: ẋₘ = Aₘ·xₘ + Bₘ·r
Actual: ẋ = A·x + B·u

Adaptation law updates controller to make x → xₘ
```

### Self-Tuning Regulator

1. RLS estimates parameters
2. Controller designed assuming estimates are true (certainty equivalence)
3. Adaptation and control operate simultaneously

## Practical Considerations

### Sampling Rate Selection

**Nyquist criterion:** Sample at ≥2× highest frequency of interest

**Motor dynamics:**
- Electrical bandwidth: f_e ≈ R/(2π·L) ≈ 500-2000 Hz
- Mechanical bandwidth: f_m ≈ b/(2π·J) ≈ 1-10 Hz

**Recommended rates:**
- Current measurement: ≥1 kHz
- Velocity/position: 100-500 Hz sufficient
- RLS updates: 10-100 Hz (decimated from current sampling)
- Control loop: 100-1000 Hz

### Sensor Noise Handling

**Current sensor:**
- Quantization noise from ADC
- Electrical noise from PWM switching
- Solution: Low-pass filter (hardware or digital)

**Encoder:**
- Position quantized to discrete counts
- Velocity from differentiation amplifies noise
- Solutions:
  1. Kalman filter for state estimation
  2. Low-pass filtered differentiation
  3. Measure position, estimate velocity

### Computational Budget

**RLS at 100 Hz:**
- 3×3 matrix operations
- ~30 floating-point operations per update
- Feasible on Arduino Uno, comfortable on STM32

**Optimization strategies:**
- Use fixed-point arithmetic (sacrifices precision)
- Exploit matrix symmetry (P is symmetric)
- Update RLS at lower rate than control loop
- Run RLS in background task/interrupt

### Parameter Bounds

**Physical constraints:**
```cpp
// Sanity checks
if (L < 0.001 || L > 0.1) L = L_default;
if (R < 1.0 || R > 100.0) R = R_default;
if (Kt < 0.001 || Kt > 1.0) Kt = Kt_default;
if (J < 0.0001 || J > 0.1) J = J_default;
if (b < 0.0 || b > 1.0) b = b_default;
```

Prevents nonsensical parameter estimates from corrupting control.

## Troubleshooting Guide

### RLS Not Converging

**Symptom:** Parameters don't stabilize, keep drifting

**Causes:**
1. Insufficient excitation
   - Solution: Use PRBS or multi-sine input
2. Forgetting factor too small (λ < 0.95)
   - Solution: Increase λ to 0.98-0.99
3. Initial covariance too small
   - Solution: Increase P₀ (try σ² = 1000)

### Parameter Estimates Unreasonable

**Symptom:** L = 50mH, R = 200Ω (way off expected range)

**Causes:**
1. Model mismatch (wrong equations)
2. Sensor calibration error
3. Timing error (Δt incorrect)
4. Numerical issues (P ill-conditioned)

**Solutions:**
- Verify sensor calibration with known inputs
- Check sample timing with oscilloscope
- Reset covariance periodically
- Add parameter bounds

### High Noise in Estimates

**Symptom:** Parameters jump around rapidly

**Causes:**
1. Measurement noise too high
2. Forgetting factor too small
3. Not enough data averaging

**Solutions:**
- Filter sensor measurements
- Increase λ (slower adaptation)
- Increase sampling rate and decimate

## Next Steps

### Implement Mechanical RLS

Extend RLS to mechanical subsystem:
```
y = Δω (velocity change)
φ = [i, -ω, -1]ᵀ (current, velocity, bias)
θ = [Δt·Kt/J, Δt·b/J, Δt·τ_load/J]ᵀ
```

Identifies J, b, and load torque simultaneously.

### Joint Electrical-Mechanical Estimation

Combine both subsystems:
```
θ = [L, R, Kt, J, b]ᵀ (5 parameters)
```

More complex but can exploit coupling between subsystems (Kt appears in both).

### Extended Kalman Filter (EKF)

Simultaneously estimate states and parameters:
```
x_augmented = [θ, ω, i, R, L, Kt, J, b]ᵀ
```

Parameters modeled as random walk. EKF provides state estimates and parameter estimates in unified framework.

### Load Torque Observer

Design observer for unknown load torque:
```
τ̂_load = J·α̂ - Kt·i + b·ω
```

Enables feedforward compensation for gravity or other disturbances.

## References

1. **RLS Algorithm:**
   - Ljung, L. (1999). *System Identification: Theory for the User*

2. **Adaptive Control:**
   - Åström, K. J., & Wittenmark, B. (1995). *Adaptive Control*

3. **Motor Control:**
   - Krishnan, R. (2001). *Electric Motor Drives*

4. **State-Space Methods:**
   - Franklin, G. F., Powell, J. D., & Emami-Naeini, A. (2015). *Feedback Control of Dynamic Systems*