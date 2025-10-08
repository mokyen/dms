# DMS Build Instructions

## Quick Start

### For STM32 NUCLEO-F411RE

```bash
# Build for STM32
pio run -e nucleo_f411re

# Upload to STM32
pio run -e nucleo_f411re -t upload

# Build, upload, and monitor
pio run -e nucleo_f411re -t upload -t monitor
```

### For Arduino Uno

```bash
# Build for Uno
pio run -e uno

# Upload to Uno
pio run -e uno -t upload

# Build, upload, and monitor
pio run -e uno -t upload -t monitor
```

## Compile-Time Features

### C++17 Features Used

Both targets now compile with **C++17** (`-std=gnu++17`), enabling:

- `constexpr` functions and classes
- `[[nodiscard]]` attributes for safer code
- Inline variables for static members
- `if constexpr` for compile-time branching (used in Config.h)
- Structured bindings (if needed)
- Fold expressions

### Board-Specific Compilation

The code automatically detects the target board and configures itself:

#### STM32 NUCLEO-F411RE (`-D NUCLEO_F411RE`)
```cpp
✓ 12-bit ADC (4095 max)
✓ 3.3V reference
✓ 100MHz CPU
✓ 115200 baud serial
✓ ENABLE_ADVANCED_CONTROL defined
✓ ENABLE_FLOAT_MATH defined
✓ DEBUG_VERBOSE enabled
✓ Performance monitoring enabled
✓ 1ms loop rate (1000Hz)
```

#### Arduino Uno (`-D ARDUINO_AVR_UNO`)
```cpp
✓ 10-bit ADC (1023 max)
✓ 5.0V reference
✓ 16MHz CPU
✓ 9600 baud serial
✓ AVOID_FLOAT_DIVISION defined
✓ Light debugging
✓ 10ms loop rate (100Hz)
```

## Compile-Time Constants

All critical math is done at compile time using `constexpr`:

```cpp
// Examples from Config.h
constexpr float ADC_VOLTS_PER_COUNT = 3.3f / 4095.0f;  // Computed at compile time
constexpr float COUNTS_PER_IN = 600.0f / 13.75f;       // Computed at compile time
constexpr float INV_COUNTS_PER_IN = 1.0f / COUNTS_PER_IN;  // Precomputed inverse

// Used in MotorDriver.h
float current = adcValue * ADC_TO_CURRENT;  // Single multiply, no division!
```

## Optimization Levels

Both targets use `-O2` optimization for good balance of:
- Code size
- Execution speed
- Debug-ability

To change optimization:
```ini
# In platformio.ini
build_flags = 
    -std=gnu++17
    -O3  # Maximum optimization (may increase code size)
```

## Code Size Comparison

After building, check code size:

```bash
# STM32
pio run -e nucleo_f411re --target size

# Arduino Uno
pio run -e uno --target size
```

**Expected sizes:**
- Arduino Uno: ~8-12KB of 32KB flash
- STM32 NUCLEO: ~15-25KB of 512KB flash (has room for much more!)

## Compile-Time Checks

The code includes compile-time assertions:

```cpp
// Example: Verify constants are sane
static_assert(MIN_DUTY_CYCLE < MAX_DUTY_CYCLE, "Invalid duty cycle range");
static_assert(MAX_TRAVEL_IN > 0, "Max travel must be positive");
static_assert(POSITION_TOLERANCE_IN > 0, "Tolerance must be positive");
```

## Build Flags Summary

### Common Flags (Both Targets)
```
-std=gnu++17          # C++17 standard with GNU extensions
-O2                   # Optimization level 2
-Wall                 # All warnings
```

### STM32-Specific
```
-D NUCLEO_F411RE
-D HAL_TIM_MODULE_ENABLED
-D HAL_ADC_MODULE_ENABLED
-Wextra              # Extra warnings (more headroom to fix)
```

### Arduino Uno-Specific
```
-D ARDUINO_AVR_UNO
```

## Debugging

### Serial Debugging

Both boards output debug info when `DEBUG_VERBOSE` is defined (auto on STM32):

```cpp
DEBUG_PRINT(F("Position: "));
DEBUG_PRINTLN(position);
```

### Hardware Debugging (STM32 only)

```bash
# Start debug session
pio debug -e nucleo_f411re

# In GDB:
(gdb) break setup
(gdb) continue
(gdb) print encoder.position
```

## Performance Testing

The STM32 version includes loop rate monitoring:

```
Performance: 952 Hz  # Actual loop frequency
```

This helps verify you have CPU headroom for advanced control algorithms.

## Migration Checklist

When moving from Uno to STM32:

- [ ] Code compiles for both targets without changes
- [ ] Current sensing calibrated for 3.3V reference
- [ ] Serial baud rate appropriate for each board
- [ ] PWM frequency verified (use oscilloscope)
- [ ] Encoder counting direction correct
- [ ] Position accuracy verified (measure actual travel)
- [ ] Current limit thresholds tested
- [ ] Timeout values appropriate for each board's speed

## Troubleshooting

### "undefined reference to..." errors
- Check that all `.cpp` files are in `src/` directory
- Verify `#include` statements are correct

### ADC readings wrong
- Verify voltage reference (5V vs 3.3V)
- Check `ADC_TO_CURRENT` constant in Config.h
- Use `motor.readVoltage()` to verify ADC is working

### Encoder not counting
- Check interrupt pins (must be pins 2, 3 on Uno)
- Verify `EncoderReader::attachInstance()` is called
- Test with `encoder.getPositionCounts()`

### Motor not moving
- Check PWM pin assignment
- Verify direction pins (INA, INB)
- Test with `motor.setSpeed(0.5f)` directly
- Check motor driver enable pin

### Compilation very slow
- First build downloads/compiles framework - this is normal
- Subsequent builds are incremental and fast
- Use `pio run` without `-t clean` for incremental builds

## Clean Build

If you encounter strange behavior:

```bash
# Clean everything
pio run -t clean

# Clean and rebuild
pio run -e nucleo_f411re -t clean
pio run -e nucleo_f411re
```

## Next Steps

1. **Test basic functions**: Upload to each board and test movement
2. **Calibrate current sensing**: Adjust `CURRENT_SENSE_CALIBRATION` in Config.h
3. **Tune motion profiles**: Adjust speed, acceleration in MotionProfiles.h
4. **Add advanced features**: Implement trapezoidal profile on STM32
5. **Create custom profiles**: Add to MotionProfiles.h namespace