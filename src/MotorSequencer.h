#pragma once
#include "MotorControlPid.h"
#include <functional>
#include <variant>

// Define the criteria types
struct TimeoutCriterion {
  unsigned long milliseconds;
};

struct CountCriterion {
  size_t iterations;
};

struct PredicateCriterion {
  std::function<bool()> condition;
};

// Variant type
using UpdateCriterion = std::variant<TimeoutCriterion, CountCriterion, PredicateCriterion>;

// Main function using std::visit with lambdas
inline void runUpdateUntil(MotorControl& controller, UpdateCriterion criterion) {
  std::visit([&controller](auto&& arg) {
    using T = std::decay_t<decltype(arg)>;
    
    if constexpr (std::is_same_v<T, TimeoutCriterion>) {
      unsigned long startMs = millis();
      while (millis() - startMs < arg.milliseconds) {
        controller.update();
        delay(1);
      }
    }
    else if constexpr (std::is_same_v<T, CountCriterion>) {
      for (size_t i = 0; i < arg.iterations; ++i) {
        controller.update();
        delay(1);
      }
    }
    else if constexpr (std::is_same_v<T, PredicateCriterion>) {
      while (!arg.condition()) {
        controller.update();
        delay(1);
      }
    }
  }, criterion);
}

// Helper factory functions
inline TimeoutCriterion timeout(unsigned long ms) { return {ms}; }
inline CountCriterion count(size_t n) { return {n}; }
inline PredicateCriterion predicate(std::function<bool()> fn) { return {fn}; }

// Convenience: Wait until arrived with timeout
inline void runUpdateUntilArrived(MotorControl& controller, unsigned long timeoutMs = 10000) {
  unsigned long startMs = millis();
  runUpdateUntil(controller, predicate([&controller, startMs, timeoutMs]() {
    return controller.hasArrived() || (millis() - startMs > timeoutMs);
  }));
}
