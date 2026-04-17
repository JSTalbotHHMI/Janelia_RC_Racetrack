#include <SPI.h>
#include "MCP4261.h"

// Teensy 4.0 / MCP4261 drift controller
// Improvements:
// - Kept fully non-blocking with millis()-based scheduling
// - Reduced repeated branching by centralizing phase targets/durations
// - Writes to the digital pot only when values change
// - Added wrap-safe time helpers and clamped output helpers
// - Smoothed steering with fixed-rate stepping, kept throttle immediate

constexpr uint8_t WP_PIN = 7;

// MCP4261 channels
constexpr uint8_t STEERING_CH = 0;
constexpr uint8_t THROTTLE_CH = 1;

// Software SPI wiring: select, shutdown, dataIn, dataOut, clock
MCP4261 pot(10, 6, 11, 12, 13);

// Optional HW SPI if rewired:
// MCP4261 pot(10, 6, &SPI);

// Steering calibration
constexpr uint8_t STR_RIGHT  = 0;
constexpr uint8_t STR_CENTER = 150;
constexpr uint8_t STR_LEFT   = 255;

// Throttle calibration
// Lower value = more forward, higher value = more reverse
constexpr uint8_t THR_FWDMAX = 90;
constexpr uint8_t THR_REVMAX = 160;

// Requested throttle setpoints
constexpr uint8_t THR_DRIFT_REQUEST = 84;
constexpr uint8_t THR_HOLD_REQUEST  = 80;
constexpr uint8_t THR_IDLE_REQUEST  = 135;

// Drift timing (ms)
constexpr uint32_t DRIFT_ENTRY_MS   = 320;
constexpr uint32_t DRIFT_COUNTER_MS = 360;
constexpr uint32_t DRIFT_RECOVER_MS = 200;

// Steering motion timing
constexpr uint32_t STEER_STEP_INTERVAL_MS = 6;
constexpr uint8_t STEER_STEP_SIZE = 3;

static_assert(STR_RIGHT <= STR_CENTER, "Invalid steering calibration");
static_assert(STR_CENTER <= STR_LEFT, "Invalid steering calibration");
static_assert(THR_FWDMAX <= THR_REVMAX, "Invalid throttle limits");
static_assert(STEER_STEP_SIZE > 0, "STEER_STEP_SIZE must be > 0");

enum class DriftPhase : uint8_t {
  ENTRY = 0,
  COUNTER,
  RECOVER
};

enum class DriftDirection : uint8_t {
  LEFT = 0,
  RIGHT
};

struct Outputs {
  uint8_t steering;
  uint8_t throttle;
};

constexpr uint8_t clampU8(const int value, const uint8_t minValue, const uint8_t maxValue) {
  return (value < minValue) ? minValue
       : (value > maxValue) ? maxValue
       : static_cast<uint8_t>(value);
}

constexpr uint8_t clampSteering(const int value) {
  return clampU8(value, STR_RIGHT, STR_LEFT);
}

constexpr uint8_t clampThrottle(const int value) {
  return clampU8(value, THR_FWDMAX, THR_REVMAX);
}

// Clamp requested throttle values at compile time.
constexpr uint8_t THR_DRIFT = clampThrottle(THR_DRIFT_REQUEST);
constexpr uint8_t THR_HOLD  = clampThrottle(THR_HOLD_REQUEST);
constexpr uint8_t THR_IDLE  = clampThrottle(THR_IDLE_REQUEST);

static Outputs current  = { STR_CENTER, THR_IDLE };
static Outputs target   = { STR_CENTER, THR_IDLE };
static Outputs lastSent = { 0xFF, 0xFF }; // Force first write.

static DriftPhase phase = DriftPhase::ENTRY;
static DriftDirection direction = DriftDirection::LEFT;

static uint32_t phaseStartedAt = 0;
static uint32_t lastSteerStepAt = 0;

inline bool intervalElapsed(const uint32_t now, const uint32_t since, const uint32_t interval) {
  return static_cast<uint32_t>(now - since) >= interval;
}

inline uint8_t stepToward(const uint8_t from, const uint8_t to, const uint8_t stepSize) {
  if (from < to) {
    const uint8_t delta = to - from;
    return from + ((delta > stepSize) ? stepSize : delta);
  }

  if (from > to) {
    const uint8_t delta = from - to;
    return from - ((delta > stepSize) ? stepSize : delta);
  }

  return from;
}

inline uint8_t steeringFor(const DriftDirection dir, const bool counterSteer) {
  if (dir == DriftDirection::LEFT) {
    return counterSteer ? STR_RIGHT : STR_LEFT;
  }
  return counterSteer ? STR_LEFT : STR_RIGHT;
}

inline uint32_t phaseDuration(const DriftPhase p) {
  switch (p) {
    case DriftPhase::ENTRY:   return DRIFT_ENTRY_MS;
    case DriftPhase::COUNTER: return DRIFT_COUNTER_MS;
    case DriftPhase::RECOVER: return DRIFT_RECOVER_MS;
  }

  // Defensive fallback for unexpected enum corruption.
  return DRIFT_ENTRY_MS;
}

void writePotIfChanged(const uint8_t channel, const uint8_t value, uint8_t &lastValue) {
  if (value != lastValue) {
    pot.setValue(channel, value);
    lastValue = value;
  }
}

void applyOutputs() {
  writePotIfChanged(THROTTLE_CH, current.throttle, lastSent.throttle);
  writePotIfChanged(STEERING_CH, current.steering, lastSent.steering);
}

void setTargets(const uint8_t steering, const uint8_t throttle) {
  target.steering = clampSteering(steering);
  target.throttle = clampThrottle(throttle);
}

void updateTargetsForPhase() {
  switch (phase) {
    case DriftPhase::ENTRY:
      setTargets(steeringFor(direction, false), THR_DRIFT);
      break;

    case DriftPhase::COUNTER:
      setTargets(steeringFor(direction, true), THR_HOLD);
      break;

    case DriftPhase::RECOVER:
      setTargets(STR_CENTER, THR_IDLE);
      break;
  }
}

void beginPhase(const DriftPhase nextPhase, const uint32_t now) {
  phase = nextPhase;
  phaseStartedAt = now;
  updateTargetsForPhase();
}

void advanceDirection() {
  direction = (direction == DriftDirection::LEFT) ? DriftDirection::RIGHT : DriftDirection::LEFT;
}

void updatePhaseMachine(const uint32_t now) {
  if (!intervalElapsed(now, phaseStartedAt, phaseDuration(phase))) {
    return;
  }

  switch (phase) {
    case DriftPhase::ENTRY:
      beginPhase(DriftPhase::COUNTER, now);
      break;

    case DriftPhase::COUNTER:
      beginPhase(DriftPhase::RECOVER, now);
      break;

    case DriftPhase::RECOVER:
      advanceDirection();
      beginPhase(DriftPhase::ENTRY, now); // No pause between drifts.
      break;
  }
}

void updateSteering(const uint32_t now) {
  if (!intervalElapsed(now, lastSteerStepAt, STEER_STEP_INTERVAL_MS)) {
    return;
  }

  // Advance by fixed intervals to keep steering cadence stable.
  lastSteerStepAt += STEER_STEP_INTERVAL_MS;
  current.steering = stepToward(current.steering, target.steering, STEER_STEP_SIZE);
}

void updateThrottle() {
  // Throttle is intentionally immediate and predictable.
  current.throttle = target.throttle;
}

void setup() {
  pinMode(WP_PIN, OUTPUT);
  digitalWrite(WP_PIN, HIGH); // Disable write protect.

  SPI.begin();
  pot.begin();

  const uint32_t now = millis();
  phaseStartedAt = now;
  lastSteerStepAt = now;

  beginPhase(DriftPhase::ENTRY, now);
  current = target; // Start from a known valid output state.
  applyOutputs();
}

void loop() {
  const uint32_t now = millis();

  updatePhaseMachine(now);
  updateSteering(now);
  updateThrottle();
  applyOutputs();
}