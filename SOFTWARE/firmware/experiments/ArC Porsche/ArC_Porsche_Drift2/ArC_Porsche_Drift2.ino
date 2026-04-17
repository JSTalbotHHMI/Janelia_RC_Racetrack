#include <SPI.h>
#include "MCP4261.h"

// Teensy 4.0 / MCP4261 drift controller
// Tuned for more controllable drifts:
// - Added a staged drift cycle (entry -> catch -> sustain -> recover -> settle)
// - Reduced full-lock steering usage so the rear can stay slipped without snapping
// - Smoothed both steering and throttle transitions
// - Kept fully non-blocking with millis()-based scheduling

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
// Lower value = more power forward.
constexpr uint8_t THR_ENTRY_REQUEST   = 82;
constexpr uint8_t THR_CATCH_REQUEST   = 88;
constexpr uint8_t THR_SUSTAIN_REQUEST = 92;
constexpr uint8_t THR_RECOVER_REQUEST = 108;
constexpr uint8_t THR_SETTLE_REQUEST  = 116;
constexpr uint8_t THR_IDLE_REQUEST    = 135;

// Drift timing (ms)
constexpr uint32_t DRIFT_ENTRY_MS   = 140;
constexpr uint32_t DRIFT_CATCH_MS   = 220;
constexpr uint32_t DRIFT_SUSTAIN_MS = 320;
constexpr uint32_t DRIFT_RECOVER_MS = 170;
constexpr uint32_t DRIFT_SETTLE_MS  = 120;

// Steering motion timing
constexpr uint32_t STEER_STEP_INTERVAL_MS = 5;
constexpr uint8_t STEER_STEP_SIZE = 4;

// Throttle motion timing
constexpr uint32_t THROTTLE_STEP_INTERVAL_MS = 8;
constexpr uint8_t THROTTLE_STEP_SIZE = 2;

// Steering targets as percentages from center toward the desired lock.
constexpr uint8_t ENTRY_STEER_PCT   = 82;
constexpr uint8_t CATCH_STEER_PCT   = 46;
constexpr uint8_t SUSTAIN_STEER_PCT = 34;

static_assert(STR_RIGHT <= STR_CENTER, "Invalid steering calibration");
static_assert(STR_CENTER <= STR_LEFT, "Invalid steering calibration");
static_assert(THR_FWDMAX <= THR_REVMAX, "Invalid throttle limits");
static_assert(STEER_STEP_SIZE > 0, "STEER_STEP_SIZE must be > 0");
static_assert(THROTTLE_STEP_SIZE > 0, "THROTTLE_STEP_SIZE must be > 0");

enum class DriftPhase : uint8_t {
  ENTRY = 0,
  CATCH,
  SUSTAIN,
  RECOVER,
  SETTLE
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

constexpr uint8_t blendToward(const uint8_t from, const uint8_t to, const uint8_t percent) {
  const int delta = static_cast<int>(to) - static_cast<int>(from);
  return static_cast<uint8_t>(from + (delta * percent) / 100);
}

// Clamp requested throttle values at compile time.
constexpr uint8_t THR_ENTRY   = clampThrottle(THR_ENTRY_REQUEST);
constexpr uint8_t THR_CATCH   = clampThrottle(THR_CATCH_REQUEST);
constexpr uint8_t THR_SUSTAIN = clampThrottle(THR_SUSTAIN_REQUEST);
constexpr uint8_t THR_RECOVER = clampThrottle(THR_RECOVER_REQUEST);
constexpr uint8_t THR_SETTLE  = clampThrottle(THR_SETTLE_REQUEST);
constexpr uint8_t THR_IDLE    = clampThrottle(THR_IDLE_REQUEST);

static Outputs current  = { STR_CENTER, THR_IDLE };
static Outputs target   = { STR_CENTER, THR_IDLE };
static Outputs lastSent = { 0xFF, 0xFF }; // Force first write.

static DriftPhase phase = DriftPhase::ENTRY;
static DriftDirection direction = DriftDirection::LEFT;

static uint32_t phaseStartedAt = 0;
static uint32_t lastSteerStepAt = 0;
static uint32_t lastThrottleStepAt = 0;

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

inline uint8_t steeringToward(const DriftDirection dir, const uint8_t percent, const bool counterSteer) {
  const bool goLeft = (dir == DriftDirection::LEFT) != counterSteer;
  const uint8_t lock = goLeft ? STR_LEFT : STR_RIGHT;
  return blendToward(STR_CENTER, lock, percent);
}

inline uint32_t phaseDuration(const DriftPhase p) {
  switch (p) {
    case DriftPhase::ENTRY:   return DRIFT_ENTRY_MS;
    case DriftPhase::CATCH:   return DRIFT_CATCH_MS;
    case DriftPhase::SUSTAIN: return DRIFT_SUSTAIN_MS;
    case DriftPhase::RECOVER: return DRIFT_RECOVER_MS;
    case DriftPhase::SETTLE:  return DRIFT_SETTLE_MS;
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
      setTargets(steeringToward(direction, ENTRY_STEER_PCT, false), THR_ENTRY);
      break;

    case DriftPhase::CATCH:
      setTargets(steeringToward(direction, CATCH_STEER_PCT, true), THR_CATCH);
      break;

    case DriftPhase::SUSTAIN:
      setTargets(steeringToward(direction, SUSTAIN_STEER_PCT, true), THR_SUSTAIN);
      break;

    case DriftPhase::RECOVER:
      setTargets(STR_CENTER, THR_RECOVER);
      break;

    case DriftPhase::SETTLE:
      setTargets(STR_CENTER, THR_SETTLE);
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
      beginPhase(DriftPhase::CATCH, now);
      break;

    case DriftPhase::CATCH:
      beginPhase(DriftPhase::SUSTAIN, now);
      break;

    case DriftPhase::SUSTAIN:
      beginPhase(DriftPhase::RECOVER, now);
      break;

    case DriftPhase::RECOVER:
      beginPhase(DriftPhase::SETTLE, now);
      break;

    case DriftPhase::SETTLE:
      advanceDirection();
      beginPhase(DriftPhase::ENTRY, now);
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

void updateThrottle(const uint32_t now) {
  if (!intervalElapsed(now, lastThrottleStepAt, THROTTLE_STEP_INTERVAL_MS)) {
    return;
  }

  lastThrottleStepAt += THROTTLE_STEP_INTERVAL_MS;
  current.throttle = stepToward(current.throttle, target.throttle, THROTTLE_STEP_SIZE);
}

void setup() {
  pinMode(WP_PIN, OUTPUT);
  digitalWrite(WP_PIN, HIGH); // Disable write protect.

  SPI.begin();
  pot.begin();

  delay(5000);

  const uint32_t now = millis();
  phaseStartedAt = now;
  lastSteerStepAt = now;
  lastThrottleStepAt = now;

  beginPhase(DriftPhase::ENTRY, now);
  current = target; // Start from a known valid output state.
  applyOutputs();
}

void loop() {
  const uint32_t now = millis();

  updatePhaseMachine(now);
  updateSteering(now);
  updateThrottle(now);
  applyOutputs();
}
