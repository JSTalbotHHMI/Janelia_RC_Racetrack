#include <SPI.h>
#include "MCP4261.h"

// Teensy 4.0 / MCP4261 circle-drift controller
// Sequence:
// 1. Drive straight to build speed
// 2. Initiate a circular drift
// 3. Hold that drift for several laps
// 4. Exit the drift cleanly
// 5. Drive straight briefly and repeat

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

enum class CircleDirection : uint8_t {
  LEFT = 0,
  RIGHT
};

// Straight driving between drift sequences.
constexpr uint8_t THR_APPROACH_REQUEST = 96;
constexpr uint8_t THR_RESET_REQUEST    = 100;
constexpr uint8_t THR_IDLE_REQUEST     = 135;

// Circular drift throttle profile.
constexpr uint8_t THR_INIT_REQUEST     = 82;
constexpr uint8_t THR_CIRCLE_HIGH_REQ  = 86;
constexpr uint8_t THR_CIRCLE_LOW_REQ   = 92;
constexpr uint8_t THR_EXIT_REQUEST     = 106;

// Circle-drift timing (ms)
constexpr uint32_t APPROACH_MS         = 720;
constexpr uint32_t INITIATE_MS         = 180;
constexpr uint32_t CIRCLE_LAP_MS       = 680;
constexpr uint8_t  CIRCLE_LAPS         = 4;
constexpr uint32_t THROTTLE_PULSE_MS   = 120;
constexpr uint32_t EXIT_MS             = 240;
constexpr uint32_t RESET_STRAIGHT_MS   = 420;

// Steering percentages from center toward full lock.
constexpr uint8_t INIT_STEER_PCT       = 88;
constexpr uint8_t CIRCLE_STEER_PCT     = 70;
constexpr uint8_t EXIT_STEER_PCT       = 24;

// Output smoothing
constexpr uint32_t STEER_STEP_INTERVAL_MS    = 5;
constexpr uint8_t  STEER_STEP_SIZE           = 4;
constexpr uint32_t THROTTLE_STEP_INTERVAL_MS = 8;
constexpr uint8_t  THROTTLE_STEP_SIZE        = 2;

static_assert(STR_RIGHT <= STR_CENTER, "Invalid steering calibration");
static_assert(STR_CENTER <= STR_LEFT, "Invalid steering calibration");
static_assert(THR_FWDMAX <= THR_REVMAX, "Invalid throttle limits");
static_assert(STEER_STEP_SIZE > 0, "STEER_STEP_SIZE must be > 0");
static_assert(THROTTLE_STEP_SIZE > 0, "THROTTLE_STEP_SIZE must be > 0");

enum class MotionPhase : uint8_t {
  APPROACH = 0,
  INITIATE,
  CIRCLE,
  EXIT,
  RESET
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

constexpr uint8_t THR_APPROACH = clampThrottle(THR_APPROACH_REQUEST);
constexpr uint8_t THR_RESET    = clampThrottle(THR_RESET_REQUEST);
constexpr uint8_t THR_IDLE     = clampThrottle(THR_IDLE_REQUEST);
constexpr uint8_t THR_INIT     = clampThrottle(THR_INIT_REQUEST);
constexpr uint8_t THR_CIRCLE_H = clampThrottle(THR_CIRCLE_HIGH_REQ);
constexpr uint8_t THR_CIRCLE_L = clampThrottle(THR_CIRCLE_LOW_REQ);
constexpr uint8_t THR_EXIT     = clampThrottle(THR_EXIT_REQUEST);

static Outputs current  = { STR_CENTER, THR_IDLE };
static Outputs target   = { STR_CENTER, THR_IDLE };
static Outputs lastSent = { 0xFF, 0xFF };

static MotionPhase phase = MotionPhase::APPROACH;
static CircleDirection direction = CircleDirection::LEFT;
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

inline uint8_t steerIntoCircle(const uint8_t percent) {
  const uint8_t lock = (direction == CircleDirection::LEFT) ? STR_LEFT : STR_RIGHT;
  return blendToward(STR_CENTER, lock, percent);
}

inline uint32_t phaseDuration(const MotionPhase p) {
  switch (p) {
    case MotionPhase::APPROACH: return APPROACH_MS;
    case MotionPhase::INITIATE: return INITIATE_MS;
    case MotionPhase::CIRCLE:   return static_cast<uint32_t>(CIRCLE_LAP_MS) * CIRCLE_LAPS;
    case MotionPhase::EXIT:     return EXIT_MS;
    case MotionPhase::RESET:    return RESET_STRAIGHT_MS;
  }

  return APPROACH_MS;
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

void updateTargetsForPhase(const uint32_t now) {
  switch (phase) {
    case MotionPhase::APPROACH:
      setTargets(STR_CENTER, THR_APPROACH);
      break;

    case MotionPhase::INITIATE:
      setTargets(steerIntoCircle(INIT_STEER_PCT), THR_INIT);
      break;

    case MotionPhase::CIRCLE: {
      const uint32_t elapsed = now - phaseStartedAt;
      const bool pulseHigh = ((elapsed / THROTTLE_PULSE_MS) % 2U) == 0U;
      setTargets(steerIntoCircle(CIRCLE_STEER_PCT), pulseHigh ? THR_CIRCLE_H : THR_CIRCLE_L);
      break;
    }

    case MotionPhase::EXIT:
      setTargets(steerIntoCircle(EXIT_STEER_PCT), THR_EXIT);
      break;

    case MotionPhase::RESET:
      setTargets(STR_CENTER, THR_RESET);
      break;
  }
}

void beginPhase(const MotionPhase nextPhase, const uint32_t now) {
  phase = nextPhase;
  phaseStartedAt = now;
  updateTargetsForPhase(now);
}

void advanceDirection() {
  direction = (direction == CircleDirection::LEFT) ? CircleDirection::RIGHT : CircleDirection::LEFT;
}

void updatePhaseMachine(const uint32_t now) {
  if (phase == MotionPhase::CIRCLE) {
    updateTargetsForPhase(now);
  }

  if (!intervalElapsed(now, phaseStartedAt, phaseDuration(phase))) {
    return;
  }

  switch (phase) {
    case MotionPhase::APPROACH:
      beginPhase(MotionPhase::INITIATE, now);
      break;

    case MotionPhase::INITIATE:
      beginPhase(MotionPhase::CIRCLE, now);
      break;

    case MotionPhase::CIRCLE:
      beginPhase(MotionPhase::EXIT, now);
      break;

    case MotionPhase::EXIT:
      beginPhase(MotionPhase::RESET, now);
      break;

    case MotionPhase::RESET:
      advanceDirection();
      beginPhase(MotionPhase::APPROACH, now);
      break;
  }
}

void updateSteering(const uint32_t now) {
  if (!intervalElapsed(now, lastSteerStepAt, STEER_STEP_INTERVAL_MS)) {
    return;
  }

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

  Serial.begin(115200);
  while(!Serial)
  {
    delay(20);
  }

  const uint32_t now = millis();
  phaseStartedAt = now;
  lastSteerStepAt = now;
  lastThrottleStepAt = now;

  beginPhase(MotionPhase::APPROACH, now);
  current = target;
  applyOutputs();
}

void loop() {
  const uint32_t now = millis();

  updatePhaseMachine(now);
  updateSteering(now);
  updateThrottle(now);
  applyOutputs();
}
