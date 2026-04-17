#include <SPI.h>
#include <stdlib.h>
#include <string.h>
#include "MCP4261.h"

// Teensy 4.0 / MCP4261 path follower
//
// Expected serial packet from Python, newline terminated:
//   ERR,<speed_error_mps>,<heading_error_rad>
//
// Example:
//   ERR,-0.18,0.12
//
// Sign assumptions used by the controller:
// - speed_error_mps   = current_speed - ideal_speed
//   Positive means the car is too fast and should reduce throttle.
// - heading_error_rad = current_heading - ideal_heading
//   Positive means the car is rotated too far left / CCW and should steer right.

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

// Base drive command while following the path.
constexpr uint8_t THR_BASE_REQUEST     = 104;
constexpr uint8_t THR_FAILSAFE_REQUEST = 118;

// Closed-loop gains.
// Right turns can need more command on small RC chassis, so the gains are split.
constexpr float STEER_GAIN_LEFT_COUNTS_PER_RAD  = 58.0f;
constexpr float STEER_GAIN_RIGHT_COUNTS_PER_RAD = 92.0f;
constexpr float STEER_RIGHT_BIAS_COUNTS         = 12.0f;
constexpr float THROTTLE_GAIN_COUNTS_PER_MPS   = 20.0f;
constexpr float THROTTLE_HEADING_DERATE_PER_RAD = 8.0f;

// Clamp the maximum correction so one bad packet cannot command full lock.
constexpr float MAX_HEADING_ERROR_RAD = 3.2f;
constexpr float MAX_SPEED_ERROR_MPS   = 2.0f;

// Serial and output timing
constexpr uint32_t SERIAL_BAUDRATE         = 115200;
constexpr uint32_t SERIAL_TIMEOUT_MS       = 250;
constexpr uint32_t STATUS_PRINT_INTERVAL_MS = 100;
constexpr size_t SERIAL_LINE_BUFFER_SIZE   = 64;

constexpr uint32_t STEER_STEP_INTERVAL_MS    = 5;
constexpr uint8_t  STEER_STEP_SIZE           = 4;
constexpr uint32_t THROTTLE_STEP_INTERVAL_MS = 8;
constexpr uint8_t  THROTTLE_STEP_SIZE        = 2;

static_assert(STR_RIGHT <= STR_CENTER, "Invalid steering calibration");
static_assert(STR_CENTER <= STR_LEFT, "Invalid steering calibration");
static_assert(THR_FWDMAX <= THR_REVMAX, "Invalid throttle limits");
static_assert(STEER_STEP_SIZE > 0, "STEER_STEP_SIZE must be > 0");
static_assert(THROTTLE_STEP_SIZE > 0, "THROTTLE_STEP_SIZE must be > 0");

struct Outputs {
  uint8_t steering;
  uint8_t throttle;
};

struct ErrorPacket {
  float speedErrorMps;
  float headingErrorRad;
  uint32_t receivedAtMs;
  bool valid;
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

float clampFloat(const float value, const float minValue, const float maxValue) {
  if (value < minValue) {
    return minValue;
  }

  if (value > maxValue) {
    return maxValue;
  }

  return value;
}

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

static Outputs current  = { STR_CENTER, clampThrottle(THR_FAILSAFE_REQUEST) };
static Outputs target   = { STR_CENTER, clampThrottle(THR_FAILSAFE_REQUEST) };
static Outputs lastSent = { 0xFF, 0xFF };

static ErrorPacket latestErrors = { 0.0f, 0.0f, 0, false };

static char serialLine[SERIAL_LINE_BUFFER_SIZE];
static size_t serialLineLength = 0;

static uint32_t lastSteerStepAt = 0;
static uint32_t lastThrottleStepAt = 0;
static uint32_t lastStatusPrintAt = 0;

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

bool parseErrorLine(char *line, float &speedErrorMps, float &headingErrorRad) {
  char *cursor = line;

  if (strncmp(cursor, "ERR,", 4) == 0) {
    cursor += 4;
  }

  char *firstSeparator = strchr(cursor, ',');
  if (firstSeparator == nullptr) {
    return false;
  }

  *firstSeparator = '\0';
  char *speedString = cursor;
  char *headingString = firstSeparator + 1;

  char *speedEnd = nullptr;
  char *headingEnd = nullptr;

  speedErrorMps = strtof(speedString, &speedEnd);
  headingErrorRad = strtof(headingString, &headingEnd);

  if (speedEnd == speedString || headingEnd == headingString) {
    return false;
  }

  while (*speedEnd == ' ' || *speedEnd == '\t') {
    ++speedEnd;
  }

  while (*headingEnd == ' ' || *headingEnd == '\t') {
    ++headingEnd;
  }

  return (*speedEnd == '\0') && (*headingEnd == '\0');
}

void updateErrorsFromSerial(const uint32_t now) {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      serialLine[serialLineLength] = '\0';

      if (serialLineLength > 0) {
        float speedErrorMps = 0.0f;
        float headingErrorRad = 0.0f;

        if (parseErrorLine(serialLine, speedErrorMps, headingErrorRad)) {
          latestErrors.speedErrorMps = clampFloat(speedErrorMps, -MAX_SPEED_ERROR_MPS, MAX_SPEED_ERROR_MPS);
          latestErrors.headingErrorRad = clampFloat(headingErrorRad, -MAX_HEADING_ERROR_RAD, MAX_HEADING_ERROR_RAD);
          latestErrors.receivedAtMs = now;
          latestErrors.valid = true;
        }
      }

      serialLineLength = 0;
      continue;
    }

    if (serialLineLength < (SERIAL_LINE_BUFFER_SIZE - 1U)) {
      serialLine[serialLineLength++] = c;
    } else {
      serialLineLength = 0;
    }
  }
}

bool errorsAreFresh(const uint32_t now) {
  return latestErrors.valid && intervalElapsed(now, latestErrors.receivedAtMs, SERIAL_TIMEOUT_MS) == false;
}

void updateTargetsFromErrors(const uint32_t now) {
  if (!errorsAreFresh(now)) {
    setTargets(STR_CENTER, clampThrottle(THR_FAILSAFE_REQUEST));
    return;
  }

  const float headingError = latestErrors.headingErrorRad;
  const float speedError = latestErrors.speedErrorMps;
  const float absHeadingError = (headingError >= 0.0f) ? headingError : -headingError;

  float steeringCorrectionCounts = 0.0f;
  if (headingError >= 0.0f) {
    steeringCorrectionCounts =
      (headingError * STEER_GAIN_RIGHT_COUNTS_PER_RAD) + STEER_RIGHT_BIAS_COUNTS;
  } else {
    steeringCorrectionCounts = headingError * STEER_GAIN_LEFT_COUNTS_PER_RAD;
  }

  const int steeringCommand =
    static_cast<int>(STR_CENTER - steeringCorrectionCounts);

  const int throttleCommand =
    static_cast<int>(
      THR_BASE_REQUEST
      + (speedError * THROTTLE_GAIN_COUNTS_PER_MPS)
      + (absHeadingError * THROTTLE_HEADING_DERATE_PER_RAD)
    );

  setTargets(
    clampSteering(steeringCommand),
    clampThrottle(throttleCommand)
  );
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

void printStatus(const uint32_t now) {
  if (!intervalElapsed(now, lastStatusPrintAt, STATUS_PRINT_INTERVAL_MS)) {
    return;
  }

  lastStatusPrintAt = now;

  Serial.print("STATUS,fresh=");
  Serial.print(errorsAreFresh(now) ? 1 : 0);
  Serial.print(",speed_err=");
  Serial.print(latestErrors.speedErrorMps, 4);
  Serial.print(",heading_err=");
  Serial.print(latestErrors.headingErrorRad, 4);
  Serial.print(",steer=");
  Serial.print(target.steering);
  Serial.print(",thr=");
  Serial.println(target.throttle);
}

void setup() {
  pinMode(WP_PIN, OUTPUT);
  digitalWrite(WP_PIN, HIGH); // Disable write protect.

  SPI.begin();
  pot.begin();

  Serial.begin(SERIAL_BAUDRATE);

  const uint32_t now = millis();
  lastSteerStepAt = now;
  lastThrottleStepAt = now;
  lastStatusPrintAt = now;

  current = target;
  applyOutputs();
}

void loop() {
  const uint32_t now = millis();

  updateErrorsFromSerial(now);
  updateTargetsFromErrors(now);
  updateSteering(now);
  updateThrottle(now);
  applyOutputs();
  printStatus(now);
}
