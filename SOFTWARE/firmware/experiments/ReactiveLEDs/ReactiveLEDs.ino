// ReactiveLEDs.ino
//
// Teensy + WS2812B reactive track lighting
//
// Serial input format (one car update per line):
//   CAR,<carId>,<x>,<y>,<travelDeg>,<headingDeg>
//
// Example:
//   CAR,2,148.5,42.0,90.0,118.0
//
// Coordinate units are arbitrary as long as your incoming serial data uses
// the same coordinate system as the patch definitions below.
//
// If a car enters a patch, that patch's LED segment flashes.
// If the same car is considered drifting while inside that patch, the patch
// switches to a moving rainbow animation instead.
// If a wall-adjacent patch sees a sudden change in travel direction, it
// erupts into a brief explosion / fire animation.

#include <FastLED.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

namespace Config {
  static const uint8_t LED_PIN = 2;
  static const EOrder COLOR_ORDER = GRB;
  static const uint8_t MAX_BRIGHTNESS = 180;
  static const uint32_t SERIAL_BAUD = 115200;

  // Treat heading disagreement above this many degrees as drifting.
  static const float DRIFT_THRESHOLD_DEG = 18.0f;

  // A trigger lingers briefly so occasional dropped serial lines do not
  // immediately turn a patch off.
  static const uint32_t PATCH_HOLD_MS = 220;
  static const uint32_t CAR_STALE_MS = 250;

  // Fade inactive segments toward black.
  static const uint8_t FADE_STEP = 18;

  // Flash animation tuning.
  static const uint8_t FLASH_BPM = 90;
  static const CRGB FLASH_COLOR_A = CRGB(255, 180, 40);
  static const CRGB FLASH_COLOR_B = CRGB(255, 40, 10);

  // Rainbow animation tuning for drifting.
  static const uint8_t RAINBOW_STEP = 9;
  static const uint8_t RAINBOW_SPEED = 5;

  // Treat a sudden change in travel direction as a wall hit.
  static const float WALL_HIT_MIN_DELTA_DEG = 22.0f;
  static const float WALL_HIT_RATE_DEG_PER_SEC = 700.0f;
  static const uint32_t WALL_HIT_MAX_SAMPLE_MS = 120;

  // Explosion / fire animation tuning.
  static const uint32_t IMPACT_HOLD_MS = 650;
  static const uint32_t IMPACT_EXPAND_MS = 180;
  static const uint32_t IMPACT_COOLDOWN_MS = 180;
}

static const uint8_t SEGMENT_COUNT = 3;
static const uint16_t SEGMENT_LENGTHS[SEGMENT_COUNT] = {
  14,  // Segment 0
  23,  // Segment 1
  11   // Segment 2
};

struct Segment {
  uint16_t start;
  uint16_t length;
};

enum PatchShape : uint8_t {
  PATCH_CIRCLE,
  PATCH_QUAD
};

struct Point2D {
  float x;
  float y;
};

struct Patch {
  const char* name;
  PatchShape shape;
  uint8_t segmentIndex;
  bool wallReactive;

  // Circle settings
  Point2D center;
  float radius;

  // Quad settings (points should be ordered around the perimeter)
  Point2D quad[4];
};

struct PatchRuntimeState {
  uint32_t lastTriggerMs;
  uint32_t lastImpactMs;
  bool driftActive;
};

struct CarState {
  bool inUse;
  uint8_t carId;
  Point2D position;
  float travelDeg;
  float headingDeg;
  float previousTravelDeg;
  uint32_t previousUpdateMs;
  bool hasPreviousTravel;
  uint32_t lastUpdateMs;
};

// ---------------------------------------------------------------------------
// Patch definitions
// Replace these coordinates with your real track coordinates.
// The first patch uses a circular trigger area.
// The second and third patches use convex quadrilaterals.
// ---------------------------------------------------------------------------
static Patch patches[] = {
  {
    "Front Straight Burst",
    PATCH_CIRCLE,
    0,
    true,
    {120.0f, 35.0f},
    20.0f,
    {{0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}}
  },
  {
    "Infield Drift Box",
    PATCH_QUAD,
    1,
    false,
    {0.0f, 0.0f},
    0.0f,
    {{180.0f, 75.0f}, {225.0f, 70.0f}, {235.0f, 105.0f}, {190.0f, 112.0f}}
  },
  {
    "Hairpin Exit Box",
    PATCH_QUAD,
    2,
    true,
    {0.0f, 0.0f},
    0.0f,
    {{70.0f, 140.0f}, {102.0f, 138.0f}, {115.0f, 172.0f}, {78.0f, 178.0f}}
  }
};

static const uint8_t PATCH_COUNT = sizeof(patches) / sizeof(patches[0]);
static const uint16_t MAX_LED_COUNT = 256;
static const uint8_t MAX_CARS = 12;

static Segment segments[SEGMENT_COUNT];
static PatchRuntimeState patchState[PATCH_COUNT];
static CarState cars[MAX_CARS];

static uint16_t totalLedCount = 0;
static CRGB leds[MAX_LED_COUNT];
static char serialLine[96];
static uint8_t serialIndex = 0;
static uint8_t rainbowBaseHue = 0;

static float wrapAngleDeg(float angleDeg) {
  while (angleDeg > 180.0f) {
    angleDeg -= 360.0f;
  }
  while (angleDeg < -180.0f) {
    angleDeg += 360.0f;
  }
  return angleDeg;
}

static float angularDifferenceDeg(float aDeg, float bDeg) {
  return fabsf(wrapAngleDeg(aDeg - bDeg));
}

static bool pointInCircle(const Point2D& point, const Patch& patch) {
  const float dx = point.x - patch.center.x;
  const float dy = point.y - patch.center.y;
  return (dx * dx + dy * dy) <= (patch.radius * patch.radius);
}

static float cross2D(const Point2D& a, const Point2D& b, const Point2D& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

static bool pointInConvexQuad(const Point2D& point, const Patch& patch) {
  bool sawPositive = false;
  bool sawNegative = false;

  for (uint8_t i = 0; i < 4; ++i) {
    const Point2D& a = patch.quad[i];
    const Point2D& b = patch.quad[(i + 1) % 4];
    const float cross = cross2D(a, b, point);

    if (cross > 0.0f) {
      sawPositive = true;
    } else if (cross < 0.0f) {
      sawNegative = true;
    }

    if (sawPositive && sawNegative) {
      return false;
    }
  }

  return true;
}

static bool pointInPatch(const Point2D& point, const Patch& patch) {
  if (patch.shape == PATCH_CIRCLE) {
    return pointInCircle(point, patch);
  }

  return pointInConvexQuad(point, patch);
}

static void buildSegments() {
  uint16_t cursor = 0;
  for (uint8_t i = 0; i < SEGMENT_COUNT; ++i) {
    segments[i].start = cursor;
    segments[i].length = SEGMENT_LENGTHS[i];
    cursor += SEGMENT_LENGTHS[i];
  }
  totalLedCount = cursor;
}

static void fillSegment(uint8_t segmentIndex, const CRGB& color) {
  const Segment& seg = segments[segmentIndex];
  for (uint16_t i = 0; i < seg.length; ++i) {
    leds[seg.start + i] = color;
  }
}

static void flashSegment(uint8_t segmentIndex, uint32_t nowMs) {
  const uint8_t phase = beatsin8(Config::FLASH_BPM, 0, 255, 0, 0);
  const CRGB blended = blend(Config::FLASH_COLOR_B, Config::FLASH_COLOR_A, phase);
  fillSegment(segmentIndex, blended);

  // Add a narrow bright pulse running through the segment for extra motion.
  const Segment& seg = segments[segmentIndex];
  const uint16_t pulseIndex = map(nowMs % 1000, 0, 999, 0, seg.length - 1);
  leds[seg.start + pulseIndex] += CRGB::White;
}

static void rainbowSegment(uint8_t segmentIndex) {
  const Segment& seg = segments[segmentIndex];
  for (uint16_t i = 0; i < seg.length; ++i) {
    leds[seg.start + i] = CHSV(rainbowBaseHue + (i * Config::RAINBOW_STEP), 255, 255);
  }
}

static bool carLikelyHitWall(const CarState& car) {
  if (!car.hasPreviousTravel) {
    return false;
  }

  const uint32_t deltaMs = car.lastUpdateMs - car.previousUpdateMs;
  if (deltaMs == 0 || deltaMs > Config::WALL_HIT_MAX_SAMPLE_MS) {
    return false;
  }

  const float deltaDeg = angularDifferenceDeg(car.travelDeg, car.previousTravelDeg);
  if (deltaDeg < Config::WALL_HIT_MIN_DELTA_DEG) {
    return false;
  }

  const float turnRateDegPerSec = (deltaDeg * 1000.0f) / static_cast<float>(deltaMs);
  return turnRateDegPerSec >= Config::WALL_HIT_RATE_DEG_PER_SEC;
}

static void impactSegment(uint8_t segmentIndex, uint32_t impactAgeMs, uint32_t nowMs) {
  const Segment& seg = segments[segmentIndex];
  if (seg.length == 0) {
    return;
  }

  const uint16_t center = seg.length / 2;
  uint16_t radius = seg.length;
  if (impactAgeMs < Config::IMPACT_EXPAND_MS) {
    radius = map(impactAgeMs, 0, Config::IMPACT_EXPAND_MS, 0, seg.length);
  }

  const uint8_t blastStrength = map(
    min(impactAgeMs, Config::IMPACT_HOLD_MS),
    0,
    Config::IMPACT_HOLD_MS,
    255,
    40
  );

  for (uint16_t i = 0; i < seg.length; ++i) {
    const uint16_t dist = (i > center) ? (i - center) : (center - i);
    uint8_t heat = 0;

    if (dist <= radius) {
      const uint16_t radialFalloff = (radius == 0) ? 0 : (dist * 180U) / radius;
      heat = qsub8(blastStrength, static_cast<uint8_t>(radialFalloff));
    } else {
      heat = scale8(blastStrength, 80);
    }

    const uint8_t flicker = sin8(static_cast<uint8_t>(i * 37U + (nowMs >> 1)));
    heat = qadd8(heat, scale8(flicker, 70));

    CRGB color = HeatColor(heat);

    if (impactAgeMs < 130 && dist <= radius + 1 && flicker > 230) {
      color += CRGB(255, 220, 160);
    }

    leds[seg.start + i] = color;
  }
}

static int8_t findCarIndex(uint8_t carId) {
  for (uint8_t i = 0; i < MAX_CARS; ++i) {
    if (cars[i].inUse && cars[i].carId == carId) {
      return static_cast<int8_t>(i);
    }
  }

  for (uint8_t i = 0; i < MAX_CARS; ++i) {
    if (!cars[i].inUse) {
      return static_cast<int8_t>(i);
    }
  }

  return -1;
}

static void refreshPatchStateFromCars(uint32_t nowMs) {
  for (uint8_t i = 0; i < PATCH_COUNT; ++i) {
    patchState[i].driftActive = false;
  }

  for (uint8_t carIndex = 0; carIndex < MAX_CARS; ++carIndex) {
    if (!cars[carIndex].inUse) {
      continue;
    }

    const uint32_t ageMs = nowMs - cars[carIndex].lastUpdateMs;
    if (ageMs > Config::CAR_STALE_MS) {
      cars[carIndex].inUse = false;
      continue;
    }

    const bool drifting =
      angularDifferenceDeg(cars[carIndex].travelDeg, cars[carIndex].headingDeg) >= Config::DRIFT_THRESHOLD_DEG;
    const bool wallHit = carLikelyHitWall(cars[carIndex]);

    for (uint8_t patchIndex = 0; patchIndex < PATCH_COUNT; ++patchIndex) {
      if (!pointInPatch(cars[carIndex].position, patches[patchIndex])) {
        continue;
      }

      patchState[patchIndex].lastTriggerMs = nowMs;
      if (drifting) {
        patchState[patchIndex].driftActive = true;
      }

      if (patches[patchIndex].wallReactive &&
          wallHit &&
          (nowMs - patchState[patchIndex].lastImpactMs) > Config::IMPACT_COOLDOWN_MS) {
        patchState[patchIndex].lastImpactMs = nowMs;
      }
    }
  }
}

static void renderPatches(uint32_t nowMs) {
  refreshPatchStateFromCars(nowMs);
  fadeToBlackBy(leds, totalLedCount, Config::FADE_STEP);

  for (uint8_t i = 0; i < PATCH_COUNT; ++i) {
    const uint32_t impactAgeMs = nowMs - patchState[i].lastImpactMs;
    if (patchState[i].lastImpactMs != 0 && impactAgeMs <= Config::IMPACT_HOLD_MS) {
      impactSegment(patches[i].segmentIndex, impactAgeMs, nowMs);
      continue;
    }

    const uint32_t ageMs = nowMs - patchState[i].lastTriggerMs;
    if (ageMs > Config::PATCH_HOLD_MS) {
      patchState[i].driftActive = false;
      continue;
    }

    if (patchState[i].driftActive) {
      rainbowSegment(patches[i].segmentIndex);
    } else {
      flashSegment(patches[i].segmentIndex, nowMs);
    }
  }
}

static void handleCarUpdate(uint8_t carId, float x, float y, float travelDeg, float headingDeg) {
  const int8_t index = findCarIndex(carId);
  if (index < 0) {
    return;
  }

  const bool hadPreviousUpdate = cars[index].inUse;
  const float previousTravelDeg = cars[index].travelDeg;
  const uint32_t previousUpdateMs = cars[index].lastUpdateMs;

  cars[index].inUse = true;
  cars[index].carId = carId;
  cars[index].position = {x, y};
  cars[index].travelDeg = travelDeg;
  cars[index].headingDeg = headingDeg;
  cars[index].hasPreviousTravel = hadPreviousUpdate;
  if (hadPreviousUpdate) {
    cars[index].previousTravelDeg = previousTravelDeg;
    cars[index].previousUpdateMs = previousUpdateMs;
  }
  cars[index].lastUpdateMs = millis();
}

static void printHelp() {
  Serial.println(F("Reactive track LED controller ready."));
  Serial.println(F("Send: CAR,<id>,<x>,<y>,<travelDeg>,<headingDeg>"));
  Serial.println(F("Example: CAR,1,120.0,35.0,87.0,110.0"));
}

static void parseLine(char* line) {
  char* token = strtok(line, ", \t");
  if (token == nullptr) {
    return;
  }

  if (strcmp(token, "HELP") == 0) {
    printHelp();
    return;
  }

  if (strcmp(token, "CAR") != 0) {
    return;
  }

  char* fields[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};
  for (uint8_t i = 0; i < 5; ++i) {
    fields[i] = strtok(nullptr, ", \t");
    if (fields[i] == nullptr) {
      return;
    }
  }

  const uint8_t carId = static_cast<uint8_t>(atoi(fields[0]));
  const float x = static_cast<float>(atof(fields[1]));
  const float y = static_cast<float>(atof(fields[2]));
  const float travelDeg = static_cast<float>(atof(fields[3]));
  const float headingDeg = static_cast<float>(atof(fields[4]));

  handleCarUpdate(carId, x, y, travelDeg, headingDeg);
}

static void serviceSerial() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      serialLine[serialIndex] = '\0';
      if (serialIndex > 0) {
        parseLine(serialLine);
      }
      serialIndex = 0;
      continue;
    }

    if (serialIndex < sizeof(serialLine) - 1) {
      serialLine[serialIndex++] = c;
    } else {
      serialIndex = 0;
    }
  }
}

void setup() {
  buildSegments();

  if (totalLedCount > MAX_LED_COUNT) {
    while (true) {
      delay(1000);
    }
  }

  FastLED.addLeds<WS2812B, Config::LED_PIN, Config::COLOR_ORDER>(leds, totalLedCount);
  FastLED.setBrightness(Config::MAX_BRIGHTNESS);
  FastLED.clear(true);

  Serial.begin(Config::SERIAL_BAUD);
  delay(300);
  printHelp();
}

void loop() {
  serviceSerial();

  const uint32_t nowMs = millis();
  rainbowBaseHue += Config::RAINBOW_SPEED;

  renderPatches(nowMs);
  FastLED.show();

  delay(16);
}
