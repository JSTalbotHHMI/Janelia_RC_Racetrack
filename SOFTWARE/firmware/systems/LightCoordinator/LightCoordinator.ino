// ReactiveLEDs.ino
//
// Mega2560 + RGBW NeoPixel reactive track lighting
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
// If a car enters a patch, that patch's LED segment turns on immediately.
// White means the heading is close to the travel direction.
// Green means the heading is rotated clockwise from the travel direction.
// Red means the heading is rotated counterclockwise from the travel direction.
// Blue means the car has crashed while in that patch.

#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

namespace Config {
  static const uint8_t LED_PIN = 2;
  static const uint8_t MAX_BRIGHTNESS = 180;
  static const uint8_t WHITE_LEVEL = 128;
  static const uint32_t SERIAL_BAUD = 115200;

  // Treat heading disagreement above this many degrees as drifting.
  static const float DRIFT_THRESHOLD_DEG = 18.0f;

  // A trigger lingers briefly so occasional dropped serial lines do not
  // immediately turn a patch off.
  static const uint32_t PATCH_HOLD_MS = 220;
  static const uint32_t CAR_STALE_MS = 250;

  // Treat a sudden change in travel direction as a wall hit.
  static const float WALL_HIT_MIN_DELTA_DEG = 45.0f;
  static const float WALL_HIT_RATE_DEG_PER_SEC = 1400.0f;
  static const uint32_t WALL_HIT_MAX_SAMPLE_MS = 90;
}

static const uint8_t SEGMENT_COUNT = 4;
static const uint16_t SEGMENT_LENGTHS[SEGMENT_COUNT] = {
  3,
  3,
  3,
  3
};

struct Segment {
  uint16_t start;
  uint16_t length;
};

enum PatchShape : uint8_t {
  PATCH_CIRCLE,
  PATCH_QUAD,
  PATCH_POLY
};

struct Point2D {
  float x;
  float y;
};

static const uint8_t MAX_PATCH_VERTICES = 16;

struct Patch {
  const char* name;
  PatchShape shape;
  uint8_t segmentIndex;
  bool wallReactive;

  // Circle settings
  Point2D center;
  float radius;

  // Polygon settings (points should be ordered around the perimeter)
  Point2D vertices[MAX_PATCH_VERTICES];
  uint8_t vertexCount;
};

struct PatchRuntimeState {
  uint32_t lastTriggerMs;
  bool crashActive;
  uint32_t color;
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

// #include "PythonCirclePath.h"
#include "PatchWithFinish.h"

static const uint16_t MAX_LED_COUNT = 256;
static const uint8_t MAX_CARS = 12;
static const uint16_t LED_COUNT = 12;

static Segment segments[SEGMENT_COUNT];
static PatchRuntimeState patchState[PATCH_COUNT];
static CarState cars[MAX_CARS];

static uint16_t totalLedCount = 0;
static Adafruit_NeoPixel strip(LED_COUNT, Config::LED_PIN, NEO_GRBW + NEO_KHZ800);
static char serialLine[96];
static uint8_t serialIndex = 0;
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

static bool pointOnSegment(const Point2D& point, const Point2D& a, const Point2D& b) {
  const float cross = cross2D(a, b, point);
  if (fabsf(cross) > 0.001f) {
    return false;
  }

  const float minX = min(a.x, b.x);
  const float maxX = max(a.x, b.x);
  const float minY = min(a.y, b.y);
  const float maxY = max(a.y, b.y);
  return point.x >= minX && point.x <= maxX && point.y >= minY && point.y <= maxY;
}

static bool pointInPolygon(const Point2D& point, const Patch& patch) {
  if (patch.vertexCount < 3) {
    return false;
  }

  bool inside = false;
  for (uint8_t i = 0, j = patch.vertexCount - 1; i < patch.vertexCount; j = i++) {
    const Point2D& a = patch.vertices[i];
    const Point2D& b = patch.vertices[j];

    if (pointOnSegment(point, a, b)) {
      return true;
    }

    const bool intersectsY = ((a.y > point.y) != (b.y > point.y));
    if (!intersectsY) {
      continue;
    }

    const float t = (point.y - a.y) / (b.y - a.y);
    const float intersectX = a.x + t * (b.x - a.x);
    if (intersectX >= point.x) {
      inside = !inside;
    }
  }

  return inside;
}

static bool pointInPatch(const Point2D& point, const Patch& patch) {
  if (patch.shape == PATCH_CIRCLE) {
    return pointInCircle(point, patch);
  }

  return pointInPolygon(point, patch);
}

static bool validatePatches() {
  for (size_t i = 0; i < PATCH_COUNT; ++i) {
    const Patch& patch = patches[i];

    if (patch.segmentIndex >= SEGMENT_COUNT) {
      return false;
    }

    if (patch.shape == PATCH_QUAD && patch.vertexCount != 4) {
      return false;
    }

    if (patch.shape == PATCH_POLY &&
        (patch.vertexCount < 3 || patch.vertexCount > MAX_PATCH_VERTICES)) {
      return false;
    }
  }

  return true;
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

static uint32_t makeColor(uint8_t red, uint8_t green, uint8_t blue, uint8_t white = 0) {
  return strip.Color(red, green, blue, white);
}

static uint8_t driftIntensity(float signedHeadingOffsetDeg) {
  const float magnitudeDeg = min(fabsf(signedHeadingOffsetDeg), 30.0f);
  if (magnitudeDeg <= Config::DRIFT_THRESHOLD_DEG) {
    return 0;
  }

  const float spanDeg = 30.0f - Config::DRIFT_THRESHOLD_DEG;
  const float normalized = (magnitudeDeg - Config::DRIFT_THRESHOLD_DEG) / spanDeg;
  return static_cast<uint8_t>(normalized * 255.0f);
}

static uint32_t colorForHeadingState(float travelDeg, float headingDeg, bool crashed) {
  if (crashed) {
    return makeColor(0, 0, 255);
  }

  const float signedHeadingOffsetDeg = wrapAngleDeg(headingDeg - travelDeg);
  const uint8_t intensity = driftIntensity(signedHeadingOffsetDeg);

  if (intensity == 0) {
    return makeColor(0, 0, 0, Config::WHITE_LEVEL);
  }

  const uint8_t whiteLevel = static_cast<uint8_t>(
    ((255 - intensity) * static_cast<uint16_t>(Config::WHITE_LEVEL)) / 255
  );

  if (signedHeadingOffsetDeg > 0.0f) {
    return makeColor(0, intensity, 0, whiteLevel);
  }

  return makeColor(intensity, 0, 0, whiteLevel);
}

static void fillSegment(uint8_t segmentIndex, uint32_t color) {
  const Segment& seg = segments[segmentIndex];
  for (uint16_t i = 0; i < seg.length; ++i) {
    strip.setPixelColor(seg.start + i, color);
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
    patchState[i].crashActive = false;
    patchState[i].color = 0;
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

    const bool wallHit = carLikelyHitWall(cars[carIndex]);

    for (uint8_t patchIndex = 0; patchIndex < PATCH_COUNT; ++patchIndex) {
      if (!pointInPatch(cars[carIndex].position, patches[patchIndex])) {
        continue;
      }

      patchState[patchIndex].lastTriggerMs = nowMs;
      if (wallHit) {
        patchState[patchIndex].crashActive = true;
        patchState[patchIndex].color = makeColor(0, 0, 255);
      } else if (!patchState[patchIndex].crashActive) {
        patchState[patchIndex].color = colorForHeadingState(
          cars[carIndex].travelDeg,
          cars[carIndex].headingDeg,
          false
        );
      }
    }
  }
}

static void renderPatches(uint32_t nowMs) {
  refreshPatchStateFromCars(nowMs);
  strip.clear();

  for (uint8_t i = 0; i < PATCH_COUNT; ++i) {
    const uint32_t ageMs = nowMs - patchState[i].lastTriggerMs;
    if (ageMs > Config::PATCH_HOLD_MS) {
      patchState[i].crashActive = false;
      patchState[i].color = 0;
      continue;
    }

    if (patchState[i].color != 0) {
      fillSegment(patches[i].segmentIndex, patchState[i].color);
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

  if (totalLedCount > LED_COUNT || !validatePatches()) {
    while (true) {
      delay(1000);
    }
  }

  strip.begin();
  strip.setBrightness(Config::MAX_BRIGHTNESS);
  strip.show();

  Serial.begin(Config::SERIAL_BAUD);
  delay(300);
  printHelp();
}

void loop() {
  serviceSerial();

  const uint32_t nowMs = millis();
  renderPatches(nowMs);
  strip.show();

  delay(16);
}
