// ReactiveLEDs.ino
//
// Teensy + RGBW NeoPixel reactive track lighting
//
// Serial input format (one car update per line):
//   CAR,<carId>,<x>,<y>,<travelDeg>,<headingDeg>
//
// Patch upload format:
//   SEGMENTS_BEGIN,<count>
//   SEGMENT,<index>,<length>
//   SEGMENTS_END
//
// Then:
//   PATCHES_BEGIN,<count>
//   PATCH,<index>,<name>,<shape>,<segmentIndex>,<wallReactive>,<geometry...>
//   PATCHES_END
//
// Shapes:
//   CIRCLE -> centerX,centerY,radius
//   QUAD   -> x1,y1,x2,y2,x3,y3,x4,y4
//   POLY   -> x1,y1,x2,y2,x3,y3,[...]
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
#include <rgbw.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

namespace Config {
  static const uint8_t LED_PIN = 2;
  static const EOrder COLOR_ORDER = GRB;
  static const EOrderW WHITE_ORDER = W3;
  static const RGBW_MODE RGBW_CONVERSION_MODE = kRGBWExactColors;
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

static const uint8_t MAX_SEGMENTS = 10;
static const uint8_t NO_SEGMENT_INDEX = 255;
static const uint8_t DEFAULT_SEGMENT_COUNT = 4;
static const uint16_t DEFAULT_SEGMENT_LENGTHS[DEFAULT_SEGMENT_COUNT] = {
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
static const uint8_t MAX_PATCHES = 10;
static const uint8_t MAX_PATCH_NAME_LENGTH = 10;

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

#define patches compiledPatches
#define PATCH_COUNT COMPILED_PATCH_COUNT
#include "NewPatches.h"
#undef patches
#undef PATCH_COUNT

static const uint16_t MAX_LED_COUNT = 100;
static const uint8_t MAX_CARS = 12;

static Segment segments[MAX_SEGMENTS];
static PatchRuntimeState patchState[MAX_PATCHES];
static CarState cars[MAX_CARS];
static Patch activePatches[MAX_PATCHES];
static Patch stagedPatches[MAX_PATCHES];
static char activePatchNames[MAX_PATCHES][MAX_PATCH_NAME_LENGTH];
static char stagedPatchNames[MAX_PATCHES][MAX_PATCH_NAME_LENGTH];
static bool stagedPatchReceived[MAX_PATCHES];
static uint16_t activeSegmentLengths[MAX_SEGMENTS];
static uint16_t stagedSegmentLengths[MAX_SEGMENTS];
static bool stagedSegmentReceived[MAX_SEGMENTS];

static uint8_t activePatchCount = 0;
static uint8_t stagedPatchCount = 0;
static uint8_t expectedPatchCount = 0;
static bool patchLoadInProgress = false;
static uint8_t activeSegmentCount = 0;
static uint8_t stagedSegmentCount = 0;
static uint8_t expectedSegmentCount = 0;
static bool segmentLoadInProgress = false;

static uint16_t totalLedCount = 0;
static CRGB leds[MAX_LED_COUNT];
static char serialLine[512];
static uint8_t serialIndex = 0;
static uint8_t rainbowBaseHue = 0;

static void buildSegments();

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

static void clearPatchRuntimeState() {
  memset(patchState, 0, sizeof(patchState));
}

static bool validatePatch(const Patch& patch) {
  if (patch.segmentIndex != NO_SEGMENT_INDEX && patch.segmentIndex >= activeSegmentCount) {
    return false;
  }

  if (patch.shape == PATCH_CIRCLE) {
    return patch.radius > 0.0f;
  }

  if (patch.shape == PATCH_QUAD && patch.vertexCount != 4) {
    return false;
  }

  if (patch.shape == PATCH_POLY &&
      (patch.vertexCount < 3 || patch.vertexCount > MAX_PATCH_VERTICES)) {
    return false;
  }

  return patch.vertexCount >= 3 && patch.vertexCount <= MAX_PATCH_VERTICES;
}

static bool validatePatches(const Patch* patchList, uint8_t patchCount) {
  if (patchCount > MAX_PATCHES) {
    return false;
  }

  for (uint8_t i = 0; i < patchCount; ++i) {
    const Patch& patch = patchList[i];

    if (!validatePatch(patch)) {
      return false;
    }
  }

  return true;
}

static bool validateSegmentLengths(const uint16_t* segmentLengths, uint8_t segmentCount) {
  if (segmentCount > MAX_SEGMENTS) {
    return false;
  }

  uint16_t total = 0;
  for (uint8_t i = 0; i < segmentCount; ++i) {
    const uint16_t length = segmentLengths[i];
    if (length == 0) {
      return false;
    }
    total += length;
    if (total > MAX_LED_COUNT) {
      return false;
    }
  }

  return true;
}

static void copyPatchName(char* destination, const char* source) {
  if (source == nullptr) {
    destination[0] = '\0';
    return;
  }

  strncpy(destination, source, MAX_PATCH_NAME_LENGTH - 1);
  destination[MAX_PATCH_NAME_LENGTH - 1] = '\0';
}

static void copyPatch(Patch& destination, char* destinationName, const Patch& source) {
  copyPatchName(destinationName, source.name);
  destination = source;
  destination.name = destinationName;
}

static void loadCompiledPatches() {
  activePatchCount = static_cast<uint8_t>(COMPILED_PATCH_COUNT);
  if (activePatchCount > MAX_PATCHES) {
    activePatchCount = MAX_PATCHES;
  }
  for (uint8_t i = 0; i < activePatchCount; ++i) {
    copyPatch(activePatches[i], activePatchNames[i], compiledPatches[i]);
  }
  clearPatchRuntimeState();
}

static void loadDefaultSegments() {
  activeSegmentCount = DEFAULT_SEGMENT_COUNT;
  for (uint8_t i = 0; i < activeSegmentCount; ++i) {
    activeSegmentLengths[i] = DEFAULT_SEGMENT_LENGTHS[i];
  }
}

static void resetStagedSegmentLoad(uint8_t expectedCount) {
  segmentLoadInProgress = true;
  expectedSegmentCount = expectedCount;
  stagedSegmentCount = expectedCount;
  memset(stagedSegmentLengths, 0, sizeof(stagedSegmentLengths));
  memset(stagedSegmentReceived, 0, sizeof(stagedSegmentReceived));
}

static void resetStagedPatchLoad(uint8_t expectedCount) {
  patchLoadInProgress = true;
  expectedPatchCount = expectedCount;
  stagedPatchCount = expectedCount;
  memset(stagedPatches, 0, sizeof(stagedPatches));
  memset(stagedPatchNames, 0, sizeof(stagedPatchNames));
  memset(stagedPatchReceived, 0, sizeof(stagedPatchReceived));
}

static bool parseBoolToken(const char* token, bool& valueOut) {
  if (token == nullptr) {
    return false;
  }

  if (strcmp(token, "1") == 0 || strcasecmp(token, "true") == 0 || strcasecmp(token, "yes") == 0) {
    valueOut = true;
    return true;
  }

  if (strcmp(token, "0") == 0 || strcasecmp(token, "false") == 0 || strcasecmp(token, "no") == 0) {
    valueOut = false;
    return true;
  }

  return false;
}

static bool parseShapeToken(const char* token, PatchShape& shapeOut) {
  if (token == nullptr) {
    return false;
  }

  if (strcasecmp(token, "CIRCLE") == 0) {
    shapeOut = PATCH_CIRCLE;
    return true;
  }
  if (strcasecmp(token, "QUAD") == 0) {
    shapeOut = PATCH_QUAD;
    return true;
  }
  if (strcasecmp(token, "POLY") == 0) {
    shapeOut = PATCH_POLY;
    return true;
  }

  return false;
}

static bool parseFloatToken(const char* token, float& valueOut) {
  if (token == nullptr) {
    return false;
  }

  bool sawDigit = false;
  for (const char* cursor = token; *cursor != '\0'; ++cursor) {
    if (*cursor >= '0' && *cursor <= '9') {
      sawDigit = true;
      break;
    }
  }

  if (!sawDigit) {
    return false;
  }

  char* parseEnd = nullptr;
  valueOut = static_cast<float>(strtod(token, &parseEnd));
  return parseEnd != token && parseEnd != nullptr && *parseEnd == '\0';
}

static void printPatchError(const __FlashStringHelper* message) {
  Serial.print(F("PATCHES_ERROR,"));
  Serial.println(message);
}

static void handlePatchUploadBegin(char* countToken) {
  if (countToken == nullptr) {
    printPatchError(F("missing_count"));
    return;
  }

  const int count = atoi(countToken);
  if (count < 0 || count > MAX_PATCHES) {
    printPatchError(F("invalid_count"));
    return;
  }

  resetStagedPatchLoad(static_cast<uint8_t>(count));
  Serial.print(F("PATCHES_BEGIN_OK,"));
  Serial.println(expectedPatchCount);
}

static void handleSegmentUploadBegin(char* countToken) {
  if (countToken == nullptr) {
    printPatchError(F("missing_segment_count"));
    return;
  }

  const int count = atoi(countToken);
  if (count < 0 || count > MAX_SEGMENTS) {
    printPatchError(F("invalid_segment_count"));
    return;
  }

  resetStagedSegmentLoad(static_cast<uint8_t>(count));
  Serial.print(F("SEGMENTS_BEGIN_OK,"));
  Serial.println(expectedSegmentCount);
}

static void commitStagedSegments() {
  activeSegmentCount = stagedSegmentCount;
  for (uint8_t i = 0; i < activeSegmentCount; ++i) {
    activeSegmentLengths[i] = stagedSegmentLengths[i];
  }
}

static void handleSegmentUploadEnd() {
  if (!segmentLoadInProgress) {
    printPatchError(F("segments_not_loading"));
    return;
  }

  for (uint8_t i = 0; i < expectedSegmentCount; ++i) {
    if (!stagedSegmentReceived[i]) {
      printPatchError(F("missing_segment"));
      segmentLoadInProgress = false;
      return;
    }
  }

  if (!validateSegmentLengths(stagedSegmentLengths, stagedSegmentCount)) {
    printPatchError(F("invalid_segment_data"));
    segmentLoadInProgress = false;
    return;
  }

  commitStagedSegments();
  buildSegments();

  if (!validatePatches(activePatches, activePatchCount)) {
    printPatchError(F("segments_conflict_with_patches"));
    loadDefaultSegments();
    buildSegments();
    segmentLoadInProgress = false;
    return;
  }

  segmentLoadInProgress = false;
  Serial.print(F("SEGMENTS_OK,"));
  Serial.println(activeSegmentCount);
}

static void handleSegmentDefinition() {
  if (!segmentLoadInProgress) {
    printPatchError(F("segments_not_loading"));
    return;
  }

  char* indexToken = strtok(nullptr, ",\t");
  char* lengthToken = strtok(nullptr, ",\t");
  if (indexToken == nullptr || lengthToken == nullptr) {
    printPatchError(F("missing_segment_fields"));
    return;
  }

  const int segmentIndex = atoi(indexToken);
  const int segmentLength = atoi(lengthToken);
  if (segmentIndex < 0 || segmentIndex >= expectedSegmentCount || segmentLength <= 0) {
    printPatchError(F("segment_out_of_range"));
    return;
  }

  stagedSegmentLengths[segmentIndex] = static_cast<uint16_t>(segmentLength);
  stagedSegmentReceived[segmentIndex] = true;
  Serial.print(F("SEGMENT_OK,"));
  Serial.println(segmentIndex);
}

static void commitStagedPatches() {
  activePatchCount = stagedPatchCount;
  for (uint8_t i = 0; i < activePatchCount; ++i) {
    copyPatch(activePatches[i], activePatchNames[i], stagedPatches[i]);
  }
  clearPatchRuntimeState();
}

static void handlePatchUploadEnd() {
  if (!patchLoadInProgress) {
    printPatchError(F("not_loading"));
    return;
  }

  for (uint8_t i = 0; i < expectedPatchCount; ++i) {
    if (!stagedPatchReceived[i]) {
      printPatchError(F("missing_patch"));
      patchLoadInProgress = false;
      return;
    }
  }

  if (!validatePatches(stagedPatches, stagedPatchCount)) {
    printPatchError(F("invalid_patch_data"));
    patchLoadInProgress = false;
    return;
  }

  commitStagedPatches();
  patchLoadInProgress = false;
  Serial.print(F("PATCHES_OK,"));
  Serial.println(activePatchCount);
}

static void handlePatchDefinition() {
  if (!patchLoadInProgress) {
    printPatchError(F("not_loading"));
    return;
  }

  char* indexToken = strtok(nullptr, ",\t");
  char* nameToken = strtok(nullptr, ",\t");
  char* shapeToken = strtok(nullptr, ",\t");
  char* segmentToken = strtok(nullptr, ",\t");
  char* wallToken = strtok(nullptr, ",\t");

  if (indexToken == nullptr || nameToken == nullptr || shapeToken == nullptr ||
      segmentToken == nullptr || wallToken == nullptr) {
    printPatchError(F("missing_patch_fields"));
    return;
  }

  const int patchIndex = atoi(indexToken);
  const int segmentIndex = atoi(segmentToken);
  if (patchIndex < 0 || patchIndex >= expectedPatchCount) {
    printPatchError(F("patch_index_out_of_range"));
    return;
  }

  if (segmentIndex < -1 || segmentIndex >= activeSegmentCount) {
    printPatchError(F("segment_out_of_range"));
    return;
  }

  PatchShape shape = PATCH_QUAD;
  if (!parseShapeToken(shapeToken, shape)) {
    printPatchError(F("invalid_shape"));
    return;
  }

  bool wallReactive = false;
  if (!parseBoolToken(wallToken, wallReactive)) {
    printPatchError(F("invalid_wall_flag"));
    return;
  }

  Patch patch = {};
  patch.name = stagedPatchNames[patchIndex];
  patch.shape = shape;
  patch.segmentIndex = (segmentIndex < 0) ? NO_SEGMENT_INDEX : static_cast<uint8_t>(segmentIndex);
  patch.wallReactive = wallReactive;
  copyPatchName(stagedPatchNames[patchIndex], nameToken);

  if (shape == PATCH_CIRCLE) {
    float values[3] = {0.0f, 0.0f, 0.0f};
    for (uint8_t i = 0; i < 3; ++i) {
      char* token = strtok(nullptr, ",\t");
      if (!parseFloatToken(token, values[i])) {
        printPatchError(F("invalid_circle_geometry"));
        return;
      }
    }
    patch.center = {values[0], values[1]};
    patch.radius = values[2];
    patch.vertexCount = 0;
  } else {
    uint8_t pointCount = 0;
    while (pointCount < MAX_PATCH_VERTICES) {
      char* xToken = strtok(nullptr, ",\t");
      if (xToken == nullptr) {
        break;
      }
      char* yToken = strtok(nullptr, ",\t");
      if (yToken == nullptr) {
        printPatchError(F("odd_polygon_coordinate_count"));
        return;
      }

      float xValue = 0.0f;
      float yValue = 0.0f;
      if (!parseFloatToken(xToken, xValue) || !parseFloatToken(yToken, yValue)) {
        printPatchError(F("invalid_polygon_geometry"));
        return;
      }

      patch.vertices[pointCount++] = {xValue, yValue};
    }

    if (strtok(nullptr, ",\t") != nullptr) {
      printPatchError(F("too_many_vertices"));
      return;
    }

    patch.vertexCount = pointCount;
    patch.center = {0.0f, 0.0f};
    patch.radius = 0.0f;
  }

  if (!validatePatch(patch)) {
    printPatchError(F("invalid_patch"));
    return;
  }

  if (shape == PATCH_QUAD && patch.vertexCount != 4) {
    printPatchError(F("quad_requires_four_points"));
    return;
  }

  if (shape == PATCH_POLY && patch.vertexCount < 3) {
    printPatchError(F("poly_requires_three_points"));
    return;
  }

  stagedPatches[patchIndex] = patch;
  stagedPatches[patchIndex].name = stagedPatchNames[patchIndex];
  stagedPatchReceived[patchIndex] = true;
  Serial.print(F("PATCH_OK,"));
  Serial.println(patchIndex);
}

static void buildSegments() {
  uint16_t cursor = 0;
  for (uint8_t i = 0; i < activeSegmentCount; ++i) {
    segments[i].start = cursor;
    segments[i].length = activeSegmentLengths[i];
    cursor += activeSegmentLengths[i];
  }
  totalLedCount = cursor;
}

static void fillSegment(uint8_t segmentIndex, const CRGB& color) {
  if (segmentIndex >= activeSegmentCount) {
    return;
  }

  const Segment& seg = segments[segmentIndex];
  for (uint16_t i = 0; i < seg.length; ++i) {
    leds[seg.start + i] = color;
  }
}

static void flashSegment(uint8_t segmentIndex, uint32_t nowMs) {
  if (segmentIndex >= activeSegmentCount) {
    return;
  }

  const uint8_t phase = beatsin8(Config::FLASH_BPM, 0, 255, 0, 0);
  const CRGB blended = blend(Config::FLASH_COLOR_B, Config::FLASH_COLOR_A, phase);
  fillSegment(segmentIndex, blended);

  // Add a narrow bright pulse running through the segment for extra motion.
  const Segment& seg = segments[segmentIndex];
  const uint16_t pulseIndex = map(nowMs % 1000, 0, 999, 0, seg.length - 1);
  leds[seg.start + pulseIndex] += CRGB::White;
}

static void rainbowSegment(uint8_t segmentIndex) {
  if (segmentIndex >= activeSegmentCount) {
    return;
  }

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
  if (segmentIndex >= activeSegmentCount) {
    return;
  }

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
  for (uint8_t i = 0; i < activePatchCount; ++i) {
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

    for (uint8_t patchIndex = 0; patchIndex < activePatchCount; ++patchIndex) {
      if (!pointInPatch(cars[carIndex].position, activePatches[patchIndex])) {
        continue;
      }

      patchState[patchIndex].lastTriggerMs = nowMs;
      if (drifting) {
        patchState[patchIndex].driftActive = true;
      }

      if (activePatches[patchIndex].segmentIndex != NO_SEGMENT_INDEX &&
          activePatches[patchIndex].wallReactive &&
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

  for (uint8_t i = 0; i < activePatchCount; ++i) {
    if (activePatches[i].segmentIndex == NO_SEGMENT_INDEX) {
      continue;
    }

    const uint32_t impactAgeMs = nowMs - patchState[i].lastImpactMs;
    if (patchState[i].lastImpactMs != 0 && impactAgeMs <= Config::IMPACT_HOLD_MS) {
      impactSegment(activePatches[i].segmentIndex, impactAgeMs, nowMs);
      continue;
    }

    const uint32_t ageMs = nowMs - patchState[i].lastTriggerMs;
    if (ageMs > Config::PATCH_HOLD_MS) {
      patchState[i].driftActive = false;
      continue;
    }

    if (patchState[i].driftActive) {
      rainbowSegment(activePatches[i].segmentIndex);
    } else {
      flashSegment(activePatches[i].segmentIndex, nowMs);
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
  Serial.println(F("Optional: SEGMENTS_BEGIN,<count> ... SEGMENTS_END"));
  Serial.println(F("Send: CAR,<id>,<x>,<y>,<travelDeg>,<headingDeg>"));
  Serial.println(F("Or: PATCHES_BEGIN,<count> ... PATCHES_END"));
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

  if (strcmp(token, "PATCHES_BEGIN") == 0) {
    handlePatchUploadBegin(strtok(nullptr, ",\t"));
    return;
  }

  if (strcmp(token, "SEGMENTS_BEGIN") == 0) {
    handleSegmentUploadBegin(strtok(nullptr, ",\t"));
    return;
  }

  if (strcmp(token, "SEGMENTS_END") == 0) {
    handleSegmentUploadEnd();
    return;
  }

  if (strcmp(token, "SEGMENT") == 0) {
    handleSegmentDefinition();
    return;
  }

  if (strcmp(token, "PATCHES_END") == 0) {
    handlePatchUploadEnd();
    return;
  }

  if (strcmp(token, "PATCH") == 0) {
    handlePatchDefinition();
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
  loadDefaultSegments();
  loadCompiledPatches();
  buildSegments();

  if (totalLedCount > MAX_LED_COUNT ||
      !validateSegmentLengths(activeSegmentLengths, activeSegmentCount) ||
      !validatePatches(activePatches, activePatchCount)) {
    while (true) {
      delay(1000);
    }
  }

  CLEDController& ledController =
    FastLED.addLeds<SK6812, Config::LED_PIN, Config::COLOR_ORDER>(leds, totalLedCount);
  ledController.setRgbw(Rgbw(kRGBWDefaultColorTemp, Config::RGBW_CONVERSION_MODE, Config::WHITE_ORDER));
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
