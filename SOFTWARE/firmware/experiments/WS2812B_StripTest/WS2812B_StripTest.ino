// WS2812B_StripTest.ino
//
// Simple Teensy 4.0 strip test for 40 WS2812B LEDs on pin 2.
// Uses FastLED, which is already used elsewhere in this firmware repo.

#include <FastLED.h>

namespace Config {
  static const uint8_t LED_PIN = 2;
  static const uint16_t LED_COUNT = 40;
  static const uint8_t BRIGHTNESS = 96;
  static const EOrder COLOR_ORDER = GRB;
  static const uint32_t SERIAL_BAUD = 115200;
}

static CRGB leds[Config::LED_COUNT];
static uint8_t rainbowHue = 0;

static void fillAndShow(const CRGB& color, uint16_t holdMs) {
  fill_solid(leds, Config::LED_COUNT, color);
  FastLED.show();
  delay(holdMs);
}

static void colorWipe(const CRGB& color, uint16_t stepDelayMs) {
  fill_solid(leds, Config::LED_COUNT, CRGB::Black);

  for (uint16_t i = 0; i < Config::LED_COUNT; ++i) {
    leds[i] = color;
    FastLED.show();
    delay(stepDelayMs);
  }
}

static void theaterChase(const CRGB& color, uint8_t cycles, uint16_t frameDelayMs) {
  for (uint8_t cycle = 0; cycle < cycles; ++cycle) {
    for (uint8_t offset = 0; offset < 3; ++offset) {
      fill_solid(leds, Config::LED_COUNT, CRGB::Black);
      for (uint16_t i = offset; i < Config::LED_COUNT; i += 3) {
        leds[i] = color;
      }
      FastLED.show();
      delay(frameDelayMs);
    }
  }
}

static void rainbowCycle(uint16_t durationMs, uint16_t frameDelayMs) {
  const uint32_t startMs = millis();

  while (millis() - startMs < durationMs) {
    for (uint16_t i = 0; i < Config::LED_COUNT; ++i) {
      leds[i] = CHSV(rainbowHue + (i * 255 / Config::LED_COUNT), 255, 255);
    }

    FastLED.show();
    rainbowHue += 2;
    delay(frameDelayMs);
  }
}

static void scanner(const CRGB& color, uint8_t passes, uint16_t frameDelayMs) {
  for (uint8_t pass = 0; pass < passes; ++pass) {
    for (int16_t i = 0; i < Config::LED_COUNT; ++i) {
      fadeToBlackBy(leds, Config::LED_COUNT, 80);
      leds[i] = color;
      FastLED.show();
      delay(frameDelayMs);
    }

    for (int16_t i = Config::LED_COUNT - 1; i >= 0; --i) {
      fadeToBlackBy(leds, Config::LED_COUNT, 80);
      leds[i] = color;
      FastLED.show();
      delay(frameDelayMs);
    }
  }
}

static void printStartupMessage() {
  Serial.println(F("WS2812B strip test"));
  Serial.println(F("Board: Teensy 4.0"));
  Serial.println(F("LED count: 40"));
  Serial.println(F("Data pin: 2"));
}

void setup() {
  Serial.begin(Config::SERIAL_BAUD);

  FastLED.addLeds<WS2812B, Config::LED_PIN, Config::COLOR_ORDER>(leds, Config::LED_COUNT);
  FastLED.setBrightness(Config::BRIGHTNESS);
  FastLED.clear(true);

  delay(250);
  printStartupMessage();
}

void loop() {
  colorWipe(CRGB::Red, 20);
  colorWipe(CRGB::Green, 20);
  colorWipe(CRGB::Blue, 20);

  fillAndShow(CRGB::White, 600);
  fillAndShow(CRGB::Black, 250);

  theaterChase(CRGB(255, 120, 0), 10, 60);
  scanner(CRGB::Cyan, 4, 18);
  rainbowCycle(3000, 20);

  fillAndShow(CRGB::Black, 500);
}
