// WS2812B_StripTest.ino
//
// Teensy 4.0 + 40 WS2812B LEDs on pin 2.
// The animation walks down the strip so each step shows:
// - current LED in red
// - previous LED in blue
// - LED before that in green

#include <FastLED.h>

namespace Config {
  static const uint8_t LED_PIN = 2;
  static const uint16_t LED_COUNT = 40;
  static const uint8_t BRIGHTNESS = 96;
  static const EOrder COLOR_ORDER = GRB;
  static const uint16_t STEP_DELAY_MS = 150;
}

static CRGB leds[Config::LED_COUNT];

void setup() {
  FastLED.addLeds<WS2812B, Config::LED_PIN, Config::COLOR_ORDER>(leds, Config::LED_COUNT);
  FastLED.setBrightness(Config::BRIGHTNESS);
  FastLED.clear(true);
}

void loop() {
  for (uint16_t i = 0; i < Config::LED_COUNT; ++i) {
    fill_solid(leds, Config::LED_COUNT, CRGB::Black);

    leds[i] = CRGB::Red;

    if (i >= 1) {
      leds[i - 1] = CRGB::Blue;
    }

    if (i >= 2) {
      leds[i - 2] = CRGB::Green;
    }

    FastLED.show();
    delay(Config::STEP_DELAY_MS);
  }

  FastLED.clear(true);
  delay(400);
}
