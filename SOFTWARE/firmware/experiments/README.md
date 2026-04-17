# Firmware Experiments

This folder contains prototype sketches, test rigs, and support assets that are still evolving.

## Experiment Areas

- `ReactiveLEDs/`
  Basic Teensy/FastLED sketch with patch definitions written directly in the `.ino` file.
- `ReactiveLEDs_LoadablePatches/`
  Variant that includes a generated header such as `PythonCirclePath.h`, making it easier to iterate on patch geometry outside the Arduino sketch.
- `ReactiveLEDs_LoadablePatches_Simulator/`
  Python script that sends synthetic `CAR,<carId>,<x>,<y>,<travelDeg>,<headingDeg>` packets while overlaying the simulated vehicle pose on a video feed.
- `WS2812B_StripTest/`
  Simple LED hardware checkout sketch for a Teensy 4.0 plus a WS2812B strip.
- `ArC Porsche/`
  RC car control experiments, including direct steering/throttle tests, drift routines, and a closed-loop path follower that accepts `ERR,<speed_error_mps>,<heading_error_rad>` packets.
- `assets/`
  Supporting design assets such as SVG racing-line references.

## Recommended Workflow

- Use `utilities/patch-generator/` to author patch CSV files.
- Use `utilities/patch-to-header/` to convert a CSV into a header.
- Copy or generate that header into the sketch folder that needs it.
- Keep exploratory sketches here until their interfaces and dependencies settle down.

## Organization Notes

- Folder names match the sketch or experiment they support.
- Generated headers that are tightly coupled to a sketch can live beside that sketch.
- If multiple experiments begin sharing the same code, promote that code into `../shared/`.
