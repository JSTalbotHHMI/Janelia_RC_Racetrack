# Firmware Experiments

This folder contains prototype sketches, tracker tools, and support assets that are still evolving.

## Experiment Areas

- `ReactiveLEDs/`
  Teensy-based reactive lighting sketch with patch definitions written directly in the `.ino` file.
- `ReactiveLEDs_LoadablePatches/`
  Reactive lighting sketch that includes a generated header such as `PythonCirclePath.h` or `PatchWithFinish.h`.
- `ReactiveLEDs_LoadablePatches_Simulator/`
  Python sender that emits synthetic `CAR,<carId>,<x>,<y>,<travelDeg>,<headingDeg>` packets for lighting-sketch testing.
- `LoadablePatches/`
  Patch-loading firmware experiment for validating generated patch headers and controller behavior.
- `BlobTracker/`
  Camera-tracking prototypes including single-car, multi-car, and patch-overlay variants plus calibration helpers and lap-board displays.
- `ArC Porsche/`
  RC car control experiments including direct steering/throttle tests, drift routines, smooth-turn tests, and a path follower that accepts `ERR,<speed_error_mps>,<heading_error_rad>` packets.
- `WS2812B_StripTest/`
  Simple LED hardware checkout sketch for a Teensy 4.0 plus a WS2812B strip.
- `assets/`
  Shared design references such as the racing-line SVG assets.

## Recommended Workflow

1. Use `../../utilities/patch-generator/` to author patch CSV files.
2. Convert the CSV into a header with `../../utilities/patch-to-header/` or export from the GUI.
3. Copy or generate that header into the sketch folder that needs it.
4. Keep experimental scripts and sketches here until their hardware assumptions and interfaces settle down.

## Organization Notes

- Folder names generally match the sketch or experiment they support.
- Generated headers that are tightly coupled to one sketch can live beside that sketch.
- Calibration CSV files and output preferences are kept near the tracker tools that use them.
- If multiple experiments begin sharing the same parsing, geometry, or hardware code, promote that code into `../shared/`.
