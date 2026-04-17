# Firmware

This folder holds embedded code for the racetrack project.

## Layout

- [`experiments/`](experiments/README.md): active prototypes, hardware bring-up sketches, and behavior tests
- [`shared/`](shared/README.md): reusable embedded components intended to be shared across sketches
- [`systems/`](systems/README.md): production or polished firmware targets

## Current Contents

Most of the implemented code currently lives in `experiments/`:

- `ReactiveLEDs/`: basic reactive lighting sketch with inline patch definitions
- `ReactiveLEDs_LoadablePatches/`: reactive lighting sketch that loads generated patch definitions from a header
- `ReactiveLEDs_LoadablePatches_Simulator/`: Python simulator that emits synthetic `CAR,...` serial packets
- `WS2812B_StripTest/`: quick LED strip validation sketch
- `ArC Porsche/`: RC car driving and path-following experiments

## Dependencies Seen In This Tree

- `FastLED` for WS2812B LED control
- `MCP4261` for steering and throttle control in the RC car experiments
- `pyserial` and `opencv-python` for companion Python scripts

As common code stabilizes, move reusable pieces from `experiments/` into `shared/` and reserve `systems/` for firmware that is ready to be treated as the canonical project build.
