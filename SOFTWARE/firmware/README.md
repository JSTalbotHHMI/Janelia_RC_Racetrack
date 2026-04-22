# Firmware

This folder holds the racetrack project's embedded controller code plus closely related Python-based tracker and operator tools.

## Layout

- [`experiments/`](experiments/README.md)
  Prototype sketches, tracker tools, diagnostics, and behavior tests.
- [`shared/`](shared/README.md)
  Reusable embedded helpers promoted out of experiments.
- [`systems/`](systems/README.md)
  More complete or preferred firmware and race-operation targets.

## Current Contents

### Experiments

- `ReactiveLEDs/`
  Early reactive lighting sketch with patch geometry defined directly in the source.
- `ReactiveLEDs_LoadablePatches/`
  Reactive lighting sketch that includes generated patch headers and supports serial updates.
- `LoadablePatches/`
  A related patch-loading experiment with generated header inputs.
- `BlobTracker/`
  Single-car and multi-car camera tracking tools, calibration helpers, overlays, and lap-board prototypes.
- `ArC Porsche/`
  RC car steering, throttle, drift, and path-following experiments.
- `WS2812B_StripTest/`
  Quick LED strip hardware validation sketch.

### Systems

- `LightCoordinator/`
  LED controller firmware intended to run generated patch sets and consume `CAR,...` serial packets.
- `Timetrials/`
  Multi-car tracking and race display workflow with calibration, patch preferences, overlays, and assets.

## Dependencies Seen In This Tree

- `FastLED` and `Adafruit_NeoPixel` for addressable LED control
- `MCP4261` for steering and throttle control in the ArC Porsche experiments
- `opencv-python`, `numpy`, `Pillow`, `tkinter`, and `pyserial` for the Python tracker and tooling workflows

As common code stabilizes, move shared pieces into `shared/` and keep `systems/` reserved for the preferred build path for each subsystem.
