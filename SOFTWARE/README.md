# Software

This directory contains the software side of the racetrack project, from embedded sketches and Python tracker tools to standalone patch-authoring utilities.

## Layout

- [`firmware/`](firmware/README.md)
  Arduino, Teensy, and Python-based race-control or vision workflows.
- [`host/`](host/README.md)
  Reserved for larger host-side applications and services.
- [`shared/`](shared/README.md)
  Shared formats, datasets, and reusable software assets.
- [`utilities/`](utilities/README.md)
  Standalone development tools such as the patch editor and CSV-to-header converter.

## What Is Implemented Today

- Patch creation and export tooling under `utilities/patch-generator/` and `utilities/patch-to-header/`
- Reactive LED controller sketches and runtime patch-loading experiments under `firmware/experiments/`
- Blob tracking, overlay, and lap-board prototypes under `firmware/experiments/BlobTracker/`
- System-level race-operation tools under `firmware/systems/Timetrials/`
- Stable lighting-controller firmware under `firmware/systems/LightCoordinator/`

## Typical Workflow

1. Build or edit the patch CSV in `utilities/patch-generator/`.
2. Convert it to a header with `utilities/patch-to-header/` or export it directly from the GUI.
3. Use that patch definition in an experiment or system sketch under `firmware/`.
4. If you are running the tracker stack, share the same calibration and patch data with the tools in `firmware/experiments/BlobTracker/` or `firmware/systems/Timetrials/`.

## Notes

- Teensy and Arduino sketches in this repo rely on LED libraries such as `FastLED` or `Adafruit_NeoPixel`, depending on the target.
- Python tools in this tree use packages such as `opencv-python`, `numpy`, `Pillow`, and `pyserial`.
- `host/` and `shared/` are still mostly placeholders, while the active implementation work is concentrated in `firmware/` and `utilities/`.
