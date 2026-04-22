# Utilities

This folder contains standalone support tools used during development and iteration.

## Available Tools

- [`patch-generator/`](patch-generator/README.md)
  Desktop patch editor for drawing and editing racetrack regions from a camera feed, saving them as CSV, exporting headers, or streaming updates to a controller.
- [`patch-to-header/`](patch-to-header/README.md)
  Command-line converter that turns a patch CSV into a C/C++ header suitable for Arduino or Teensy sketches.

## Typical Patch Workflow

1. Run `patch-generator` and create or edit a CSV that describes the track regions.
2. Save generated CSV files in an `output/` folder.
3. Export the CSV to a header from the GUI, or run `patch-to-header` manually.
4. Include the generated header in the target firmware sketch, such as `ReactiveLEDs_LoadablePatches`, `LoadablePatches`, or `LightCoordinator`.
5. If you want live iteration without reflashing, use `patch-generator/serial_patch_loader.py` to stream updates over serial.

Keep each utility self-contained with its own README, assets, and generated output folder when needed.
