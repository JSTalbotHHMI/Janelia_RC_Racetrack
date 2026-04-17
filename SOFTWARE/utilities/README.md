# Utilities

This folder contains standalone support tools used during development.

## Available Tools

- [`patch-generator/`](patch-generator/README.md)
  Desktop GUI for drawing and editing track patches from a live webcam image, then saving them as CSV.
- [`patch-to-header/`](patch-to-header/README.md)
  Command-line converter that turns a patch CSV into a C/C++ header suitable for inclusion in Arduino or Teensy sketches.

## Typical Patch Workflow

1. Run `patch-generator` and create or edit a CSV file that describes the track regions.
2. Save generated CSV files in an `output/` folder.
3. Run `patch-to-header` on the CSV to produce a header with `Patch` initializers.
4. Include the header in the target firmware sketch, such as `ReactiveLEDs_LoadablePatches`.

Keep each utility self-contained with its own README, assets, and generated output folder when needed.
