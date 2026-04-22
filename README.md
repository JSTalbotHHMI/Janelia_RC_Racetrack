# Janelia RC Racetrack

Software, firmware, and fabrication assets for the Janelia RC racetrack project.

The repository currently centers on three active workflows:

- reactive LED patch authoring and controller firmware
- camera-based blob tracking and lap-timing tools
- RC car steering and throttle experiments for the ArC Porsche platform

The `ECAD/` and `MCAD/` folders are kept in the tree for future electrical and mechanical design files, while most of the implemented work today lives under `SOFTWARE/`.

## Repository Layout

```text
Janelia_RC_Racetrack/
  ECAD/                       Electrical design workspace
  MCAD/                       Mechanical design workspace
  SOFTWARE/
    firmware/
      experiments/            Prototype sketches, tracker tools, and test rigs
      shared/                 Reusable embedded code promoted out of experiments
      systems/                More complete controller and race-operation builds
    host/                     Future host-side applications and services
    shared/                   Shared formats, assets, and cross-tool software data
    utilities/                Standalone development tools
```

## Current Highlights

- `SOFTWARE/utilities/patch-generator/`
  Desktop patch editor for drawing racetrack regions from a camera view, saving them as CSV, exporting headers, or streaming live patch updates over serial.
- `SOFTWARE/utilities/patch-to-header/`
  Command-line converter that turns patch CSV files into Arduino-friendly headers.
- `SOFTWARE/firmware/experiments/`
  Embedded sketches and prototype Python tools, including `BlobTracker`, `ReactiveLEDs`, `ReactiveLEDs_LoadablePatches`, `LoadablePatches`, and the `ArC Porsche` RC car control experiments.
- `SOFTWARE/firmware/systems/LightCoordinator/`
  A more complete LED controller target that consumes generated patch headers and `CAR,...` serial updates.
- `SOFTWARE/firmware/systems/Timetrials/`
  Multi-car tracking and lap-timing application with calibration, patch-selection, preview, and race-display assets.

## Common Workflows

### Patch-Driven Lighting

1. Use [`SOFTWARE/utilities/patch-generator/`](SOFTWARE/utilities/patch-generator/README.md) to create or edit patch CSV files from the camera view.
2. Export the CSV to a header, or run [`SOFTWARE/utilities/patch-to-header/`](SOFTWARE/utilities/patch-to-header/README.md) directly.
3. Include the generated header in a sketch such as `ReactiveLEDs_LoadablePatches`, `LoadablePatches`, or `LightCoordinator`.
4. Optionally use `serial_patch_loader.py` to push CSV changes to a running controller without reflashing.

### Vision And Timing

1. Calibrate the camera and patch layout with the tools under `SOFTWARE/firmware/experiments/BlobTracker/`.
2. Reuse those calibration and patch CSV files in `SOFTWARE/firmware/systems/Timetrials/`.
3. Stream pose or event data to lighting or race-display tools over serial and UDP, depending on the workflow you are testing.

### ArC Porsche Vehicle Control

- The `SOFTWARE/firmware/experiments/ArC Porsche/` folder contains steering, throttle, drift, turn, and path-following sketches.
- The companion Python sender can generate synthetic `ERR,...` packets so the path follower can be tested without a live vision loop.

## Conventions

- Keep exploratory sketches and prototype scripts in `SOFTWARE/firmware/experiments/`.
- Move stable, preferred builds into `SOFTWARE/firmware/systems/`.
- Put standalone desktop tools in `SOFTWARE/utilities/`.
- Keep generated CSV or header outputs in nearby `output/` folders when practical.
- Add or refresh a local README when a folder gains its own setup steps, assets, or generated artifacts.
