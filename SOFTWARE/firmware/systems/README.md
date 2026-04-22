# Firmware Systems

This folder holds the racetrack project's more complete or preferred build targets.

Use this area when an experiment becomes stable enough that it should be treated as the main supported build for a subsystem.

## Current System Targets

- `LightCoordinator/`
  Arduino-style LED controller firmware that includes a generated patch header, manages LED segments, and reacts to `CAR,<carId>,<x>,<y>,<travelDeg>,<headingDeg>` serial packets.
- `Timetrials/`
  Python-based race-operation tool for tracking up to five marker pairs, managing calibration and patch preferences, and rendering lap-timing UI assets.

## Promotion Guidelines

- Keep bring-up sketches, one-off diagnostics, and rapidly changing ideas in `../experiments/`.
- Move a target here when it becomes the canonical workflow for a subsystem.
- Promote any reusable helper code into `../shared/` instead of duplicating it between `systems/` folders.
