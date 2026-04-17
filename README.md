# Janelia RC Racetrack

Software, firmware, and fabrication assets for the Janelia RC racetrack project.

This repository currently leans heavily toward embedded experiments and support tools for interactive track lighting and RC car control. The electrical and mechanical design folders are present as landing zones for future project files.

## What Is Here Today

- `SOFTWARE/utilities/patch-generator/`: desktop GUI for drawing track patches from a live camera feed and saving them as CSV files
- `SOFTWARE/utilities/patch-to-header/`: command-line converter that turns patch CSV files into Arduino-friendly headers
- `SOFTWARE/firmware/experiments/ReactiveLEDs*`: Teensy/FastLED lighting sketches that react to synthetic or camera-derived car pose data
- `SOFTWARE/firmware/experiments/ArC Porsche/`: RC car control sketches and helper scripts for MCP4261-based steering/throttle experiments
- `SOFTWARE/firmware/experiments/WS2812B_StripTest/`: quick LED hardware validation sketch

## Repository Layout

```text
Janelia_RC_Racetrack/
  ECAD/                       Electrical design workspace
  MCAD/                       Mechanical design workspace
  SOFTWARE/
    firmware/
      experiments/            Active sketches, test rigs, and supporting assets
      shared/                 Reusable embedded code intended for multiple sketches
      systems/                Production-ready firmware targets
    host/                     Future host-side apps and services
    shared/                   Cross-tool formats, protocols, and shared software assets
    utilities/                Standalone support tools and generated outputs
```

See the local README files in those folders for area-specific guidance.

## Common Workflows

### Reactive LED Track Workflow

1. Use [`SOFTWARE/utilities/patch-generator/`](SOFTWARE/utilities/patch-generator/README.md) to define track regions from a camera view and save a CSV.
2. Use [`SOFTWARE/utilities/patch-to-header/`](SOFTWARE/utilities/patch-to-header/README.md) to convert that CSV into a header file.
3. Include the generated header in [`ReactiveLEDs_LoadablePatches`](SOFTWARE/firmware/experiments/README.md) or a related sketch.
4. Optionally use the simulator script in `ReactiveLEDs_LoadablePatches_Simulator` to send synthetic `CAR,...` packets while tuning behavior.

### RC Car Control Experiments

- The `ArC Porsche` experiment set contains direct control, path-following, drift, and turn-profile sketches.
- A companion Python sender can stream synthetic `ERR,...` packets so the path-following sketch can be tested without a live vision stack.

## Conventions

- Put production or deployable firmware in `SOFTWARE/firmware/systems/`.
- Put exploratory sketches, diagnostics, and one-off prototypes in `SOFTWARE/firmware/experiments/`.
- Put standalone desktop tools in `SOFTWARE/utilities/`.
- Put generated outputs in an `output/` folder near the tool that creates them when practical.
- Add a local README when a folder starts to carry its own workflow, dependencies, or generated artifacts.
