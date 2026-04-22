# ArC Porsche Experiments

This folder contains RC car control experiments built around an `MCP4261` digital potentiometer for steering and throttle control.

## Contents

- `ArC_Porsche/`
  Early direct-control sketch for exercising steering and throttle ranges.
- `ArC_Porsche_Circle/`
  Circle-driving experiment.
- `ArC_Porsche_Drift/`
  Drift controller with phased steering and throttle behavior.
- `ArC_Porsche_Drift2/`
  Additional drift iteration.
- `ArC_Porsche_DriftCircle/`
  Drift-oriented circular driving experiment.
- `ArC_Porsche_PathFollower/`
  Closed-loop path-following sketch that consumes `ERR,<speed_error_mps>,<heading_error_rad>` serial packets.
- `ArC_Porsche_SmoothTurn/`
  Steering sweep experiment for smoother turn behavior.
- `dummyController_FigureEight_ErrorCommunication.py`
  Python sender that streams synthetic error packets so the path follower can be tested without a live vision pipeline.

## Hardware And Dependencies

- The sketches target a Teensy-based setup with software SPI connections to the `MCP4261`.
- Calibration values are still sketch-local, so steering and throttle ranges are not automatically consistent across experiments.
- The helper sender script only needs Python standard-library modules and can be used to bench-test serial handling before the vision stack is connected.

## Suggested Usage

1. Start with `ArC_Porsche/` or `ArC_Porsche_SmoothTurn/` when validating wiring and steering direction.
2. Move to `ArC_Porsche_PathFollower/` once you are ready to feed external error signals into the control loop.
3. Use `dummyController_FigureEight_ErrorCommunication.py` to generate repeatable `ERR,...` traffic while tuning.

If one of these variants becomes the preferred control baseline, promote it to `../../systems/` and move shared helpers into `../../shared/`.
