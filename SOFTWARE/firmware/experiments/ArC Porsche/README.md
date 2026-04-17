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

## Notes

- Several sketches reference Teensy 4.0 wiring and software SPI connections for the MCP4261.
- Calibration values are sketch-local right now, so compare constants before assuming one experiment's steering or throttle ranges match another's.
- If one of these variants becomes the preferred control baseline, consider promoting it to `../../systems/` and moving shared helpers into `../../shared/`.
