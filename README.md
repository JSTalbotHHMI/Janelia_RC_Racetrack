# Janelia_RC_Racetrack
![Version](https://img.shields.io/badge/version-dev-lightgrey)

Software tools and fabrication files for the Janelia "Take Your Kid to Work Day" racetrack

## Repository Layout

```text
Janelia_RC_Racetrack/
  ECAD/        Electrical design files
  MCAD/        Mechanical design files
  SOFTWARE/
    firmware/  Arduino and other embedded code
      systems/
      experiments/
      shared/
    host/      Computer-side software that talks to hardware
    shared/    Shared software assets, protocols, and reusable code
    utilities/ Standalone tools and generated outputs
```

## Notes

- Put production firmware in `SOFTWARE/firmware/systems/`.
- Put one-off or diagnostic sketches in `SOFTWARE/firmware/experiments/`.
- Put standalone desktop tools in `SOFTWARE/utilities/`.
- Put generated files in an `output/` folder inside the relevant tool or workflow folder.
