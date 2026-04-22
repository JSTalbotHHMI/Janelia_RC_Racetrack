# Host

This folder is reserved for larger host-side applications and services that interact with the racetrack hardware.

Possible future contents include:

- camera ingestion or vision pipelines that grow beyond single-file utilities
- operator dashboards
- serial or network bridge services
- calibration, monitoring, or race-management apps with their own application structure

Today, most host-side logic still lives in `../utilities/` or in Python tools under `../firmware/`, so this directory remains a placeholder for when those workflows need a more formal application layout.
