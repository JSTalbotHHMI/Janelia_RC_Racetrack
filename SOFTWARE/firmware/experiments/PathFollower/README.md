# PathFollower

This directory contains two desktop tools for defining and following closed driving paths with a camera-tracked RC car.

- `janelia_path_generator.py` lets you draw a closed path over a live camera image or a black canvas and save it as JSON.
- `janelia_path_follower.py` loads that JSON path, tracks red and blue blobs on the car, selects a target point on the path, and sends control error terms over serial.

## Overview

The intended workflow is:

1. Open the path generator.
2. Select the overhead camera from the camera dropdown.
3. Draw or edit control points over the track image.
4. Generate and save a path JSON file.
5. Open the follower.
6. Select the camera and serial port.
7. Load the saved path JSON.
8. Start following.

The follower assumes:

- blue is the front marker
- red is the rear marker
- vehicle position is the midpoint between the two markers
- heading points from red to blue

## Requirements

Install Python 3 and the required packages:

```bash
pip install numpy opencv-python pillow pyserial
```

`pyserial` is only required for the follower.

## Path Generator

Run:

```bash
python janelia_path_generator.py
```

Features:

- camera dropdown with refresh button
- live camera background when a camera is available
- black-canvas fallback when no camera is available
- closed spline generation from control points
- resampling into evenly distributed path samples
- JSON export with per-sample speed and heading-offset metadata

Controls:

- single-click empty space: add a control point
- drag a point: move it
- double-click near a segment: insert a point
- `Undo`: remove the last point
- `Clear`: remove all control points
- `Generate`: build sampled path points
- `Save As`: save the path JSON

Important fields:

- `Samples`: number of path samples to export
- `Lookahead px`: default lookahead radius stored in the JSON file
- `Default speed`: speed value written into each sample
- `Heading offset`: heading offset written into each sample

## Path File Format

Saved paths use JSON format `janelia_rc_path_v1` and include:

- `lookahead_radius_px`
- `control_points`
- `samples`

Each sample contains:

- `sample_id`
- `x`, `y`
- `s_norm`
- `speed_mps`
- `heading_offset_deg`

## Path Follower

Run:

```bash
python janelia_path_follower.py
```

Features:

- camera dropdown with refresh button
- serial port and baud selection
- path JSON loading
- red/blue blob tracking in HSV color space
- lookahead-circle target selection on the path
- nearest-point fallback when the lookahead circle does not intersect the path
- live overlay showing blobs, pose, trail, lookahead circle, and target point

Serial output format:

```text
ERR,<speed_error_mps>,<heading_error_rad>
```

Current control behavior:

- heading is computed from the red marker toward the blue marker
- target heading is the bearing to the selected target point plus that sample's heading offset
- heading error is angle-wrapped before transmission
- if the lookahead circle misses the path, the nearest point on the path is used until intersections reappear
- if tracking is lost for too long, the app sends `ERR,0,0`
- stopping the app also sends `ERR,0,0`

## Camera Selection

Both apps probe camera indices `0` through `8` at startup and populate the dropdown with the indices that open successfully. Use `Refresh` if you connect or disconnect a camera while the app is open.

## Notes

- The apps use pixel coordinates from the camera image. There is no world-coordinate calibration in this directory.
- The follower expects a closed path.
- The serial consumer must interpret the `ERR,...` packet format correctly on the firmware side.
