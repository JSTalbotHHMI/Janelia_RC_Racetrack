# Software

This directory contains the software side of the racetrack project, from embedded sketches to desktop utilities.

## Layout

- [`firmware/`](firmware/README.md): Arduino and Teensy sketches, plus embedded support code
- [`host/`](host/README.md): future host-side applications that talk to hardware or cameras
- [`shared/`](shared/README.md): future shared formats, protocol definitions, and reusable software assets
- [`utilities/`](utilities/README.md): standalone support scripts and tools used during development

## Current Focus

The most complete software workflow in the repository today is the reactive LED pipeline:

1. Define track regions as patches with `utilities/patch-generator/`.
2. Convert those patch CSV files into an Arduino header with `utilities/patch-to-header/`.
3. Include the generated header in the loadable reactive LED sketch under `firmware/experiments/`.

The firmware experiments directory also contains RC car control sketches for the ArC Porsche platform, including a Python sender for synthetic path-following error packets.

## Notes

- Several sketches target Teensy hardware and reference `FastLED`.
- The ArC Porsche control sketches use the `MCP4261` digital potentiometer library.
- Python helper scripts in this repo use packages such as `opencv-python`, `Pillow`, `numpy`, and `pyserial` depending on the tool.
