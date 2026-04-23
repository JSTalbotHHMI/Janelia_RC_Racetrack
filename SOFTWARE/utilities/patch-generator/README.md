# Patch Generator

`patch_generator.py` is a desktop tool for creating and editing patch-definition CSV files from a live webcam feed.

It fits into the lighting workflow like this:

1. draw or edit patches here and save a CSV
2. export that CSV to a header, or convert it with [`../patch-to-header/`](../patch-to-header/README.md)
3. optionally stream the CSV live to a controller with `serial_patch_loader.py`
4. include the generated header in a firmware sketch when you want a compiled-in startup patch set

## Features

The tool lets you:

- view a live camera image
- place `quadpatch` rectangles by clicking two points for one side and a third point for the width
- place `polypatch` regions by clicking three or more points, including concave shapes
- place `circlepatch` regions by clicking a center, then dragging to set the radius
- modify existing patches directly in the video display
- load a tracker-style camera calibration CSV and apply those webcam settings
- save patch definitions to a CSV file
- export the active CSV directly to an Arduino header
- reload an existing CSV and redraw its saved patches
- mark whether a patch is wall-adjacent
- reorder patches in the CSV and table
- assign patch names from a fixed function list instead of free text

## Requirements

- Python 3
- `opencv-python`
- `numpy`
- `Pillow`
- `pyserial`
- `tkinter`
- a working webcam

Install the Python packages with:

```powershell
pip install opencv-python numpy pillow pyserial
```

## Running

From this folder:

```powershell
python patch_generator.py
```

When the camera opens successfully, the live feed appears and the patch controls become available.

## Main Controls

- `New`
  Create a new patch CSV.
- `Load`
  Open an existing patch CSV from disk.
- `Reload`
  Re-read the currently active CSV.
- `Export`
  Run the shared `patch-to-header.py` converter and write a `.h` file beside the active CSV.
- `Calibration`
  Choose a camera-settings CSV in the tracker format.
- `Clear`
  Clear the current patch list and unload the active CSV.
- `Quadpatch`
  Create a rectangle by clicking two points for a side and a third point for its width.
- `Polypatch`
  Create a simple polygon patch with three or more points.
- `Circlepatch`
  Create a circular patch.
- `Up` / `Down`
  Move the selected patch earlier or later in the table and CSV order.
- `Delete`
  Delete the selected patch.

## Editing Existing Patches

After a CSV is loaded, existing patches can be modified directly in the video display:

- click a patch to select it
- drag inside a selected patch to move it
- drag a vertex handle on a quadpatch or polypatch to reshape it
- `Shift+Click` near a selected polypatch edge to insert a new vertex
- click a polypatch vertex handle, then press `Delete` or `Backspace`, to remove that vertex
- drag the center handle on a circlepatch to move it
- drag the outer handle on a circlepatch to change its radius
- press `Esc` to cancel an in-progress edit

Edits are written back to the active CSV when the patch is deselected.

If a patch becomes invalid while it is selected, the app lets you either continue editing or discard those changes when the patch is deselected.

## Patch Naming

New patches no longer use free-text names. Instead, the save dialog lets you choose from a fixed function list:

- `finish`
- `third`
- `corner`
- `point`
- `DQ`
- `ignore`

The generator appends the next available number for numbered function names, such as `corner1`, `corner2`, or `point3`.

Special rules:

- `finish` can exist only once, and it is saved as `finish`
- `third` can exist only twice, as `third1` and `third2`

## Creating A New CSV

When you create a new CSV:

- choosing a normal folder creates the file inside that folder's `output/` subfolder
- the `output/` subfolder is created automatically when needed
- choosing an `output/` folder directly uses that folder as-is

Generated CSV files are intended to live in `output/` rather than beside the source code.

## Live Serial Patch Streaming

[`serial_patch_loader.py`](C:\Users\talbotj\Documents\GitHub\Janelia_RC_Racetrack\SOFTWARE\utilities\patch-generator\serial_patch_loader.py) keeps a serial connection open to the controller, uploads the current CSV once, and re-uploads whenever the CSV changes on disk.

That is useful with patch-loading controller sketches when you want edits to take effect immediately without reflashing.

Example:

```powershell
python serial_patch_loader.py COM5 .\output\NewPatches.csv
```

Upload a runtime segment layout before the patches:

```powershell
python serial_patch_loader.py COM5 .\output\NewPatches.csv --segment-lengths 12,8,15
```

One-shot upload:

```powershell
python serial_patch_loader.py COM5 .\output\NewPatches.csv --once
```

Notes:

- the script reuses the shared CSV parser from `patch-to-header.py`
- `--segment-lengths` updates the active segment count and per-segment LED lengths before uploading patches
- patches can target `segmentIndex=-1` to exist without an LED segment
- the controller keeps its last valid patch set active if an upload fails validation
- the serial port stays open, so repeated CSV edits do not require reconnecting

## Camera Calibration CSV

The `Calibration` button accepts the same CSV format used by the tracker tools:

```text
property_name,property_id,value
```

Those settings are applied through OpenCV `capture.set(...)`. If the generator finds a tracker calibration CSV on disk, it uses that as the initial calibration file on startup.

## CSV Format

The tool writes rows in one of these forms:

```text
quadpatch,name,wall_adjacent=true,x1,y1,x2,y2,x3,y3,x4,y4[,segmentIndex]
polypatch,name,wall_adjacent=true,x1,y1,x2,y2,x3,y3,...[,segmentIndex]
circlepatch,name,wall_adjacent=true,center_x,center_y,radius[,segmentIndex]
```

Notes:

- quadpatch rows are saved as four rectangle corners
- polypatch rows may be concave, but they must remain simple non-self-intersecting polygons with at least three point pairs
- circlepatch rows must have a positive radius
- `wall_adjacent=true` or `wall_adjacent=false` metadata is supported
- blank rows are ignored
- simple header-style rows starting with `label` or `type` are ignored

## Typical Workflow

1. Start the tool.
2. Click `New` to create a CSV, or `Load` to open an existing one.
3. Click `Quadpatch`, `Polypatch`, or `Circlepatch`.
4. Click points in the video feed to define the patch.
   `Quadpatch` uses three clicks: side start, side end, then width.
5. For `Circlepatch`, click once to place the center, then drag and release to set the radius.
6. For `Polypatch`, press `Enter` after selecting at least three points.
7. Enter a patch name when prompted.
8. Continue adding or editing patches as needed.
9. Click `Export` to generate the matching header file.
10. Use `serial_patch_loader.py` if you want the active controller to pick up CSV changes live.

## Files In This Folder

- [patch_generator.py](C:\Users\talbotj\Documents\GitHub\Janelia_RC_Racetrack\SOFTWARE\utilities\patch-generator\patch_generator.py)
  Main GUI application.
- [serial_patch_loader.py](C:\Users\talbotj\Documents\GitHub\Janelia_RC_Racetrack\SOFTWARE\utilities\patch-generator\serial_patch_loader.py)
  Serial watcher and uploader for runtime patch updates.
- `assets/`
  Static assets used by the GUI, including the window icon.
- `output/`
  Generated CSV and header files.
