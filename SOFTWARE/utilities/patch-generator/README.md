# Patch Generator

`patch_generator.py` is a desktop tool for creating and editing patch-definition CSV files from a live webcam feed.

It lets a user:
- view a live camera image
- place `quadpatch` regions by clicking 4 points
- place `polypatch` regions by clicking 3 or more points
- place `circlepatch` regions by clicking a center, then dragging to set the radius
- modify existing patches directly in the video display
- save patch definitions to a CSV file
- reload an existing CSV and redraw its saved patches

## Requirements

- Python 3
- [Pillow](https://pypi.org/project/Pillow/)
- [OpenCV for Python](https://pypi.org/project/opencv-python/)
- a working webcam

Install dependencies with:

```powershell
pip install pillow opencv-python numpy
```

## Running

From this folder, run:

```powershell
python patch_generator.py
```

When the camera is available, the live feed will appear and the patch controls will enable.

## Interface

The main controls are:
- `New`: create a new patch CSV
- `Load`: open an existing patch CSV
- `Reload`: reload the currently active CSV from disk
- `Clear`: clear the current patch list and unload the active CSV
- `Quadpatch`: create a 4-corner polygon patch
- `Polypatch`: create a convex polygon patch with 3 or more points
- `Circlepatch`: create a circular patch

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

While a patch is selected, its geometry is allowed to become temporarily invalid.
When an invalid patch is deselected, the app asks whether to:
- `Edit`: keep the patch selected and continue editing from its current shape
- `Cancel`: discard all changes made while that patch was selected

The table also includes a `Wall` checkbox column:
- click the checkbox cell to mark or clear whether a patch is wall-adjacent
- the wall-adjacent flag is saved into the CSV

## Creating A New CSV

When a user creates a new CSV:
- if they choose a normal folder, the file is created in that folder's `output/` subfolder
- if the `output/` subfolder does not exist, it is created automatically
- if they choose an `output/` folder directly, that same folder is used

Generated CSV files are intended to live in `output/` instead of beside the source code.

## Loading And Reloading

- `Load` opens an existing CSV file from any location.
- `Reload` re-reads the currently active CSV from disk.
- If the active CSV file was deleted or moved, `Reload` resets the app to the same state as having no CSV loaded and shows a warning.

## CSV Format

The tool writes rows in one of these forms:

```text
quadpatch,name,wall_adjacent=true,x1,y1,x2,y2,x3,y3,x4,y4
polypatch,name,wall_adjacent=true,x1,y1,x2,y2,x3,y3,...
circlepatch,name,wall_adjacent=true,center_x,center_y,radius
```

Notes:
- quadpatch rows must define a convex quadrilateral
- polypatch rows must define a convex polygon and include at least 3 point pairs
- circlepatch rows must have a positive radius
- `wall_adjacent=true` or `wall_adjacent=false` metadata is supported and optional when loading older files
- blank rows are ignored
- simple header-style rows starting with `label` or `type` are ignored

## Typical Workflow

1. Start the tool.
2. Click `New` to create a CSV, or `Load` to open an existing one.
3. Click `Quadpatch`, `Polypatch`, or `Circlepatch`.
4. Click points in the video feed to define the patch.
5. For `Circlepatch`, click once to place the center, then drag and release to set the radius.
6. For `Polypatch`, press `Enter` after selecting at least 3 points.
7. Enter a patch name when prompted.
8. Continue adding patches as needed.
8. Use `Reload` if the CSV changed on disk outside the app.

## Files In This Folder

- [patch_generator.py](C:\Users\talbotj\Documents\GitHub\Janelia_RC_Racetrack\SOFTWARE\utilities\patch-generator\patch_generator.py): main application
- `assets/`: static assets used by the GUI, including the window icon
- `output/`: generated CSV files
