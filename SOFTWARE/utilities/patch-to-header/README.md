# Patch To Header

`patch-to-header.py` converts a patch-definition CSV into an Arduino-friendly header file.

It is designed to pair with the CSV files created by [`../patch-generator/`](../patch-generator/README.md), though it can also be used with hand-edited CSV files that follow the same format.

## What It Produces

The script writes a header that defines:

- a `Patch` array
- a patch count constant

The generated header assumes the including sketch already defines:

- `Patch`
- `PATCH_CIRCLE`
- `PATCH_QUAD`
- `PATCH_POLY`

## Input Format

Supported rows look like:

```text
quadpatch,name,wall_adjacent=true,x1,y1,x2,y2,x3,y3,x4,y4[,segmentIndex]
polypatch,name,wall_adjacent=true,x1,y1,x2,y2,x3,y3,...[,segmentIndex]
circlepatch,name,wall_adjacent=true,center_x,center_y,radius[,segmentIndex]
```

Notes:

- `wall_adjacent` is optional.
- `segmentIndex` can be supplied explicitly or omitted and assigned automatically.
- blank rows and simple header rows beginning with `label`, `labels`, or `type` are ignored.

## Usage

From this folder:

```powershell
python patch-to-header.py ..\patch-generator\output\PythonCirclePath.csv
```

Write to a specific output path:

```powershell
python patch-to-header.py ..\patch-generator\output\PythonCirclePath.csv --output .\output\PythonCirclePath.h
```

Customize symbol names:

```powershell
python patch-to-header.py ..\patch-generator\output\PythonCirclePath.csv --array-name patches --count-name PATCH_COUNT
```

If `--output` is omitted, the script writes the header into an `output/` folder beside the CSV.
