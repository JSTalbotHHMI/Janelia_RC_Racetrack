#!/usr/bin/env python3
"""
Interactive webcam calibration utility for blob tracking.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import dataclass
from pathlib import Path

try:
    import cv2
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "opencv-python is required. Install it with: pip install opencv-python"
    ) from exc

try:
    import tkinter as tk
    from tkinter import filedialog, ttk
except ImportError as exc:  # pragma: no cover
    raise SystemExit("tkinter is required for the calibration GUI.") from exc


DEFAULT_CALIBRATION_CSV = Path(__file__).with_name("camera_calibration.csv")
PREVIEW_WINDOW_NAME = "Camera Calibration - Video"
PREVIEW_INTERVAL_MS = 15
VALUE_POLL_INTERVAL_MS = 120
VALUE_HIGHLIGHT_DURATION_S = 3.0
SELECTED_ROW_BG = "#dbeafe"
NORMAL_CELL_BG = "#ffffff"


@dataclass(frozen=True)
class CameraProperty:
    name: str
    prop_id: int
    min_value: float
    max_value: float
    display_decimals: int = 0
    is_toggle: bool = False


CAMERA_PROPERTIES = (
    CameraProperty("auto_exposure", cv2.CAP_PROP_AUTO_EXPOSURE, 0.0, 1.0, 0, True),
    CameraProperty("exposure", cv2.CAP_PROP_EXPOSURE, -13.0, 0.0, 2),
    CameraProperty("auto_wb", cv2.CAP_PROP_AUTO_WB, 0.0, 1.0, 0, True),
    CameraProperty("wb_temperature", cv2.CAP_PROP_WB_TEMPERATURE, 2800.0, 7500.0, 0),
    CameraProperty("autofocus", cv2.CAP_PROP_AUTOFOCUS, 0.0, 1.0, 0, True),
    CameraProperty("focus", cv2.CAP_PROP_FOCUS, 0.0, 255.0, 0),
    CameraProperty("brightness", cv2.CAP_PROP_BRIGHTNESS, 0.0, 255.0, 0),
    CameraProperty("contrast", cv2.CAP_PROP_CONTRAST, 0.0, 255.0, 0),
    CameraProperty("saturation", cv2.CAP_PROP_SATURATION, 0.0, 255.0, 0),
    CameraProperty("hue", cv2.CAP_PROP_HUE, 0.0, 255.0, 0),
    CameraProperty("gain", cv2.CAP_PROP_GAIN, 0.0, 255.0, 0),
    CameraProperty("gamma", cv2.CAP_PROP_GAMMA, 1.0, 500.0, 0),
    CameraProperty("sharpness", cv2.CAP_PROP_SHARPNESS, 0.0, 255.0, 0),
    CameraProperty("backlight", cv2.CAP_PROP_BACKLIGHT, 0.0, 10.0, 0),
    CameraProperty("zoom", cv2.CAP_PROP_ZOOM, 0.0, 255.0, 0),
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Tune webcam properties through a GUI and save them to a CSV for "
            "blob_tracker.py to reuse."
        )
    )
    parser.add_argument("--camera", default="0", help="OpenCV camera index or video path. Default: 0.")
    parser.add_argument(
        "--backend",
        choices=("auto", "any", "dshow", "msmf"),
        default="auto",
        help="Preferred OpenCV backend for camera sources on Windows. Default: auto.",
    )
    parser.add_argument(
        "--output",
        default=str(DEFAULT_CALIBRATION_CSV),
        help=(
            "Default CSV path to suggest when creating or loading files. "
            f"Default: {DEFAULT_CALIBRATION_CSV.name} beside this script."
        ),
    )
    parser.add_argument("--width", type=int, default=1920, help="Requested capture width in pixels. Default: 1920.")
    parser.add_argument("--height", type=int, default=1080, help="Requested capture height in pixels. Default: 1080.")
    return parser


def parse_camera_source(raw_value: str) -> int | str:
    try:
        return int(raw_value)
    except ValueError:
        return raw_value


def backend_candidates(video_source: int | str, backend_name: str) -> list[tuple[str, int]]:
    cap_any = getattr(cv2, "CAP_ANY", 0)
    cap_dshow = getattr(cv2, "CAP_DSHOW", cap_any)
    cap_msmf = getattr(cv2, "CAP_MSMF", cap_any)
    named_backends = {
        "any": [("any", cap_any)],
        "dshow": [("dshow", cap_dshow)],
        "msmf": [("msmf", cap_msmf)],
    }
    if backend_name != "auto":
        return named_backends[backend_name]
    if isinstance(video_source, int):
        return [("dshow", cap_dshow), ("any", cap_any), ("msmf", cap_msmf)]
    return [("any", cap_any)]


def open_video_capture(
    video_source: int | str,
    backend_name: str,
    requested_width: int,
    requested_height: int,
) -> tuple[cv2.VideoCapture, str]:
    for candidate_name, backend_id in backend_candidates(video_source, backend_name):
        capture = cv2.VideoCapture(video_source, backend_id)
        if capture.isOpened():
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if isinstance(video_source, int):
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, requested_width)
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, requested_height)
            return capture, candidate_name
        capture.release()
    raise SystemExit(
        f"Could not open video source {video_source!r}. "
        "If this is a webcam on Windows, try --backend dshow."
    )


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def format_property_value(property_info: CameraProperty, value: float) -> str:
    if property_info.is_toggle:
        return "ON" if value >= 0.5 else "OFF"
    return f"{value:.{property_info.display_decimals}f}"


def values_match(property_info: CameraProperty, old_value: float, new_value: float) -> bool:
    if property_info.is_toggle:
        return (old_value >= 0.5) == (new_value >= 0.5)
    tolerance = max(1e-6, 10.0 ** (-(property_info.display_decimals + 1)))
    return abs(old_value - new_value) <= tolerance


def blend_hex(color_a: tuple[int, int, int], color_b: tuple[int, int, int], amount: float) -> str:
    clamped = max(0.0, min(1.0, amount))
    blended = tuple(
        int(round((1.0 - clamped) * channel_a + clamped * channel_b))
        for channel_a, channel_b in zip(color_a, color_b)
    )
    return f"#{blended[0]:02x}{blended[1]:02x}{blended[2]:02x}"


def load_camera_settings(csv_path: str) -> dict[str, float]:
    calibration_path = Path(csv_path)
    if not calibration_path.exists():
        return {}
    loaded: dict[str, float] = {}
    with calibration_path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            property_name = row.get("property_name", "").strip()
            if not property_name:
                continue
            loaded[property_name] = float(row["value"])
    return loaded


def save_csv_settings(capture: cv2.VideoCapture, output_path: str) -> None:
    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["property_name", "property_id", "value"])
        for property_info in CAMERA_PROPERTIES:
            writer.writerow([property_info.name, property_info.prop_id, f"{capture.get(property_info.prop_id):.6f}"])


def apply_named_settings(capture: cv2.VideoCapture, values_by_name: dict[str, float]) -> None:
    properties_by_name = {prop.name: prop for prop in CAMERA_PROPERTIES}
    for property_name in ("auto_exposure", "auto_wb", "autofocus"):
        if property_name not in values_by_name:
            continue
        property_info = properties_by_name[property_name]
        capture.set(property_info.prop_id, property_info.max_value if values_by_name[property_name] >= 0.5 else property_info.min_value)
    for property_info in CAMERA_PROPERTIES:
        if property_info.name not in values_by_name:
            continue
        raw_value = values_by_name[property_info.name]
        normalized = (
            property_info.max_value if (property_info.is_toggle and raw_value >= 0.5)
            else property_info.min_value if property_info.is_toggle
            else clamp(raw_value, property_info.min_value, property_info.max_value)
        )
        capture.set(property_info.prop_id, normalized)


def preview_window_is_open(window_name: str) -> bool:
    try:
        return cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) >= 1
    except cv2.error:
        return False


def fit_frame_to_window(window_name: str, frame):
    try:
        _x, _y, target_width, target_height = cv2.getWindowImageRect(window_name)
    except cv2.error:
        return frame
    if target_width <= 0 or target_height <= 0:
        return frame
    frame_height, frame_width = frame.shape[:2]
    scale = min(target_width / frame_width, target_height / frame_height)
    scaled_width = max(1, int(round(frame_width * scale)))
    scaled_height = max(1, int(round(frame_height * scale)))
    resized = cv2.resize(frame, (scaled_width, scaled_height), interpolation=cv2.INTER_LINEAR)
    import numpy as np
    canvas = np.zeros((target_height, target_width, 3), dtype=frame.dtype)
    x_offset = (target_width - scaled_width) // 2
    y_offset = (target_height - scaled_height) // 2
    canvas[y_offset:y_offset + scaled_height, x_offset:x_offset + scaled_width] = resized
    return canvas


class CalibrationApp:
    def __init__(self, capture: cv2.VideoCapture, backend_name: str, default_output: str) -> None:
        self.capture = capture
        self.backend_name = backend_name
        self.default_output = default_output
        self.active_csv_path: str | None = None
        self.decimal_place = 0
        self.running = True
        self.selected_property_name = CAMERA_PROPERTIES[0].name
        self.property_rows: dict[str, dict[str, object]] = {}
        self.last_values = {prop.name: self.capture.get(prop.prop_id) for prop in CAMERA_PROPERTIES}
        self.last_change_times = {prop.name: 0.0 for prop in CAMERA_PROPERTIES}
        self.display_values = dict(self.last_values)
        self.held_properties = {prop.name: False for prop in CAMERA_PROPERTIES}
        self.preview_after_id: str | None = None
        self.poll_after_id: str | None = None

        self.root = tk.Tk()
        self.root.title("Camera Calibration Controls")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.status_var = tk.StringVar(value="Create a new CSV or load an existing one to enable Save and Reload.")
        self.active_csv_var = tk.StringVar(value="Active CSV: (none)")
        self.decimal_var = tk.StringVar(value="Adjustment step: 10^0")

        self._build_layout()
        self._populate_table()
        self._refresh_button_states()
        self._schedule_preview()
        self._schedule_value_poll()

    def _build_layout(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(2, weight=1)

        top_bar = ttk.Frame(self.root, padding=10)
        top_bar.grid(row=0, column=0, sticky="ew")
        for column in range(4):
            top_bar.columnconfigure(column, weight=1)
        ttk.Button(top_bar, text="New", command=self.create_new_file).grid(row=0, column=0, padx=4, sticky="ew")
        self.load_button = ttk.Button(top_bar, text="Load", command=self.load_file)
        self.load_button.grid(row=0, column=1, padx=4, sticky="ew")
        self.save_button = ttk.Button(top_bar, text="Save", command=self.save_active_file)
        self.save_button.grid(row=0, column=2, padx=4, sticky="ew")
        self.reload_button = ttk.Button(top_bar, text="Reload", command=self.reload_active_file)
        self.reload_button.grid(row=0, column=3, padx=4, sticky="ew")

        info_frame = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        info_frame.grid(row=1, column=0, sticky="ew")
        ttk.Label(info_frame, text=f"Backend: {self.backend_name}").grid(row=0, column=0, sticky="w")
        ttk.Label(info_frame, textvariable=self.active_csv_var).grid(row=1, column=0, sticky="w", pady=(4, 0))
        ttk.Label(info_frame, textvariable=self.decimal_var).grid(row=2, column=0, sticky="w", pady=(4, 0))

        table_outer = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        table_outer.grid(row=2, column=0, sticky="nsew")
        table_outer.columnconfigure(0, weight=1)
        table_outer.rowconfigure(0, weight=1)
        self.canvas = tk.Canvas(table_outer, highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(table_outer, orient="vertical", command=self.canvas.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.canvas.configure(yscrollcommand=scrollbar.set)
        self.table_frame = ttk.Frame(self.canvas)
        self.canvas_window = self.canvas.create_window((0, 0), window=self.table_frame, anchor="nw")
        self.table_frame.bind("<Configure>", self._on_table_configure)
        self.canvas.bind("<Configure>", self._on_canvas_configure)

        controls_frame = ttk.Frame(self.root, padding=10)
        controls_frame.grid(row=3, column=0, sticky="ew")
        for column in range(7):
            controls_frame.columnconfigure(column, weight=1)
        ttk.Button(controls_frame, text="Previous", command=self.select_previous).grid(row=0, column=0, padx=4, pady=4, sticky="ew")
        ttk.Button(controls_frame, text="Next", command=self.select_next).grid(row=0, column=1, padx=4, pady=4, sticky="ew")
        ttk.Button(controls_frame, text="Decrease", command=lambda: self.adjust_selected(-1)).grid(row=0, column=2, padx=4, pady=4, sticky="ew")
        ttk.Button(controls_frame, text="Increase", command=lambda: self.adjust_selected(1)).grid(row=0, column=3, padx=4, pady=4, sticky="ew")
        ttk.Button(controls_frame, text="Decimal Left", command=self.decimal_left).grid(row=0, column=4, padx=4, pady=4, sticky="ew")
        ttk.Button(controls_frame, text="Decimal Right", command=self.decimal_right).grid(row=0, column=5, padx=4, pady=4, sticky="ew")
        ttk.Button(controls_frame, text="Toggle Selected", command=self.toggle_selected).grid(row=0, column=6, padx=4, pady=4, sticky="ew")

        status_frame = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        status_frame.grid(row=4, column=0, sticky="ew")
        ttk.Label(status_frame, textvariable=self.status_var, wraplength=780).grid(row=0, column=0, sticky="w")

    def _populate_table(self) -> None:
        headers = ("Property", "Value", "Range", "Type", "Hold", "Held")
        for column, header in enumerate(headers):
            tk.Label(self.table_frame, text=header, font=("TkDefaultFont", 9, "bold"), bg="#e5e7eb", relief="solid", borderwidth=1, padx=6, pady=6).grid(row=0, column=column, sticky="ew", padx=1, pady=1)
        for i, weight in enumerate((3, 2, 2, 1, 1, 1)):
            self.table_frame.columnconfigure(i, weight=weight)
        for row_index, property_info in enumerate(CAMERA_PROPERTIES, start=1):
            property_label = self._make_cell(property_info.name, "w")
            property_label.grid(row=row_index, column=0, sticky="ew", padx=1, pady=1)
            value_label = self._make_cell("", "center")
            value_label.grid(row=row_index, column=1, sticky="ew", padx=1, pady=1)
            range_label = self._make_cell(f"{property_info.min_value:.{property_info.display_decimals}f} - {property_info.max_value:.{property_info.display_decimals}f}", "center")
            range_label.grid(row=row_index, column=2, sticky="ew", padx=1, pady=1)
            type_label = self._make_cell("Toggle" if property_info.is_toggle else "Numeric", "center")
            type_label.grid(row=row_index, column=3, sticky="ew", padx=1, pady=1)
            hold_button = ttk.Button(self.table_frame, text="Hold", command=lambda name=property_info.name: self.hold_property(name))
            hold_button.grid(row=row_index, column=4, sticky="ew", padx=1, pady=1)
            held_label = self._make_cell("No", "center")
            held_label.grid(row=row_index, column=5, sticky="ew", padx=1, pady=1)
            for widget in (property_label, value_label, range_label, type_label, held_label):
                widget.bind("<Button-1>", lambda _event, name=property_info.name: self.select_property_by_name(name))
            self.property_rows[property_info.name] = {
                "property_label": property_label,
                "value_label": value_label,
                "range_label": range_label,
                "type_label": type_label,
                "held_label": held_label,
                "hold_button": hold_button,
            }
        self.refresh_table()

    def _make_cell(self, text: str, anchor: str) -> tk.Label:
        return tk.Label(self.table_frame, text=text, anchor=anchor, bg=NORMAL_CELL_BG, relief="solid", borderwidth=1, padx=6, pady=5)

    def _on_table_configure(self, _event) -> None:
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _on_canvas_configure(self, event) -> None:
        self.canvas.itemconfigure(self.canvas_window, width=event.width)

    def refresh_table(self) -> None:
        if not self.running:
            return
        now = time.monotonic()
        for property_info in CAMERA_PROPERTIES:
            current_value = self.capture.get(property_info.prop_id)
            row = self.property_rows[property_info.name]
            row["value_label"].configure(text=format_property_value(property_info, self.display_values[property_info.name]))
            if not values_match(property_info, self.last_values[property_info.name], current_value):
                self.last_values[property_info.name] = current_value
                self.last_change_times[property_info.name] = now
                if not property_info.is_toggle:
                    self.display_values[property_info.name] = current_value
            fade_amount = min(max((now - self.last_change_times[property_info.name]) / VALUE_HIGHLIGHT_DURATION_S, 0.0), 1.0)
            row["value_label"].configure(bg=blend_hex((255, 242, 122), (255, 255, 255), fade_amount))
            row_bg = SELECTED_ROW_BG if property_info.name == self.selected_property_name else NORMAL_CELL_BG
            for label_key in ("property_label", "range_label", "type_label"):
                row[label_key].configure(bg=row_bg)
            held = self.held_properties[property_info.name]
            row["held_label"].configure(
                text="Yes" if held else "No",
                bg="#dcfce7" if held else row_bg,
                fg="#166534" if held else "#111111",
            )
            row["hold_button"].configure(text="Release" if held else "Hold")

    def _refresh_button_states(self) -> None:
        state = "normal" if self.active_csv_path is not None else "disabled"
        self.save_button.configure(state=state)
        self.reload_button.configure(state=state)
        self.active_csv_var.set(f"Active CSV: {self.active_csv_path if self.active_csv_path else '(none)'}")

    def select_property_by_name(self, property_name: str) -> None:
        self.selected_property_name = property_name
        self.refresh_table()

    def selected_property(self) -> CameraProperty | None:
        for property_info in CAMERA_PROPERTIES:
            if property_info.name == self.selected_property_name:
                return property_info
        return None

    def select_previous(self) -> None:
        current = self.selected_property()
        if current is None:
            return
        index = next(i for i, prop in enumerate(CAMERA_PROPERTIES) if prop.name == current.name)
        self.select_property_by_name(CAMERA_PROPERTIES[(index - 1) % len(CAMERA_PROPERTIES)].name)

    def select_next(self) -> None:
        current = self.selected_property()
        if current is None:
            return
        index = next(i for i, prop in enumerate(CAMERA_PROPERTIES) if prop.name == current.name)
        self.select_property_by_name(CAMERA_PROPERTIES[(index + 1) % len(CAMERA_PROPERTIES)].name)

    def write_property(self, property_info: CameraProperty, requested_value: float) -> float:
        self.capture.set(property_info.prop_id, requested_value)
        actual_value = self.capture.get(property_info.prop_id)
        self.display_values[property_info.name] = requested_value if property_info.is_toggle else actual_value
        return actual_value

    def adjust_selected(self, direction: int) -> None:
        property_info = self.selected_property()
        if property_info is None:
            self.status_var.set("Select a property first.")
            return
        current_value = self.capture.get(property_info.prop_id)
        next_value = property_info.max_value if (property_info.is_toggle and direction > 0) else (
            property_info.min_value if property_info.is_toggle else clamp(current_value + (direction * (10.0 ** self.decimal_place)), property_info.min_value, property_info.max_value)
        )
        actual_value = self.write_property(property_info, next_value)
        self.last_values[property_info.name] = actual_value
        self.last_change_times[property_info.name] = time.monotonic()
        self.refresh_table()
        self.status_var.set(f"{property_info.name} set to {format_property_value(property_info, self.display_values[property_info.name])}")

    def toggle_selected(self) -> None:
        property_info = self.selected_property()
        if property_info is None:
            self.status_var.set("Select a property first.")
            return
        current_value = self.capture.get(property_info.prop_id)
        if property_info.is_toggle:
            next_value = property_info.min_value if current_value >= 0.5 else property_info.max_value
        else:
            midpoint = 0.5 * (property_info.min_value + property_info.max_value)
            next_value = property_info.min_value if current_value > midpoint else property_info.max_value
        actual_value = self.write_property(property_info, next_value)
        self.last_values[property_info.name] = actual_value
        self.last_change_times[property_info.name] = time.monotonic()
        self.refresh_table()
        self.status_var.set(f"{property_info.name} toggled to {format_property_value(property_info, self.display_values[property_info.name])}")

    def hold_property(self, property_name: str) -> None:
        property_info = next(prop for prop in CAMERA_PROPERTIES if prop.name == property_name)
        if self.held_properties[property_name]:
            self.held_properties[property_name] = False
            self.refresh_table()
            self.status_var.set(f"Released hold on {property_name}")
            return
        requested_value = self.capture.get(property_info.prop_id)
        if property_info.name in ("auto_exposure", "auto_wb", "autofocus"):
            requested_value = property_info.min_value
        actual_value = self.write_property(property_info, requested_value)
        self.last_values[property_info.name] = actual_value
        self.last_change_times[property_info.name] = time.monotonic()
        self.held_properties[property_name] = True
        self.refresh_table()
        self.status_var.set(f"Held {property_info.name} at {format_property_value(property_info, self.display_values[property_info.name])}")

    def decimal_left(self) -> None:
        self.decimal_place = max(self.decimal_place - 1, -3)
        self.decimal_var.set(f"Adjustment step: 10^{self.decimal_place}")

    def decimal_right(self) -> None:
        self.decimal_place = min(self.decimal_place + 1, 3)
        self.decimal_var.set(f"Adjustment step: 10^{self.decimal_place}")

    def enforce_held_properties(self) -> None:
        for property_info in CAMERA_PROPERTIES:
            if not self.held_properties[property_info.name]:
                continue
            requested_value = self.display_values[property_info.name]
            actual_value = self.write_property(property_info, requested_value)
            self.last_values[property_info.name] = actual_value

    def create_new_file(self) -> None:
        target = filedialog.asksaveasfilename(parent=self.root, title="Create camera calibration CSV", initialdir=str(Path(self.active_csv_path or self.default_output).parent), initialfile=Path(self.active_csv_path or self.default_output).name, defaultextension=".csv", filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        if not target:
            return
        self.active_csv_path = target
        save_csv_settings(self.capture, self.active_csv_path)
        self.held_properties = {prop.name: False for prop in CAMERA_PROPERTIES}
        self._refresh_button_states()
        self.refresh_table()
        self.status_var.set(f"Created calibration file {self.active_csv_path}")

    def load_file(self) -> None:
        target = filedialog.askopenfilename(parent=self.root, title="Load camera calibration CSV", initialdir=str(Path(self.active_csv_path or self.default_output).parent), filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        if not target:
            return
        apply_named_settings(self.capture, load_camera_settings(target))
        self.active_csv_path = target
        self.last_values = {prop.name: self.capture.get(prop.prop_id) for prop in CAMERA_PROPERTIES}
        self.display_values = dict(self.last_values)
        self.held_properties = {prop.name: False for prop in CAMERA_PROPERTIES}
        self.refresh_table()
        self._refresh_button_states()
        self.status_var.set(f"Loaded calibration file {self.active_csv_path}")

    def save_active_file(self) -> None:
        if self.active_csv_path is None:
            return
        save_csv_settings(self.capture, self.active_csv_path)
        self.status_var.set(f"Saved calibration to {self.active_csv_path}")

    def reload_active_file(self) -> None:
        if self.active_csv_path is None:
            return
        apply_named_settings(self.capture, load_camera_settings(self.active_csv_path))
        self.last_values = {prop.name: self.capture.get(prop.prop_id) for prop in CAMERA_PROPERTIES}
        self.display_values = dict(self.last_values)
        self.held_properties = {prop.name: False for prop in CAMERA_PROPERTIES}
        self.refresh_table()
        self.status_var.set(f"Reloaded calibration from {self.active_csv_path}")

    def _schedule_preview(self) -> None:
        if not self.running:
            return
        try:
            if not self.root.winfo_exists():
                self.close()
                return
        except tk.TclError:
            self.close()
            return
        ok, frame = self.capture.read()
        if ok and frame is not None:
            cv2.imshow(PREVIEW_WINDOW_NAME, fit_frame_to_window(PREVIEW_WINDOW_NAME, frame))
            cv2.waitKey(1)
        else:
            self.status_var.set("Camera frame read failed.")
        if not preview_window_is_open(PREVIEW_WINDOW_NAME):
            self.close()
            return
        try:
            self.preview_after_id = self.root.after(PREVIEW_INTERVAL_MS, self._schedule_preview)
        except tk.TclError:
            self.close()

    def _schedule_value_poll(self) -> None:
        if not self.running:
            return
        self.enforce_held_properties()
        self.refresh_table()
        try:
            self.poll_after_id = self.root.after(VALUE_POLL_INTERVAL_MS, self._schedule_value_poll)
        except tk.TclError:
            self.close()

    def close(self) -> None:
        if not self.running:
            return
        self.running = False
        try:
            if self.preview_after_id is not None and self.root.winfo_exists():
                self.root.after_cancel(self.preview_after_id)
        except tk.TclError:
            pass
        try:
            if self.poll_after_id is not None and self.root.winfo_exists():
                self.root.after_cancel(self.poll_after_id)
        except tk.TclError:
            pass
        self.capture.release()
        cv2.destroyAllWindows()
        try:
            if self.root.winfo_exists():
                self.root.destroy()
        except tk.TclError:
            pass

    def run(self) -> int:
        self.root.mainloop()
        return 0


def main() -> int:
    args = build_parser().parse_args()
    if args.width <= 0:
        raise SystemExit("--width must be greater than 0")
    if args.height <= 0:
        raise SystemExit("--height must be greater than 0")
    video_source = parse_camera_source(args.camera)
    capture, backend_name = open_video_capture(video_source, args.backend, args.width, args.height)
    actual_width = int(round(capture.get(cv2.CAP_PROP_FRAME_WIDTH)))
    actual_height = int(round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    print(
        f"Video source: {args.camera} (backend={backend_name}, "
        f"requested={args.width}x{args.height}, actual={actual_width}x{actual_height})"
    )
    cv2.namedWindow(PREVIEW_WINDOW_NAME, cv2.WINDOW_NORMAL)
    app = CalibrationApp(capture=capture, backend_name=backend_name, default_output=args.output)
    return app.run()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        sys.exit(0)
