#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import List, Tuple, Optional

import cv2
import numpy as np
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    from PIL import Image, ImageTk
except ImportError as exc:
    raise SystemExit('This script requires Pillow. Install with: pip install pillow') from exc

APP_TITLE = 'Janelia Path Generator'
DISPLAY_POLL_MS = 20
CAMERA_FRAME_WIDTH = 1280
CAMERA_FRAME_HEIGHT = 720
CAMERA_FPS = 60
MAX_CAMERA_INDEX_TO_PROBE = 8
DEFAULT_CALIBRATION_CSV = Path(__file__).with_name('camera_calibration.csv')
OUTPUT_DIR = Path(__file__).with_name('output')
DEFAULT_EXTERNAL_CALIBRATION_CSV = OUTPUT_DIR / 'Calibrated.csv'
CALIBRATION_PREFERENCE_FILE = OUTPUT_DIR / 'janelia_path_generator.default_calibration.txt'
POINT_RADIUS = 6
HOVER_RADIUS = 12
LINE_COLOR = (0, 255, 255)
POINT_COLOR = (0, 0, 255)
POINT_HOVER_COLOR = (0, 255, 0)
TEXT_COLOR = (255, 255, 255)
PATH_COLOR = (255, 200, 0)
SEGMENT_COLOR = (255, 100, 255)
DEFAULT_LOOKAHEAD_PX = 75.0
DEFAULT_SPEED = 1.0
DEFAULT_HEADING_DEG = 0.0


@dataclass
class PathSample:
    sample_id: int
    x: float
    y: float
    s_norm: float
    speed_mps: float
    heading_offset_deg: float


def load_camera_calibration(csv_path: str) -> list[tuple[str, int, float]]:
    calibration_path = Path(csv_path)
    if not calibration_path.exists():
        return []
    loaded_settings: list[tuple[str, int, float]] = []
    with calibration_path.open('r', newline='', encoding='utf-8') as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            property_name = row.get('property_name', '').strip()
            if not property_name:
                continue
            loaded_settings.append((property_name, int(row['property_id']), float(row['value'])))
    return loaded_settings


def save_default_calibration_path(csv_path: str) -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    CALIBRATION_PREFERENCE_FILE.write_text(csv_path, encoding='utf-8')


def load_default_calibration_path() -> str | None:
    if not CALIBRATION_PREFERENCE_FILE.exists():
        return None
    saved_path = CALIBRATION_PREFERENCE_FILE.read_text(encoding='utf-8').strip()
    return saved_path or None


def resolve_calibration_source(preferred_path: str | None = None) -> tuple[str | None, list[tuple[str, int, float]], str]:
    candidate_paths: list[str] = []
    if preferred_path:
        candidate_paths.append(preferred_path)
    remembered_path = load_default_calibration_path()
    if remembered_path:
        candidate_paths.append(remembered_path)
    candidate_paths.append(str(DEFAULT_EXTERNAL_CALIBRATION_CSV))
    candidate_paths.append(str(DEFAULT_CALIBRATION_CSV))

    seen_paths: set[str] = set()
    for candidate_path in candidate_paths:
        normalized_path = str(Path(candidate_path))
        if normalized_path in seen_paths:
            continue
        seen_paths.add(normalized_path)
        settings = load_camera_calibration(normalized_path)
        if settings:
            return normalized_path, settings, normalized_path

    return None, [], 'none loaded'


def apply_calibration_settings(
    capture: cv2.VideoCapture,
    settings: list[tuple[str, int, float]],
    source_label: str,
) -> None:
    if not settings:
        return
    print(f'Applying camera calibration from {source_label}')
    for property_name, property_id, value in settings:
        applied = capture.set(property_id, value)
        actual_value = capture.get(property_id)
        status = 'ok' if applied else 'ignored'
        print(
            f'  {property_name}: requested {value:.3f}, '
            f'camera reports {actual_value:.3f} ({status})'
        )


def apply_default_camera_calibration(
    capture: cv2.VideoCapture,
    preferred_path: str | None = None,
) -> tuple[str | None, str]:
    resolved_path, settings, source_label = resolve_calibration_source(preferred_path)
    apply_calibration_settings(capture, settings, source_label)
    return resolved_path, source_label


class PathGeneratorApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title(APP_TITLE)
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

        self.capture: Optional[cv2.VideoCapture] = None
        self.latest_bgr: Optional[np.ndarray] = None
        self.photo: Optional[ImageTk.PhotoImage] = None
        self.video_item: Optional[int] = None
        self.frame_size = (960, 720)
        self.display_region: Optional[Tuple[int, int, int, int]] = None

        self.control_points: List[List[float]] = []
        self.hover_index: Optional[int] = None
        self.drag_index: Optional[int] = None
        self.samples: List[PathSample] = []
        self.output_path: Optional[Path] = None

        self.camera_var = tk.StringVar(value='')
        self.segment_count_var = tk.StringVar(value='200')
        self.lookahead_var = tk.StringVar(value=str(int(DEFAULT_LOOKAHEAD_PX)))
        self.default_speed_var = tk.StringVar(value=f'{DEFAULT_SPEED:.2f}')
        self.default_heading_var = tk.StringVar(value=f'{DEFAULT_HEADING_DEG:.1f}')
        self.status_var = tk.StringVar(value='Click to add control points. First point is the start line and travel direction anchor.')
        self.available_camera_indices: List[int] = []
        self.camera_combo: Optional[ttk.Combobox] = None
        self.calibration_path = load_default_calibration_path()
        self.calibration_source_label = self.calibration_path or 'none loaded'
        self.calibration_label: Optional[ttk.Label] = None

        self._build_ui()
        self.refresh_camera_list(open_selected=True, announce=False)
        self._schedule_refresh()

    def _build_ui(self) -> None:
        self.root.configure(bg='#202225')
        main = ttk.Frame(self.root, padding=10)
        main.grid(row=0, column=0, sticky='nsew')
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main.columnconfigure(0, weight=1)
        main.rowconfigure(0, weight=1)

        self.canvas = tk.Canvas(main, width=self.frame_size[0], height=self.frame_size[1], bg='black', highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky='nsew', padx=(0, 10))
        self.canvas.bind('<Button-1>', self.on_canvas_press)
        self.canvas.bind('<B1-Motion>', self.on_canvas_drag)
        self.canvas.bind('<ButtonRelease-1>', self.on_canvas_release)
        self.canvas.bind('<Motion>', self.on_canvas_motion)
        self.canvas.bind('<Double-Button-1>', self.on_canvas_double_click)

        side = ttk.Frame(main)
        side.grid(row=0, column=1, sticky='ns')
        side.columnconfigure(0, weight=1)

        ttk.Label(side, text='Path generation', font=('', 12, 'bold')).grid(row=0, column=0, sticky='w', pady=(0, 8))

        fields = ttk.Frame(side)
        fields.grid(row=1, column=0, sticky='ew')
        fields.columnconfigure(1, weight=1)

        ttk.Label(fields, text='Camera').grid(row=0, column=0, sticky='w')
        self.camera_combo = ttk.Combobox(fields, textvariable=self.camera_var, state='readonly')
        self.camera_combo.grid(row=0, column=1, sticky='ew', pady=2)
        self.camera_combo.bind('<<ComboboxSelected>>', self.on_camera_selected)
        ttk.Button(fields, text='Refresh', command=self.refresh_camera_list).grid(row=0, column=2, padx=(6, 0), pady=2)
        self.calibration_label = ttk.Label(fields, text=self.format_calibration_label(), justify='left')
        self.calibration_label.grid(row=1, column=0, columnspan=3, sticky='ew', pady=(6, 2))
        ttk.Button(fields, text='Choose Calibration CSV', command=self.browse_calibration_file).grid(row=2, column=0, columnspan=3, sticky='ew', pady=(0, 6))
        ttk.Label(fields, text='Samples').grid(row=3, column=0, sticky='w')
        ttk.Entry(fields, textvariable=self.segment_count_var, width=10).grid(row=3, column=1, sticky='ew', pady=2)
        ttk.Label(fields, text='Lookahead px').grid(row=4, column=0, sticky='w')
        ttk.Entry(fields, textvariable=self.lookahead_var, width=10).grid(row=4, column=1, sticky='ew', pady=2)
        ttk.Label(fields, text='Default speed').grid(row=5, column=0, sticky='w')
        ttk.Entry(fields, textvariable=self.default_speed_var, width=10).grid(row=5, column=1, sticky='ew', pady=2)
        ttk.Label(fields, text='Heading offset').grid(row=6, column=0, sticky='w')
        ttk.Entry(fields, textvariable=self.default_heading_var, width=10).grid(row=6, column=1, sticky='ew', pady=2)

        btns = ttk.Frame(side)
        btns.grid(row=2, column=0, sticky='ew', pady=(10, 0))
        btns.columnconfigure((0, 1), weight=1)
        ttk.Button(btns, text='Undo', command=self.undo_point).grid(row=0, column=0, sticky='ew', padx=(0, 4), pady=2)
        ttk.Button(btns, text='Clear', command=self.clear_points).grid(row=0, column=1, sticky='ew', padx=(4, 0), pady=2)
        ttk.Button(btns, text='Generate', command=self.generate_path).grid(row=1, column=0, sticky='ew', padx=(0, 4), pady=2)
        ttk.Button(btns, text='Save As', command=self.save_path).grid(row=1, column=1, sticky='ew', padx=(4, 0), pady=2)

        help_text = (
            'Single-click empty space: add point\n'
            'Drag point: move point\n'
            'Double-click near segment: insert point\n'
            'Spline is always closed\n'
            'IDs increase in the order you drew the path'
        )
        ttk.Label(side, text=help_text, justify='left', wraplength=240).grid(row=3, column=0, sticky='ew', pady=(12, 0))
        ttk.Label(side, textvariable=self.status_var, justify='left', wraplength=240).grid(row=4, column=0, sticky='ew', pady=(12, 0))

    def _camera_backend(self) -> int:
        return cv2.CAP_DSHOW if hasattr(cv2, 'CAP_DSHOW') else cv2.CAP_ANY

    def _create_capture(self, camera_index: int) -> cv2.VideoCapture:
        backend = self._camera_backend()
        return cv2.VideoCapture(camera_index, backend) if backend != cv2.CAP_ANY else cv2.VideoCapture(camera_index)

    def _release_camera(self) -> None:
        if self.capture is not None:
            self.capture.release()
            self.capture = None

    def _probe_camera_indices(self) -> List[int]:
        indices: List[int] = []
        for camera_index in range(MAX_CAMERA_INDEX_TO_PROBE + 1):
            cap = self._create_capture(camera_index)
            if cap.isOpened():
                indices.append(camera_index)
            cap.release()
        return indices

    def refresh_camera_list(self, open_selected: bool = True, announce: bool = True) -> None:
        previous_selection = self._selected_camera_index()
        self.available_camera_indices = self._probe_camera_indices()
        if self.camera_combo is None:
            return
        if self.available_camera_indices:
            selected_index = previous_selection if previous_selection in self.available_camera_indices else self.available_camera_indices[0]
            values = [str(index) for index in self.available_camera_indices]
            self.camera_combo.configure(values=values, state='readonly')
            self.camera_var.set(str(selected_index))
            if open_selected:
                self._open_camera(selected_index, announce=announce)
        else:
            self.camera_combo.configure(values=(), state='disabled')
            self.camera_var.set('')
            self._release_camera()
            self.latest_bgr = None
            self.status_var.set('No cameras detected. You can still create a path on a black canvas.')

    def _selected_camera_index(self) -> Optional[int]:
        try:
            return int(self.camera_var.get())
        except ValueError:
            return None

    def on_camera_selected(self, _event: tk.Event) -> None:
        self._open_camera(announce=True)

    def format_calibration_label(self) -> str:
        if self.calibration_path:
            return f'Calibration CSV: {Path(self.calibration_path).name}'
        return f'Calibration: {self.calibration_source_label}'

    def browse_calibration_file(self) -> None:
        initial_path = self.calibration_path or str(DEFAULT_EXTERNAL_CALIBRATION_CSV)
        selected = filedialog.askopenfilename(
            parent=self.root,
            title='Select camera calibration CSV',
            initialdir=str(Path(initial_path).parent),
            initialfile=Path(initial_path).name,
            filetypes=[('CSV files', '*.csv'), ('All files', '*.*')],
        )
        if not selected:
            return
        self.calibration_path = selected
        self.calibration_source_label = selected
        save_default_calibration_path(selected)
        if self.calibration_label is not None:
            self.calibration_label.configure(text=self.format_calibration_label())
        if self.capture is not None:
            resolved_path, source_label = apply_default_camera_calibration(self.capture, selected)
            self.calibration_path = resolved_path
            self.calibration_source_label = source_label
            if self.calibration_label is not None:
                self.calibration_label.configure(text=self.format_calibration_label())
            self.status_var.set(f'Applied camera calibration from {source_label}.')

    def _open_camera(self, camera_index: Optional[int] = None, announce: bool = True) -> None:
        if camera_index is None:
            camera_index = self._selected_camera_index()
        if camera_index is None:
            self._release_camera()
            self.latest_bgr = None
            self.status_var.set('Select a camera first.')
            return
        self._release_camera()
        cap = self._create_capture(camera_index)
        if not cap.isOpened():
            self.latest_bgr = None
            self.status_var.set(f'Camera {camera_index} failed to open. You can still create a path on a black canvas.')
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
        self.capture = cap
        self.calibration_path, self.calibration_source_label = apply_default_camera_calibration(cap, self.calibration_path)
        if self.calibration_label is not None:
            self.calibration_label.configure(text=self.format_calibration_label())
        if announce:
            self.status_var.set(f'Using camera {camera_index}. Calibration: {self.calibration_source_label}.')

    def _schedule_refresh(self) -> None:
        self.root.after(DISPLAY_POLL_MS, self.update_frame)

    def update_frame(self) -> None:
        if self.capture is not None:
            ok, frame = self.capture.read()
            if ok:
                self.latest_bgr = frame
        self.redraw()
        self._schedule_refresh()

    def redraw(self) -> None:
        if self.latest_bgr is None:
            frame = np.zeros((self.frame_size[1], self.frame_size[0], 3), dtype=np.uint8)
        else:
            frame = cv2.resize(self.latest_bgr, self.frame_size)
        self.display_region = (0, 0, self.frame_size[0], self.frame_size[1])

        if len(self.control_points) >= 2:
            pts = np.array(self.control_points, dtype=np.float32)
            smooth = self._build_closed_curve(pts, max(300, len(self.control_points) * 40))
            cv2.polylines(frame, [smooth.astype(np.int32)], True, PATH_COLOR, 2, cv2.LINE_AA)

        if self.samples:
            for sample in self.samples:
                p0 = (int(sample.x), int(sample.y))
                cv2.circle(frame, p0, 2, SEGMENT_COLOR, -1)
                cv2.putText(frame, str(sample.sample_id), (p0[0] + 4, p0[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, SEGMENT_COLOR, 1, cv2.LINE_AA)

        for idx, point in enumerate(self.control_points):
            color = POINT_HOVER_COLOR if idx == self.hover_index else POINT_COLOR
            cv2.circle(frame, (int(point[0]), int(point[1])), POINT_RADIUS, color, -1)
            label = 'start' if idx == 0 else str(idx)
            cv2.putText(frame, label, (int(point[0]) + 8, int(point[1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT_COLOR, 1, cv2.LINE_AA)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(rgb)
        self.photo = ImageTk.PhotoImage(image=image)
        if self.video_item is None:
            self.video_item = self.canvas.create_image(0, 0, anchor='nw', image=self.photo)
        else:
            self.canvas.itemconfig(self.video_item, image=self.photo)

    def canvas_to_frame(self, event_x: int, event_y: int) -> Tuple[float, float]:
        return float(event_x), float(event_y)

    def nearest_point_index(self, x: float, y: float) -> Optional[int]:
        best = None
        best_d = float('inf')
        for idx, (px, py) in enumerate(self.control_points):
            d = math.hypot(px - x, py - y)
            if d < best_d and d <= HOVER_RADIUS:
                best_d = d
                best = idx
        return best

    def nearest_segment_insert_index(self, x: float, y: float) -> Optional[int]:
        if len(self.control_points) < 2:
            return None
        best_index = None
        best_d = float('inf')
        points = self.control_points
        for idx in range(len(points)):
            a = np.array(points[idx], dtype=float)
            b = np.array(points[(idx + 1) % len(points)], dtype=float)
            ab = b - a
            denom = float(np.dot(ab, ab))
            if denom <= 1e-9:
                continue
            t = float(np.clip(np.dot(np.array([x, y]) - a, ab) / denom, 0.0, 1.0))
            proj = a + (t * ab)
            d = float(np.linalg.norm(np.array([x, y]) - proj))
            if d < best_d and d <= HOVER_RADIUS:
                best_d = d
                best_index = idx + 1
        return best_index

    def on_canvas_press(self, event: tk.Event) -> None:
        x, y = self.canvas_to_frame(event.x, event.y)
        idx = self.nearest_point_index(x, y)
        if idx is not None:
            self.drag_index = idx
            return
        self.control_points.append([x, y])
        self.samples = []
        self.status_var.set(f'Added control point {len(self.control_points) - 1}.')

    def on_canvas_drag(self, event: tk.Event) -> None:
        if self.drag_index is None:
            return
        x, y = self.canvas_to_frame(event.x, event.y)
        self.control_points[self.drag_index] = [x, y]
        self.samples = []

    def on_canvas_release(self, event: tk.Event) -> None:
        self.drag_index = None

    def on_canvas_motion(self, event: tk.Event) -> None:
        x, y = self.canvas_to_frame(event.x, event.y)
        self.hover_index = self.nearest_point_index(x, y)

    def on_canvas_double_click(self, event: tk.Event) -> None:
        x, y = self.canvas_to_frame(event.x, event.y)
        insert_index = self.nearest_segment_insert_index(x, y)
        if insert_index is not None:
            self.control_points.insert(insert_index, [x, y])
            self.samples = []
            self.status_var.set(f'Inserted control point at index {insert_index}.')

    def undo_point(self) -> None:
        if self.control_points:
            self.control_points.pop()
            self.samples = []
            self.status_var.set('Removed last control point.')

    def clear_points(self) -> None:
        self.control_points.clear()
        self.samples = []
        self.status_var.set('Cleared all control points.')

    def _build_closed_curve(self, control_points: np.ndarray, num_samples: int) -> np.ndarray:
        n = len(control_points)
        if n < 2:
            return control_points.copy()
        if n == 2:
            return np.vstack([np.linspace(control_points[0], control_points[1], num_samples // 2, endpoint=False),
                              np.linspace(control_points[1], control_points[0], num_samples // 2 + 1)])
        samples = []
        per_seg = max(8, int(math.ceil(num_samples / n)))
        for i in range(n):
            p0 = control_points[(i - 1) % n]
            p1 = control_points[i % n]
            p2 = control_points[(i + 1) % n]
            p3 = control_points[(i + 2) % n]
            for j in range(per_seg):
                t = j / per_seg
                t2 = t * t
                t3 = t2 * t
                point = 0.5 * (
                    (2 * p1)
                    + (-p0 + p2) * t
                    + (2*p0 - 5*p1 + 4*p2 - p3) * t2
                    + (-p0 + 3*p1 - 3*p2 + p3) * t3
                )
                samples.append(point)
        return np.array(samples, dtype=np.float32)

    def _resample_by_curvature(self, curve: np.ndarray, sample_count: int) -> np.ndarray:
        if len(curve) < 4:
            return curve
        diffs = np.roll(curve, -1, axis=0) - curve
        lengths = np.linalg.norm(diffs, axis=1)
        total_length = float(np.sum(lengths))
        if total_length <= 1e-6:
            return curve[:sample_count]

        tangents_prev = curve - np.roll(curve, 1, axis=0)
        tangents_next = np.roll(curve, -1, axis=0) - curve
        angles_prev = np.arctan2(tangents_prev[:, 1], tangents_prev[:, 0])
        angles_next = np.arctan2(tangents_next[:, 1], tangents_next[:, 0])
        curvature = np.abs(np.arctan2(np.sin(angles_next - angles_prev), np.cos(angles_next - angles_prev)))
        weights = lengths * (1.0 + 3.0 * curvature)
        cumulative = np.concatenate([[0.0], np.cumsum(weights)])
        targets = np.linspace(0.0, cumulative[-1], sample_count, endpoint=False)
        result = []
        for target in targets:
            seg = int(np.searchsorted(cumulative, target, side='right') - 1)
            seg = min(max(seg, 0), len(curve) - 1)
            next_seg = (seg + 1) % len(curve)
            denom = cumulative[seg + 1] - cumulative[seg]
            alpha = 0.0 if denom <= 1e-9 else (target - cumulative[seg]) / denom
            point = (1.0 - alpha) * curve[seg] + alpha * curve[next_seg]
            result.append(point)
        return np.array(result, dtype=np.float32)

    def generate_path(self) -> None:
        if len(self.control_points) < 3:
            messagebox.showwarning(APP_TITLE, 'Add at least 3 control points before generating a path.')
            return
        try:
            sample_count = max(8, int(self.segment_count_var.get()))
            lookahead_px = float(self.lookahead_var.get())
            default_speed = float(self.default_speed_var.get())
            default_heading = float(self.default_heading_var.get())
        except ValueError:
            messagebox.showerror(APP_TITLE, 'Samples, lookahead, speed, and heading offset must be numeric.')
            return

        control = np.array(self.control_points, dtype=np.float32)
        dense = self._build_closed_curve(control, max(800, sample_count * 6))
        sampled = self._resample_by_curvature(dense, sample_count)

        seg_lengths = np.linalg.norm(np.roll(sampled, -1, axis=0) - sampled, axis=1)
        total_length = float(np.sum(seg_lengths))
        cumulative = 0.0
        self.samples = []
        for idx, point in enumerate(sampled):
            s_norm = 0.0 if total_length <= 1e-9 else cumulative / total_length
            self.samples.append(PathSample(
                sample_id=idx,
                x=float(point[0]),
                y=float(point[1]),
                s_norm=s_norm,
                speed_mps=default_speed,
                heading_offset_deg=default_heading,
            ))
            cumulative += float(seg_lengths[idx])
        self.status_var.set(f'Generated {len(self.samples)} path samples. Lookahead radius recorded as {lookahead_px:.1f}px.')

    def save_path(self) -> None:
        if not self.samples:
            self.generate_path()
            if not self.samples:
                return
        try:
            lookahead_px = float(self.lookahead_var.get())
        except ValueError:
            messagebox.showerror(APP_TITLE, 'Lookahead must be numeric.')
            return

        file_path = filedialog.asksaveasfilename(
            title='Save path file',
            defaultextension='.json',
            filetypes=[('JSON files', '*.json')],
            initialfile='path_definition.json',
        )
        if not file_path:
            return
        payload = {
            'format': 'janelia_rc_path_v1',
            'created_unix_s': time.time(),
            'closed': True,
            'lookahead_radius_px': lookahead_px,
            'control_points': [{'x': float(x), 'y': float(y)} for x, y in self.control_points],
            'samples': [asdict(sample) for sample in self.samples],
        }
        path = Path(file_path)
        path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
        self.output_path = path
        self.status_var.set(f'Saved path to {path.name}.')

    def on_close(self) -> None:
        self._release_camera()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    app = PathGeneratorApp(root)
    root.mainloop()


if __name__ == '__main__':
    main()
