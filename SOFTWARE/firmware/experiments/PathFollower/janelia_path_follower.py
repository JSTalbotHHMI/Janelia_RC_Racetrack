#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import serial
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    from PIL import Image, ImageTk
except ImportError as exc:
    raise SystemExit('This script requires Pillow. Install with: pip install pillow') from exc

APP_TITLE = 'Janelia Path Follower'
DISPLAY_POLL_MS = 20
CAMERA_FRAME_WIDTH = 1280
CAMERA_FRAME_HEIGHT = 720
CAMERA_FPS = 60
MAX_CAMERA_INDEX_TO_PROBE = 8
DEFAULT_SERIAL_PORT = 'COM6'
DEFAULT_SERIAL_BAUD = 115200
DEFAULT_STARTUP_DELAY_S = 2.0
DEFAULT_MIN_BLOB_AREA = 80
DEFAULT_MAX_BLOB_AREA = 20000
DEFAULT_TRAIL_LEN = 20
DEFAULT_LOST_TIMEOUT_S = 0.4
HSV_RED_RANGES = [((0, 90, 60), (12, 255, 255)), ((165, 90, 60), (179, 255, 255))]
HSV_BLUE_RANGES = [((90, 80, 60), (130, 255, 255))]


@dataclass
class Blob:
    centroid: Tuple[float, float]
    area: float
    contour: np.ndarray


@dataclass
class PoseSample:
    position: Tuple[float, float]
    orientation_deg: float
    travel_deg: float
    speed_px_s: float
    timestamp_s: float


@dataclass
class PathSample:
    sample_id: int
    x: float
    y: float
    s_norm: float
    speed_mps: float
    heading_offset_deg: float


class SerialConnection:
    def __init__(self, port: str, baud: int, startup_delay_s: float) -> None:
        self.port = port
        self.baud = baud
        self.startup_delay_s = startup_delay_s
        self.handle: Optional[serial.Serial] = None

    def connect(self) -> None:
        if self.handle is not None:
            return
        self.handle = serial.serial_for_url(self.port, self.baud, timeout=0.05)
        time.sleep(self.startup_delay_s)

    def disconnect(self) -> None:
        if self.handle is not None:
            self.handle.close()
            self.handle = None

    def send_error(self, speed_error_mps: float, heading_error_rad: float) -> None:
        if self.handle is None:
            return
        packet = f'ERR,{speed_error_mps:.4f},{heading_error_rad:.4f}\n'.encode('ascii')
        self.handle.write(packet)
        self.handle.flush()


class PathFollowerApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title(APP_TITLE)
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

        self.capture: Optional[cv2.VideoCapture] = None
        self.photo: Optional[ImageTk.PhotoImage] = None
        self.video_item: Optional[int] = None
        self.frame_size = (960, 720)
        self.latest_bgr: Optional[np.ndarray] = None

        self.path_file_var = tk.StringVar(value='No path selected')
        self.camera_var = tk.StringVar(value='')
        self.serial_port_var = tk.StringVar(value=DEFAULT_SERIAL_PORT)
        self.serial_baud_var = tk.StringVar(value=str(DEFAULT_SERIAL_BAUD))
        self.lookahead_override_var = tk.StringVar(value='')
        self.status_var = tk.StringVar(value='Choose a path file, then press Start.')
        self.start_stop_var = tk.StringVar(value='Idle')

        self.path_samples: List[PathSample] = []
        self.path_lookahead_px: float = 75.0
        self.path_segments: List[Tuple[np.ndarray, np.ndarray, PathSample, PathSample]] = []
        self.serial_conn: Optional[SerialConnection] = None
        self.running = False
        self.last_pose: Optional[PoseSample] = None
        self.trail: List[Tuple[int, int]] = []
        self.last_detection_time_s: Optional[float] = None
        self.available_camera_indices: List[int] = []
        self.camera_combo: Optional[ttk.Combobox] = None

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

        side = ttk.Frame(main)
        side.grid(row=0, column=1, sticky='ns')
        side.columnconfigure(0, weight=1)

        ttk.Label(side, text='Follower control', font=('', 12, 'bold')).grid(row=0, column=0, sticky='w', pady=(0, 8))
        ttk.Button(side, text='Select path file', command=self.select_path_file).grid(row=1, column=0, sticky='ew', pady=2)
        ttk.Label(side, textvariable=self.path_file_var, wraplength=260, justify='left').grid(row=2, column=0, sticky='ew', pady=(0, 8))

        form = ttk.Frame(side)
        form.grid(row=3, column=0, sticky='ew')
        form.columnconfigure(1, weight=1)
        ttk.Label(form, text='Camera').grid(row=0, column=0, sticky='w')
        self.camera_combo = ttk.Combobox(form, textvariable=self.camera_var, state='readonly')
        self.camera_combo.grid(row=0, column=1, sticky='ew', pady=2)
        self.camera_combo.bind('<<ComboboxSelected>>', self.on_camera_selected)
        ttk.Button(form, text='Refresh', command=self.refresh_camera_list).grid(row=0, column=2, padx=(6, 0), pady=2)
        ttk.Label(form, text='Serial port').grid(row=1, column=0, sticky='w')
        ttk.Entry(form, textvariable=self.serial_port_var).grid(row=1, column=1, sticky='ew', pady=2)
        ttk.Label(form, text='Baud').grid(row=2, column=0, sticky='w')
        ttk.Entry(form, textvariable=self.serial_baud_var).grid(row=2, column=1, sticky='ew', pady=2)
        ttk.Label(form, text='Lookahead px').grid(row=3, column=0, sticky='w')
        ttk.Entry(form, textvariable=self.lookahead_override_var).grid(row=3, column=1, sticky='ew', pady=2)

        btns = ttk.Frame(side)
        btns.grid(row=4, column=0, sticky='ew', pady=(10, 0))
        btns.columnconfigure((0, 1), weight=1)
        ttk.Button(btns, text='Start', command=self.start_following).grid(row=0, column=0, sticky='ew', padx=(0, 4))
        ttk.Button(btns, text='Stop', command=self.stop_following).grid(row=0, column=1, sticky='ew', padx=(4, 0))

        ttk.Label(side, textvariable=self.start_stop_var, font=('', 11, 'bold')).grid(row=5, column=0, sticky='w', pady=(10, 0))
        ttk.Label(side, textvariable=self.status_var, wraplength=260, justify='left').grid(row=6, column=0, sticky='ew', pady=(10, 0))

        help_text = (
            'Tracking uses red and blue blobs.\n'
            'Pose center = midpoint of the pair.\n'
            'Orientation points from red to blue.\n'
            'Target point = circle/path intersection with higher path ID.'
        )
        ttk.Label(side, text=help_text, wraplength=260, justify='left').grid(row=7, column=0, sticky='ew', pady=(12, 0))

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
            self.status_var.set('No cameras detected.')

    def _selected_camera_index(self) -> Optional[int]:
        try:
            return int(self.camera_var.get())
        except ValueError:
            return None

    def on_camera_selected(self, _event: tk.Event) -> None:
        self._open_camera(announce=True)

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
            self.status_var.set(f'Camera {camera_index} failed to open.')
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
        self.capture = cap
        if announce:
            self.status_var.set(f'Using camera {camera_index}.')

    def _schedule_refresh(self) -> None:
        self.root.after(DISPLAY_POLL_MS, self.update_frame)

    def update_frame(self) -> None:
        if self.capture is not None:
            ok, frame = self.capture.read()
            if ok:
                self.latest_bgr = cv2.resize(frame, self.frame_size)
        self.process_and_draw()
        self._schedule_refresh()

    def load_path(self, path: Path) -> None:
        data = json.loads(path.read_text(encoding='utf-8'))
        if data.get('format') != 'janelia_rc_path_v1':
            raise ValueError('Unsupported path file format.')
        self.path_lookahead_px = float(data.get('lookahead_radius_px', 75.0))
        self.path_samples = [PathSample(**sample) for sample in data['samples']]
        if len(self.path_samples) < 2:
            raise ValueError('Path file must contain at least 2 samples.')
        self.path_segments.clear()
        for idx in range(len(self.path_samples)):
            a = self.path_samples[idx]
            b = self.path_samples[(idx + 1) % len(self.path_samples)]
            self.path_segments.append((np.array([a.x, a.y], dtype=float), np.array([b.x, b.y], dtype=float), a, b))

    def select_path_file(self) -> None:
        chosen = filedialog.askopenfilename(title='Select path file', filetypes=[('JSON files', '*.json')])
        if not chosen:
            return
        path = Path(chosen)
        try:
            self.load_path(path)
        except Exception as exc:
            messagebox.showerror(APP_TITLE, f'Failed to load path file:\n\n{exc}')
            return
        self.path_file_var.set(str(path))
        self.status_var.set(f'Loaded {len(self.path_samples)} path samples from {path.name}.')

    def start_following(self) -> None:
        if not self.path_samples:
            messagebox.showwarning(APP_TITLE, 'Select a path file first.')
            return
        try:
            baud = int(self.serial_baud_var.get())
        except ValueError:
            messagebox.showerror(APP_TITLE, 'Baud must be an integer.')
            return
        try:
            self.serial_conn = SerialConnection(self.serial_port_var.get().strip(), baud, DEFAULT_STARTUP_DELAY_S)
            self.serial_conn.connect()
        except Exception as exc:
            messagebox.showerror(APP_TITLE, f'Failed to open serial port:\n\n{exc}')
            self.serial_conn = None
            return
        self.running = True
        self.start_stop_var.set('Running')
        self.status_var.set('Following path. Press Stop to halt.')

    def stop_following(self) -> None:
        self.running = False
        self.start_stop_var.set('Stopped')
        if self.serial_conn is not None:
            try:
                self.serial_conn.send_error(0.0, 0.0)
            except Exception:
                pass
            self.serial_conn.disconnect()
            self.serial_conn = None
        self.status_var.set('Stopped.')

    def detect_blobs(self, hsv: np.ndarray, ranges: List[Tuple[Tuple[int, int, int], Tuple[int, int, int]]]) -> List[Blob]:
        mask = None
        for low, high in ranges:
            part = cv2.inRange(hsv, np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8))
            mask = part if mask is None else cv2.bitwise_or(mask, part)
        assert mask is not None
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blobs: List[Blob] = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < DEFAULT_MIN_BLOB_AREA or area > DEFAULT_MAX_BLOB_AREA:
                continue
            moments = cv2.moments(contour)
            if abs(moments['m00']) < 1e-9:
                continue
            cx = float(moments['m10'] / moments['m00'])
            cy = float(moments['m01'] / moments['m00'])
            blobs.append(Blob((cx, cy), area, contour))
        return blobs

    def choose_pair(self, red_blobs: List[Blob], blue_blobs: List[Blob]) -> Optional[Tuple[Blob, Blob]]:
        if not red_blobs or not blue_blobs:
            return None
        best_pair = None
        best_score = float('inf')
        for red in red_blobs:
            for blue in blue_blobs:
                distance = math.hypot(red.centroid[0] - blue.centroid[0], red.centroid[1] - blue.centroid[1])
                area_score = abs(red.area - blue.area)
                score = distance + 0.01 * area_score
                if score < best_score:
                    best_score = score
                    best_pair = (red, blue)
        return best_pair

    def compute_pose(self, pair: Tuple[Blob, Blob], timestamp_s: float) -> PoseSample:
        red_blob, blue_blob = pair
        cx = 0.5 * (red_blob.centroid[0] + blue_blob.centroid[0])
        cy = 0.5 * (red_blob.centroid[1] + blue_blob.centroid[1])
        orientation_rad = math.atan2(blue_blob.centroid[1] - red_blob.centroid[1], blue_blob.centroid[0] - red_blob.centroid[0])
        orientation_deg = math.degrees(orientation_rad)
        travel_deg = orientation_deg
        speed_px_s = 0.0
        if self.last_pose is not None:
            dt = max(1e-6, timestamp_s - self.last_pose.timestamp_s)
            dx = cx - self.last_pose.position[0]
            dy = cy - self.last_pose.position[1]
            speed_px_s = math.hypot(dx, dy) / dt
            if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                travel_deg = math.degrees(math.atan2(dy, dx))
        return PoseSample((cx, cy), orientation_deg, travel_deg, speed_px_s, timestamp_s)

    def interpolate_path_sample(
        self,
        point: np.ndarray,
        sample_a: PathSample,
        sample_b: PathSample,
        t: float,
    ) -> PathSample:
        interp_id = sample_a.sample_id + t * ((sample_b.sample_id - sample_a.sample_id) % len(self.path_samples))
        interp_speed = (1.0 - t) * sample_a.speed_mps + t * sample_b.speed_mps
        interp_heading = (1.0 - t) * sample_a.heading_offset_deg + t * sample_b.heading_offset_deg
        return PathSample(
            sample_id=int(round(interp_id)) % len(self.path_samples),
            x=float(point[0]),
            y=float(point[1]),
            s_norm=0.0,
            speed_mps=float(interp_speed),
            heading_offset_deg=float(interp_heading),
        )

    def find_target_sample(self, center: Tuple[float, float], lookahead_radius_px: float) -> Optional[PathSample]:
        center_vec = np.array(center, dtype=float)
        intersections: List[PathSample] = []
        closest_sample: Optional[PathSample] = None
        closest_distance = float('inf')
        r = lookahead_radius_px
        for a, b, sample_a, sample_b in self.path_segments:
            d = b - a
            f = a - center_vec
            A = float(np.dot(d, d))
            B = 2.0 * float(np.dot(f, d))
            C = float(np.dot(f, f) - (r * r))
            disc = (B * B) - (4.0 * A * C)
            if A <= 1e-9:
                continue

            t_closest = float(np.clip(-float(np.dot(f, d)) / A, 0.0, 1.0))
            closest_point = a + (t_closest * d)
            closest_dist = float(np.linalg.norm(center_vec - closest_point))
            if closest_dist < closest_distance:
                closest_distance = closest_dist
                closest_sample = self.interpolate_path_sample(closest_point, sample_a, sample_b, t_closest)

            if disc < 0.0:
                continue

            sqrt_disc = math.sqrt(max(0.0, disc))
            for sign in (-1.0, 1.0):
                t = (-B + sign * sqrt_disc) / (2.0 * A)
                if 0.0 <= t <= 1.0:
                    point = a + (t * d)
                    intersections.append(self.interpolate_path_sample(point, sample_a, sample_b, t))
        if not intersections:
            return closest_sample
        intersections.sort(key=lambda item: item.sample_id)
        return intersections[-1]

    @staticmethod
    def wrap_angle_rad(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def process_and_draw(self) -> None:
        if self.latest_bgr is None:
            frame = np.zeros((self.frame_size[1], self.frame_size[0], 3), dtype=np.uint8)
        else:
            frame = self.latest_bgr.copy()

        timestamp_s = time.monotonic()
        pose = None
        target = None
        lookahead_px = self.path_lookahead_px
        if self.lookahead_override_var.get().strip():
            try:
                lookahead_px = float(self.lookahead_override_var.get().strip())
            except ValueError:
                pass

        if self.latest_bgr is not None:
            hsv = cv2.cvtColor(self.latest_bgr, cv2.COLOR_BGR2HSV)
            red_blobs = self.detect_blobs(hsv, HSV_RED_RANGES)
            blue_blobs = self.detect_blobs(hsv, HSV_BLUE_RANGES)
            pair = self.choose_pair(red_blobs, blue_blobs)
            if pair is not None:
                pose = self.compute_pose(pair, timestamp_s)
                self.last_pose = pose
                self.last_detection_time_s = timestamp_s
                self.trail.append((int(pose.position[0]), int(pose.position[1])))
                self.trail = self.trail[-DEFAULT_TRAIL_LEN:]

                if self.path_samples:
                    target = self.find_target_sample(pose.position, lookahead_px)
                    if target is not None and self.running and self.serial_conn is not None:
                        target_bearing_rad = math.atan2(target.y - pose.position[1], target.x - pose.position[0])
                        target_heading_rad = target_bearing_rad + math.radians(target.heading_offset_deg)
                        current_heading_rad = math.radians(pose.orientation_deg)
                        heading_error_rad = self.wrap_angle_rad(current_heading_rad - target_heading_rad)
                        speed_error_mps = 0.0 - target.speed_mps
                        try:
                            self.serial_conn.send_error(speed_error_mps, heading_error_rad)
                        except Exception as exc:
                            self.status_var.set(f'Serial send failed: {exc}')
                            self.stop_following()

                red_blob, blue_blob = pair
                cv2.drawContours(frame, [red_blob.contour], -1, (0, 0, 255), 2)
                cv2.drawContours(frame, [blue_blob.contour], -1, (255, 0, 0), 2)
                cv2.circle(frame, (int(red_blob.centroid[0]), int(red_blob.centroid[1])), 5, (0, 0, 255), -1)
                cv2.circle(frame, (int(blue_blob.centroid[0]), int(blue_blob.centroid[1])), 5, (255, 0, 0), -1)
                cv2.circle(frame, (int(pose.position[0]), int(pose.position[1])), 5, (0, 255, 255), -1)

        if self.path_samples:
            pts = np.array([(int(s.x), int(s.y)) for s in self.path_samples], dtype=np.int32)
            cv2.polylines(frame, [pts], True, (0, 255, 255), 2, cv2.LINE_AA)
            step = max(1, len(self.path_samples) // 25)
            for s in self.path_samples[::step]:
                cv2.putText(frame, str(s.sample_id), (int(s.x) + 3, int(s.y) - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (255, 255, 0), 1, cv2.LINE_AA)

        if pose is not None:
            heading_rad = math.radians(pose.orientation_deg)
            hx = int(pose.position[0] + 35.0 * math.cos(heading_rad))
            hy = int(pose.position[1] + 35.0 * math.sin(heading_rad))
            cv2.arrowedLine(frame, (int(pose.position[0]), int(pose.position[1])), (hx, hy), (0, 255, 0), 2, tipLength=0.2)
            cv2.circle(frame, (int(pose.position[0]), int(pose.position[1])), int(round(lookahead_px)), (100, 100, 100), 1)
            cv2.putText(frame, f'pose=({pose.position[0]:.1f},{pose.position[1]:.1f}) orient={pose.orientation_deg:.1f} deg', (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)

        if target is not None:
            cv2.circle(frame, (int(target.x), int(target.y)), 7, (255, 0, 255), -1)
            cv2.putText(frame, f'target id={target.sample_id} speed={target.speed_mps:.2f} hdgOff={target.heading_offset_deg:.1f}', (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)

        for i in range(1, len(self.trail)):
            cv2.line(frame, self.trail[i - 1], self.trail[i], (0, 200, 0), 2)

        if self.running and self.last_detection_time_s is not None:
            if (timestamp_s - self.last_detection_time_s) > DEFAULT_LOST_TIMEOUT_S and self.serial_conn is not None:
                try:
                    self.serial_conn.send_error(0.0, 0.0)
                except Exception:
                    pass
                self.status_var.set('Tracking lost; sent zeroed error command.')

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(rgb)
        self.photo = ImageTk.PhotoImage(image=image)
        if self.video_item is None:
            self.video_item = self.canvas.create_image(0, 0, anchor='nw', image=self.photo)
        else:
            self.canvas.itemconfig(self.video_item, image=self.photo)

    def on_close(self) -> None:
        self.stop_following()
        self._release_camera()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    app = PathFollowerApp(root)
    root.mainloop()


if __name__ == '__main__':
    main()
