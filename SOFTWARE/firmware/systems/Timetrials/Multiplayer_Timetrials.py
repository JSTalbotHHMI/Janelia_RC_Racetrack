#!/usr/bin/env python3
"""
Track up to five red/blue marker pairs from a USB webcam and visualize RC-car motion.
"""

from __future__ import annotations

import argparse
import csv
import io
import importlib.util
import json
import math
import random
import socket
import struct
import subprocess
import sys
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

try:
    import cv2
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "opencv-python is required. Install it with: pip install opencv-python"
    ) from exc

try:
    import numpy as np
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "numpy is required. Install it with: pip install numpy"
    ) from exc

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "pyserial is required. Install it with: pip install pyserial"
    ) from exc

try:
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk
except ImportError as exc:  # pragma: no cover
    raise SystemExit("tkinter is required for the tracker GUI.") from exc

try:
    from PIL import Image, ImageTk
except ImportError:  # pragma: no cover
    Image = None
    ImageTk = None


TRAIL_LENGTH = 10
TRAIL_MIN_ALPHA = 0.12
TRAIL_MAX_ALPHA = 0.92
DOT_RADIUS_PX = 5
MIN_TRAVEL_ARROW_LENGTH_PX = 12
MAX_TRAVEL_ARROW_LENGTH_PX = 42
ORIENTATION_ARROW_LENGTH_PX = 34
TRAIL_THICKNESS_PX = 2
ARROW_THICKNESS_PX = 2
BLACK_BGR = (0, 0, 0)
BLUE_BGR = (255, 0, 0)
RED_BGR = (0, 0, 255)
GREEN_BGR = (0, 255, 0)
YELLOW_BGR = (0, 255, 255)
CYAN_BGR = (255, 255, 0)
ORANGE_BGR = (0, 165, 255)
GRAY_BGR = (150, 150, 150)
DEFAULT_CALIBRATION_CSV = Path(__file__).with_name("camera_calibration.csv")
OUTPUT_DIR = Path(__file__).with_name("output")
DEFAULT_EXTERNAL_CALIBRATION_CSV = OUTPUT_DIR / "Calibrated.csv"
DEFAULT_PATCH_CSV = Path(
    r"C:\Users\talbotj\Documents\GitHub\Janelia_RC_Racetrack\SOFTWARE\utilities\patch-generator\output\FinishThird1Third2.csv"
)
CALIBRATION_PREFERENCE_FILE = OUTPUT_DIR / "Multiplayer_Timetrials.default_calibration.txt"
PATCH_PREFERENCE_FILE = OUTPUT_DIR / "Multiplayer_Timetrials.default_patch.txt"
DEFAULT_FAVICON_PNG = Path(__file__).with_name("assets") / "multiblobtracker.png"
LAPTIME_SPLASH_PNG = Path(__file__).with_name("assets") / "laptime_board_splash.png"
LAPTIME_BACKGROUND_PNG = Path(__file__).with_name("assets") / "laptime_board_background.png"
SERIAL_PATCH_LOADER_PATH = (
    Path(__file__).resolve().parents[3] / "utilities" / "patch-generator" / "serial_patch_loader.py"
)
SERIAL_PATCH_LOADER_MODULE_NAME = "tracker_serial_patch_loader"
PATCH_UPLOAD_TIMEOUT_S = 4.0
REQUIRED_PATCH_NAMES = ("finish", "third1", "third2")
PATCH_SEQUENCE = ("finish", "third1", "third2")
PREVIEW_CONNECT_TIMEOUT_S = 5.0
PREVIEW_SEND_TIMEOUT_S = 0.01
TRACKING_LOSS_DQ_FRAMES = 12
FRAME_INTERVAL_MS = 15
SIDE_PANEL_WIDTH_PX = 360
SIDE_PANEL_HEIGHT_PX = 640
STATUS_TEXT_HEIGHT = 18
PREVIEW_WIDTH_PX = 1280
PREVIEW_HEIGHT_PX = 720
PATCH_SOURCE_WIDTH_PX = 1280
PATCH_SOURCE_HEIGHT_PX = 720
ROI_MARGIN_PX = 60
PROCESSING_WIDTH_PX = 960
PROCESSING_HEIGHT_PX = 540
FULL_FRAME_RECOVERY_INTERVAL = 6
TRACK_COUNT = 5
WHITE_BGR = (255, 255, 255)
DEFAULT_SERIAL_PORT = "COM6"
DEFAULT_SERIAL_BAUD = 115200
DEFAULT_CAR_ID_BASE = 1
DEFAULT_SERIAL_STARTUP_DELAY_S = 2.0
MIN_SPLASH_DURATION_S = 3.0
PATCH_FLASH_DURATION_S = 0.12
TRACK_LABEL_COLORS = [
    (0, 255, 255),
    (255, 255, 0),
    (255, 0, 255),
    (0, 165, 255),
    (255, 255, 255),
]
EMBEDDED_CALIBRATION_SETTINGS: list[tuple[str, int, float]] = [
    ("auto_exposure", 21, -1.0),
    ("exposure", 15, -5.0),
    ("auto_wb", 44, 0.0),
    ("wb_temperature", 45, 4000.0),
    ("autofocus", 39, 2.0),
    ("focus", 28, 10.0),
    ("brightness", 10, 128.0),
    ("contrast", 11, 128.0),
    ("saturation", 12, 128.0),
    ("hue", 13, -1.0),
    ("gain", 14, 0.0),
    ("gamma", 22, -1.0),
    ("sharpness", 20, 128.0),
    ("backlight", 32, 1.0),
    ("zoom", 27, 100.0),
]
BOARD_MAX_CARS = 5
BOARD_MAX_NAME_LENGTH = 25
BOARD_ALLOWED_COLORS = {
    "red": "#d7263d",
    "yellow": "#f4d35e",
    "green": "#2a9d8f",
    "blue": "#2b59c3",
    "purple": "#7b2cbf",
    "white": "#f5f5f5",
    "pink": "#ff85a1",
}
BOARD_BACKGROUND = "#15151b"
BOARD_HEADER_BACKGROUND = "#252531"
BOARD_HEADER_FOREGROUND = "#f7f7f7"
BOARD_CELL_BORDER = "#3d3d4c"
SPLASH_BACKGROUND = "#0b0d14"
SPLASH_PROGRESS_BG = "#1d2230"
SPLASH_PROGRESS_FG = "#ef233c"
ROW_HEIGHT = 44
LEADER_ROW_HEIGHT = 66
ROW_GAP = 3
ANIMATION_SMOOTHING = 0.28
ANIMATION_EPSILON = 0.6
EMBEDDED_DEFAULT_PATCH_CSV = """\
quadpatch,finish,wall_adjacent=false,498,65,610,63,616,236,496,236
quadpatch,third1,wall_adjacent=false,739,353,908,510,840,583,664,425
quadpatch,third2,wall_adjacent=false,376,349,445,422,272,580,209,504
circlepatch,points,wall_adjacent=false,824,140,56.3205
polypatch,DQ1,wall_adjacent=true,182,59,397,62,191,229
polypatch,DQ2,wall_adjacent=true,149,6,148,53,998,46,999,7
polypatch,DQ3,wall_adjacent=true,917,56,930,577,1012,572,1001,58
polypatch,DQ4,wall_adjacent=true,163,612,1007,598,1002,688,147,679
polypatch,DQ5,wall_adjacent=true,122,68,167,61,183,593,123,600
"""


@dataclass(frozen=True)
class Blob:
    centroid: tuple[float, float]
    area: float
    contour: np.ndarray


@dataclass(frozen=True)
class PoseSample:
    position: tuple[float, float]
    orientation_deg: float
    travel_deg: float
    speed_px_s: float
    timestamp_s: float


@dataclass
class TrackState:
    samples: deque[PoseSample] = field(default_factory=lambda: deque(maxlen=TRAIL_LENGTH))
    previous_position: tuple[float, float] | None = None
    previous_timestamp_s: float | None = None
    motion_reference_position: tuple[float, float] | None = None
    motion_reference_timestamp_s: float | None = None
    last_pose: PoseSample | None = None
    detected: bool = False


@dataclass(frozen=True)
class BlobPairCandidate:
    red_blob: Blob
    blue_blob: Blob
    center: tuple[float, float]
    score: float


@dataclass(frozen=True)
class PatchOverlayRecord:
    name: str
    kind: str
    numbers: tuple[float, ...]
    segment_index: int
    wall_adjacent: bool


@dataclass
class PatchOverlayState:
    occupants: set[int] = field(default_factory=set)
    flash_color_bgr: tuple[int, int, int] | None = None
    flash_until_s: float = 0.0


@dataclass
class TimeTrialCarState:
    last_seen_required_patch: str = "finish"
    expected_patch: str = "third1"
    current_lap: int = 1
    current_third: int = 1
    completed_laps: int = 0
    last_checkpoint_time_s: float | None = None
    lap_start_time_s: float | None = None
    disqualified: bool = False
    finished: bool = False
    active_required_patches: set[str] = field(default_factory=set)
    lost_tracking_frames: int = 0
    pending_third_adjustment_s: float = 0.0
    lap_adjustment_s: float = 0.0


@dataclass
class PreviewWorkerState:
    process: subprocess.Popen[bytes] | None = None
    server_socket: socket.socket | None = None
    client_socket: socket.socket | None = None
    endpoint: tuple[str, int] | None = None


@dataclass
class BoardCarState:
    car_id: int
    name_var: tk.StringVar
    color_var: tk.StringVar
    current_lap: int = 1
    current_third: int = 1
    last_third_time: float | None = None
    previous_third_time: float | None = None
    last_lap_time: float | None = None
    previous_lap_time: float | None = None
    total_elapsed: float = 0.0
    finished: bool = False
    disqualified: bool = False
    finish_place: int | None = None
    row_y: float = 0.0
    row_height: float = ROW_HEIGHT
    frames: dict[str, tk.Widget] = field(default_factory=dict)

    @property
    def completed_thirds(self) -> int:
        return max(0, ((self.current_lap - 1) * 3) + (self.current_third - 1))

    @property
    def has_started(self) -> bool:
        return self.completed_thirds > 0 or self.total_elapsed > 0


@dataclass
class SerialConnectionState:
    port: str
    baud: int
    startup_delay_s: float
    enabled: bool = True
    handle: serial.SerialBase | None = None
    last_error: str | None = None

    @property
    def is_connected(self) -> bool:
        return self.handle is not None

    def connect(self) -> None:
        if not self.enabled or self.handle is not None:
            return
        self.handle = serial.serial_for_url(self.port, self.baud, timeout=0.1)
        time.sleep(self.startup_delay_s)
        self.last_error = None
        print(f"Serial connected on {self.port} at {self.baud} baud")

    def disconnect(self) -> None:
        if self.handle is None:
            return
        self.handle.close()
        self.handle = None
        print(f"Serial disconnected from {self.port}")

    def upload_patch_file(self, patch_path: Path, response_timeout_s: float = PATCH_UPLOAD_TIMEOUT_S) -> None:
        if self.handle is None:
            raise RuntimeError("Serial controller is not connected.")
        patch_loader_module = load_serial_patch_loader_module()
        patch_parser = patch_loader_module.load_patch_to_header_module()
        patch_loader_module.send_patch_set(
            self.handle,
            patch_parser,
            patch_path,
            response_timeout_s,
            None,
        )
        self.last_error = None

    def upload_patch_records(
        self,
        patches: list[PatchOverlayRecord],
        response_timeout_s: float = PATCH_UPLOAD_TIMEOUT_S,
    ) -> None:
        if self.handle is None:
            raise RuntimeError("Serial controller is not connected.")
        patch_loader_module = load_serial_patch_loader_module()
        segment_lengths = patch_loader_module.infer_segment_lengths(patches, None)
        segment_lines = patch_loader_module.build_segment_lines(segment_lengths)
        patch_lines = patch_loader_module.build_patch_lines(patches)

        self.handle.reset_input_buffer()
        deadline = time.monotonic() + response_timeout_s

        if segment_lines:
            self.handle.write(f"{segment_lines[0]}\n".encode("utf-8"))
            self.handle.flush()
            patch_loader_module.wait_for_prefixes(self.handle, deadline, ("SEGMENTS_BEGIN_OK,",))

            for index, line in enumerate(segment_lines[1:-1]):
                self.handle.write(f"{line}\n".encode("utf-8"))
                self.handle.flush()
                patch_loader_module.wait_for_prefixes(self.handle, deadline, (f"SEGMENT_OK,{index}",))

            self.handle.write(f"{segment_lines[-1]}\n".encode("utf-8"))
            self.handle.flush()
            patch_loader_module.wait_for_prefixes(self.handle, deadline, ("SEGMENTS_OK,",))

        self.handle.write(f"{patch_lines[0]}\n".encode("utf-8"))
        self.handle.flush()
        patch_loader_module.wait_for_prefixes(self.handle, deadline, ("PATCHES_BEGIN_OK,",))

        for index, line in enumerate(patch_lines[1:-1]):
            self.handle.write(f"{line}\n".encode("utf-8"))
            self.handle.flush()
            patch_loader_module.wait_for_prefixes(self.handle, deadline, (f"PATCH_OK,{index}",))

        self.handle.write(f"{patch_lines[-1]}\n".encode("utf-8"))
        self.handle.flush()
        patch_loader_module.wait_for_prefixes(self.handle, deadline, ("PATCHES_OK,",))
        self.last_error = None

    def send_pose(
        self,
        car_id: int,
        x: float,
        y: float,
        travel_deg: float,
        heading_deg: float,
    ) -> None:
        if self.handle is None:
            return
        packet = (
            f"CAR,{car_id},{x:.2f},{y:.2f},{travel_deg:.2f},{heading_deg:.2f}\n"
        ).encode("ascii")
        try:
            self.handle.write(packet)
            self.handle.flush()
            self.last_error = None
        except serial.SerialException as exc:
            self.last_error = str(exc)
            self.disconnect()
            print(f"Serial write failed for {self.port}: {exc}")


def parse_board_time(value: object) -> float | None:
    if value is None:
        return None
    if isinstance(value, str):
        cleaned = value.strip()
        if not cleaned or cleaned == "-":
            return None
        return float(cleaned)
    return float(value)


def format_board_time(value: float | None) -> str:
    if value is None:
        return "-"
    return f"{value:.3f}"


def format_board_diff(current: float | None, previous: float | None) -> str:
    if current is None or previous is None:
        return "-"
    delta = current - previous
    sign = "+" if delta >= 0 else "-"
    return f"{sign}{abs(delta):.3f}"


def best_text_color(hex_color: str) -> str:
    hex_color = hex_color.lstrip("#")
    red = int(hex_color[0:2], 16)
    green = int(hex_color[2:4], 16)
    blue = int(hex_color[4:6], 16)
    luminance = (0.299 * red) + (0.587 * green) + (0.114 * blue)
    return "#111111" if luminance > 160 else "#f7f7f7"


def blend_hex_colors(foreground: str, background: str, alpha: float) -> str:
    foreground = foreground.lstrip("#")
    background = background.lstrip("#")
    fg_rgb = [int(foreground[index:index + 2], 16) for index in (0, 2, 4)]
    bg_rgb = [int(background[index:index + 2], 16) for index in (0, 2, 4)]
    mixed = [
        round((alpha * fg_channel) + ((1.0 - alpha) * bg_channel))
        for fg_channel, bg_channel in zip(fg_rgb, bg_rgb)
    ]
    return "#{:02x}{:02x}{:02x}".format(*mixed)


def format_place(place: int | None) -> str:
    if place is None:
        return ""
    if 10 <= (place % 100) <= 20:
        suffix = "th"
    else:
        suffix = {1: "st", 2: "nd", 3: "rd"}.get(place % 10, "th")
    return f"{place}{suffix}"


def parse_car_packet(value: int | str) -> tuple[int, bool]:
    if isinstance(value, int):
        return value, False
    cleaned = str(value).strip().upper()
    is_disqualified = cleaned.endswith("DQ")
    if is_disqualified:
        cleaned = cleaned[:-2].strip()
    return int(cleaned), is_disqualified


class IntegratedLapTimeBoard(tk.Toplevel):
    def __init__(
        self,
        parent: tk.Misc,
        on_close: Callable[[], None] | None = None,
        laps_var: tk.StringVar | None = None,
        timer_var: tk.StringVar | None = None,
        on_start: Callable[[], None] | None = None,
        on_restart: Callable[[], None] | None = None,
        on_stop: Callable[[], None] | None = None,
    ) -> None:
        super().__init__(parent)
        self.title("Formula 1 Style Lap Time Board")
        self.configure(bg=BOARD_BACKGROUND)
        self.geometry("1220x700")
        self.minsize(1080, 420)
        self._on_close = on_close or self.destroy
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.withdraw()
        self._icon_image: tk.PhotoImage | None = None
        self._background_source: Image.Image | None = None
        self._background_image: ImageTk.PhotoImage | None = None
        self._background_label: tk.Label | None = None
        self._rows_background_image: ImageTk.PhotoImage | None = None
        self._rows_background_item: int | None = None
        self._background_resize_job: str | None = None
        self._start_race_callback = on_start
        self._restart_race_callback = on_restart
        self._stop_race_callback = on_stop
        self.start_button: tk.Button | None = None
        self.restart_button: tk.Button | None = None
        self.stop_button: tk.Button | None = None
        self._medal_images = self._load_medal_images()
        self.cars: dict[int, BoardCarState] = {}
        self.row_frames: dict[int, tk.Frame] = {}
        self.row_window_ids: dict[int, int] = {}
        self.finishers: list[int] = []
        self.title_hold_job: str | None = None
        self.demo_panel_visible = False
        self._row_animation_job: str | None = None
        self._row_targets: dict[int, tuple[float, float]] = {}
        self.row_widget_order = [
            "position",
            "car",
            "name",
            "color",
            "lap",
            "third",
            "third_time",
            "third_diff",
            "lap_time",
            "lap_diff",
            "medal",
        ]
        self.column_specs = [
            ("POS", 70),
            ("CAR", 70),
            ("NAME", 230),
            ("COLOR", 130),
            ("LAP", 70),
            ("THIRD", 80),
            ("1/3 TIME", 105),
            ("1/3 DIFF", 105),
            ("LAP TIME", 105),
            ("LAP DIFF", 105),
            ("Placed", 85),
        ]
        self._load_window_icon()
        self._load_background_asset()
        self.run_laps_var = laps_var or tk.StringVar(master=self, value="6")
        self.timer_var = timer_var or tk.StringVar(master=self, value="00:00.000")
        self._build_header()
        self._build_rows()
        self._build_test_panel()
        self.refresh_board()
        self.bind("<Configure>", self._handle_window_configure)
        self.deiconify()
        self._apply_windowed_fullscreen()
        self._refresh_background_image()

    def _load_window_icon(self) -> None:
        if not DEFAULT_FAVICON_PNG.exists():
            return
        try:
            self._icon_image = tk.PhotoImage(master=self, file=str(DEFAULT_FAVICON_PNG))
            self.iconphoto(True, self._icon_image)
        except tk.TclError:
            self._icon_image = None

    def _get_asset_path(self, filename: str) -> Path:
        return Path(__file__).resolve().parent / "assets" / filename

    def _load_background_asset(self) -> None:
        if Image is None or ImageTk is None or not LAPTIME_BACKGROUND_PNG.exists():
            return
        try:
            self._background_source = Image.open(LAPTIME_BACKGROUND_PNG).convert("RGB")
        except OSError:
            self._background_source = None
            return
        self._background_label = tk.Label(self, bd=0, highlightthickness=0)
        self._background_label.place(x=0, y=0, relwidth=1, relheight=1)
        self._background_label.lower()

    def _handle_window_configure(self, event: tk.Event) -> None:
        if event.widget is not self:
            return
        if self._background_resize_job is not None:
            self.after_cancel(self._background_resize_job)
        self._background_resize_job = self.after(40, self._refresh_background_image)

    def _handle_rows_container_configure(self, _: tk.Event) -> None:
        for state in self.cars.values():
            self._place_row(state)

    def _refresh_background_image(self) -> None:
        self._background_resize_job = None
        if (
            self._background_source is None
            or self._background_label is None
            or ImageTk is None
        ):
            return
        width = max(1, self.winfo_width())
        height = max(1, self.winfo_height())
        resized = self._background_source.resize((width, height), Image.LANCZOS)
        self._background_image = ImageTk.PhotoImage(resized, master=self)
        self._background_label.configure(image=self._background_image)
        self._background_label.lower()
        self._refresh_rows_background(resized)

    def _refresh_rows_background(self, resized_background: Image.Image) -> None:
        if not hasattr(self, "rows_container") or self._rows_background_item is None or ImageTk is None:
            return
        self.update_idletasks()
        container_width = max(1, self.rows_container.winfo_width())
        container_height = max(1, self.rows_container.winfo_height())
        origin_x = max(0, self.rows_container.winfo_rootx() - self.winfo_rootx())
        origin_y = max(0, self.rows_container.winfo_rooty() - self.winfo_rooty())
        max_x = max(0, resized_background.width - container_width)
        max_y = max(0, resized_background.height - container_height)
        crop_box = (
            min(origin_x, max_x),
            min(origin_y, max_y),
            min(origin_x, max_x) + container_width,
            min(origin_y, max_y) + container_height,
        )
        rows_background = resized_background.crop(crop_box)
        self._rows_background_image = ImageTk.PhotoImage(rows_background, master=self.rows_container)
        self.rows_container.itemconfigure(
            self._rows_background_item,
            image=self._rows_background_image,
        )
        self.rows_container.coords(self._rows_background_item, 0, 0)

    def _load_medal_images(self) -> dict[int, tk.PhotoImage]:
        medal_files = {
            1: "gold_medal.png",
            2: "silver_medal.png",
            3: "bronze_medal.png",
        }
        medal_images: dict[int, tk.PhotoImage] = {}
        target_size = 24
        for place, filename in medal_files.items():
            medal_path = self._get_asset_path(filename)
            if not medal_path.exists():
                continue
            try:
                image = tk.PhotoImage(master=self, file=str(medal_path))
                shrink_factor = max(1, math.ceil(max(image.width(), image.height()) / target_size))
                medal_images[place] = image.subsample(shrink_factor, shrink_factor)
            except tk.TclError:
                continue
        return medal_images

    def _apply_windowed_fullscreen(self) -> None:
        try:
            self.state("zoomed")
            return
        except tk.TclError:
            pass
        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        self.geometry(f"{screen_width}x{screen_height}+0+0")

    def _build_header(self) -> None:
        header_bar = tk.Frame(self, bg=BOARD_BACKGROUND, padx=12, pady=10)
        header_bar.grid(row=0, column=0, sticky="ew")
        header_bar.grid_columnconfigure(0, weight=1)

        title = tk.Label(
            header_bar,
            text="LAPTIME BOARD",
            bg=BOARD_BACKGROUND,
            fg="#f8f9fa",
            font=("Bahnschrift SemiBold", 20),
            pady=2,
            cursor="hand2",
        )
        title.grid(row=0, column=0, sticky="w")
        title.bind("<ButtonPress-1>", self._start_title_hold)
        title.bind("<ButtonRelease-1>", self._cancel_title_hold)
        title.bind("<Leave>", self._cancel_title_hold)
        self.title_label = title

        stopwatch_frame = tk.Frame(header_bar, bg=BOARD_HEADER_BACKGROUND, bd=1, relief="solid", padx=12, pady=6)
        stopwatch_frame.grid(row=0, column=1, sticky="e")
        tk.Label(
            stopwatch_frame,
            text="STOPWATCH",
            bg=BOARD_HEADER_BACKGROUND,
            fg="#d7d7df",
            font=("Bahnschrift SemiBold", 10),
        ).grid(row=0, column=0, sticky="e")
        tk.Label(
            stopwatch_frame,
            textvariable=self.timer_var,
            bg=BOARD_HEADER_BACKGROUND,
            fg=BOARD_HEADER_FOREGROUND,
            font=("Consolas", 18),
            padx=12,
        ).grid(row=0, column=1, sticky="e")

        self.board_frame = tk.Frame(self, bg=BOARD_BACKGROUND, padx=12, pady=6)
        self.board_frame.grid(row=1, column=0, sticky="nsew")
        self.grid_rowconfigure(1, weight=1)
        self.grid_columnconfigure(0, weight=1)
        self.board_frame.grid_columnconfigure(0, weight=1)
        self.board_frame.grid_rowconfigure(1, weight=1)

        header = tk.Frame(self.board_frame, bg=BOARD_HEADER_BACKGROUND, bd=1, relief="solid")
        header.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        self._configure_table_columns(header)
        for index, (label, _) in enumerate(self.column_specs):
            cell = tk.Label(
                header,
                text=label,
                bg=BOARD_HEADER_BACKGROUND,
                fg=BOARD_HEADER_FOREGROUND,
                font=("Bahnschrift SemiBold", 11),
                padx=6,
                pady=6,
                bd=1,
                relief="solid",
            )
            cell.grid(row=0, column=index, sticky="nsew")

        self.rows_container = tk.Canvas(
            self.board_frame,
            bg=BOARD_BACKGROUND,
            highlightthickness=0,
            bd=0,
        )
        self.rows_container.grid(row=1, column=0, sticky="nsew")
        self.rows_container.bind("<Configure>", self._handle_rows_container_configure)
        self._rows_background_item = self.rows_container.create_image(0, 0, anchor="nw")

    def _build_rows(self) -> None:
        available_colors = list(BOARD_ALLOWED_COLORS.keys())
        starting_colors = random.sample(available_colors, k=BOARD_MAX_CARS)
        for car_id in range(1, BOARD_MAX_CARS + 1):
            name_var = tk.StringVar(value=f"Driver {car_id}")
            color_var = tk.StringVar(value=starting_colors[car_id - 1])
            state = BoardCarState(car_id=car_id, name_var=name_var, color_var=color_var)
            self.cars[car_id] = state

            name_var.trace_add("write", self._limit_name_length(state))
            color_var.trace_add("write", self._refresh_row_from_trace(state))

            row_frame = tk.Frame(self.rows_container, bg=BOARD_ALLOWED_COLORS["white"], bd=1, relief="solid")
            self.row_frames[car_id] = row_frame
            self._configure_table_columns(row_frame)

            widgets: dict[str, tk.Widget] = {}
            widgets["position"] = self._build_label(row_frame, 0, text="-")
            widgets["car"] = self._build_label(row_frame, 1, text=str(car_id))
            widgets["name"] = self._build_entry(row_frame, 2, state.name_var)
            widgets["color"] = self._build_color_menu(row_frame, 3, state.color_var)
            widgets["lap"] = self._build_label(row_frame, 4, text="1")
            widgets["third"] = self._build_label(row_frame, 5, text="1")
            widgets["third_time"] = self._build_label(row_frame, 6, text="-")
            widgets["third_diff"] = self._build_label(row_frame, 7, text="-")
            widgets["lap_time"] = self._build_label(row_frame, 8, text="-")
            widgets["lap_diff"] = self._build_label(row_frame, 9, text="-")
            widgets["medal"] = self._build_label(row_frame, 10, text="")
            state.frames = widgets
            self._apply_row_style(state)
            self.row_window_ids[car_id] = self.rows_container.create_window(
                0,
                0,
                anchor="nw",
                window=row_frame,
                width=1,
                height=ROW_HEIGHT,
            )

    def _configure_table_columns(self, parent: tk.Widget) -> None:
        for index, (_, minsize) in enumerate(self.column_specs):
            parent.grid_columnconfigure(index, minsize=minsize, weight=1, uniform="board")

    def _build_label(self, parent: tk.Widget, column: int, text: str) -> tk.Label:
        label = tk.Label(parent, text=text, font=("Consolas", 11), padx=6, pady=7, bd=1, relief="solid")
        label.grid(row=0, column=column, sticky="nsew")
        return label

    def _build_entry(self, parent: tk.Widget, column: int, textvariable: tk.StringVar) -> tk.Entry:
        entry = tk.Entry(parent, textvariable=textvariable, font=("Bahnschrift", 11), justify="center", bd=1, relief="solid")
        entry.grid(row=0, column=column, sticky="nsew")
        return entry

    def _build_color_menu(self, parent: tk.Widget, column: int, variable: tk.StringVar) -> tk.OptionMenu:
        menu = tk.OptionMenu(parent, variable, *BOARD_ALLOWED_COLORS.keys())
        menu.config(width=1, font=("Bahnschrift", 10), indicatoron=False, bd=1, relief="solid", highlightthickness=0)
        color_menu = menu["menu"]
        for index, color_name in enumerate(BOARD_ALLOWED_COLORS.keys()):
            swatch_color = BOARD_ALLOWED_COLORS[color_name]
            text_color = best_text_color(swatch_color)
            color_menu.entryconfigure(
                index,
                background=swatch_color,
                foreground=text_color,
                activebackground=swatch_color,
                activeforeground=text_color,
            )
        menu.grid(row=0, column=column, sticky="nsew")
        return menu

    def _build_test_panel(self) -> None:
        panel = tk.Frame(self, bg=BOARD_BACKGROUND, padx=12, pady=10)
        self.demo_panel = panel

        help_text = (
            "Test an incoming checkpoint packet: car id, current third (1-3), latest 1/3 time, "
            "current lap, latest lap time. Use '-' when a new time does not exist yet."
        )
        help_label = tk.Label(
            panel,
            text=help_text,
            bg=BOARD_BACKGROUND,
            fg="#d7d7df",
            anchor="w",
            justify="left",
            font=("Bahnschrift", 10),
        )
        help_label.grid(row=0, column=0, columnspan=12, sticky="ew", pady=(0, 8))

        self.test_vars = {
            "car_id": tk.StringVar(value="1"),
            "current_third": tk.StringVar(value="2"),
            "third_time": tk.StringVar(value="4.320"),
            "current_lap": tk.StringVar(value="1"),
            "lap_time": tk.StringVar(value="-"),
        }
        fields = [
            ("Car", "car_id"),
            ("Current Third", "current_third"),
            ("1/3 Time", "third_time"),
            ("Current Lap", "current_lap"),
            ("Lap Time", "lap_time"),
        ]
        for index, (label_text, key) in enumerate(fields):
            tk.Label(
                panel,
                text=label_text,
                bg=BOARD_BACKGROUND,
                fg="#f1f1f1",
                font=("Bahnschrift SemiBold", 10),
            ).grid(row=1, column=index * 2, sticky="w", padx=(0, 4))
            tk.Entry(
                panel,
                textvariable=self.test_vars[key],
                width=10,
                font=("Consolas", 11),
                justify="center",
            ).grid(row=1, column=(index * 2) + 1, padx=(0, 10))

        tk.Button(
            panel,
            text="Apply Update",
            command=self._apply_test_update,
            font=("Bahnschrift SemiBold", 10),
            bg="#e63946",
            fg="#ffffff",
            activebackground="#d62839",
            activeforeground="#ffffff",
            padx=10,
            pady=4,
        ).grid(row=1, column=10, padx=(8, 8))
        tk.Button(
            panel,
            text="Run Demo Sequence",
            command=self._run_demo_sequence,
            font=("Bahnschrift SemiBold", 10),
            bg="#457b9d",
            fg="#ffffff",
            activebackground="#35627e",
            activeforeground="#ffffff",
            padx=10,
            pady=4,
        ).grid(row=1, column=11, padx=(0, 8))

        self.control_panel = tk.Frame(self, bg=BOARD_BACKGROUND, padx=12, pady=10)
        self.control_panel.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        tk.Label(
            self.control_panel,
            text="Run Laps",
            bg=BOARD_BACKGROUND,
            fg="#f1f1f1",
            font=("Bahnschrift SemiBold", 10),
        ).grid(row=0, column=0, sticky="w", padx=(0, 6))
        tk.Entry(
            self.control_panel,
            textvariable=self.run_laps_var,
            width=6,
            font=("Consolas", 11),
            justify="center",
        ).grid(row=0, column=1, sticky="w", padx=(0, 12))
        self.start_button = tk.Button(
            self.control_panel,
            text="Start",
            command=self._handle_start_race,
            font=("Bahnschrift SemiBold", 10),
            bg="#2a9d8f",
            fg="#ffffff",
            activebackground="#23867a",
            activeforeground="#ffffff",
            padx=12,
            pady=4,
            state="disabled",
        )
        self.start_button.grid(row=0, column=2, sticky="w", padx=(0, 8))
        self.restart_button = tk.Button(
            self.control_panel,
            text="Reset",
            command=self._handle_restart_race,
            font=("Bahnschrift SemiBold", 10),
            bg="#6c757d",
            fg="#ffffff",
            activebackground="#5c636a",
            activeforeground="#ffffff",
            padx=12,
            pady=4,
        )
        self.restart_button.grid(row=0, column=3, sticky="w", padx=(0, 8))
        self.stop_button = tk.Button(
            self.control_panel,
            text="Stop",
            command=self._handle_stop_race,
            font=("Bahnschrift SemiBold", 10),
            bg="#c1121f",
            fg="#ffffff",
            activebackground="#9f0f19",
            activeforeground="#ffffff",
            padx=12,
            pady=4,
            state="disabled",
        )
        self.stop_button.grid(row=0, column=4, sticky="w")
    def _get_run_laps(self) -> int:
        try:
            run_laps = int(self.run_laps_var.get().strip())
        except (AttributeError, ValueError):
            run_laps = 6
        return max(1, run_laps)

    def _handle_start_race(self) -> None:
        if self._start_race_callback is not None:
            self._start_race_callback()

    def _handle_restart_race(self) -> None:
        if self._restart_race_callback is not None:
            self._restart_race_callback()

    def _handle_stop_race(self) -> None:
        if self._stop_race_callback is not None:
            self._stop_race_callback()

    def _limit_name_length(self, state: BoardCarState):
        def callback(*_: object) -> None:
            current = state.name_var.get()
            if len(current) > BOARD_MAX_NAME_LENGTH:
                state.name_var.set(current[:BOARD_MAX_NAME_LENGTH])
        return callback

    def _refresh_row_from_trace(self, state: BoardCarState):
        def callback(*_: object) -> None:
            selected = state.color_var.get().strip().lower()
            if selected not in BOARD_ALLOWED_COLORS:
                state.color_var.set("white")
                return
            self._apply_row_style(state)
        return callback

    def _apply_row_style(self, state: BoardCarState) -> None:
        base_color = BOARD_ALLOWED_COLORS[state.color_var.get()]
        row_frame = self.row_frames[state.car_id]
        row_frame.configure(bg=BOARD_BACKGROUND, highlightbackground=BOARD_CELL_BORDER)
        total_cells = len(self.row_widget_order)
        for index, key in enumerate(self.row_widget_order):
            widget = state.frames[key]
            alpha = 1.0 - (0.8 * index / max(1, total_cells - 1))
            bg_color = blend_hex_colors(base_color, BOARD_BACKGROUND, alpha)
            fg_color = best_text_color(bg_color)
            if isinstance(widget, tk.Label):
                widget.configure(bg=bg_color, fg=fg_color, highlightbackground=BOARD_CELL_BORDER)
            elif isinstance(widget, tk.Entry):
                widget.configure(
                    bg=bg_color,
                    fg=fg_color,
                    insertbackground=fg_color,
                    highlightbackground=BOARD_CELL_BORDER,
                    disabledbackground=bg_color,
                    disabledforeground=fg_color,
                )
            elif isinstance(widget, tk.OptionMenu):
                widget.configure(
                    bg=bg_color,
                    fg=fg_color,
                    activebackground=bg_color,
                    activeforeground=fg_color,
                    highlightbackground=BOARD_CELL_BORDER,
                )
                widget["menu"].configure(
                    bg=blend_hex_colors(base_color, BOARD_BACKGROUND, 0.75),
                    fg=best_text_color(blend_hex_colors(base_color, BOARD_BACKGROUND, 0.75)),
                    activebackground=bg_color,
                    activeforeground=fg_color,
                )
                for menu_index, color_name in enumerate(BOARD_ALLOWED_COLORS.keys()):
                    swatch_color = BOARD_ALLOWED_COLORS[color_name]
                    swatch_text = best_text_color(swatch_color)
                    widget["menu"].entryconfigure(
                        menu_index,
                        background=swatch_color,
                        foreground=swatch_text,
                        activebackground=swatch_color,
                        activeforeground=swatch_text,
                    )

    def _set_row_emphasis(self, state: BoardCarState, is_leader: bool) -> None:
        label_font = ("Consolas", 13 if is_leader else 11)
        entry_font = ("Bahnschrift", 13 if is_leader else 11)
        menu_font = ("Bahnschrift", 12 if is_leader else 10)
        pady = 12 if is_leader else 7

        for widget in state.frames.values():
            if isinstance(widget, tk.Label):
                widget.configure(font=label_font, pady=pady)
            elif isinstance(widget, tk.Entry):
                widget.configure(font=entry_font)
            elif isinstance(widget, tk.OptionMenu):
                widget.configure(font=menu_font)
                widget["menu"].configure(font=menu_font)

    def _place_row(self, state: BoardCarState) -> None:
        width = max(1, self.rows_container.winfo_width())
        self.rows_container.coords(
            self.row_window_ids[state.car_id],
            0,
            int(round(state.row_y)),
        )
        self.rows_container.itemconfigure(
            self.row_window_ids[state.car_id],
            width=width,
            height=int(round(state.row_height)),
        )

    def _animate_rows(self) -> None:
        self._row_animation_job = None
        animation_pending = False
        for car_id, state in self.cars.items():
            target_y, target_height = self._row_targets.get(car_id, (state.row_y, state.row_height))
            dy = target_y - state.row_y
            dh = target_height - state.row_height
            if abs(dy) <= ANIMATION_EPSILON and abs(dh) <= ANIMATION_EPSILON:
                state.row_y = target_y
                state.row_height = target_height
            else:
                state.row_y += dy * ANIMATION_SMOOTHING
                state.row_height += dh * ANIMATION_SMOOTHING
                animation_pending = True
            self._place_row(state)
        if animation_pending and self.winfo_exists():
            self._row_animation_job = self.after(16, self._animate_rows)

    def _start_row_animation(self) -> None:
        if self._row_animation_job is None:
            self._row_animation_job = self.after(16, self._animate_rows)

    def _start_title_hold(self, _: tk.Event) -> None:
        self._cancel_title_hold()
        self.title_hold_job = self.after(3000, self._toggle_demo_panel)

    def _cancel_title_hold(self, _: tk.Event | None = None) -> None:
        if self.title_hold_job is not None:
            self.after_cancel(self.title_hold_job)
            self.title_hold_job = None

    def _toggle_demo_panel(self) -> None:
        self.title_hold_job = None
        self.demo_panel_visible = not self.demo_panel_visible
        if self.demo_panel_visible:
            self.demo_panel.grid(row=3, column=0, sticky="ew")
        else:
            self.demo_panel.grid_remove()

    def reset_board(self, laps: int | None = None) -> None:
        if laps is not None:
            self.run_laps_var.set(str(max(1, laps)))
        self.finishers.clear()
        for state in self.cars.values():
            state.current_lap = 1
            state.current_third = 1
            state.last_third_time = None
            state.previous_third_time = None
            state.last_lap_time = None
            state.previous_lap_time = None
            state.total_elapsed = 0.0
            state.finished = False
            state.disqualified = False
            state.finish_place = None
            state.row_y = 0.0
            state.row_height = ROW_HEIGHT
        self.refresh_board()

    def process_update(
        self,
        car_id: int | str,
        current_third: int,
        latest_third_time: int | float | str | None,
        current_lap: int,
        latest_lap_time: int | float | str | None,
    ) -> None:
        parsed_car_id, is_disqualified = parse_car_packet(car_id)
        if parsed_car_id not in self.cars:
            return
        if current_third not in (1, 2, 3):
            raise ValueError("Current third must be 1, 2, or 3.")
        if current_lap < 1:
            raise ValueError("Current lap must be 1 or higher.")
        state = self.cars[parsed_car_id]
        if state.finished or state.disqualified:
            return

        incoming_progress = ((current_lap - 1) * 3) + current_third
        existing_progress = ((state.current_lap - 1) * 3) + state.current_third
        if state.has_started and incoming_progress < existing_progress:
            return

        if is_disqualified:
            state.current_third = current_third
            state.current_lap = current_lap
            state.disqualified = True
            self.refresh_board()
            return

        third_time = parse_board_time(latest_third_time)
        lap_time = parse_board_time(latest_lap_time)
        if third_time is not None:
            state.previous_third_time = state.last_third_time
            state.last_third_time = third_time
            state.total_elapsed += third_time
        if lap_time is not None:
            state.previous_lap_time = state.last_lap_time
            state.last_lap_time = lap_time

        state.current_third = current_third
        state.current_lap = current_lap
        if current_lap > self._get_run_laps():
            state.finished = True
            if state.car_id not in self.finishers:
                self.finishers.append(state.car_id)
                state.finish_place = len(self.finishers)
        self.refresh_board()

    def refresh_board(self) -> None:
        sorted_cars = sorted(
            self.cars.values(),
            key=lambda state: (
                state.disqualified,
                state.finished,
                -state.completed_thirds,
                math.inf if not state.has_started else state.total_elapsed,
                state.car_id,
            ),
        )
        total_height = 0
        for position in range(1, len(sorted_cars) + 1):
            total_height += LEADER_ROW_HEIGHT if position == 1 else ROW_HEIGHT
            if position < len(sorted_cars):
                total_height += ROW_GAP
        self.rows_container.configure(height=total_height)
        self.rows_container.grid_propagate(False)

        current_y = 0.0
        for position, state in enumerate(sorted_cars, start=1):
            is_leader = position == 1
            target_height = LEADER_ROW_HEIGHT if is_leader else ROW_HEIGHT
            self._row_targets[state.car_id] = (current_y, target_height)
            self._set_row_emphasis(state, is_leader)
            state.frames["position"].configure(text=str(position) if state.has_started else "-")
            state.frames["car"].configure(text=str(state.car_id))
            state.frames["lap"].configure(text="Finished" if state.finished else str(state.current_lap))
            state.frames["third"].configure(text=str(state.current_third))
            state.frames["third_time"].configure(text=format_board_time(state.last_third_time))
            state.frames["third_diff"].configure(text=format_board_diff(state.last_third_time, state.previous_third_time))
            state.frames["lap_time"].configure(text=format_board_time(state.last_lap_time))
            state.frames["lap_diff"].configure(text=format_board_diff(state.last_lap_time, state.previous_lap_time))
            if state.disqualified:
                state.frames["medal"].configure(text="DQ", image="", compound="center")
            elif state.finish_place in self._medal_images:
                state.frames["medal"].configure(text="", image=self._medal_images[state.finish_place], compound="center")
            else:
                state.frames["medal"].configure(text=format_place(state.finish_place), image="", compound="center")
            self._apply_row_style(state)
            current_y += target_height + ROW_GAP

        for state in self.cars.values():
            if state.car_id not in self._row_targets:
                continue
            target_y, target_height = self._row_targets[state.car_id]
            if state.row_y == 0.0 and state.row_height == ROW_HEIGHT:
                state.row_y = target_y
                state.row_height = target_height
                self._place_row(state)
        self._start_row_animation()

    def update_race_control_states(self, start_enabled: bool, stop_enabled: bool) -> None:
        if self.start_button is not None:
            self.start_button.configure(state="normal" if start_enabled else "disabled")
        if self.stop_button is not None:
            self.stop_button.configure(state="normal" if stop_enabled else "disabled")

    def finalize_current_standings(self) -> list[int]:
        placed_states = sorted(
            (
                state
                for state in self.cars.values()
                if not state.disqualified and state.finish_place is not None
            ),
            key=lambda state: (state.finish_place or math.inf, state.car_id),
        )
        ordered_unplaced = [
            state
            for state in sorted(
                self.cars.values(),
                key=lambda state: (
                    state.disqualified,
                    state.finished,
                    -state.completed_thirds,
                    math.inf if not state.has_started else state.total_elapsed,
                    state.car_id,
                ),
            )
            if not state.disqualified and state.finish_place is None
        ]
        next_place = len(placed_states) + 1
        ranked_car_ids = [state.car_id for state in placed_states]
        for state in ordered_unplaced:
            state.finish_place = next_place
            state.finished = True
            ranked_car_ids.append(state.car_id)
            next_place += 1
        self.refresh_board()
        return ranked_car_ids

    def _apply_test_update(self) -> None:
        try:
            self.process_update(
                car_id=self.test_vars["car_id"].get(),
                current_third=int(self.test_vars["current_third"].get()),
                latest_third_time=self.test_vars["third_time"].get(),
                current_lap=int(self.test_vars["current_lap"].get()),
                latest_lap_time=self.test_vars["lap_time"].get(),
            )
        except ValueError as exc:
            messagebox.showerror("Invalid update", str(exc))

    def _build_demo_sequence(self) -> list[tuple[int | str, int, float | str, int, float | str]]:
        demo_splits = {
            1: [4.15, 4.10, 4.05, 4.10, 4.05, 4.00, 4.05, 4.00, 3.95, 4.00, 3.95, 3.90, 3.95, 3.90, 3.85, 3.90, 3.85, 3.80, 3.88, 3.83, 3.78, 3.86, 3.81, 3.76, 3.84, 3.79, 3.74, 3.82, 3.77, 3.72],
            2: [4.00, 3.95, 3.90, 3.95, 3.90, 3.85, 3.90, 3.85, 3.80, 4.10, 4.10, 4.10, 4.15, 4.15, 4.15, 4.20, 4.20, 4.20, 4.22, 4.22, 4.22, 4.24, 4.24, 4.24, 4.26, 4.26, 4.26, 4.28, 4.28, 4.28],
            3: [4.70, 4.65, 4.60, 4.65, 4.60, 4.55, 4.60, 4.55, 4.50, 4.55, 4.50, 4.45, 3.95, 3.90, 3.85, 3.90, 3.85, 3.80, 3.88, 3.83, 3.78, 3.86, 3.81, 3.76, 3.84, 3.79, 3.74, 3.82, 3.77, 3.72],
            4: [4.25, 4.20, 4.15, 4.22, 4.17, 4.12, 4.20, 4.15, 4.10, 4.20, 4.18, 4.17, 4.18, 4.17, 4.15, 4.16, 4.15, 4.14, 4.15, 4.14, 4.13, 4.14, 4.13, 4.12, 4.13, 4.12, 4.11, 4.12, 4.11, 4.10],
            5: [4.40, 4.35, 4.30, 4.35, 4.30, 4.25, 4.30, 4.25, 4.20, 4.28, 4.24, 4.20, 4.26, 4.22, 4.18, 4.24, 4.20, 4.16, 4.22, 4.18, 4.14, 4.20, 4.16, 4.12, 4.18, 4.14, 4.10, 4.16, 4.12, 4.08],
        }
        events: list[tuple[float, tuple[int | str, int, float | str, int, float | str]]] = []
        dq_progress = ((6 - 1) * 3) + 2
        for car_id, splits in demo_splits.items():
            cumulative_time = 0.0
            lap_running_time = 0.0
            for completed_thirds, third_time in enumerate(splits, start=1):
                if car_id == 5 and completed_thirds == dq_progress:
                    current_lap = ((completed_thirds - 1) // 3) + 1
                    current_third = ((completed_thirds - 1) % 3) + 1
                    events.append((round(cumulative_time + 0.001, 6), (f"{car_id}DQ", current_third, "-", current_lap, "-")))
                    break
                cumulative_time += third_time
                lap_running_time += third_time
                current_lap = ((completed_thirds - 1) // 3) + 1
                next_third = (completed_thirds % 3) + 1
                next_lap = current_lap + 1 if completed_thirds % 3 == 0 else current_lap
                lap_time: float | str = round(lap_running_time, 3) if completed_thirds % 3 == 0 else "-"
                if completed_thirds % 3 == 0:
                    lap_running_time = 0.0
                events.append(
                    (
                        round(cumulative_time, 6),
                        (car_id, next_third, round(third_time, 3), next_lap, lap_time),
                    )
                )
        events.sort(key=lambda item: (item[0], item[1][0]))
        return [update for _, update in events]

    def _run_demo_sequence(self) -> None:
        self.reset_board()
        self.run_laps_var.set("10")
        sequence = self._build_demo_sequence()

        def step(index: int = 0) -> None:
            if index >= len(sequence):
                return
            self.process_update(*sequence[index])
            self.after(550, lambda: step(index + 1))

        step()


class SplashScreen:
    def __init__(self, image_path: Path) -> None:
        self.root: tk.Tk | None = None
        self.image: tk.PhotoImage | None = None
        self.progress: ttk.Progressbar | None = None
        self.progress_fill_id: int | None = None
        self.status_label: tk.Label | None = None
        self.image_size: tuple[int, int] = (256, 256)
        try:
            self.root = tk.Tk()
            self.root.overrideredirect(True)
            self.root.attributes("-topmost", True)
            self.root.configure(bg=SPLASH_BACKGROUND)

            image_width, image_height = self.image_size
            if LAPTIME_SPLASH_PNG.exists():
                self.image = tk.PhotoImage(file=str(LAPTIME_SPLASH_PNG))
                image_width = self.image.width()
                image_height = self.image.height()
                self.image_size = (image_width, image_height)
            elif image_path.exists():
                self.image = tk.PhotoImage(file=str(image_path))
                image_width = self.image.width()
                image_height = self.image.height()

            splash_width = image_width + 32
            splash_height = image_height + 88
            canvas = tk.Canvas(
                self.root,
                width=image_width,
                height=image_height,
                highlightthickness=0,
                bd=0,
                bg=SPLASH_BACKGROUND,
            )
            canvas.pack(padx=16, pady=(16, 10))
            if self.image is not None:
                canvas.create_image(image_width // 2, image_height // 2, image=self.image)
            else:
                canvas.create_rectangle(0, 0, image_width, image_height, fill=BOARD_HEADER_BACKGROUND, outline="")
                canvas.create_text(
                    image_width // 2,
                    image_height // 2,
                    text="LAPTIME\nBOARD",
                    fill=BOARD_HEADER_FOREGROUND,
                    font=("Bahnschrift SemiBold", 24),
                    justify="center",
                )

            bar_margin = 18
            bar_height = 14
            bar_top = image_height - 28
            canvas.create_rectangle(
                bar_margin,
                bar_top,
                image_width - bar_margin,
                bar_top + bar_height,
                fill=SPLASH_PROGRESS_BG,
                outline="",
            )
            self.progress_fill_id = canvas.create_rectangle(
                bar_margin,
                bar_top,
                bar_margin,
                bar_top + bar_height,
                fill=SPLASH_PROGRESS_FG,
                outline="",
            )
            self.progress = canvas
            self.status_label = tk.Label(
                self.root,
                text="Starting up",
                bg=SPLASH_BACKGROUND,
                fg="#f8f9fa",
                font=("Bahnschrift", 11),
            )
            self.status_label.pack(pady=(0, 14))

            self.root.update_idletasks()
            screen_width = self.root.winfo_screenwidth()
            screen_height = self.root.winfo_screenheight()
            x_pos = max(0, (screen_width - splash_width) // 2)
            y_pos = max(0, (screen_height - splash_height) // 2)
            self.root.geometry(f"{splash_width}x{splash_height}+{x_pos}+{y_pos}")
            self.root.update()
        except tk.TclError:
            self.close()

    def set_progress(self, value: float) -> None:
        if self.root is None or self.progress is None:
            return
        if self.progress_fill_id is not None:
            image_width, image_height = self.image_size
            bar_margin = 18
            bar_top = image_height - 28
            bar_height = 14
            total_width = image_width - (bar_margin * 2)
            fill_width = bar_margin + int(total_width * max(0.0, min(100.0, value)) / 100.0)
            self.progress.coords(
                self.progress_fill_id,
                bar_margin,
                bar_top,
                fill_width,
                bar_top + bar_height,
            )
        if self.status_label is not None:
            self.status_label.configure(text="Loading tracker")
        self.root.update_idletasks()
        self.root.update()

    def close(self) -> None:
        if self.root is None:
            return
        try:
            self.root.destroy()
        except tk.TclError:
            pass
        finally:
            self.root = None
            self.image = None
            self.progress = None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Track up to five red and blue blob pairs from a webcam, estimate vehicle pose, "
            "and overlay motion/orientation diagnostics."
        )
    )
    parser.add_argument("--camera", default="0", help="OpenCV camera index or video path. Default: 0.")
    parser.add_argument(
        "--backend",
        choices=("auto", "any", "dshow", "msmf"),
        default="auto",
        help="Preferred OpenCV backend for camera sources on Windows. Default: auto.",
    )
    parser.add_argument("--window-name", default="Multi Blob Tracker", help="GUI window title. Default: Multi Blob Tracker.")
    parser.add_argument("--width", type=int, default=960, help="Requested capture width in pixels. Default: 960.")
    parser.add_argument("--height", type=int, default=540, help="Requested capture height in pixels. Default: 540.")
    parser.add_argument("--min-area", type=float, default=15.0, help="Minimum contour area in pixels for each color blob. Default: 15.")
    parser.add_argument("--max-area", type=float, default=20000.0, help="Maximum contour area in pixels for each color blob. Default: 20000.")
    parser.add_argument("--max-pair-distance", type=float, default=160.0, help="Maximum distance in pixels between red and blue centroids. Default: 160.")
    parser.add_argument("--trail-length", type=int, default=TRAIL_LENGTH, help=f"Number of recent positions to display. Default: {TRAIL_LENGTH}.")
    parser.add_argument("--blur-kernel", type=int, default=3, help="Gaussian blur kernel size, must be odd. Default: 3.")
    parser.add_argument("--morph-kernel", type=int, default=3, help="Morphology kernel size in pixels. Default: 3.")
    parser.add_argument("--orientation-full-scale-deg", type=float, default=30.0, help="Relative orientation angle that maps to full red/green intensity. Default: 30.")
    parser.add_argument("--speed-reference", type=float, default=360.0, help="Reference speed in px/s for full-green trail coloring. Default: 360.")
    parser.add_argument("--stop-speed-threshold", type=float, default=6.0, help="Speeds below this px/s threshold are treated as stopped. Default: 6.0.")
    parser.add_argument(
        "--camera-calibration",
        default=None,
        help=(
            "Optional CSV file containing camera settings to prefer on startup. "
            "If omitted, the tracker tries the remembered calibration file, then "
            f"{DEFAULT_EXTERNAL_CALIBRATION_CSV.name}, then embedded defaults."
        ),
    )
    parser.add_argument(
        "--browse-calibration",
        action="store_true",
        help="Open a file browser to choose the camera calibration CSV at startup.",
    )
    parser.add_argument(
        "--serial-port",
        default=DEFAULT_SERIAL_PORT,
        help=f"Serial port for the ReactiveLEDs device. Default: {DEFAULT_SERIAL_PORT}.",
    )
    parser.add_argument(
        "--serial-baud",
        type=int,
        default=DEFAULT_SERIAL_BAUD,
        help=f"Serial baud rate. Default: {DEFAULT_SERIAL_BAUD}.",
    )
    parser.add_argument(
        "--car-id-base",
        type=int,
        default=DEFAULT_CAR_ID_BASE,
        help=f"Base car identifier embedded in CAR packets. Default: {DEFAULT_CAR_ID_BASE}.",
    )
    parser.add_argument(
        "--serial-startup-delay",
        type=float,
        default=DEFAULT_SERIAL_STARTUP_DELAY_S,
        help=(
            "Seconds to wait after opening the serial port before sending. "
            f"Default: {DEFAULT_SERIAL_STARTUP_DELAY_S}."
        ),
    )
    parser.add_argument(
        "--no-serial",
        action="store_true",
        help="Disable serial output and run as a local tracker only.",
    )
    parser.add_argument("--preview-worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--preview-host", default="127.0.0.1", help=argparse.SUPPRESS)
    parser.add_argument("--preview-port", type=int, default=0, help=argparse.SUPPRESS)
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


def load_camera_calibration(csv_path: str) -> list[tuple[str, int, float]]:
    calibration_path = Path(csv_path)
    if not calibration_path.exists():
        return []
    loaded_settings: list[tuple[str, int, float]] = []
    with calibration_path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            property_name = row.get("property_name", "").strip()
            if not property_name:
                continue
            loaded_settings.append((property_name, int(row["property_id"]), float(row["value"])))
    return loaded_settings


def save_default_calibration_path(csv_path: str) -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    CALIBRATION_PREFERENCE_FILE.write_text(csv_path, encoding="utf-8")


def load_default_calibration_path() -> str | None:
    if not CALIBRATION_PREFERENCE_FILE.exists():
        return None
    saved_path = CALIBRATION_PREFERENCE_FILE.read_text(encoding="utf-8").strip()
    return saved_path or None


def save_default_patch_path(csv_path: str) -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    PATCH_PREFERENCE_FILE.write_text(csv_path, encoding="utf-8")


def load_default_patch_path() -> str | None:
    if not PATCH_PREFERENCE_FILE.exists():
        return None
    saved_path = PATCH_PREFERENCE_FILE.read_text(encoding="utf-8").strip()
    return saved_path or None


def parse_patch_bool(raw_value: str) -> bool:
    normalized = raw_value.strip().lower()
    if normalized in {"1", "true", "t", "yes", "y", "on"}:
        return True
    if normalized in {"0", "false", "f", "no", "n", "off"}:
        return False
    raise ValueError(f"Invalid boolean value: {raw_value!r}")


def parse_patch_metadata(cells: list[str]) -> tuple[list[str], int | None, bool]:
    number_cells: list[str] = []
    segment_index: int | None = None
    wall_adjacent = False
    for cell in cells:
        lowered = cell.strip().lower()
        if not lowered:
            continue
        if "=" in lowered:
            key, value = lowered.split("=", 1)
            if key == "wall_adjacent":
                wall_adjacent = parse_patch_bool(value)
                continue
            if key in {"segmentindex", "segment_index"}:
                segment_index = int(value)
                continue
            continue
        number_cells.append(cell)
    return number_cells, segment_index, wall_adjacent


def parse_patch_row(
    row: list[str],
    row_number: int,
    next_segment_index: int,
) -> tuple[PatchOverlayRecord | None, int]:
    cells = [cell.strip() for cell in row]
    if not any(cells):
        return None, next_segment_index

    row_type = cells[0].lower()
    if row_type in {"label", "labels", "type"}:
        return None, next_segment_index
    if len(cells) < 2 or not cells[1]:
        raise ValueError(f"Row {row_number}: missing patch name.")

    name = cells[1]
    payload, metadata_segment_index, wall_adjacent = parse_patch_metadata(cells[2:])

    if row_type == "circlepatch":
        if len(payload) == 4:
            segment_index = int(payload[-1])
            payload = payload[:-1]
        elif metadata_segment_index is not None:
            segment_index = metadata_segment_index
        else:
            segment_index = next_segment_index
        if len(payload) != 3:
            raise ValueError(
                f"Row {row_number}: circlepatch rows must contain center x, center y, and radius."
            )
    elif row_type == "quadpatch":
        if len(payload) == 9:
            segment_index = int(payload[-1])
            payload = payload[:-1]
        elif metadata_segment_index is not None:
            segment_index = metadata_segment_index
        else:
            segment_index = next_segment_index
        if len(payload) != 8:
            raise ValueError(
                f"Row {row_number}: quadpatch rows must contain 8 coordinate values."
            )
    elif row_type == "polypatch":
        if len(payload) < 6:
            raise ValueError(
                f"Row {row_number}: polypatch rows must contain at least 3 x/y point pairs."
            )
        if len(payload) % 2 == 1:
            segment_index = int(payload[-1])
            payload = payload[:-1]
        elif metadata_segment_index is not None:
            segment_index = metadata_segment_index
        else:
            segment_index = next_segment_index
        if len(payload) < 6 or len(payload) % 2 != 0:
            raise ValueError(
                f"Row {row_number}: polypatch rows must contain 3 or more complete x/y pairs."
            )
    else:
        raise ValueError(f"Row {row_number}: unsupported patch type {cells[0]!r}.")

    numbers = tuple(float(value) for value in payload)
    next_segment_index = max(next_segment_index, segment_index + 1)
    return (
        PatchOverlayRecord(name, row_type, numbers, segment_index, wall_adjacent),
        next_segment_index,
    )


def parse_patch_rows(rows: list[list[str]], source_label: str) -> list[PatchOverlayRecord]:
    patches: list[PatchOverlayRecord] = []
    next_segment_index = 0
    for row_number, row in enumerate(rows, start=1):
        patch, next_segment_index = parse_patch_row(row, row_number, next_segment_index)
        if patch is not None:
            patches.append(patch)
    if not patches:
        raise ValueError(f"No patch rows found in {source_label}.")
    return patches


def load_patch_overlay(csv_path: str) -> list[PatchOverlayRecord]:
    patch_path = Path(csv_path)
    if not patch_path.exists():
        raise FileNotFoundError(f"Patch CSV not found: {patch_path}")

    with patch_path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.reader(handle)
        return parse_patch_rows(list(reader), str(patch_path))


def load_embedded_default_patches() -> list[PatchOverlayRecord]:
    reader = csv.reader(io.StringIO(EMBEDDED_DEFAULT_PATCH_CSV))
    return parse_patch_rows(list(reader), "embedded time-trial patch data")


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

    return None, EMBEDDED_CALIBRATION_SETTINGS.copy(), "embedded calibration"


def resolve_default_patch_source(preferred_path: str | None = None) -> str | None:
    candidate_paths: list[str] = []
    if preferred_path:
        candidate_paths.append(preferred_path)
    remembered_path = load_default_patch_path()
    if remembered_path:
        candidate_paths.append(remembered_path)
    candidate_paths.append(str(DEFAULT_PATCH_CSV))

    seen_paths: set[str] = set()
    for candidate_path in candidate_paths:
        normalized_path = str(Path(candidate_path))
        if normalized_path in seen_paths:
            continue
        seen_paths.add(normalized_path)
        if Path(normalized_path).exists():
            return normalized_path
    return None


def load_serial_patch_loader_module():
    spec = importlib.util.spec_from_file_location(
        SERIAL_PATCH_LOADER_MODULE_NAME,
        SERIAL_PATCH_LOADER_PATH,
    )
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not load serial patch loader from {SERIAL_PATCH_LOADER_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[SERIAL_PATCH_LOADER_MODULE_NAME] = module
    spec.loader.exec_module(module)
    return module


def normalize_patch_name(name: str) -> str:
    return name.strip().lower()


def find_required_patches(
    patches: list[PatchOverlayRecord],
) -> tuple[dict[str, PatchOverlayRecord], list[str]]:
    patch_lookup = {normalize_patch_name(patch.name): patch for patch in patches}
    missing = [name for name in REQUIRED_PATCH_NAMES if name not in patch_lookup]
    required = {
        name: patch_lookup[name]
        for name in REQUIRED_PATCH_NAMES
        if name in patch_lookup
    }
    return required, missing


def apply_calibration_settings(
    capture: cv2.VideoCapture,
    settings: list[tuple[str, int, float]],
    source_label: str,
) -> None:
    if not settings:
        return
    print(f"Applying camera calibration from {source_label}")
    for property_name, property_id, value in settings:
        applied = capture.set(property_id, value)
        actual_value = capture.get(property_id)
        status = "ok" if applied else "ignored"
        print(
            f"  {property_name}: requested {value:.3f}, "
            f"camera reports {actual_value:.3f} ({status})"
        )


def apply_default_camera_calibration(
    capture: cv2.VideoCapture,
    preferred_path: str | None = None,
) -> tuple[str | None, str]:
    resolved_path, settings, source_label = resolve_calibration_source(preferred_path)
    apply_calibration_settings(capture, settings, source_label)
    return resolved_path, source_label


def choose_calibration_file(initial_path: str) -> str | None:
    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    try:
        selected = filedialog.askopenfilename(
            parent=root,
            title="Select camera calibration CSV",
            initialdir=str(Path(initial_path).parent),
            initialfile=Path(initial_path).name,
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        return selected or None
    finally:
        root.destroy()


def validate_args(args: argparse.Namespace) -> None:
    if args.width <= 0:
        raise SystemExit("--width must be greater than 0")
    if args.height <= 0:
        raise SystemExit("--height must be greater than 0")
    if args.min_area <= 0:
        raise SystemExit("--min-area must be greater than 0")
    if args.max_area <= args.min_area:
        raise SystemExit("--max-area must be greater than --min-area")
    if args.max_pair_distance <= 0:
        raise SystemExit("--max-pair-distance must be greater than 0")
    if args.trail_length < 2:
        raise SystemExit("--trail-length must be at least 2")
    if args.blur_kernel < 1 or args.blur_kernel % 2 == 0:
        raise SystemExit("--blur-kernel must be an odd integer >= 1")
    if args.morph_kernel < 1:
        raise SystemExit("--morph-kernel must be >= 1")
    if args.orientation_full_scale_deg <= 0:
        raise SystemExit("--orientation-full-scale-deg must be > 0")
    if args.speed_reference < 0:
        raise SystemExit("--speed-reference must be >= 0")
    if args.stop_speed_threshold < 0:
        raise SystemExit("--stop-speed-threshold must be >= 0")
    if args.serial_baud <= 0:
        raise SystemExit("--serial-baud must be greater than 0")
    if args.car_id_base < 0:
        raise SystemExit("--car-id-base must be >= 0")
    if args.serial_startup_delay < 0:
        raise SystemExit("--serial-startup-delay must be >= 0")


def wrap_angle_deg(angle_deg: float) -> float:
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg <= -180.0:
        angle_deg += 360.0
    return angle_deg


def angle_deg_from_points(start: tuple[float, float], end: tuple[float, float]) -> float:
    return math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))


def distance(point_a: tuple[float, float], point_b: tuple[float, float]) -> float:
    return math.hypot(point_b[0] - point_a[0], point_b[1] - point_a[1])


def midpoint(point_a: tuple[float, float], point_b: tuple[float, float]) -> tuple[float, float]:
    return (0.5 * (point_a[0] + point_b[0]), 0.5 * (point_a[1] + point_b[1]))


def blend_color(color_a: tuple[int, int, int], color_b: tuple[int, int, int], amount: float) -> tuple[int, int, int]:
    clamped = max(0.0, min(1.0, amount))
    return tuple(
        int(round((1.0 - clamped) * channel_a + clamped * channel_b))
        for channel_a, channel_b in zip(color_a, color_b)
    )


def speed_color_bgr(speed_px_s: float, reference_speed_px_s: float) -> tuple[int, int, int]:
    if reference_speed_px_s <= 1e-6:
        return RED_BGR
    ratio = max(0.0, min(1.0, speed_px_s / reference_speed_px_s))
    return blend_color(RED_BGR, GREEN_BGR, ratio)


def travel_arrow_length_px(speed_px_s: float, reference_speed_px_s: float) -> int:
    if reference_speed_px_s <= 1e-6:
        return MIN_TRAVEL_ARROW_LENGTH_PX
    ratio = max(0.0, min(1.0, speed_px_s / reference_speed_px_s))
    return int(round(MIN_TRAVEL_ARROW_LENGTH_PX + ratio * (MAX_TRAVEL_ARROW_LENGTH_PX - MIN_TRAVEL_ARROW_LENGTH_PX)))


def orientation_color_bgr(travel_deg: float, orientation_deg: float, full_scale_deg: float) -> tuple[int, int, int]:
    delta_deg = wrap_angle_deg(orientation_deg - travel_deg)
    magnitude = max(0.0, min(1.0, abs(delta_deg) / full_scale_deg))
    if delta_deg > 0.0:
        return blend_color(BLACK_BGR, GREEN_BGR, magnitude)
    if delta_deg < 0.0:
        return blend_color(BLACK_BGR, RED_BGR, magnitude)
    return BLACK_BGR


def endpoint_from_angle(origin: tuple[int, int], angle_deg: float, length_px: int) -> tuple[int, int]:
    angle_rad = math.radians(angle_deg)
    return (
        origin[0] + int(round(length_px * math.cos(angle_rad))),
        origin[1] + int(round(length_px * math.sin(angle_rad))),
    )


def blend_line(frame: np.ndarray, start_point: tuple[int, int], end_point: tuple[int, int], color_bgr: tuple[int, int, int], alpha: float, thickness: int) -> None:
    overlay = frame.copy()
    cv2.line(overlay, start_point, end_point, color_bgr, thickness, lineType=cv2.LINE_AA)
    cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0.0, dst=frame)


def preprocess_frame(frame: np.ndarray, blur_kernel: int) -> np.ndarray:
    if blur_kernel > 1:
        frame = cv2.GaussianBlur(frame, (blur_kernel, blur_kernel), 0)
    return cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


def make_color_mask(hsv_frame: np.ndarray, color_name: str, morph_kernel: int) -> np.ndarray:
    if color_name == "red":
        mask = cv2.inRange(hsv_frame, np.array([0, 110, 70], dtype=np.uint8), np.array([12, 255, 255], dtype=np.uint8))
        mask |= cv2.inRange(hsv_frame, np.array([170, 110, 70], dtype=np.uint8), np.array([180, 255, 255], dtype=np.uint8))
    elif color_name == "blue":
        mask = cv2.inRange(hsv_frame, np.array([95, 100, 60], dtype=np.uint8), np.array([130, 255, 255], dtype=np.uint8))
    else:  # pragma: no cover
        raise ValueError(f"Unsupported color {color_name!r}")
    kernel = np.ones((morph_kernel, morph_kernel), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask


def extract_blobs(mask: np.ndarray, min_area: float, max_area: float) -> list[Blob]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs: list[Blob] = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area or area > max_area:
            continue
        moments = cv2.moments(contour)
        if abs(moments["m00"]) < 1e-6:
            continue
        blobs.append(
            Blob(
                centroid=(moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]),
                area=area,
                contour=contour,
            )
        )
    blobs.sort(key=lambda blob: blob.area, reverse=True)
    return blobs


def offset_blobs(blobs: list[Blob], x_offset: int, y_offset: int) -> list[Blob]:
    if x_offset == 0 and y_offset == 0:
        return blobs
    shifted: list[Blob] = []
    for blob in blobs:
        contour = blob.contour.copy()
        contour[:, 0, 0] += x_offset
        contour[:, 0, 1] += y_offset
        shifted.append(
            Blob(
                centroid=(blob.centroid[0] + x_offset, blob.centroid[1] + y_offset),
                area=blob.area,
                contour=contour,
            )
        )
    return shifted


def scale_blobs(blobs: list[Blob], scale: float) -> list[Blob]:
    if abs(scale - 1.0) < 1e-6:
        return blobs
    scaled: list[Blob] = []
    for blob in blobs:
        contour = np.rint(blob.contour.astype(np.float32) / scale).astype(np.int32)
        scaled.append(
            Blob(
                centroid=(blob.centroid[0] / scale, blob.centroid[1] / scale),
                area=blob.area / (scale * scale),
                contour=contour,
            )
        )
    return scaled


def pair_score(red_blob: Blob, blue_blob: Blob, previous_position: tuple[float, float] | None) -> float:
    pair_distance = distance(red_blob.centroid, blue_blob.centroid)
    pair_center = midpoint(red_blob.centroid, blue_blob.centroid)
    continuity_penalty = distance(pair_center, previous_position) * 1.5 if previous_position is not None else 0.0
    return (red_blob.area + blue_blob.area) - (pair_distance * 2.0) - continuity_penalty


def build_blob_pair_candidates(
    red_blobs: list[Blob],
    blue_blobs: list[Blob],
    max_pair_distance: float,
) -> list[BlobPairCandidate]:
    candidates: list[BlobPairCandidate] = []
    for red_blob in red_blobs:
        for blue_blob in blue_blobs:
            if distance(red_blob.centroid, blue_blob.centroid) > max_pair_distance:
                continue
            candidates.append(
                BlobPairCandidate(
                    red_blob=red_blob,
                    blue_blob=blue_blob,
                    center=midpoint(red_blob.centroid, blue_blob.centroid),
                    score=pair_score(red_blob, blue_blob, None),
                )
            )
    candidates.sort(key=lambda candidate: candidate.score, reverse=True)
    return candidates


def choose_blob_pairs(
    red_blobs: list[Blob],
    blue_blobs: list[Blob],
    max_pair_distance: float,
    previous_positions: list[tuple[float, float] | None],
    pair_count: int,
) -> list[tuple[Blob, Blob] | None]:
    candidates = build_blob_pair_candidates(red_blobs, blue_blobs, max_pair_distance)
    if not candidates:
        return [None] * pair_count

    best_assignment: list[tuple[Blob, Blob] | None] = [None] * pair_count
    best_cost = float("inf")

    def search(
        track_index: int,
        used_red_ids: set[int],
        used_blue_ids: set[int],
        current_assignment: list[tuple[Blob, Blob] | None],
        current_cost: float,
    ) -> None:
        nonlocal best_assignment, best_cost
        if track_index == pair_count:
            if current_cost < best_cost:
                best_cost = current_cost
                best_assignment = current_assignment.copy()
            return
        if current_cost >= best_cost:
            return

        previous_position = previous_positions[track_index]

        # Allow a missed detection, but penalize it so stable matches win when available.
        miss_penalty = 250.0 if previous_position is not None else 50.0
        current_assignment[track_index] = None
        search(track_index + 1, used_red_ids, used_blue_ids, current_assignment, current_cost + miss_penalty)

        for candidate in candidates:
            red_id = id(candidate.red_blob)
            blue_id = id(candidate.blue_blob)
            if red_id in used_red_ids or blue_id in used_blue_ids:
                continue
            continuity_cost = (
                distance(candidate.center, previous_position)
                if previous_position is not None
                else 0.0
            )
            candidate_cost = continuity_cost - (0.02 * candidate.score)
            current_assignment[track_index] = (candidate.red_blob, candidate.blue_blob)
            used_red_ids.add(red_id)
            used_blue_ids.add(blue_id)
            search(
                track_index + 1,
                used_red_ids,
                used_blue_ids,
                current_assignment,
                current_cost + candidate_cost,
            )
            used_red_ids.remove(red_id)
            used_blue_ids.remove(blue_id)

        current_assignment[track_index] = None

    search(0, set(), set(), [None] * pair_count, 0.0)

    # Fill the lowest-numbered empty tracks first when there is no continuity history
    # for those slots yet, so initial detections appear as Car 1, Car 2, etc.
    compacted_assignment: list[tuple[Blob, Blob] | None] = [None] * pair_count
    preserved_pair_ids: set[int] = set()

    for track_index, previous_position in enumerate(previous_positions):
        if previous_position is not None and best_assignment[track_index] is not None:
            compacted_assignment[track_index] = best_assignment[track_index]
            preserved_pair_ids.add(id(best_assignment[track_index]))

    pending_pairs = [
        pair
        for pair in best_assignment
        if pair is not None and id(pair) not in preserved_pair_ids
    ]

    for track_index, previous_position in enumerate(previous_positions):
        if compacted_assignment[track_index] is not None:
            continue
        if not pending_pairs:
            break
        compacted_assignment[track_index] = pending_pairs.pop(0)

    return compacted_assignment


def point_in_polygon(point: tuple[float, float], vertices: list[tuple[float, float]]) -> bool:
    inside = False
    point_x, point_y = point
    for index, start in enumerate(vertices):
        end = vertices[index - 1]
        start_x, start_y = start
        end_x, end_y = end
        intersects_y = (start_y > point_y) != (end_y > point_y)
        if not intersects_y:
            continue
        intersect_x = ((end_x - start_x) * (point_y - start_y) / ((end_y - start_y) + 1e-12)) + start_x
        if point_x < intersect_x:
            inside = not inside
    return inside


def point_in_patch(point: tuple[float, float], patch: PatchOverlayRecord) -> bool:
    if patch.kind == "circlepatch":
        center_x, center_y, radius = patch.numbers
        return distance(point, (center_x, center_y)) <= radius

    vertices = [
        (patch.numbers[index], patch.numbers[index + 1])
        for index in range(0, len(patch.numbers), 2)
    ]
    return point_in_polygon(point, vertices)


def patch_name_contains(name: str, token: str) -> bool:
    return token.strip().upper() in name.strip().upper()


def build_patch_mask(
    patches: list[PatchOverlayRecord],
    width: int,
    height: int,
    source_width: int,
    source_height: int,
) -> np.ndarray:
    mask = np.zeros((height, width), dtype=np.uint8)
    if not patches:
        return mask

    scale_x = width / max(source_width, 1)
    scale_y = height / max(source_height, 1)

    def scale_point(x_value: float, y_value: float) -> tuple[int, int]:
        return (
            int(round(x_value * scale_x)),
            int(round(y_value * scale_y)),
        )

    for patch in patches:
        if patch.kind == "circlepatch":
            center_x, center_y, radius = patch.numbers
            center = scale_point(center_x, center_y)
            radius_px = max(1, int(round(radius * 0.5 * (scale_x + scale_y))))
            cv2.circle(mask, center, radius_px, 255, thickness=-1, lineType=cv2.LINE_AA)
        else:
            points = np.array(
                [
                    scale_point(patch.numbers[index], patch.numbers[index + 1])
                    for index in range(0, len(patch.numbers), 2)
                ],
                dtype=np.int32,
            )
            cv2.fillPoly(mask, [points], 255, lineType=cv2.LINE_AA)
    return mask


def patch_base_color_bgr(patch_state: PatchOverlayState) -> tuple[int, int, int]:
    return YELLOW_BGR if patch_state.occupants else GRAY_BGR


def draw_blob_marker(frame: np.ndarray, blob: Blob, color_bgr: tuple[int, int, int]) -> None:
    cv2.drawContours(frame, [blob.contour], -1, color_bgr, thickness=-1, lineType=cv2.LINE_AA)


def draw_patch_overlay(
    frame: np.ndarray,
    patches: list[PatchOverlayRecord],
    patch_states: list[PatchOverlayState],
    timestamp_s: float,
    source_width: int,
    source_height: int,
) -> None:
    if not patches:
        return

    frame_height, frame_width = frame.shape[:2]
    scale_x = frame_width / max(source_width, 1)
    scale_y = frame_height / max(source_height, 1)

    def scale_point(x_value: float, y_value: float) -> tuple[int, int]:
        return (
            int(round(x_value * scale_x)),
            int(round(y_value * scale_y)),
        )

    overlay = frame.copy()
    labels: list[tuple[str, tuple[int, int], tuple[int, int, int]]] = []
    for patch, patch_state in zip(patches, patch_states):
        color_bgr = patch_base_color_bgr(patch_state)
        if patch_state.flash_color_bgr is not None and timestamp_s <= patch_state.flash_until_s:
            color_bgr = patch_state.flash_color_bgr
        if patch.kind == "circlepatch":
            center_x, center_y, radius = patch.numbers
            center = scale_point(center_x, center_y)
            radius_px = max(1, int(round(radius * 0.5 * (scale_x + scale_y))))
            cv2.circle(overlay, center, radius_px, color_bgr, thickness=-1, lineType=cv2.LINE_AA)
            cv2.circle(frame, center, radius_px, color_bgr, thickness=2, lineType=cv2.LINE_AA)
            label_point = (center[0] + radius_px + 6, center[1] - 6)
        else:
            points = np.array(
                [
                    scale_point(patch.numbers[index], patch.numbers[index + 1])
                    for index in range(0, len(patch.numbers), 2)
                ],
                dtype=np.int32,
            )
            cv2.fillPoly(overlay, [points], color_bgr, lineType=cv2.LINE_AA)
            cv2.polylines(frame, [points], isClosed=True, color=color_bgr, thickness=2, lineType=cv2.LINE_AA)
            top_left = points[np.argmin(points[:, 0] + points[:, 1])]
            label_point = (int(top_left[0]) + 6, int(top_left[1]) - 6)

        labels.append((patch.name, label_point, color_bgr))

    cv2.addWeighted(overlay, 0.2, frame, 0.8, 0.0, dst=frame)
    for patch_name, label_point, color_bgr in labels:
        cv2.putText(
            frame,
            patch_name,
            label_point,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color_bgr,
            1,
            cv2.LINE_AA,
        )


def draw_pose_overlay(
    frame: np.ndarray,
    samples: deque[PoseSample],
    orientation_full_scale_deg: float,
    speed_reference_px_s: float,
    stop_speed_threshold_px_s: float,
    label: str,
    label_color_bgr: tuple[int, int, int],
    show_label: bool,
    show_history: bool,
    draw_history_this_frame: bool,
    show_travel_arrow: bool,
    show_orientation_arrow: bool,
) -> None:
    if not samples:
        return
    points = [(int(round(sample.position[0])), int(round(sample.position[1]))) for sample in samples]
    segment_count = len(samples) - 1
    if show_history and draw_history_this_frame and segment_count > 0:
        for index in range(segment_count):
            alpha = TRAIL_MAX_ALPHA if segment_count == 1 else TRAIL_MIN_ALPHA + (index / (segment_count - 1)) * (TRAIL_MAX_ALPHA - TRAIL_MIN_ALPHA)
            segment_speed = samples[index + 1].speed_px_s
            blend_line(frame, points[index], points[index + 1], speed_color_bgr(segment_speed, speed_reference_px_s), alpha, TRAIL_THICKNESS_PX)
    current = samples[-1]
    current_point = points[-1]
    orientation_tip = endpoint_from_angle(current_point, current.orientation_deg, ORIENTATION_ARROW_LENGTH_PX)
    if show_orientation_arrow:
        cv2.circle(frame, current_point, DOT_RADIUS_PX, BLACK_BGR, thickness=-1, lineType=cv2.LINE_AA)
    if show_label:
        label_origin = (current_point[0] + 8, current_point[1] - 8)
        cv2.putText(
            frame,
            label,
            label_origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            BLACK_BGR,
            4,
            lineType=cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            label,
            label_origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            label_color_bgr,
            2,
            lineType=cv2.LINE_AA,
        )
    if show_travel_arrow and current.speed_px_s >= stop_speed_threshold_px_s:
        travel_tip = endpoint_from_angle(current_point, current.travel_deg, travel_arrow_length_px(current.speed_px_s, speed_reference_px_s))
        cv2.arrowedLine(frame, current_point, travel_tip, BLUE_BGR, ARROW_THICKNESS_PX, line_type=cv2.LINE_AA, tipLength=0.24)
    if show_orientation_arrow:
        cv2.arrowedLine(
            frame,
            current_point,
            orientation_tip,
            orientation_color_bgr(current.travel_deg, current.orientation_deg, orientation_full_scale_deg),
            ARROW_THICKNESS_PX,
            line_type=cv2.LINE_AA,
            tipLength=0.24,
        )


def read_frame(capture: cv2.VideoCapture) -> tuple[np.ndarray | None, float | None]:
    ok, frame = capture.read()
    if ok and frame is not None:
        return frame, time.perf_counter()
    return None, None


def fit_frame_to_canvas(frame: np.ndarray, target_width: int, target_height: int) -> np.ndarray:
    frame_height, frame_width = frame.shape[:2]
    scale = min(target_width / frame_width, target_height / frame_height)
    scaled_width = max(1, int(round(frame_width * scale)))
    scaled_height = max(1, int(round(frame_height * scale)))
    resized = cv2.resize(frame, (scaled_width, scaled_height), interpolation=cv2.INTER_LINEAR)
    canvas = np.zeros((target_height, target_width, 3), dtype=np.uint8)
    x_offset = (target_width - scaled_width) // 2
    y_offset = (target_height - scaled_height) // 2
    canvas[y_offset:y_offset + scaled_height, x_offset:x_offset + scaled_width] = resized
    return canvas


def get_preview_size(window_name: str) -> tuple[int, int]:
    default_size = (PREVIEW_WIDTH_PX, PREVIEW_HEIGHT_PX)
    if not hasattr(cv2, "getWindowImageRect"):
        return default_size
    try:
        _, _, width, height = cv2.getWindowImageRect(window_name)
    except cv2.error:
        return default_size
    if width <= 0 or height <= 0:
        return default_size
    return width, height


def process_preview_events() -> None:
    if hasattr(cv2, "pollKey"):
        cv2.pollKey()
        return
    cv2.waitKey(1)


def recv_exact(connection: socket.socket, size: int) -> bytes | None:
    chunks = bytearray()
    while len(chunks) < size:
        try:
            chunk = connection.recv(size - len(chunks))
        except socket.timeout:
            continue
        if not chunk:
            return None
        chunks.extend(chunk)
    return bytes(chunks)


def run_preview_worker(host: str, port: int, window_name: str) -> int:
    connection = socket.create_connection((host, port), timeout=PREVIEW_CONNECT_TIMEOUT_S)
    connection.settimeout(0.1)
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, PREVIEW_WIDTH_PX, PREVIEW_HEIGHT_PX)
    try:
        while True:
            header = recv_exact(connection, 4)
            if header is None:
                break
            frame_size = struct.unpack("!I", header)[0]
            if frame_size == 0:
                break
            encoded_frame = recv_exact(connection, frame_size)
            if encoded_frame is None:
                break
            frame_buffer = np.frombuffer(encoded_frame, dtype=np.uint8)
            frame = cv2.imdecode(frame_buffer, cv2.IMREAD_COLOR)
            if frame is None:
                continue
            cv2.imshow(window_name, frame)
            process_preview_events()
            try:
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                break
    finally:
        try:
            connection.close()
        except OSError:
            pass
        try:
            cv2.destroyWindow(window_name)
        except cv2.error:
            pass
    return 0


def compute_search_roi(
    frame_shape: tuple[int, int, int],
    previous_position: tuple[float, float] | None,
    previous_speed_px_s: float,
    dt_s: float | None,
    max_pair_distance: float,
) -> tuple[int, int, int, int] | None:
    if previous_position is None:
        return None
    frame_height, frame_width = frame_shape[:2]
    travel_margin = previous_speed_px_s * dt_s if dt_s is not None else 0.0
    radius = int(round((0.75 * max_pair_distance) + ROI_MARGIN_PX + (0.75 * travel_margin)))
    center_x = int(round(previous_position[0]))
    center_y = int(round(previous_position[1]))
    x0 = max(0, center_x - radius)
    y0 = max(0, center_y - radius)
    x1 = min(frame_width, center_x + radius)
    y1 = min(frame_height, center_y + radius)
    if x1 <= x0 or y1 <= y0:
        return None
    return x0, y0, x1, y1


def expand_roi(
    roi: tuple[int, int, int, int] | None,
    frame_shape: tuple[int, int, int],
    extra_margin_px: int,
) -> tuple[int, int, int, int] | None:
    if roi is None:
        return None
    frame_height, frame_width = frame_shape[:2]
    x0, y0, x1, y1 = roi
    return (
        max(0, x0 - extra_margin_px),
        max(0, y0 - extra_margin_px),
        min(frame_width, x1 + extra_margin_px),
        min(frame_height, y1 + extra_margin_px),
    )


class TrackerApp:
    def __init__(self, capture: cv2.VideoCapture, args: argparse.Namespace, backend_name: str, actual_width: int, actual_height: int) -> None:
        self.capture = capture
        self.args = args
        self.backend_name = backend_name
        self.actual_width = actual_width
        self.actual_height = actual_height
        self.running = True
        self.tracks = [TrackState(deque(maxlen=args.trail_length)) for _ in range(TRACK_COUNT)]
        self.last_search_mode = "Full frame"
        self.calibration_path = args.camera_calibration
        self.calibration_source_label = args.camera_calibration
        self.patch_path = resolve_default_patch_source()
        self.loaded_patches: list[PatchOverlayRecord] = []
        self.patch_states: list[PatchOverlayState] = []
        self.patch_load_error: str | None = None
        self.required_patch_map: dict[str, PatchOverlayRecord] = {}
        self.patch_source_width = PATCH_SOURCE_WIDTH_PX
        self.patch_source_height = PATCH_SOURCE_HEIGHT_PX
        self.last_frame_time_s: float | None = None
        self.frame_counter = 0
        self.race_running = False
        self.race_started_at_s: float | None = None
        self.completed_car_count = 0
        self.time_trial_states = [TimeTrialCarState() for _ in range(TRACK_COUNT)]
        self.preview_worker = PreviewWorkerState()
        self.serial_state = SerialConnectionState(
            port=args.serial_port,
            baud=args.serial_baud,
            startup_delay_s=args.serial_startup_delay,
            enabled=not args.no_serial,
        )
        self.processing_scale = min(
            1.0,
            PROCESSING_WIDTH_PX / max(1, actual_width),
            PROCESSING_HEIGHT_PX / max(1, actual_height),
        )
        self.processing_width = max(1, int(round(actual_width * self.processing_scale)))
        self.processing_height = max(1, int(round(actual_height * self.processing_scale)))

        self.root = tk.Tk()
        self.root.title("Multiplayer Timetrials")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.resizable(False, False)
        self.root.geometry(f"{SIDE_PANEL_WIDTH_PX}x{SIDE_PANEL_HEIGHT_PX}")
        self.root.columnconfigure(0, weight=0)
        self.root.rowconfigure(0, weight=0)
        self.show_label_var = tk.BooleanVar(master=self.root, value=False)
        self.show_history_var = tk.BooleanVar(master=self.root, value=False)
        self.show_travel_arrow_var = tk.BooleanVar(master=self.root, value=False)
        self.show_orientation_arrow_var = tk.BooleanVar(master=self.root, value=False)
        self.show_blob_overlay_var = tk.BooleanVar(master=self.root, value=False)
        self.show_patches_var = tk.BooleanVar(master=self.root, value=False)
        self.laps_var = tk.StringVar(master=self.root, value="6")
        self.timer_var = tk.StringVar(master=self.root, value="00:00.000")
        self.start_button: ttk.Button | None = None
        self.reset_button: ttk.Button | None = None
        self.stop_button: ttk.Button | None = None
        self.icon_image: tk.PhotoImage | None = None
        self.apply_window_icon()
        self.lap_board = IntegratedLapTimeBoard(
            self.root,
            on_close=self.close,
            laps_var=self.laps_var,
            timer_var=self.timer_var,
            on_start=self.start_race,
            on_restart=self.reset_race,
            on_stop=self.stop_race,
        )

        self.side_panel = ttk.Frame(
            self.root,
            padding=12,
            width=SIDE_PANEL_WIDTH_PX,
            height=SIDE_PANEL_HEIGHT_PX,
        )
        self.side_panel.grid(row=0, column=0, sticky="nsew")
        self.side_panel.grid_propagate(False)
        self.side_panel.columnconfigure(0, weight=1)
        self.side_panel.rowconfigure(8, weight=1)

        ttk.Label(
            self.side_panel,
            text=(
                f"Source: {args.camera}\n"
                f"Backend: {backend_name}\n"
                f"Resolution: requested {args.width}x{args.height}, actual {actual_width}x{actual_height}"
            ),
            justify="left",
        ).grid(row=0, column=0, sticky="ew")

        self.calibration_label = ttk.Label(
            self.side_panel,
            text=self.format_calibration_label(),
            justify="left",
        )
        self.calibration_label.grid(row=1, column=0, sticky="ew", pady=(12, 6))

        ttk.Button(
            self.side_panel,
            text="Choose Calibration CSV",
            command=self.browse_calibration_file,
        ).grid(row=2, column=0, sticky="ew")

        self.patch_label = ttk.Label(
            self.side_panel,
            text=self.format_patch_label(),
            justify="left",
        )
        self.patch_label.grid(row=3, column=0, sticky="ew", pady=(12, 6))

        ttk.Button(
            self.side_panel,
            text="Choose Patch CSV",
            command=self.browse_patch_file,
        ).grid(row=4, column=0, sticky="ew")

        self.race_controls_frame = ttk.Frame(self.side_panel)
        self.race_controls_frame.grid(row=5, column=0, sticky="ew", pady=(12, 0))
        self.race_controls_frame.columnconfigure(2, weight=1)
        self.race_controls_frame.columnconfigure(3, weight=1)
        self.race_controls_frame.columnconfigure(4, weight=1)
        ttk.Label(self.race_controls_frame, text="Laps").grid(row=0, column=0, sticky="w")
        ttk.Entry(
            self.race_controls_frame,
            textvariable=self.laps_var,
            width=6,
            justify="center",
        ).grid(row=0, column=1, sticky="w", padx=(6, 12))
        self.start_button = ttk.Button(
            self.race_controls_frame,
            text="Start",
            command=self.start_race,
        )
        self.start_button.grid(row=0, column=2, sticky="ew", padx=(0, 8))
        self.reset_button = ttk.Button(
            self.race_controls_frame,
            text="Reset",
            command=self.reset_race,
        )
        self.reset_button.grid(row=0, column=3, sticky="ew", padx=(0, 8))
        self.stop_button = ttk.Button(
            self.race_controls_frame,
            text="Stop",
            command=self.stop_race,
            state="disabled",
        )
        self.stop_button.grid(row=0, column=4, sticky="ew")

        self.status_text = tk.Text(
            self.side_panel,
            width=36,
            height=STATUS_TEXT_HEIGHT,
            wrap="word",
        )
        self.status_scrollbar = ttk.Scrollbar(
            self.side_panel,
            orient="vertical",
            command=self.status_text.yview,
        )
        self.status_text.configure(yscrollcommand=self.status_scrollbar.set)
        self.status_text.grid(row=6, column=0, sticky="nsew", pady=(12, 12))
        self.status_scrollbar.grid(row=6, column=1, sticky="ns", pady=(12, 12))
        self.status_text.configure(state="disabled")

        self.controls_frame = ttk.Frame(self.side_panel)
        self.controls_frame.grid(row=7, column=0, sticky="ew", pady=(0, 12))
        self.controls_frame.columnconfigure(0, weight=1)
        self.controls_frame.columnconfigure(1, weight=1)

        ttk.Label(self.controls_frame, text="Overlay Elements").grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(
            self.controls_frame,
            text="Show Patches",
            variable=self.show_patches_var,
        ).grid(row=1, column=0, sticky="w")
        ttk.Checkbutton(
            self.controls_frame,
            text="Show Blob Overlay",
            variable=self.show_blob_overlay_var,
        ).grid(row=1, column=1, sticky="w", padx=(12, 0))
        ttk.Checkbutton(
            self.controls_frame,
            text="Car ID Labels",
            variable=self.show_label_var,
        ).grid(row=2, column=0, sticky="w")
        ttk.Checkbutton(
            self.controls_frame,
            text="History Lines",
            variable=self.show_history_var,
        ).grid(row=2, column=1, sticky="w", padx=(12, 0))
        ttk.Checkbutton(
            self.controls_frame,
            text="Travel Arrow",
            variable=self.show_travel_arrow_var,
        ).grid(row=3, column=0, sticky="w")
        ttk.Checkbutton(
            self.controls_frame,
            text="Orientation Arrow",
            variable=self.show_orientation_arrow_var,
        ).grid(row=3, column=1, sticky="w", padx=(12, 0))

        self.spacer_frame = ttk.Frame(self.side_panel)
        self.spacer_frame.grid(row=8, column=0, sticky="nsew")

        self.start_preview_worker()
        self.load_patch_file(self.patch_path)
        self.update_start_button_state({}, [])

        if self.serial_state.enabled:
            try:
                self.serial_state.connect()
            except serial.SerialException as exc:
                self.serial_state.handle = None
                self.serial_state.last_error = str(exc)
                print(
                    f"Starting disconnected because {self.serial_state.port!r} "
                    f"could not be opened: {exc}"
                )
        self.reset_race()

    def apply_window_icon(self) -> None:
        if not DEFAULT_FAVICON_PNG.exists():
            return
        try:
            self.icon_image = tk.PhotoImage(file=str(DEFAULT_FAVICON_PNG))
            self.root.iconphoto(True, self.icon_image)
        except tk.TclError:
            self.icon_image = None

    def format_calibration_label(self) -> str:
        if self.calibration_path:
            calibration_name = Path(self.calibration_path).name
            return f"Calibration CSV: {calibration_name}"
        return f"Calibration: {self.calibration_source_label}"

    def format_patch_label(self) -> str:
        if self.patch_path:
            return f"Patch CSV: {Path(self.patch_path).name}"
        if self.loaded_patches:
            return f"Patch CSV: embedded {DEFAULT_PATCH_CSV.name}"
        return "Patch CSV: none loaded"

    def browse_calibration_file(self) -> None:
        selected = filedialog.askopenfilename(
            parent=self.root,
            title="Select camera calibration CSV",
            initialdir=str(Path(self.calibration_path).parent) if self.calibration_path else str(DEFAULT_CALIBRATION_CSV.parent),
            initialfile=Path(self.calibration_path).name if self.calibration_path else DEFAULT_CALIBRATION_CSV.name,
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not selected:
            return
        self.calibration_path = selected
        self.calibration_source_label = selected
        self.args.camera_calibration = selected
        save_default_calibration_path(selected)
        self.calibration_label.configure(text=self.format_calibration_label())
        resolved_path, source_label = apply_default_camera_calibration(self.capture, selected)
        self.calibration_path = resolved_path
        self.calibration_source_label = source_label
        self.args.camera_calibration = resolved_path or selected
        self.calibration_label.configure(text=self.format_calibration_label())
        self.set_status(
            [
                "Applied camera calibration.",
                f"Source: {source_label}",
            ]
        )

    def browse_patch_file(self) -> None:
        initial_path = self.patch_path or str(DEFAULT_PATCH_CSV)
        selected = filedialog.askopenfilename(
            parent=self.root,
            title="Select patch CSV",
            initialdir=str(Path(initial_path).parent),
            initialfile=Path(initial_path).name,
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not selected:
            return
        self.load_patch_file(selected, persist=True)

    def led_controller_patches(self, patches: list[PatchOverlayRecord]) -> list[PatchOverlayRecord]:
        return [patch for patch in patches if patch_name_contains(patch.name, "corner")]

    def upload_patch_file_to_controller(self, patch_path: Path) -> None:
        if not self.serial_state.enabled:
            self.serial_state.last_error = None
            return
        if self.serial_state.handle is None:
            self.serial_state.connect()
        patches = load_patch_overlay(str(patch_path))
        self.serial_state.upload_patch_records(self.led_controller_patches(patches))

    def upload_patch_records_to_controller(self, patches: list[PatchOverlayRecord]) -> None:
        if not self.serial_state.enabled:
            self.serial_state.last_error = None
            return
        if self.serial_state.handle is None:
            self.serial_state.connect()
        self.serial_state.upload_patch_records(self.led_controller_patches(patches))

    def load_patch_file(self, patch_path: str | None, persist: bool = False) -> None:
        if not patch_path:
            patches = load_embedded_default_patches()
            patch_path = None
            patch_error = (
                f"Patch CSV not found. Loaded embedded fallback for {DEFAULT_PATCH_CSV.name}."
            )
        else:
            try:
                patches = load_patch_overlay(patch_path)
                patch_error = None
            except (FileNotFoundError, ValueError, OSError) as exc:
                if Path(patch_path) == DEFAULT_PATCH_CSV:
                    patches = load_embedded_default_patches()
                    patch_error = (
                        f"{exc} Loaded embedded fallback for {DEFAULT_PATCH_CSV.name}."
                    )
                    patch_path = None
                else:
                    self.patch_load_error = str(exc)
                    self.loaded_patches = []
                    self.patch_states = []
                    self.required_patch_map = {}
                    self.patch_path = patch_path
                    self.patch_label.configure(text=self.format_patch_label())
                    return

        required_patch_map, missing_required = find_required_patches(patches)
        if missing_required:
            fallback_path = resolve_default_patch_source(str(DEFAULT_PATCH_CSV))
            fallback_reason = (
                f"Patch CSV must include Finish, third1, and third2. Missing: {', '.join(missing_required)}."
            )
            if patch_path != fallback_path and fallback_path:
                self.patch_load_error = f"{fallback_reason} Reverting to {Path(fallback_path).name}."
                self.load_patch_file(fallback_path, persist=False)
                return
            self.patch_path = fallback_path or patch_path
            self.loaded_patches = []
            self.patch_states = []
            self.required_patch_map = {}
            self.patch_load_error = fallback_reason
            self.patch_label.configure(text=self.format_patch_label())
            return

        self.patch_path = patch_path
        self.loaded_patches = patches
        self.patch_states = [PatchOverlayState() for _ in patches]
        self.required_patch_map = required_patch_map
        self.patch_load_error = patch_error
        self.patch_label.configure(text=self.format_patch_label())
        try:
            if patch_path and Path(patch_path).exists():
                self.upload_patch_file_to_controller(Path(patch_path))
            else:
                self.upload_patch_records_to_controller(patches)
        except (ImportError, OSError, RuntimeError, serial.SerialException, TimeoutError, ValueError) as exc:
            self.patch_load_error = f"Patch upload failed: {exc}"
        if persist and patch_path:
            save_default_patch_path(patch_path)

    def current_run_laps(self) -> int:
        try:
            run_laps = int(self.laps_var.get().strip())
        except (AttributeError, ValueError):
            run_laps = 1
        return max(1, run_laps)

    def format_stopwatch(self, elapsed_s: float | None) -> str:
        if elapsed_s is None:
            return "00:00.000"
        total_ms = max(0, int(round(elapsed_s * 1000.0)))
        minutes, remaining_ms = divmod(total_ms, 60000)
        seconds, milliseconds = divmod(remaining_ms, 1000)
        return f"{minutes:02d}:{seconds:02d}.{milliseconds:03d}"

    def reset_time_trial_state(self) -> None:
        self.time_trial_states = [TimeTrialCarState() for _ in range(TRACK_COUNT)]
        self.race_running = False
        self.race_started_at_s = None
        self.completed_car_count = 0
        self.timer_var.set("00:00.000")

    def reset_race(self) -> None:
        self.reset_time_trial_state()
        self.lap_board.reset_board(self.current_run_laps())
        self.update_start_button_state({}, self.current_outside_finish())

    def start_race(self) -> None:
        if self.race_running or not self.required_patch_map:
            return
        outside_finish = self.current_outside_finish()
        if outside_finish:
            return

        self.reset_time_trial_state()
        self.race_running = True
        self.race_started_at_s = time.perf_counter()
        for state in self.time_trial_states:
            state.last_checkpoint_time_s = self.race_started_at_s
            state.lap_start_time_s = self.race_started_at_s
        self.lap_board.reset_board(self.current_run_laps())
        self.update_start_button_state({}, [])

    def stop_race(self) -> None:
        if not self.race_running:
            return
        if self.race_started_at_s is not None:
            self.timer_var.set(self.format_stopwatch(time.perf_counter() - self.race_started_at_s))
        ranked_car_ids = set(self.lap_board.finalize_current_standings())
        self.race_running = False
        self.race_started_at_s = None
        self.completed_car_count = len(ranked_car_ids)
        for track_index, state in enumerate(self.time_trial_states):
            car_id = self.args.car_id_base + track_index
            if state.disqualified:
                continue
            if car_id in ranked_car_ids:
                state.finished = True
        self.update_start_button_state({}, self.current_outside_finish())

    def current_detected_car_positions(self) -> dict[int, tuple[float, float]]:
        positions: dict[int, tuple[float, float]] = {}
        for track_index, track in enumerate(self.tracks):
            if track.last_pose is None or not track.detected:
                continue
            positions[track_index] = track.last_pose.position
        return positions

    def current_outside_finish(self) -> list[int]:
        finish_patch = self.required_patch_map.get("finish")
        if finish_patch is None:
            return []
        return sorted(
            track_index
            for track_index, car_position in self.current_detected_car_positions().items()
            if not point_in_patch(self.to_patch_coordinates(car_position), finish_patch)
        )

    @staticmethod
    def is_disqualifying_patch_name(patch_name: str) -> bool:
        return "DQ" in patch_name.strip().upper()

    @staticmethod
    def is_point_patch_name(patch_name: str) -> bool:
        return "POINT" in patch_name.strip().upper()

    def to_patch_coordinates(self, position: tuple[float, float]) -> tuple[float, float]:
        scale_x = self.patch_source_width / max(float(self.actual_width), 1.0)
        scale_y = self.patch_source_height / max(float(self.actual_height), 1.0)
        return (position[0] * scale_x, position[1] * scale_y)

    def update_start_button_state(
        self,
        current_required_patches: dict[int, set[str]],
        outside_finish: list[int],
    ) -> None:
        start_enabled = bool(not self.race_running and self.required_patch_map and not outside_finish)
        if self.start_button is not None:
            self.start_button.configure(state="normal" if start_enabled else "disabled")
        if self.stop_button is not None:
            self.stop_button.configure(state="normal" if self.race_running else "disabled")
        self.lap_board.update_race_control_states(start_enabled, self.race_running)

    def send_board_progress_update(
        self,
        track_index: int,
        current_third: int,
        latest_third_time: float | str,
        current_lap: int,
        latest_lap_time: float | str,
    ) -> None:
        self.lap_board.process_update(
            car_id=self.args.car_id_base + track_index,
            current_third=current_third,
            latest_third_time=latest_third_time,
            current_lap=current_lap,
            latest_lap_time=latest_lap_time,
        )

    def disqualify_car(self, track_index: int) -> None:
        state = self.time_trial_states[track_index]
        if state.disqualified or state.finished:
            return
        state.disqualified = True
        self.lap_board.process_update(
            car_id=f"{self.args.car_id_base + track_index}DQ",
            current_third=state.current_third,
            latest_third_time="-",
            current_lap=state.current_lap,
            latest_lap_time="-",
        )

    def apply_point_bonus(self, track_index: int) -> None:
        state = self.time_trial_states[track_index]
        if state.disqualified or state.finished:
            return
        state.pending_third_adjustment_s -= 1.0
        state.lap_adjustment_s -= 1.0

    def update_tracking_loss_disqualifications(self) -> None:
        if not self.race_running:
            for state in self.time_trial_states:
                state.lost_tracking_frames = 0
            return

        for track_index, track in enumerate(self.tracks):
            state = self.time_trial_states[track_index]
            if state.disqualified or state.finished:
                state.lost_tracking_frames = 0
                continue
            if track.detected:
                state.lost_tracking_frames = 0
                continue
            state.lost_tracking_frames += 1
            if state.lost_tracking_frames > TRACKING_LOSS_DQ_FRAMES:
                self.disqualify_car(track_index)

    def handle_legal_patch_entry(self, track_index: int, patch_name: str, timestamp_s: float) -> None:
        state = self.time_trial_states[track_index]
        if state.disqualified or state.finished:
            return

        if state.last_checkpoint_time_s is None or state.lap_start_time_s is None:
            state.last_checkpoint_time_s = timestamp_s
            state.lap_start_time_s = timestamp_s

        split_time_s = max(
            0.0,
            (timestamp_s - state.last_checkpoint_time_s) + state.pending_third_adjustment_s,
        )
        state.last_checkpoint_time_s = timestamp_s
        state.pending_third_adjustment_s = 0.0
        run_laps = self.current_run_laps()

        if patch_name == "third1":
            state.current_third = 2
            state.expected_patch = "third2"
            state.last_seen_required_patch = "third1"
            self.send_board_progress_update(
                track_index,
                current_third=2,
                latest_third_time=round(split_time_s, 3),
                current_lap=state.current_lap,
                latest_lap_time="-",
            )
            return

        if patch_name == "third2":
            state.current_third = 3
            state.expected_patch = "finish"
            state.last_seen_required_patch = "third2"
            self.send_board_progress_update(
                track_index,
                current_third=3,
                latest_third_time=round(split_time_s, 3),
                current_lap=state.current_lap,
                latest_lap_time="-",
            )
            return

        lap_time_s = max(0.0, (timestamp_s - state.lap_start_time_s) + state.lap_adjustment_s)
        state.completed_laps += 1
        state.current_lap += 1
        state.current_third = 1
        state.expected_patch = "third1"
        state.last_seen_required_patch = "finish"
        state.lap_start_time_s = timestamp_s
        state.lap_adjustment_s = 0.0
        self.send_board_progress_update(
            track_index,
            current_third=1,
            latest_third_time=round(split_time_s, 3),
            current_lap=state.current_lap,
            latest_lap_time=round(lap_time_s, 3),
        )
        if state.completed_laps >= run_laps:
            state.finished = True
            self.completed_car_count += 1

    def update_time_trial_progress(
        self,
        detected_car_positions: list[tuple[int, tuple[float, float]]],
        timestamp_s: float,
    ) -> tuple[dict[int, set[str]], list[int]]:
        current_required_patches: dict[int, set[str]] = {}
        outside_finish: list[int] = []
        finish_patch = self.required_patch_map.get("finish")
        for track_index, car_position in detected_car_positions:
            patch_position = self.to_patch_coordinates(car_position)
            current_patch_names = {
                patch.name
                for patch in self.loaded_patches
                if point_in_patch(patch_position, patch)
            }
            current_required_patches[track_index] = {
                patch_name for patch_name in current_patch_names if patch_name in self.required_patch_map
            }
            if finish_patch is not None and "finish" not in current_required_patches[track_index]:
                outside_finish.append(track_index)

            state = self.time_trial_states[track_index]
            previous_patches = state.active_required_patches
            entering_patches = [
                patch.name
                for patch in self.loaded_patches
                if patch.name in current_patch_names and patch.name not in previous_patches
            ]
            state.active_required_patches = current_patch_names
            if not self.race_running or state.disqualified or state.finished:
                continue

            entered_point_patch = False

            for patch_name in entering_patches:
                if self.is_point_patch_name(patch_name):
                    self.apply_point_bonus(track_index)
                    entered_point_patch = True

            if not entered_point_patch:
                for patch_name in entering_patches:
                    if self.is_disqualifying_patch_name(patch_name):
                        self.disqualify_car(track_index)
                        break
                if state.disqualified:
                    continue

            for patch_name in PATCH_SEQUENCE:
                if patch_name not in entering_patches:
                    continue
                if patch_name == state.expected_patch:
                    self.handle_legal_patch_entry(track_index, patch_name, timestamp_s)
                elif patch_name != state.last_seen_required_patch:
                    self.disqualify_car(track_index)
                break

        for track_index, state in enumerate(self.time_trial_states):
            if track_index not in current_required_patches:
                state.active_required_patches = set()

        return current_required_patches, sorted(outside_finish)

    def set_status(self, lines: list[str]) -> None:
        if not self.running:
            return
        yview = self.status_text.yview()
        was_at_bottom = bool(yview) and yview[1] >= 0.999
        self.status_text.configure(state="normal")
        self.status_text.delete("1.0", tk.END)
        self.status_text.insert("1.0", "\n".join(lines))
        self.status_text.configure(state="disabled")
        if was_at_bottom:
            self.status_text.see(tk.END)
        elif yview:
            self.status_text.yview_moveto(yview[0])

    def schedule_next_frame(self) -> None:
        return

    def start_preview_worker(self) -> None:
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind(("127.0.0.1", 0))
        server_socket.listen(1)
        server_socket.settimeout(PREVIEW_CONNECT_TIMEOUT_S)
        host, port = server_socket.getsockname()
        process = subprocess.Popen(
            [
                sys.executable,
                str(Path(__file__).resolve()),
                "--preview-worker",
                "--preview-host",
                str(host),
                "--preview-port",
                str(port),
                "--window-name",
                self.args.window_name,
            ],
            cwd=str(Path(__file__).resolve().parent),
        )
        try:
            client_socket, _ = server_socket.accept()
        except socket.timeout:
            process.kill()
            server_socket.close()
            raise RuntimeError("Preview worker could not connect.")
        client_socket.settimeout(PREVIEW_SEND_TIMEOUT_S)
        self.preview_worker = PreviewWorkerState(
            process=process,
            server_socket=server_socket,
            client_socket=client_socket,
            endpoint=(host, port),
        )

    def send_preview_frame(self, frame: np.ndarray) -> None:
        connection = self.preview_worker.client_socket
        if connection is None:
            return
        success, encoded_frame = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not success:
            return
        payload = encoded_frame.tobytes()
        packet = struct.pack("!I", len(payload)) + payload
        try:
            connection.sendall(packet)
        except (BrokenPipeError, ConnectionResetError, socket.timeout, OSError):
            self.preview_worker.client_socket = None

    def preview_worker_closed(self) -> bool:
        process = self.preview_worker.process
        if process is not None and process.poll() is not None:
            return True
        return self.preview_worker.process is not None and self.preview_worker.client_socket is None

    def stop_preview_worker(self) -> None:
        connection = self.preview_worker.client_socket
        if connection is not None:
            try:
                connection.sendall(struct.pack("!I", 0))
            except OSError:
                pass
            try:
                connection.close()
            except OSError:
                pass
        server_socket = self.preview_worker.server_socket
        if server_socket is not None:
            try:
                server_socket.close()
            except OSError:
                pass
        process = self.preview_worker.process
        if process is not None:
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
        self.preview_worker = PreviewWorkerState()

    def serial_status_lines(self) -> list[str]:
        if not self.serial_state.enabled:
            return ["Serial: disabled"]
        state = (
            f"Serial: connected to {self.serial_state.port} @ {self.serial_state.baud}"
            if self.serial_state.is_connected
            else f"Serial: disconnected from {self.serial_state.port}"
        )
        if self.serial_state.last_error:
            return [state, f"Serial error: {self.serial_state.last_error}"]
        return [state]

    def update_patch_states(
        self,
        car_positions: list[tuple[int, tuple[float, float]]],
        timestamp_s: float,
    ) -> None:
        if len(self.patch_states) != len(self.loaded_patches):
            self.patch_states = [PatchOverlayState() for _ in self.loaded_patches]

        scale_x = self.patch_source_width / max(float(self.actual_width), 1.0)
        scale_y = self.patch_source_height / max(float(self.actual_height), 1.0)

        for patch, patch_state in zip(self.loaded_patches, self.patch_states):
            current_occupants: set[int] = set()
            for track_index, car_position in car_positions:
                patch_point = (
                    car_position[0] * scale_x,
                    car_position[1] * scale_y,
                )
                if point_in_patch(patch_point, patch):
                    current_occupants.add(track_index)

            entering = current_occupants - patch_state.occupants
            leaving = patch_state.occupants - current_occupants

            if entering:
                patch_state.flash_color_bgr = GREEN_BGR
                patch_state.flash_until_s = timestamp_s + PATCH_FLASH_DURATION_S
            elif leaving:
                patch_state.flash_color_bgr = RED_BGR
                patch_state.flash_until_s = timestamp_s + PATCH_FLASH_DURATION_S
            elif timestamp_s > patch_state.flash_until_s:
                patch_state.flash_color_bgr = None

            patch_state.occupants = current_occupants

    def detect_blob_pairs(
        self,
        processing_frame: np.ndarray,
    ) -> tuple[list[Blob], list[Blob], list[tuple[Blob, Blob] | None]]:
        hsv_frame = preprocess_frame(processing_frame, self.args.blur_kernel)
        ignore_patches = [patch for patch in self.loaded_patches if patch_name_contains(patch.name, "ignore")]
        ignore_mask = build_patch_mask(
            ignore_patches,
            processing_frame.shape[1],
            processing_frame.shape[0],
            self.patch_source_width,
            self.patch_source_height,
        )
        scaled_min_area = self.args.min_area * self.processing_scale * self.processing_scale
        scaled_max_area = self.args.max_area * self.processing_scale * self.processing_scale
        scaled_max_pair_distance = self.args.max_pair_distance * self.processing_scale
        red_mask = make_color_mask(hsv_frame, "red", self.args.morph_kernel)
        blue_mask = make_color_mask(hsv_frame, "blue", self.args.morph_kernel)
        if np.any(ignore_mask):
            red_mask = cv2.bitwise_and(red_mask, cv2.bitwise_not(ignore_mask))
            blue_mask = cv2.bitwise_and(blue_mask, cv2.bitwise_not(ignore_mask))
        red_blobs = extract_blobs(
            red_mask,
            scaled_min_area,
            scaled_max_area,
        )
        blue_blobs = extract_blobs(
            blue_mask,
            scaled_min_area,
            scaled_max_area,
        )
        previous_positions = [
            None
            if track.previous_position is None
            else (
                track.previous_position[0] * self.processing_scale,
                track.previous_position[1] * self.processing_scale,
            )
            for track in self.tracks
        ]
        chosen_pairs = choose_blob_pairs(
            red_blobs,
            blue_blobs,
            scaled_max_pair_distance,
            previous_positions,
            TRACK_COUNT,
        )
        self.last_search_mode = (
            f"Full frame ({self.processing_width}x{self.processing_height}), "
            f"ignore patches: {len(ignore_patches)}"
        )
        scaled_pairs = [
            None
            if pair is None
            else (
                scale_blobs([pair[0]], self.processing_scale)[0],
                scale_blobs([pair[1]], self.processing_scale)[0],
            )
            for pair in chosen_pairs
        ]
        return scale_blobs(red_blobs, self.processing_scale), scale_blobs(blue_blobs, self.processing_scale), scaled_pairs

    def update_frame(self) -> None:
        if not self.running:
            return
        try:
            if not self.root.winfo_exists():
                self.close()
                return
        except tk.TclError:
            self.close()
            return

        frame, frame_timestamp_s = read_frame(self.capture)
        if frame is None or frame_timestamp_s is None:
            self.set_status(["Camera frame read failed."])
            self.close()
            return
        if self.race_running and self.race_started_at_s is not None:
            self.timer_var.set(self.format_stopwatch(frame_timestamp_s - self.race_started_at_s))
        elif not self.race_running:
            self.timer_var.set("00:00.000")
        fps = 0.0
        if self.last_frame_time_s is not None:
            fps = 1.0 / max(frame_timestamp_s - self.last_frame_time_s, 1e-6)
        self.last_frame_time_s = frame_timestamp_s
        self.frame_counter += 1
        draw_history_this_frame = (self.frame_counter % 2) == 0

        annotated = frame.copy()
        if self.processing_scale < 1.0:
            processing_frame = cv2.resize(
                frame,
                (self.processing_width, self.processing_height),
                interpolation=cv2.INTER_LINEAR,
            )
        else:
            processing_frame = frame
        red_blobs, blue_blobs, chosen_pairs = self.detect_blob_pairs(processing_frame)
        detected_car_positions: list[tuple[int, tuple[float, float]]] = []

        detected_tracks = 0
        status_lines = [
            f"Detected pairs: {sum(pair is not None for pair in chosen_pairs)}/{TRACK_COUNT}",
            f"Framerate: {fps:.1f} fps",
            f"Search region: {self.last_search_mode}",
            f"Detected red blobs: {len(red_blobs)}",
            f"Detected blue blobs: {len(blue_blobs)}",
            (
                f"Patches loaded: {len(self.loaded_patches)} from {Path(self.patch_path).name}"
                if self.loaded_patches and self.patch_path
                else "Patches loaded: 0"
            ),
            f"Patch coordinate map: {self.patch_source_width}x{self.patch_source_height} -> {frame.shape[1]}x{frame.shape[0]}",
            (
                f"Patch overlay: {'shown' if self.show_patches_var.get() else 'hidden'}"
                if self.loaded_patches
                else "Patch overlay: unavailable"
            ),
            f"Blob overlay: {'shown' if self.show_blob_overlay_var.get() else 'hidden'}",
            *self.serial_status_lines(),
            "",
        ]
        if self.patch_load_error:
            status_lines.extend([f"Patch load error: {self.patch_load_error}", ""])

        for track_index, track in enumerate(self.tracks):
            chosen_pair = chosen_pairs[track_index]
            label = f"Car {track_index + 1}"
            label_color = TRACK_LABEL_COLORS[track_index % len(TRACK_LABEL_COLORS)]
            if chosen_pair is None:
                track.detected = False
                status_lines.append(f"{label}: waiting for red+blue pair")
                continue

            detected_tracks += 1
            track.detected = True
            red_blob, blue_blob = chosen_pair
            if self.show_blob_overlay_var.get():
                draw_blob_marker(annotated, red_blob, RED_BGR)
                draw_blob_marker(annotated, blue_blob, BLUE_BGR)
            car_position = midpoint(red_blob.centroid, blue_blob.centroid)
            orientation_deg = angle_deg_from_points(red_blob.centroid, blue_blob.centroid)
            if track.motion_reference_position is None or track.motion_reference_timestamp_s is None:
                speed_px_s = 0.0
                travel_deg = track.last_pose.travel_deg if track.last_pose is not None else orientation_deg
            else:
                motion_dt_s = max(frame_timestamp_s - track.motion_reference_timestamp_s, 1e-6)
                dx = car_position[0] - track.motion_reference_position[0]
                dy = car_position[1] - track.motion_reference_position[1]
                measured_speed_px_s = math.hypot(dx, dy) / motion_dt_s
                if measured_speed_px_s >= self.args.stop_speed_threshold:
                    speed_px_s = measured_speed_px_s
                    travel_deg = math.degrees(math.atan2(dy, dx))
                    track.motion_reference_position = car_position
                    track.motion_reference_timestamp_s = frame_timestamp_s
                else:
                    speed_px_s = 0.0
                    travel_deg = track.last_pose.travel_deg if track.last_pose is not None else orientation_deg

            pose = PoseSample(car_position, orientation_deg, travel_deg, speed_px_s, frame_timestamp_s)
            detected_car_positions.append((track_index, car_position))
            track.samples.append(pose)
            track.last_pose = pose
            track.previous_position = car_position
            track.previous_timestamp_s = frame_timestamp_s
            if track.motion_reference_position is None or track.motion_reference_timestamp_s is None:
                track.motion_reference_position = car_position
                track.motion_reference_timestamp_s = frame_timestamp_s

            self.serial_state.send_pose(
                self.args.car_id_base + track_index,
                car_position[0],
                car_position[1],
                travel_deg,
                orientation_deg,
            )

            draw_pose_overlay(
                annotated,
                track.samples,
                self.args.orientation_full_scale_deg,
                self.args.speed_reference,
                self.args.stop_speed_threshold,
                str(track_index + 1),
                label_color,
                self.show_label_var.get(),
                self.show_history_var.get(),
                draw_history_this_frame,
                self.show_travel_arrow_var.get(),
                self.show_orientation_arrow_var.get(),
            )
            status_lines.extend(
                [
                    f"{label}: position ({car_position[0]:.1f}, {car_position[1]:.1f}) px",
                    f"{label}: orientation {orientation_deg:.1f} deg",
                    f"{label}: travel {travel_deg:.1f} deg",
                    f"{label}: speed {speed_px_s:.1f} px/s",
                    f"{label}: relative heading {wrap_angle_deg(orientation_deg - travel_deg):+.1f} deg",
                    "",
                ]
            )

        self.update_tracking_loss_disqualifications()

        current_required_patches: dict[int, set[str]] = {}
        outside_finish: list[int] = []
        if self.required_patch_map:
            current_required_patches, outside_finish = self.update_time_trial_progress(
                detected_car_positions,
                frame_timestamp_s,
            )
        self.update_start_button_state(current_required_patches, outside_finish)

        if self.loaded_patches:
            self.update_patch_states(detected_car_positions, frame_timestamp_s)
            status_lines.extend(
                [
                    f"Patch occupancy: {sum(1 for state in self.patch_states if state.occupants)}/{len(self.patch_states)} active",
                    "",
                ]
            )

        status_lines[0:0] = [
            (
                f"Race: running ({self.format_stopwatch(frame_timestamp_s - self.race_started_at_s)})"
                if self.race_running and self.race_started_at_s is not None
                else "Race: ready"
            ),
            f"Stopwatch: {self.timer_var.get()}",
            f"Target laps: {self.current_run_laps()}",
            (
                "Required patches: Finish, third1, third2 loaded"
                if self.required_patch_map
                else "Required patches: unavailable"
            ),
            (
                "Start eligibility: all detected cars inside Finish"
                if not outside_finish
                else "Start eligibility: blocked, detected car outside Finish"
            ),
            "",
        ]

        if self.race_running:
            for track_index, state in enumerate(self.time_trial_states, start=1):
                race_state = "finished" if state.finished else "DQ" if state.disqualified else state.expected_patch
                status_lines.extend(
                    [
                        (
                            f"Race Car {track_index}: lap {state.current_lap}/{self.current_run_laps()}, "
                            f"third {state.current_third}, next {race_state}"
                        ),
                    ]
                )
            status_lines.append("")

        if self.show_patches_var.get() and self.loaded_patches:
            draw_patch_overlay(
                annotated,
                self.loaded_patches,
                self.patch_states,
                frame_timestamp_s,
                self.patch_source_width,
                self.patch_source_height,
            )

        if detected_tracks == 0:
            status_lines.insert(0, "Waiting for two red+blue blob pairs.")
        else:
            status_lines.insert(0, "Tracking active.")

        try:
            self.set_status(status_lines)
        except tk.TclError:
            self.close()
            return

        try:
            preview_width, preview_height = get_preview_size(self.args.window_name)
            preview = fit_frame_to_canvas(annotated, preview_width, preview_height)
            self.send_preview_frame(preview)
        except cv2.error:
            return
        if self.preview_worker_closed():
            self.close()

    def close(self) -> None:
        if not self.running:
            return
        self.running = False
        self.serial_state.disconnect()
        self.capture.release()
        self.stop_preview_worker()
        try:
            self.lap_board.destroy()
        except tk.TclError:
            pass
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def run(self) -> int:
        frame_interval_s = FRAME_INTERVAL_MS / 1000.0
        while self.running:
            loop_started_at = time.perf_counter()
            try:
                self.root.update_idletasks()
                self.root.update()
            except tk.TclError:
                self.close()
                break
            self.update_frame()
            remaining_s = frame_interval_s - (time.perf_counter() - loop_started_at)
            if remaining_s > 0.0:
                time.sleep(remaining_s)
        return 0


def main() -> int:
    args = build_parser().parse_args()
    if args.preview_worker:
        return run_preview_worker(args.preview_host, args.preview_port, args.window_name)
    validate_args(args)
    splash = SplashScreen(DEFAULT_FAVICON_PNG)
    splash_start_time = time.perf_counter()
    try:
        splash.set_progress(10.0)
        if args.browse_calibration:
            selected_calibration = choose_calibration_file(
                args.camera_calibration or str(DEFAULT_EXTERNAL_CALIBRATION_CSV)
            )
            if selected_calibration is None:
                raise SystemExit("Calibration file selection canceled.")
            args.camera_calibration = selected_calibration
            save_default_calibration_path(selected_calibration)

        splash.set_progress(28.0)
        resolved_calibration_path, resolved_settings, calibration_source_label = resolve_calibration_source(
            args.camera_calibration
        )
        args.camera_calibration = resolved_calibration_path

        splash.set_progress(48.0)
        video_source = parse_camera_source(args.camera)
        capture, backend_name = open_video_capture(video_source, args.backend, args.width, args.height)

        splash.set_progress(68.0)
        apply_calibration_settings(
            capture,
            resolved_settings,
            calibration_source_label,
        )
        actual_width = int(round(capture.get(cv2.CAP_PROP_FRAME_WIDTH)))
        actual_height = int(round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))

        print(
            f"Video source: {args.camera} (backend={backend_name}, "
            f"requested={args.width}x{args.height}, actual={actual_width}x{actual_height})"
        )

        splash.set_progress(88.0)
        app = TrackerApp(capture, args, backend_name, actual_width, actual_height)
        app.calibration_source_label = calibration_source_label
        app.calibration_label.configure(text=app.format_calibration_label())

        splash.set_progress(100.0)
        remaining_splash_time = MIN_SPLASH_DURATION_S - (time.perf_counter() - splash_start_time)
        if remaining_splash_time > 0.0:
            time.sleep(remaining_splash_time)
        splash.close()
        return app.run()
    finally:
        splash.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        sys.exit(0)
