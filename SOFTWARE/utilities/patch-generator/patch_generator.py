from __future__ import annotations

import ctypes
import csv
import colorsys
import importlib.util
import math
import random
import socket
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
import zlib

import numpy as np
import tkinter as tk
import tkinter.font as tkfont
from tkinter import filedialog, messagebox, ttk

try:
    from PIL import Image, ImageTk
except ModuleNotFoundError as exc:
    root = tk.Tk()
    root.withdraw()
    messagebox.showerror(
        "Missing Dependency",
        "This script requires Pillow.\n\nInstall it with:\n\npip install pillow",
    )
    raise SystemExit(1) from exc

try:
    import cv2
except ModuleNotFoundError as exc:
    root = tk.Tk()
    root.withdraw()
    messagebox.showerror(
        "Missing Dependency",
        "This script requires OpenCV.\n\nInstall it with:\n\npip install opencv-python",
    )
    raise SystemExit(1) from exc


APP_TITLE = "Webcam Patch Tracker"
WINDOWS_APP_ID = "org.janelia.patch_generator"
SCRIPT_DIR = Path(__file__).resolve().parent
ASSETS_DIR = SCRIPT_DIR / "assets"
FAVICON_PATH = ASSETS_DIR / "patch_generator_favicon.png"
PATCH_TO_HEADER_PATH = SCRIPT_DIR.parent / "patch-to-header" / "patch-to-header.py"
PREVIEW_RED_BGR = (0, 0, 255)
PREVIEW_RED_OUTLINE = "#FF0000"
PREVIEW_RED_FILL_ALPHA = 0.30
SAVED_FILL_ALPHA = 0.30
CROSSHAIR_SIZE = 8
CROSSHAIR_THICKNESS = 1
DISPLAY_POLL_MS = 15
CAMERA_FRAME_WIDTH = 1280
CAMERA_FRAME_HEIGHT = 720
CAMERA_FPS = 60
DEFAULT_CALIBRATION_CSV = (
    SCRIPT_DIR.parent.parent / "firmware" / "experiments" / "tracker" / "camera_calibration.csv"
)
PATCH_FUNCTIONS = ("finish", "third", "corner", "point", "DQ", "ignore")
PATCH_FUNCTION_LIMITS = {"finish": 1, "third": 2}
SELECTED_OUTLINE_BGR = (0, 255, 255)
SELECTED_FILL_ALPHA = 0.18
EDIT_HANDLE_RADIUS = 7
EDIT_HIT_RADIUS = 12
EDGE_HIT_DISTANCE = 10
SHIFT_MASK = 0x0001
SELECTED_VERTEX_BGR = (255, 255, 255)
HOVER_HANDLE_RADIUS_DELTA = 3
UI_BG = "#232529"
UI_PANEL_BG = "#2C2F34"
UI_PANEL_ALT_BG = "#353940"
UI_BORDER = "#4A5059"
UI_TEXT = "#ECEFF4"
UI_MUTED_TEXT = "#B4BCC7"
UI_ACCENT = "#8FA3B8"
UI_ACCENT_ACTIVE = "#A7B7C7"
UI_BUTTON_DISABLED = "#23262B"
UI_BUTTON_DISABLED_TEXT = "#727B86"
UI_CANVAS_BG = "#1B1D21"
INSTANCE_HOST = "127.0.0.1"
INSTANCE_PORT = 49152 + (zlib.crc32(str(SCRIPT_DIR).encode("utf-8")) % 12000)
INSTANCE_SHUTDOWN_MESSAGE = b"shutdown"
INSTANCE_STARTUP_TIMEOUT_S = 5.0
PATCH_TO_HEADER_MODULE_NAME = "patch_to_header_exporter"
_PATCH_TO_HEADER_MODULE = None


@dataclass
class PatchRecord:
    name: str
    kind: str
    geometry: tuple[float, ...]
    color_rgb: tuple[int, int, int]
    table_tag: str
    wall_adjacent: bool = False

    @property
    def outline_hex(self) -> str:
        r, g, b = self.color_rgb
        return f"#{r:02X}{g:02X}{b:02X}"

    @property
    def text_hex(self) -> str:
        return ideal_text_color(self.color_rgb)

    @property
    def color_bgr(self) -> tuple[int, int, int]:
        r, g, b = self.color_rgb
        return (b, g, r)


@dataclass
class EditDragState:
    patch_tag: str
    drag_mode: str
    start_point: tuple[int, int]
    original_geometry: tuple[float, ...]
    handle_index: int | None = None
    original_circle_handle_angle: float | None = None


@dataclass(frozen=True)
class CameraCalibrationSetting:
    property_name: str
    property_id: int
    value: float


class InvalidPatchDialog(tk.Toplevel):
    def __init__(self, parent: tk.Misc, patch_name: str, reason: str) -> None:
        super().__init__(parent)
        self.result = "edit"
        self.title("Invalid Patch")
        self.resizable(False, False)
        self.transient(parent)
        self.grab_set()
        self.configure(bg=UI_BG)

        self.columnconfigure(0, weight=1)

        message = (
            f'"{patch_name}" is currently invalid:\n{reason}\n\n'
            "Choose Edit to keep working on it, or Cancel to discard changes made while it was selected."
        )
        ttk.Label(self, text=message, justify="left", wraplength=340).grid(
            row=0, column=0, padx=16, pady=(16, 10), sticky="w"
        )

        button_row = ttk.Frame(self)
        button_row.grid(row=1, column=0, padx=16, pady=(0, 16), sticky="e")
        ttk.Button(button_row, text="Cancel", command=self.choose_cancel, style="Compact.TButton").pack(
            side=tk.LEFT, padx=(0, 8)
        )
        ttk.Button(button_row, text="Edit", command=self.choose_edit, style="Compact.TButton").pack(side=tk.LEFT)

        self.protocol("WM_DELETE_WINDOW", self.choose_edit)
        self.bind("<Escape>", lambda _event: self.choose_edit())
        self.bind("<Return>", lambda _event: self.choose_edit())

        self.update_idletasks()
        self._center_on_parent(parent)

    def _center_on_parent(self, parent: tk.Misc) -> None:
        parent.update_idletasks()
        parent_x = parent.winfo_rootx()
        parent_y = parent.winfo_rooty()
        parent_w = parent.winfo_width()
        parent_h = parent.winfo_height()
        width = self.winfo_width()
        height = self.winfo_height()
        x_pos = parent_x + max((parent_w - width) // 2, 0)
        y_pos = parent_y + max((parent_h - height) // 2, 0)
        self.geometry(f"+{x_pos}+{y_pos}")

    def choose_edit(self) -> None:
        self.result = "edit"
        self.destroy()

    def choose_cancel(self) -> None:
        self.result = "cancel"
        self.destroy()


class PatchFunctionPrompt(tk.Toplevel):
    def __init__(self, parent: tk.Misc, title: str, prompt: str, options: list[tuple[str, str]]) -> None:
        super().__init__(parent)
        self.result: str | None = None
        self._option_map = {label: generated_name for label, generated_name in options}
        self.title(title)
        self.resizable(False, False)
        self.transient(parent)
        self.grab_set()
        self.configure(bg=UI_BG)

        self.columnconfigure(0, weight=1)

        ttk.Label(self, text=prompt, justify="left", wraplength=340).grid(
            row=0, column=0, padx=16, pady=(16, 8), sticky="w"
        )

        self.selection_var = tk.StringVar(value=options[0][0])
        self.preview_var = tk.StringVar(value=f"Patch name: {options[0][1]}")

        combo = ttk.Combobox(
            self,
            textvariable=self.selection_var,
            values=[label for label, _generated_name in options],
            state="readonly",
            width=28,
        )
        combo.grid(row=1, column=0, padx=16, pady=(0, 8), sticky="ew")
        combo.focus_set()
        combo.bind("<<ComboboxSelected>>", self.update_preview)

        ttk.Label(self, textvariable=self.preview_var, justify="left").grid(
            row=2, column=0, padx=16, pady=(0, 12), sticky="w"
        )

        button_row = ttk.Frame(self)
        button_row.grid(row=3, column=0, padx=16, pady=(0, 16), sticky="e")
        ttk.Button(button_row, text="Cancel", command=self.cancel, style="Compact.TButton").pack(
            side=tk.LEFT, padx=(0, 8)
        )
        ttk.Button(button_row, text="Save", command=self.save, style="Compact.TButton").pack(side=tk.LEFT)

        self.bind("<Return>", lambda _event: self.save())
        self.bind("<Escape>", lambda _event: self.cancel())
        self.protocol("WM_DELETE_WINDOW", self.cancel)

        self.update_idletasks()
        self._center_on_parent(parent)

    def _center_on_parent(self, parent: tk.Misc) -> None:
        parent.update_idletasks()
        parent_x = parent.winfo_rootx()
        parent_y = parent.winfo_rooty()
        parent_w = parent.winfo_width()
        parent_h = parent.winfo_height()
        width = self.winfo_width()
        height = self.winfo_height()
        x_pos = parent_x + max((parent_w - width) // 2, 0)
        y_pos = parent_y + max((parent_h - height) // 2, 0)
        self.geometry(f"+{x_pos}+{y_pos}")

    def update_preview(self, _event: tk.Event | None = None) -> None:
        selected_label = self.selection_var.get()
        generated_name = self._option_map.get(selected_label, "")
        self.preview_var.set(f"Patch name: {generated_name}")

    def cancel(self) -> None:
        self.result = None
        self.destroy()

    def save(self) -> None:
        selected_label = self.selection_var.get()
        generated_name = self._option_map.get(selected_label)
        if not generated_name:
            messagebox.showwarning("Function Required", "Please choose a patch function.", parent=self)
            return
        self.result = generated_name
        self.destroy()


class WebcamPatchApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title(APP_TITLE)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.bind("<Return>", self.on_return_key)
        self.root.bind("<Escape>", self.on_escape_key)
        self.root.bind_all("<Delete>", self.on_delete_key)
        self.root.bind_all("<BackSpace>", self.on_delete_key)

        self.current_csv_path: Path | None = None
        self.patches: list[PatchRecord] = []
        self.current_mode: str | None = None
        self.temp_points: list[tuple[int, int]] = []
        self.circle_creation_drag_active = False
        self.mouse_position: tuple[int, int] | None = None
        self.frame_size: tuple[int, int] | None = None
        self.display_region: tuple[int, int, int, int] | None = None
        self.video_photo: ImageTk.PhotoImage | None = None
        self.window_icon: ImageTk.PhotoImage | None = None
        self.video_item: int | None = None
        self.waiting_text_item: int | None = None
        self.tag_counter = 0
        self.next_hue = random.random()
        self.target_canvas_size = (960, 720)
        self.state_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.capture: cv2.VideoCapture | None = None
        self.render_thread: threading.Thread | None = None
        self.latest_display_rgb: np.ndarray | None = None
        self.latest_display_region: tuple[int, int, int, int] | None = None
        self.latest_frame_size: tuple[int, int] | None = None
        self.latest_render_token = 0
        self.last_shown_render_token = -1
        self.video_ready = False
        self.selected_patch_tag: str | None = None
        self.selected_patch_original_geometry: tuple[float, ...] | None = None
        self.selected_patch_original_wall_adjacent: bool | None = None
        self.selected_patch_original_circle_handle_angle: float | None = None
        self.selected_patch_has_unsaved_changes = False
        self.edit_drag_state: EditDragState | None = None
        self.selected_vertex_index: int | None = None
        self.hovered_handle_key: tuple[str, str, int | None] | None = None
        self.circle_handle_angles: dict[str, float] = {}
        self.ignore_table_select_event = False
        self.button_style_name = "Compact.TButton"
        self.instance_listener_socket: socket.socket | None = None
        self.instance_listener_thread: threading.Thread | None = None
        self.camera_calibration_path: Path | None = DEFAULT_CALIBRATION_CSV if DEFAULT_CALIBRATION_CSV.exists() else None

        self.status_var = tk.StringVar(value="Waiting for video feed...")
        self.file_var = tk.StringVar(value="CSV: none")
        self.calibration_var = tk.StringVar(value=self.format_calibration_label())

        self._configure_window_icon()
        self._build_ui()
        self._configure_window_chrome()
        self.render_thread = threading.Thread(target=self.render_loop, name="camera-render", daemon=True)
        self.render_thread.start()
        self._schedule_display_refresh()

    def _configure_window_icon(self) -> None:
        if not FAVICON_PATH.exists():
            return

        try:
            self.window_icon = ImageTk.PhotoImage(file=str(FAVICON_PATH))
            self.root.iconphoto(True, self.window_icon)
        except (tk.TclError, OSError):
            self.window_icon = None

    def _configure_window_chrome(self) -> None:
        if sys.platform != "win32":
            return

        try:
            self.root.update_idletasks()
            hwnd = self.root.winfo_id()
            dark_mode = ctypes.c_int(1)
            dwmapi = ctypes.windll.dwmapi
            for attribute in (20, 19):
                result = dwmapi.DwmSetWindowAttribute(
                    hwnd,
                    attribute,
                    ctypes.byref(dark_mode),
                    ctypes.sizeof(dark_mode),
                )
                if result == 0:
                    break
        except Exception:
            return

    def start_instance_listener(self, listener_socket: socket.socket) -> None:
        self.instance_listener_socket = listener_socket
        self.instance_listener_thread = threading.Thread(
            target=self.instance_listener_loop,
            name="instance-listener",
            daemon=True,
        )
        self.instance_listener_thread.start()

    def instance_listener_loop(self) -> None:
        listener_socket = self.instance_listener_socket
        if listener_socket is None:
            return

        while not self.stop_event.is_set():
            try:
                connection, _address = listener_socket.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            with connection:
                try:
                    message = connection.recv(64)
                except OSError:
                    continue
                if message.strip() == INSTANCE_SHUTDOWN_MESSAGE:
                    self.root.after(0, self.on_close)
                    break

    def format_calibration_label(self) -> str:
        calibration_name = self.camera_calibration_path.name if self.camera_calibration_path is not None else "(none)"
        return f"Calibration: {calibration_name}"

    def _build_ui(self) -> None:
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        self.root.configure(bg=UI_BG)
        style.configure(".", background=UI_BG, foreground=UI_TEXT)
        style.configure("TFrame", background=UI_BG)
        style.configure("TLabel", background=UI_BG, foreground=UI_TEXT)
        style.configure("TLabelframe", background=UI_BG, foreground=UI_TEXT, borderwidth=1, relief="solid")
        style.configure("TLabelframe.Label", background=UI_BG, foreground=UI_TEXT)
        self._configure_button_style(style)
        style.configure(
            "TEntry",
            fieldbackground=UI_PANEL_ALT_BG,
            foreground=UI_TEXT,
            insertcolor=UI_TEXT,
            bordercolor=UI_BORDER,
            lightcolor=UI_BORDER,
            darkcolor=UI_BORDER,
        )
        style.map("TEntry", fieldbackground=[("focus", UI_PANEL_ALT_BG)])
        style.configure(
            "Vertical.TScrollbar",
            background=UI_PANEL_ALT_BG,
            troughcolor=UI_BG,
            bordercolor=UI_BORDER,
            arrowcolor=UI_TEXT,
            lightcolor=UI_PANEL_ALT_BG,
            darkcolor=UI_PANEL_ALT_BG,
        )

        self.canvas_bg = UI_CANVAS_BG
        tree_bg = UI_PANEL_BG
        tree_fg = UI_TEXT
        self.table_style_name = "Patch.Treeview"
        style.configure(self.table_style_name, background=tree_bg, fieldbackground=tree_bg, foreground=tree_fg)
        style.map(
            self.table_style_name,
            background=[("selected", tree_bg)],
            foreground=[("selected", tree_fg)],
        )
        style.configure(
            "Treeview.Heading",
            background=UI_PANEL_ALT_BG,
            foreground=UI_TEXT,
            bordercolor=UI_BORDER,
            lightcolor=UI_PANEL_ALT_BG,
            darkcolor=UI_PANEL_ALT_BG,
        )
        style.map(
            "Treeview.Heading",
            background=[("active", UI_ACCENT_ACTIVE), ("pressed", UI_ACCENT)],
            foreground=[("active", UI_BG), ("pressed", UI_BG)],
        )
        base_font = tkfont.nametofont("TkDefaultFont")
        self.table_row_font = base_font.copy()
        self.table_selected_font = base_font.copy()
        self.table_selected_font.configure(weight="bold", slant="italic", underline=1)

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        main = ttk.Frame(self.root, padding=12)
        main.grid(row=0, column=0, sticky="nsew")
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=0)
        main.rowconfigure(0, weight=1)

        video_frame = ttk.Frame(main)
        video_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        video_frame.columnconfigure(0, weight=1)
        video_frame.rowconfigure(0, weight=1)

        self.canvas = tk.Canvas(video_frame, width=960, height=720, bg=self.canvas_bg, highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.canvas.bind("<ButtonPress-1>", self.on_canvas_press)
        self.canvas.bind("<B1-Motion>", self.on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_canvas_release)
        self.canvas.bind("<Motion>", self.on_canvas_motion)
        self.canvas.bind("<Leave>", self.on_canvas_leave)
        self.canvas.bind("<Configure>", self.on_canvas_configure)
        self.waiting_text_item = self.canvas.create_text(
            480,
            360,
            text="Waiting for video feed",
            fill=UI_MUTED_TEXT,
            font=("Arial", 28, "bold"),
        )

        side = ttk.Frame(main, width=280)
        side.grid(row=0, column=1, sticky="ns")
        side.columnconfigure(0, weight=1)
        side.rowconfigure(5, weight=1)

        controls_label = ttk.Label(side, text="Controls", font=("", 12, "bold"))
        controls_label.grid(row=0, column=0, sticky="w", pady=(0, 10))

        patch_button_row = ttk.Frame(side)
        patch_button_row.grid(row=1, column=0, sticky="ew", pady=2)
        patch_button_row.columnconfigure(0, weight=1)
        patch_button_row.columnconfigure(1, weight=1)
        patch_button_row.columnconfigure(2, weight=1)

        self.quad_button = ttk.Button(
            patch_button_row,
            text="Quadpatch",
            command=self.start_quadpatch,
            state=tk.DISABLED,
            style=self.button_style_name,
        )
        self.quad_button.grid(row=0, column=0, sticky="ew", padx=(0, 4))

        self.circle_button = ttk.Button(
            patch_button_row,
            text="Circlepatch",
            command=self.start_circlepatch,
            state=tk.DISABLED,
            style=self.button_style_name,
        )
        self.circle_button.grid(row=0, column=1, sticky="ew", padx=2)

        self.poly_button = ttk.Button(
            patch_button_row,
            text="Polypatch",
            command=self.start_polypatch,
            state=tk.DISABLED,
            style=self.button_style_name,
        )
        self.poly_button.grid(row=0, column=2, sticky="ew", padx=(4, 0))

        file_button_row = ttk.Frame(side)
        file_button_row.grid(row=2, column=0, sticky="ew", pady=2)
        file_button_row.columnconfigure(0, weight=1)
        file_button_row.columnconfigure(1, weight=1)

        self.new_button = ttk.Button(
            file_button_row,
            text="New",
            command=self.create_new_csv,
            state=tk.DISABLED,
            style=self.button_style_name,
        )
        self.new_button.grid(row=0, column=0, sticky="ew", padx=(0, 4))

        self.clear_button = ttk.Button(
            file_button_row,
            text="Clear",
            command=self.clear_all,
            state=tk.DISABLED,
            style=self.button_style_name,
        )
        self.clear_button.grid(row=0, column=1, sticky="ew", padx=(4, 0))

        load_button_row = ttk.Frame(side)
        load_button_row.grid(row=3, column=0, sticky="ew", pady=2)
        load_button_row.columnconfigure(0, weight=1)
        load_button_row.columnconfigure(1, weight=1)
        load_button_row.columnconfigure(2, weight=1)

        self.load_button = ttk.Button(
            load_button_row,
            text="Load",
            command=self.load_csv,
            state=tk.DISABLED,
            style=self.button_style_name,
        )
        self.load_button.grid(row=0, column=0, sticky="ew", padx=(0, 4))

        self.reload_button = ttk.Button(
            load_button_row,
            text="Reload",
            command=self.reload_csv,
            state=tk.DISABLED,
            style=self.button_style_name,
        )
        self.reload_button.grid(row=0, column=1, sticky="ew", padx=2)

        self.export_button = ttk.Button(
            load_button_row,
            text="Export",
            command=self.export_header,
            state=tk.DISABLED,
            style=self.button_style_name,
        )
        self.export_button.grid(row=0, column=2, sticky="ew", padx=(2, 0))

        calibration_button_row = ttk.Frame(side)
        calibration_button_row.grid(row=4, column=0, sticky="ew", pady=(2, 0))
        calibration_button_row.columnconfigure(0, weight=1)

        self.calibration_button = ttk.Button(
            calibration_button_row,
            text="Calibration",
            command=self.choose_calibration_csv,
            state=tk.DISABLED,
            style=self.button_style_name,
        )
        self.calibration_button.grid(row=0, column=0, sticky="ew")

        table_frame = ttk.LabelFrame(side, text="Patches", padding=8)
        table_frame.grid(row=5, column=0, sticky="nsew", pady=(12, 0))
        table_frame.columnconfigure(0, weight=1)
        table_frame.rowconfigure(0, weight=0)
        table_frame.rowconfigure(1, weight=1)

        reorder_button_row = ttk.Frame(table_frame)
        reorder_button_row.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))
        reorder_button_row.columnconfigure(0, weight=1)
        reorder_button_row.columnconfigure(1, weight=1)

        self.move_up_button = ttk.Button(
            reorder_button_row,
            text="Up",
            command=lambda: self.move_selected_patch(-1),
            state=tk.DISABLED,
            style=self.button_style_name,
        )
        self.move_up_button.grid(row=0, column=0, sticky="ew", padx=(0, 4))

        self.move_down_button = ttk.Button(
            reorder_button_row,
            text="Down",
            command=lambda: self.move_selected_patch(1),
            state=tk.DISABLED,
            style=self.button_style_name,
        )
        self.move_down_button.grid(row=0, column=1, sticky="ew", padx=(4, 0))

        self.delete_patch_button = ttk.Button(
            table_frame,
            text="Delete",
            command=self.delete_selected_patch,
            state=tk.DISABLED,
            style=self.button_style_name,
        )
        self.delete_patch_button.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(8, 0))

        self.table = ttk.Treeview(
            table_frame,
            columns=("wall", "name", "type"),
            show="headings",
            height=18,
            style=self.table_style_name,
        )
        self.table.heading("wall", text="Wall")
        self.table.heading("name", text="Name")
        self.table.heading("type", text="Type")
        self.table.column("wall", width=52, anchor="center", stretch=False)
        self.table.column("name", width=150, anchor="w")
        self.table.column("type", width=100, anchor="w")
        self.table.grid(row=1, column=0, sticky="nsew")
        self.table.bind("<<TreeviewSelect>>", self.on_table_select)
        self.table.bind("<ButtonRelease-1>", self.on_table_click)
        self.table.tag_configure("selected_patch_row", font=self.table_selected_font)

        scrollbar = ttk.Scrollbar(table_frame, orient=tk.VERTICAL, command=self.table.yview)
        scrollbar.grid(row=1, column=1, sticky="ns")
        self.table.configure(yscrollcommand=scrollbar.set)

        calibration_label = ttk.Label(side, textvariable=self.calibration_var, wraplength=250)
        calibration_label.grid(row=6, column=0, sticky="ew", pady=(12, 4))
        file_label = ttk.Label(side, textvariable=self.file_var, wraplength=250)
        file_label.grid(row=7, column=0, sticky="ew", pady=(4, 4))
        status_label = ttk.Label(side, textvariable=self.status_var, wraplength=250, justify="left")
        status_label.grid(row=8, column=0, sticky="ew")

        for widget in (
            side,
            controls_label,
            patch_button_row,
            file_button_row,
            load_button_row,
            calibration_button_row,
            table_frame,
            calibration_label,
            file_label,
            status_label,
        ):
            widget.bind("<ButtonPress-1>", self.on_sidebar_background_click, add="+")

    def _configure_button_style(self, style: ttk.Style) -> None:
        style.configure(
            self.button_style_name,
            foreground=UI_TEXT,
            background=UI_PANEL_ALT_BG,
            padding=(6, 2),
            borderwidth=1,
            focusthickness=0,
            focuscolor=UI_ACCENT,
            anchor="center",
            relief="flat",
            bordercolor=UI_BORDER,
            lightcolor=UI_PANEL_ALT_BG,
            darkcolor=UI_PANEL_ALT_BG,
        )
        style.map(
            self.button_style_name,
            background=[
                ("disabled", UI_BUTTON_DISABLED),
                ("pressed", UI_ACCENT),
                ("active", UI_ACCENT_ACTIVE),
            ],
            foreground=[("disabled", UI_BUTTON_DISABLED_TEXT), ("active", UI_BG), ("pressed", UI_BG)],
            bordercolor=[
                ("disabled", UI_BORDER),
                ("pressed", UI_ACCENT),
                ("active", UI_ACCENT_ACTIVE),
            ],
        )

    def load_camera_calibration(self, csv_path: Path) -> list[CameraCalibrationSetting]:
        if not csv_path.exists():
            return []

        loaded_settings: list[CameraCalibrationSetting] = []
        with csv_path.open("r", newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                property_name = row.get("property_name", "").strip()
                if not property_name:
                    continue
                loaded_settings.append(
                    CameraCalibrationSetting(
                        property_name=property_name,
                        property_id=int(row["property_id"]),
                        value=float(row["value"]),
                    )
                )
        return loaded_settings

    def apply_camera_calibration(self, capture: cv2.VideoCapture, csv_path: Path) -> list[str]:
        settings = self.load_camera_calibration(csv_path)
        if not settings:
            return []

        results: list[str] = []
        for setting in settings:
            applied = capture.set(setting.property_id, setting.value)
            actual_value = capture.get(setting.property_id)
            status = "ok" if applied else "ignored"
            results.append(
                f"{setting.property_name}: requested {setting.value:.3f}, camera reports {actual_value:.3f} ({status})"
            )
        return results

    def choose_calibration_csv(self) -> None:
        initial_path = self.camera_calibration_path or DEFAULT_CALIBRATION_CSV
        selected = filedialog.askopenfilename(
            parent=self.root,
            title="Select camera calibration CSV",
            initialdir=str(initial_path.parent) if initial_path is not None else str(SCRIPT_DIR),
            initialfile=initial_path.name if initial_path is not None else "",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not selected:
            return

        path = Path(selected)
        try:
            with self.state_lock:
                capture = self.capture
            results = [] if capture is None else self.apply_camera_calibration(capture, path)
        except (OSError, ValueError, KeyError) as exc:
            messagebox.showerror(
                "Calibration Error",
                f"Could not load the camera calibration CSV.\n\n{exc}",
                parent=self.root,
            )
            return

        self.camera_calibration_path = path
        self.calibration_var.set(self.format_calibration_label())

        if capture is None:
            self.status_var.set(f"Calibration CSV selected: {path.name}. It will be applied when the camera opens.")
            return

        if results:
            self.status_var.set(f"Applied calibration from {path.name}.")
        else:
            self.status_var.set(f"Calibration CSV selected: {path.name}. No settings were found to apply.")

    def _open_camera(self) -> cv2.VideoCapture | None:
        capture: cv2.VideoCapture | None = None
        if hasattr(cv2, "CAP_DSHOW"):
            capture = cv2.VideoCapture(0, cv2.CAP_DSHOW)
            if not capture.isOpened():
                capture.release()
                capture = None

        if capture is None:
            capture = cv2.VideoCapture(0)

        if not capture.isOpened():
            capture.release()
            return None

        if hasattr(cv2, "VideoWriter_fourcc"):
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_FRAME_WIDTH)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_FRAME_HEIGHT)
        capture.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
        if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        calibration_path = self.camera_calibration_path
        if calibration_path is not None and calibration_path.exists():
            try:
                self.apply_camera_calibration(capture, calibration_path)
            except (OSError, ValueError, KeyError) as exc:
                self.root.after(
                    0,
                    lambda: self.status_var.set(
                        f"Failed to apply calibration {calibration_path.name}: {exc}"
                    ),
                )

        return capture

    def _schedule_display_refresh(self) -> None:
        self.root.after(DISPLAY_POLL_MS, self.update_display)

    def update_display(self) -> None:
        if not self.root.winfo_exists():
            return

        render_rgb: np.ndarray | None = None
        display_region: tuple[int, int, int, int] | None = None
        frame_size: tuple[int, int] | None = None

        with self.state_lock:
            if self.latest_render_token != self.last_shown_render_token and self.latest_display_rgb is not None:
                render_rgb = self.latest_display_rgb
                display_region = self.latest_display_region
                frame_size = self.latest_frame_size
                self.last_shown_render_token = self.latest_render_token

        if render_rgb is not None and display_region is not None and frame_size is not None:
            self.show_frame(render_rgb, display_region, frame_size)
            if not self.video_ready:
                self.video_ready = True
                self.hide_waiting_overlay()
                self.update_button_states()
                if self.current_csv_path is None:
                    self.status_var.set("Create or load a CSV file to begin.")

        self._schedule_display_refresh()

    def render_loop(self) -> None:
        while not self.stop_event.is_set():
            capture = self.capture
            if capture is None:
                capture = self._open_camera()
                if capture is None:
                    time.sleep(0.5)
                    continue
                self.capture = capture

            ok, frame = capture.read()
            if not ok:
                capture.release()
                if self.capture is capture:
                    self.capture = None
                time.sleep(0.05)
                continue

            render_rgb, display_region, frame_size = self.render_frame_for_display(frame)
            with self.state_lock:
                self.latest_display_rgb = render_rgb
                self.latest_display_region = display_region
                self.latest_frame_size = frame_size
                self.latest_render_token += 1

    def render_frame_for_display(
        self, frame_bgr: np.ndarray
    ) -> tuple[np.ndarray, tuple[int, int, int, int], tuple[int, int]]:
        with self.state_lock:
            patches = list(self.patches)
            current_mode = self.current_mode
            temp_points = list(self.temp_points)
            mouse_position = self.mouse_position
            canvas_width, canvas_height = self.target_canvas_size
            selected_patch_tag = self.selected_patch_tag
            selected_vertex_index = self.selected_vertex_index
            hovered_handle_key = self.hovered_handle_key
            edit_drag_state = self.edit_drag_state

        display = frame_bgr.copy()
        height, width = display.shape[:2]

        for patch in patches:
            if patch.kind in {"quadpatch", "polypatch"}:
                points = self._geometry_to_points(patch.geometry)
                self.draw_translucent_polygon(display, points, patch.color_bgr, SAVED_FILL_ALPHA)
            elif patch.kind == "circlepatch":
                cx, cy, radius = patch.geometry
                self.draw_translucent_circle(
                    display,
                    (int(round(cx)), int(round(cy))),
                    int(round(radius)),
                    patch.color_bgr,
                    SAVED_FILL_ALPHA,
                )

        selected_patch = next((patch for patch in patches if patch.table_tag == selected_patch_tag), None)
        if selected_patch is not None:
            self.draw_selected_patch(
                display,
                selected_patch,
                selected_vertex_index,
                hovered_handle_key,
                edit_drag_state,
                mouse_position,
            )

        self.draw_preview(display, current_mode, temp_points, mouse_position)

        scale = min(canvas_width / width, canvas_height / height)
        display_width = max(1, int(round(width * scale)))
        display_height = max(1, int(round(height * scale)))
        interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
        if display_width != width or display_height != height:
            display = cv2.resize(display, (display_width, display_height), interpolation=interpolation)

        offset_x = max((canvas_width - display_width) // 2, 0)
        offset_y = max((canvas_height - display_height) // 2, 0)
        display_region = (offset_x, offset_y, display_width, display_height)
        display_rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
        return (display_rgb, display_region, (width, height))

    def draw_selected_patch(
        self,
        frame_bgr: np.ndarray,
        patch: PatchRecord,
        selected_vertex_index: int | None,
        hovered_handle_key: tuple[str, str, int | None] | None,
        edit_drag_state: EditDragState | None,
        mouse_position: tuple[int, int] | None,
    ) -> None:
        if patch.kind in {"quadpatch", "polypatch"}:
            points = self._geometry_to_points(patch.geometry)
            self.draw_translucent_polygon(
                frame_bgr,
                points,
                SELECTED_OUTLINE_BGR,
                SELECTED_FILL_ALPHA,
                outline_bgr=patch.color_bgr,
            )
            for index, point in enumerate(points):
                is_selected = patch.kind == "polypatch" and index == selected_vertex_index
                handle_key = (patch.table_tag, "vertex", index)
                is_hovered = hovered_handle_key == handle_key
                self.draw_edit_handle(
                    frame_bgr,
                    point,
                    patch.color_bgr,
                    hovered=is_hovered,
                    selected=is_selected,
                )
            return

        if patch.kind == "circlepatch":
            cx, cy, radius = patch.geometry
            center = (int(round(cx)), int(round(cy)))
            radius_int = int(round(radius))
            center_selected = bool(
                edit_drag_state is not None
                and edit_drag_state.patch_tag == patch.table_tag
                and edit_drag_state.drag_mode == "center"
            )
            radius_selected = bool(
                edit_drag_state is not None
                and edit_drag_state.patch_tag == patch.table_tag
                and edit_drag_state.drag_mode == "radius"
            )
            radius_handle = self.get_circle_radius_handle(patch)
            if radius_selected and mouse_position is not None:
                radius_handle = mouse_position
            self.draw_translucent_circle(
                frame_bgr,
                center,
                radius_int,
                SELECTED_OUTLINE_BGR,
                SELECTED_FILL_ALPHA,
                outline_bgr=patch.color_bgr,
            )
            self.draw_edit_handle(
                frame_bgr,
                center,
                patch.color_bgr,
                hovered=hovered_handle_key == (patch.table_tag, "center", None),
                selected=center_selected,
            )
            self.draw_edit_handle(
                frame_bgr,
                radius_handle,
                patch.color_bgr,
                hovered=hovered_handle_key == (patch.table_tag, "radius", None),
                selected=radius_selected,
            )
            if radius_selected:
                cv2.line(frame_bgr, center, radius_handle, patch.color_bgr, 1, cv2.LINE_AA)

    def draw_edit_handle(
        self,
        frame_bgr: np.ndarray,
        point: tuple[int, int],
        normal_bgr: tuple[int, int, int],
        hovered: bool = False,
        selected: bool = False,
    ) -> None:
        fill_color = complementary_bgr(normal_bgr) if selected else normal_bgr
        radius = EDIT_HANDLE_RADIUS + HOVER_HANDLE_RADIUS_DELTA if hovered else EDIT_HANDLE_RADIUS
        if selected:
            radius += 1
        cv2.circle(frame_bgr, point, radius, fill_color, thickness=-1, lineType=cv2.LINE_AA)
        cv2.circle(frame_bgr, point, radius + 2, (0, 0, 0), thickness=1, lineType=cv2.LINE_AA)

    def show_frame(
        self,
        frame_rgb: np.ndarray,
        display_region: tuple[int, int, int, int],
        frame_size: tuple[int, int],
    ) -> None:
        image = Image.fromarray(frame_rgb)
        self.video_photo = ImageTk.PhotoImage(image=image)
        offset_x, offset_y, display_width, display_height = display_region
        self.display_region = display_region
        self.frame_size = frame_size

        if self.video_item is None:
            self.video_item = self.canvas.create_image(offset_x, offset_y, anchor="nw", image=self.video_photo)
        else:
            self.canvas.itemconfigure(self.video_item, image=self.video_photo)
            self.canvas.coords(self.video_item, offset_x, offset_y)

        self.canvas.itemconfigure(self.video_item, state="normal")
        self.canvas.configure(scrollregion=(0, 0, max(self.canvas.winfo_width(), display_width), max(self.canvas.winfo_height(), display_height)))

    def draw_preview(
        self,
        frame_bgr: np.ndarray,
        current_mode: str | None,
        temp_points: list[tuple[int, int]],
        mouse_position: tuple[int, int] | None,
    ) -> None:
        if current_mode == "quadpatch":
            self.draw_quadpatch_preview(frame_bgr, temp_points, mouse_position)
        elif current_mode == "circlepatch":
            if not temp_points:
                return
            center = temp_points[0]
            self.draw_crosshair(frame_bgr, center, PREVIEW_RED_BGR)
            if mouse_position is None:
                return
            self.draw_crosshair(frame_bgr, mouse_position, PREVIEW_RED_BGR)
            radius = int(round(distance_between(center, mouse_position)))
            self.draw_translucent_circle(
                frame_bgr,
                center,
                radius,
                PREVIEW_RED_BGR,
                PREVIEW_RED_FILL_ALPHA,
                outline_bgr=PREVIEW_RED_BGR,
            )
        elif current_mode == "polypatch":
            self.draw_polypatch_preview(frame_bgr, temp_points, mouse_position)

    def draw_quadpatch_preview(
        self,
        frame_bgr: np.ndarray,
        temp_points: list[tuple[int, int]],
        mouse_position: tuple[int, int] | None,
    ) -> None:
        if not temp_points:
            return

        for point in temp_points:
            self.draw_crosshair(frame_bgr, point, PREVIEW_RED_BGR)

        if mouse_position is None:
            return

        self.draw_crosshair(frame_bgr, mouse_position, PREVIEW_RED_BGR)

        if len(temp_points) == 1:
            cv2.line(frame_bgr, temp_points[0], mouse_position, PREVIEW_RED_BGR, 2, cv2.LINE_AA)
            return

        if len(temp_points) == 2:
            preview_points = rectangle_from_three_points(temp_points[0], temp_points[1], mouse_position)
            if preview_points is None:
                cv2.line(frame_bgr, temp_points[0], temp_points[1], PREVIEW_RED_BGR, 2, cv2.LINE_AA)
                cv2.line(frame_bgr, temp_points[1], mouse_position, PREVIEW_RED_BGR, 1, cv2.LINE_AA)
                return

            self.draw_translucent_polygon(
                frame_bgr,
                preview_points,
                PREVIEW_RED_BGR,
                PREVIEW_RED_FILL_ALPHA,
                outline_bgr=PREVIEW_RED_BGR,
            )

    def draw_polypatch_preview(
        self,
        frame_bgr: np.ndarray,
        temp_points: list[tuple[int, int]],
        mouse_position: tuple[int, int] | None,
    ) -> None:
        if not temp_points:
            return

        for point in temp_points:
            self.draw_crosshair(frame_bgr, point, PREVIEW_RED_BGR)

        if mouse_position is None:
            return

        if len(temp_points) == 1:
            cv2.line(frame_bgr, temp_points[0], mouse_position, PREVIEW_RED_BGR, 2, cv2.LINE_AA)
            return

        preview_points = temp_points + [mouse_position]
        self.draw_translucent_polygon(
            frame_bgr,
            preview_points,
            PREVIEW_RED_BGR,
            PREVIEW_RED_FILL_ALPHA,
            outline_bgr=PREVIEW_RED_BGR,
        )

    def on_canvas_configure(self, event: tk.Event) -> None:
        with self.state_lock:
            self.target_canvas_size = (max(int(event.width), 1), max(int(event.height), 1))
        if self.waiting_text_item is not None:
            self.canvas.coords(self.waiting_text_item, event.width // 2, event.height // 2)

    def on_canvas_motion(self, event: tk.Event) -> None:
        point = self.clamp_point((event.x, event.y))
        hovered_handle_key: tuple[str, str, int | None] | None = None
        with self.state_lock:
            self.mouse_position = point
            current_mode = self.current_mode
            edit_drag_state = self.edit_drag_state
        if point is not None and current_mode is None and edit_drag_state is None:
            handle = self.find_edit_handle_at_point(point)
            if handle is not None:
                patch, drag_mode, handle_index = handle
                hovered_handle_key = (patch.table_tag, drag_mode, handle_index)
        with self.state_lock:
            self.hovered_handle_key = hovered_handle_key

    def on_canvas_leave(self, _event: tk.Event) -> None:
        with self.state_lock:
            self.mouse_position = None
            self.hovered_handle_key = None

    def on_canvas_press(self, event: tk.Event) -> None:
        self.canvas.focus_set()
        point = self.clamp_point((event.x, event.y))
        with self.state_lock:
            current_mode = self.current_mode
        if current_mode is not None:
            if point is not None:
                if current_mode == "circlepatch":
                    with self.state_lock:
                        point_count = len(self.temp_points)
                    if point_count == 0:
                        self.handle_creation_click(current_mode, point)
                    elif point_count == 1:
                        with self.state_lock:
                            self.mouse_position = point
                            self.circle_creation_drag_active = True
                    return
                self.handle_creation_click(current_mode, point)
            return

        if point is None:
            self.select_patch(None)
            return

        handle = self.find_edit_handle_at_point(point)
        if handle is not None:
            patch, drag_mode, handle_index = handle
            if not self.select_patch(patch.table_tag):
                return
            with self.state_lock:
                self.selected_vertex_index = handle_index if patch.kind == "polypatch" else None
            self.edit_drag_state = EditDragState(
                patch_tag=patch.table_tag,
                drag_mode=drag_mode,
                start_point=point,
                original_geometry=patch.geometry,
                handle_index=handle_index,
                original_circle_handle_angle=self.circle_handle_angles.get(patch.table_tag),
            )
            return

        if self.try_insert_polypatch_vertex(event, point):
            return

        patch = self.find_patch_at_point(point)
        if patch is None:
            self.select_patch(None)
            return

        if not self.select_patch(patch.table_tag):
            return
        with self.state_lock:
            self.selected_vertex_index = None
        self.edit_drag_state = EditDragState(
            patch_tag=patch.table_tag,
            drag_mode="move",
            start_point=point,
            original_geometry=patch.geometry,
        )

    def handle_creation_click(self, current_mode: str, point: tuple[int, int]) -> None:
        if current_mode == "quadpatch":
            with self.state_lock:
                self.temp_points.append(point)
                point_count = len(self.temp_points)
            if point_count == 1:
                self.status_var.set("Quadpatch mode: click a second point to set the side length and orientation.")
            elif point_count == 2:
                self.status_var.set("Quadpatch mode: click a third point to set the rectangle width.")
            elif point_count == 3:
                self.finish_quadpatch()
        elif current_mode == "circlepatch":
            with self.state_lock:
                if self.temp_points:
                    return
                self.temp_points.append(point)
            self.status_var.set("Circlepatch mode: drag to size the circle, then release to save.")
        elif current_mode == "polypatch":
            with self.state_lock:
                self.temp_points.append(point)
                point_count = len(self.temp_points)
            if point_count < 3:
                self.status_var.set(
                    f"Polypatch mode: {point_count} point(s) selected. Click another point."
                )
            else:
                self.status_var.set(
                    f"Polypatch mode: {point_count} point(s) selected. Press Enter to finish."
                )

    def on_canvas_drag(self, event: tk.Event) -> None:
        point = self.clamp_point((event.x, event.y))
        if point is None:
            return

        with self.state_lock:
            self.mouse_position = point
            current_mode = self.current_mode
            circle_creation_drag_active = self.circle_creation_drag_active
        if current_mode == "circlepatch" and circle_creation_drag_active:
            return
        drag_state = self.edit_drag_state
        if drag_state is None:
            return

        patch = self.get_patch_by_tag(drag_state.patch_tag)
        if patch is None:
            self.edit_drag_state = None
            return

        candidate_geometry = self.get_drag_geometry_candidate(patch.kind, drag_state, point)
        with self.state_lock:
            patch.geometry = candidate_geometry
            if patch.kind == "circlepatch" and drag_state.drag_mode == "radius":
                self.circle_handle_angles[patch.table_tag] = self.calculate_circle_handle_angle(
                    (candidate_geometry[0], candidate_geometry[1]),
                    point,
                )

    def on_canvas_release(self, event: tk.Event) -> None:
        point = self.clamp_point((event.x, event.y))
        with self.state_lock:
            current_mode = self.current_mode
            circle_creation_drag_active = self.circle_creation_drag_active
            temp_points = list(self.temp_points)
        if current_mode == "circlepatch" and circle_creation_drag_active:
            with self.state_lock:
                self.circle_creation_drag_active = False
            if point is None or len(temp_points) != 1:
                return
            center = temp_points[0]
            if distance_between(center, point) <= 0:
                self.status_var.set("Circlepatch mode: drag farther from the center to set the radius.")
                return
            with self.state_lock:
                self.temp_points = [center, point]
            self.finish_circlepatch()
            return

        drag_state = self.edit_drag_state
        self.edit_drag_state = None
        if drag_state is None:
            return

        patch = self.get_patch_by_tag(drag_state.patch_tag)
        if patch is None:
            return

        if point is None:
            with self.state_lock:
                current_geometry = patch.geometry
            if current_geometry != drag_state.original_geometry:
                self.mark_selected_patch_dirty(patch.table_tag)
            return

        candidate_geometry = self.get_drag_geometry_candidate(patch.kind, drag_state, point)
        with self.state_lock:
            patch.geometry = candidate_geometry
            if patch.kind == "circlepatch" and drag_state.drag_mode == "radius":
                self.circle_handle_angles[patch.table_tag] = self.calculate_circle_handle_angle(
                    (candidate_geometry[0], candidate_geometry[1]),
                    point,
                )
        if candidate_geometry != drag_state.original_geometry:
            self.mark_selected_patch_dirty(patch.table_tag)

    def on_return_key(self, _event: tk.Event) -> None:
        with self.state_lock:
            current_mode = self.current_mode
            point_count = len(self.temp_points)

        if current_mode != "polypatch":
            return

        if point_count < 3:
            messagebox.showwarning(
                "Incomplete Polypatch",
                "A polypatch needs at least 3 points before it can be saved.",
                parent=self.root,
            )
            return

        self.finish_polypatch()

    def on_escape_key(self, _event: tk.Event) -> None:
        with self.state_lock:
            current_mode = self.current_mode

        if current_mode is not None:
            self.cancel_current_mode(current_mode)
            return

        drag_state = self.edit_drag_state
        if drag_state is None:
            return

        patch = self.get_patch_by_tag(drag_state.patch_tag)
        if patch is not None:
            with self.state_lock:
                patch.geometry = drag_state.original_geometry
                if patch.kind == "circlepatch" and drag_state.original_circle_handle_angle is not None:
                    self.circle_handle_angles[patch.table_tag] = drag_state.original_circle_handle_angle
        self.edit_drag_state = None
        self.status_var.set("Edit cancelled.")

    def on_delete_key(self, _event: tk.Event) -> None:
        focus_widget = self.root.focus_get()
        if focus_widget is not None and focus_widget.winfo_class() in {"Entry", "TEntry", "Text", "Spinbox", "TCombobox"}:
            return

        with self.state_lock:
            current_mode = self.current_mode
            selected_patch_tag = self.selected_patch_tag
            selected_vertex_index = self.selected_vertex_index

        if current_mode is not None or selected_patch_tag is None or selected_vertex_index is None:
            return

        patch = self.get_patch_by_tag(selected_patch_tag)
        if patch is None or patch.kind != "polypatch":
            return

        points = self._geometry_to_points(patch.geometry)
        if len(points) <= 3:
            messagebox.showwarning(
                "Cannot Delete Vertex",
                "A polypatch must keep at least 3 points.",
                parent=self.root,
            )
            return

        candidate_points = points[:selected_vertex_index] + points[selected_vertex_index + 1 :]
        candidate_geometry = tuple(float(value) for vertex in candidate_points for value in vertex)
        with self.state_lock:
            patch.geometry = candidate_geometry
            self.selected_vertex_index = min(selected_vertex_index, len(candidate_points) - 1)
        self.mark_selected_patch_dirty(patch.table_tag)
        self.status_var.set(f'Removed a vertex from "{patch.name}".')

    def clamp_point(self, point: tuple[int, int]) -> tuple[int, int] | None:
        if self.frame_size is None or self.display_region is None:
            return None

        frame_width, frame_height = self.frame_size
        offset_x, offset_y, display_width, display_height = self.display_region
        if not (
            offset_x <= point[0] < offset_x + display_width
            and offset_y <= point[1] < offset_y + display_height
        ):
            return None

        relative_x = (point[0] - offset_x) / display_width
        relative_y = (point[1] - offset_y) / display_height
        x_pos = max(0, min(int(relative_x * frame_width), frame_width - 1))
        y_pos = max(0, min(int(relative_y * frame_height), frame_height - 1))
        return (x_pos, y_pos)

    def start_quadpatch(self) -> None:
        if self.current_csv_path is None:
            return
        with self.state_lock:
            self.current_mode = "quadpatch"
            self.temp_points = []
            self.mouse_position = None
        self.status_var.set(
            "Quadpatch mode: click two points to define a side, then a third point to define the width."
        )

    def finish_quadpatch(self) -> None:
        with self.state_lock:
            raw_points = list(self.temp_points)
            self.current_mode = None
            self.mouse_position = None

        rectangle_points = None
        if len(raw_points) == 3:
            rectangle_points = rectangle_from_three_points(raw_points[0], raw_points[1], raw_points[2])

        if rectangle_points is None:
            messagebox.showerror(
                "Invalid Quadpatch",
                "The selected quadpatch could not define a valid rectangle. The quadpatch was cancelled.",
                parent=self.root,
            )
            with self.state_lock:
                self.temp_points = []
            self.status_var.set("Quadpatch cancelled.")
            return

        name = self.prompt_for_patch_name("Save Quadpatch", "Choose a function for this quadpatch:")
        if name is None:
            with self.state_lock:
                self.temp_points = []
            self.status_var.set("Quadpatch cancelled.")
            return

        with self.state_lock:
            self.temp_points = []
        geometry = tuple(float(value) for point in rectangle_points for value in point)
        self.save_patch_record(name=name, kind="quadpatch", geometry=geometry)
        self.status_var.set(f'Saved quadpatch "{name}".')

    def start_circlepatch(self) -> None:
        if self.current_csv_path is None:
            return
        with self.state_lock:
            self.current_mode = "circlepatch"
            self.temp_points = []
            self.circle_creation_drag_active = False
            self.mouse_position = None
        self.status_var.set("Circlepatch mode: click the center, then drag to set the radius.")

    def finish_circlepatch(self) -> None:
        with self.state_lock:
            center, edge = self.temp_points
            self.current_mode = None
            self.mouse_position = None
        radius = distance_between(center, edge)
        if radius <= 0:
            with self.state_lock:
                self.temp_points = []
            self.status_var.set("Circlepatch cancelled.")
            return

        name = self.prompt_for_patch_name("Save Circlepatch", "Choose a function for this circlepatch:")
        if name is None:
            with self.state_lock:
                self.temp_points = []
            self.status_var.set("Circlepatch cancelled.")
            return

        with self.state_lock:
            self.temp_points = []
        geometry = (float(center[0]), float(center[1]), float(radius))
        self.save_patch_record(
            name=name,
            kind="circlepatch",
            geometry=geometry,
            circle_handle_angle=self.calculate_circle_handle_angle(center, edge),
        )
        self.status_var.set(f'Saved circlepatch "{name}".')

    def start_polypatch(self) -> None:
        if self.current_csv_path is None:
            return
        with self.state_lock:
            self.current_mode = "polypatch"
            self.temp_points = []
            self.mouse_position = None
        self.status_var.set("Polypatch mode: click points around the polygon. Press Enter to finish.")

    def finish_polypatch(self) -> None:
        with self.state_lock:
            raw_points = list(self.temp_points)
            self.current_mode = None
            self.mouse_position = None

        invalid_reason = simple_polygon_invalid_reason(raw_points, minimum_points=3)
        if invalid_reason is not None:
            messagebox.showerror(
                "Invalid Polypatch",
                f"The selected polypatch is invalid because {invalid_reason} The polypatch was cancelled.",
                parent=self.root,
            )
            with self.state_lock:
                self.temp_points = []
            self.status_var.set("Polypatch cancelled.")
            return

        name = self.prompt_for_patch_name("Save Polypatch", "Choose a function for this polypatch:")
        if name is None:
            with self.state_lock:
                self.temp_points = []
            self.status_var.set("Polypatch cancelled.")
            return

        with self.state_lock:
            self.temp_points = []
        geometry = tuple(float(value) for point in raw_points for value in point)
        self.save_patch_record(name=name, kind="polypatch", geometry=geometry)
        self.status_var.set(f'Saved polypatch "{name}".')

    def cancel_current_mode(self, current_mode: str) -> None:
        with self.state_lock:
            self.current_mode = None
            self.temp_points = []
            self.circle_creation_drag_active = False
            self.mouse_position = None

        if current_mode == "quadpatch":
            self.status_var.set("Quadpatch cancelled.")
        elif current_mode == "circlepatch":
            self.status_var.set("Circlepatch cancelled.")
        elif current_mode == "polypatch":
            self.status_var.set("Polypatch cancelled.")

    def get_patch_by_tag(self, table_tag: str) -> PatchRecord | None:
        with self.state_lock:
            return next((patch for patch in self.patches if patch.table_tag == table_tag), None)

    def select_patch(self, table_tag: str | None) -> None:
        with self.state_lock:
            current_tag = self.selected_patch_tag

        if table_tag == current_tag:
            self.apply_table_selection(table_tag)
            self.refresh_table_selection_styles()
            return True

        if not self.finalize_selected_patch_on_deselect():
            self.apply_table_selection(current_tag)
            self.refresh_table_selection_styles()
            return False

        self.start_selected_patch_session(table_tag)
        self.apply_table_selection(table_tag)
        self.refresh_table_selection_styles()
        return True

    def on_table_select(self, _event: tk.Event) -> None:
        if self.ignore_table_select_event:
            return

        selection = self.table.selection()
        new_patch_tag = selection[0] if selection else None
        self.select_patch(new_patch_tag)
        self.update_reorder_button_states()

    def on_sidebar_background_click(self, event: tk.Event) -> None:
        with self.state_lock:
            current_mode = self.current_mode
        if current_mode is not None:
            return
        if event.widget is self.table:
            return
        self.select_patch(None)

    def refresh_table_selection_styles(self) -> None:
        selection = set(self.table.selection())
        for item_id in self.table.get_children():
            base_tags = [item_id]
            if item_id in selection:
                base_tags.append("selected_patch_row")
            self.table.item(item_id, tags=tuple(base_tags))

    def update_reorder_button_states(self) -> None:
        if not getattr(self, "video_ready", False):
            self.move_up_button.config(state=tk.DISABLED)
            self.move_down_button.config(state=tk.DISABLED)
            self.delete_patch_button.config(state=tk.DISABLED)
            return

        selected_tag = None
        children = list(self.table.get_children())
        selection = self.table.selection()
        if selection:
            selected_tag = selection[0]

        if selected_tag is None or selected_tag not in children:
            self.move_up_button.config(state=tk.DISABLED)
            self.move_down_button.config(state=tk.DISABLED)
            self.delete_patch_button.config(state=tk.DISABLED)
            return

        selected_index = children.index(selected_tag)
        self.move_up_button.config(state=tk.NORMAL if selected_index > 0 else tk.DISABLED)
        self.move_down_button.config(state=tk.NORMAL if selected_index < len(children) - 1 else tk.DISABLED)
        self.delete_patch_button.config(state=tk.NORMAL)

    def apply_table_selection(self, table_tag: str | None) -> None:
        self.ignore_table_select_event = True
        try:
            if table_tag is None:
                self.table.selection_remove(self.table.selection())
                self.update_reorder_button_states()
                return
            if self.table.exists(table_tag):
                self.table.selection_set((table_tag,))
                self.table.focus(table_tag)
                self.table.see(table_tag)
            self.update_reorder_button_states()
        finally:
            self.root.after_idle(self.clear_ignore_table_select_event)

    def clear_ignore_table_select_event(self) -> None:
        self.ignore_table_select_event = False

    def start_selected_patch_session(self, table_tag: str | None) -> None:
        patch_geometry: tuple[float, ...] | None = None
        wall_adjacent: bool | None = None
        circle_handle_angle: float | None = None
        if table_tag is not None:
            patch = self.get_patch_by_tag(table_tag)
            if patch is not None:
                patch_geometry = tuple(patch.geometry)
                wall_adjacent = patch.wall_adjacent
                circle_handle_angle = self.circle_handle_angles.get(table_tag)

        with self.state_lock:
            self.selected_patch_tag = table_tag
            self.selected_patch_original_geometry = patch_geometry
            self.selected_patch_original_wall_adjacent = wall_adjacent
            self.selected_patch_original_circle_handle_angle = circle_handle_angle
            self.selected_patch_has_unsaved_changes = False
            self.selected_vertex_index = None
            self.hovered_handle_key = None

    def mark_selected_patch_dirty(self, table_tag: str | None) -> None:
        with self.state_lock:
            if self.selected_patch_tag == table_tag:
                self.selected_patch_has_unsaved_changes = True

    def prompt_invalid_patch_choice(self, patch_name: str, reason: str) -> str:
        dialog = InvalidPatchDialog(self.root, patch_name, reason)
        self.root.wait_window(dialog)
        return dialog.result

    def finalize_selected_patch_on_deselect(self) -> bool:
        with self.state_lock:
            selected_tag = self.selected_patch_tag
            original_geometry = self.selected_patch_original_geometry
            original_wall_adjacent = self.selected_patch_original_wall_adjacent
            original_circle_handle_angle = self.selected_patch_original_circle_handle_angle
            has_unsaved_changes = self.selected_patch_has_unsaved_changes

        if selected_tag is None or original_geometry is None or not has_unsaved_changes:
            return True

        patch = self.get_patch_by_tag(selected_tag)
        if patch is None:
            return True

        invalid_reason = self.invalid_geometry_reason(patch.kind, patch.geometry)
        if invalid_reason is None:
            if not self.write_all_patches_to_csv():
                return False
            self.status_var.set(f'Saved changes to "{patch.name}".')
            return True

        choice = self.prompt_invalid_patch_choice(patch.name, invalid_reason)
        if choice == "edit":
            self.status_var.set(f'Continue editing "{patch.name}" before deselecting it.')
            return False

        with self.state_lock:
            patch.geometry = original_geometry
            if original_wall_adjacent is not None:
                patch.wall_adjacent = original_wall_adjacent
            if patch.kind == "circlepatch":
                self.circle_handle_angles[patch.table_tag] = (
                    0.0 if original_circle_handle_angle is None else original_circle_handle_angle
                )
            self.selected_patch_has_unsaved_changes = False
        self.update_table_row(patch)
        self.status_var.set(f'Discarded unsaved changes to "{patch.name}".')
        return True

    def finalize_selected_patch_for_close(self) -> None:
        with self.state_lock:
            selected_tag = self.selected_patch_tag
            original_geometry = self.selected_patch_original_geometry
            original_wall_adjacent = self.selected_patch_original_wall_adjacent
            original_circle_handle_angle = self.selected_patch_original_circle_handle_angle
            has_unsaved_changes = self.selected_patch_has_unsaved_changes

        if selected_tag is None:
            return

        patch = self.get_patch_by_tag(selected_tag)
        if patch is None:
            return

        if has_unsaved_changes and original_geometry is not None:
            invalid_reason = self.invalid_geometry_reason(patch.kind, patch.geometry)
            if invalid_reason is None:
                try:
                    with self.state_lock:
                        rows = [self.serialize_record(record) for record in self.patches]
                    if self.current_csv_path is not None:
                        with self.current_csv_path.open("w", newline="", encoding="utf-8") as handle:
                            writer = csv.writer(handle)
                            writer.writerows(rows)
                except OSError:
                    pass
            else:
                with self.state_lock:
                    patch.geometry = original_geometry
                    if original_wall_adjacent is not None:
                        patch.wall_adjacent = original_wall_adjacent
                    if patch.kind == "circlepatch":
                        self.circle_handle_angles[patch.table_tag] = (
                            0.0 if original_circle_handle_angle is None else original_circle_handle_angle
                        )

        with self.state_lock:
            self.selected_patch_tag = None
            self.selected_patch_original_geometry = None
            self.selected_patch_original_wall_adjacent = None
            self.selected_patch_original_circle_handle_angle = None
            self.selected_patch_has_unsaved_changes = False
            self.selected_vertex_index = None
            self.hovered_handle_key = None
        self.apply_table_selection(None)
        self.refresh_table_selection_styles()

    def on_table_click(self, event: tk.Event) -> None:
        region = self.table.identify_region(event.x, event.y)
        row_id = self.table.identify_row(event.y)
        if not row_id:
            if region != "heading":
                self.select_patch(None)
            return

        if region != "cell":
            return

        column_id = self.table.identify_column(event.x)
        if column_id != "#1":
            return

        if not self.select_patch(row_id):
            return

        patch = self.get_patch_by_tag(row_id)
        if patch is None:
            return

        patch.wall_adjacent = not patch.wall_adjacent
        self.update_table_row(patch)
        self.mark_selected_patch_dirty(patch.table_tag)
        if self.selected_patch_tag != patch.table_tag:
            if not self.write_all_patches_to_csv():
                patch.wall_adjacent = not patch.wall_adjacent
                self.update_table_row(patch)
                return
        state_text = "enabled" if patch.wall_adjacent else "cleared"
        self.status_var.set(f'Wall-adjacent flag {state_text} for "{patch.name}".')

    def move_selected_patch(self, direction: int) -> None:
        if direction not in (-1, 1):
            raise ValueError("direction must be -1 or 1")

        selected_items = self.table.selection()
        if not selected_items:
            self.status_var.set("Select a patch in the table to reorder it.")
            self.update_reorder_button_states()
            return

        selected_tag = selected_items[0]
        patch = self.get_patch_by_tag(selected_tag)
        if patch is None:
            self.update_reorder_button_states()
            return

        invalid_reason = self.invalid_geometry_reason(patch.kind, patch.geometry)
        if invalid_reason is not None:
            self.status_var.set(f'Finish fixing "{patch.name}" before reordering it.')
            return

        with self.state_lock:
            current_index = next(
                (index for index, existing_patch in enumerate(self.patches) if existing_patch.table_tag == selected_tag),
                None,
            )
            if current_index is None:
                return
            target_index = current_index + direction
            if target_index < 0 or target_index >= len(self.patches):
                self.update_reorder_button_states()
                return
            moved_patch = self.patches.pop(current_index)
            self.patches.insert(target_index, moved_patch)

        self.table.move(selected_tag, "", target_index)
        self.apply_table_selection(selected_tag)
        self.refresh_table_selection_styles()

        if not self.write_all_patches_to_csv():
            with self.state_lock:
                reverted_index = next(
                    (index for index, existing_patch in enumerate(self.patches) if existing_patch.table_tag == selected_tag),
                    None,
                )
                if reverted_index is not None:
                    reverted_patch = self.patches.pop(reverted_index)
                    self.patches.insert(current_index, reverted_patch)
            self.table.move(selected_tag, "", current_index)
            self.apply_table_selection(selected_tag)
            self.refresh_table_selection_styles()
            return

        direction_text = "up" if direction < 0 else "down"
        self.status_var.set(f'Moved "{patch.name}" {direction_text}.')

    def find_patch_at_point(self, point: tuple[int, int]) -> PatchRecord | None:
        with self.state_lock:
            patches = list(self.patches)

        for patch in reversed(patches):
            if self.patch_contains_point(patch, point):
                return patch
        return None

    def patch_contains_point(self, patch: PatchRecord, point: tuple[int, int]) -> bool:
        if patch.kind == "circlepatch":
            cx, cy, radius = patch.geometry
            return distance_between((int(round(cx)), int(round(cy))), point) <= radius

        points = np.array(self._geometry_to_points(patch.geometry), dtype=np.int32)
        return cv2.pointPolygonTest(points, point, False) >= 0

    def find_edit_handle_at_point(
        self, point: tuple[int, int]
    ) -> tuple[PatchRecord, str, int | None] | None:
        if self.selected_patch_tag is None:
            return None

        patch = self.get_patch_by_tag(self.selected_patch_tag)
        if patch is None:
            return None

        if patch.kind == "circlepatch":
            center = (int(round(patch.geometry[0])), int(round(patch.geometry[1])))
            radius_handle = self.get_circle_radius_handle(patch)
            if distance_between(center, point) <= EDIT_HIT_RADIUS:
                return (patch, "center", None)
            if distance_between(radius_handle, point) <= EDIT_HIT_RADIUS:
                return (patch, "radius", None)
            return None

        for index, vertex in enumerate(self._geometry_to_points(patch.geometry)):
            if distance_between(vertex, point) <= EDIT_HIT_RADIUS:
                return (patch, "vertex", index)
        return None

    def get_circle_radius_handle(self, patch: PatchRecord) -> tuple[int, int]:
        cx, cy, radius = patch.geometry
        angle = self.circle_handle_angles.get(patch.table_tag, 0.0)
        return (
            int(round(cx + (radius * math.cos(angle)))),
            int(round(cy + (radius * math.sin(angle)))),
        )

    def calculate_circle_handle_angle(
        self, center: tuple[float, float] | tuple[int, int], point: tuple[int, int]
    ) -> float:
        return math.atan2(point[1] - center[1], point[0] - center[0])

    def try_insert_polypatch_vertex(self, event: tk.Event, point: tuple[int, int]) -> bool:
        if not event.state & SHIFT_MASK:
            return False

        with self.state_lock:
            selected_patch_tag = self.selected_patch_tag

        if selected_patch_tag is None:
            return False

        patch = self.get_patch_by_tag(selected_patch_tag)
        if patch is None or patch.kind != "polypatch":
            return False

        insert_index = self.find_polypatch_edge_insert_index(patch, point)
        if insert_index is None:
            return False

        points = self._geometry_to_points(patch.geometry)
        candidate_points = points[:insert_index] + [point] + points[insert_index:]
        candidate_geometry = tuple(float(value) for vertex in candidate_points for value in vertex)
        with self.state_lock:
            patch.geometry = candidate_geometry
            self.selected_vertex_index = insert_index
        self.mark_selected_patch_dirty(patch.table_tag)
        self.status_var.set(f'Added a vertex to "{patch.name}".')
        return True

    def find_polypatch_edge_insert_index(self, patch: PatchRecord, point: tuple[int, int]) -> int | None:
        points = self._geometry_to_points(patch.geometry)
        best_index: int | None = None
        best_distance = float("inf")

        for index, start in enumerate(points):
            end = points[(index + 1) % len(points)]
            distance = point_to_segment_distance(point, start, end)
            if distance <= EDGE_HIT_DISTANCE and distance < best_distance:
                best_distance = distance
                best_index = index + 1

        return best_index

    def get_drag_geometry_candidate(
        self, kind: str, drag_state: EditDragState, point: tuple[int, int]
    ) -> tuple[float, ...]:
        if drag_state.drag_mode == "move":
            dx = point[0] - drag_state.start_point[0]
            dy = point[1] - drag_state.start_point[1]
            original = drag_state.original_geometry
            if kind == "circlepatch":
                return (original[0] + dx, original[1] + dy, original[2])
            moved: list[float] = []
            for index, value in enumerate(original):
                moved.append(value + (dx if index % 2 == 0 else dy))
            return tuple(moved)

        if kind == "circlepatch":
            cx, cy, radius = drag_state.original_geometry
            if drag_state.drag_mode == "center":
                return (float(point[0]), float(point[1]), radius)
            if drag_state.drag_mode == "radius":
                return (cx, cy, float(distance_between((int(round(cx)), int(round(cy))), point)))

        if drag_state.drag_mode == "vertex" and drag_state.handle_index is not None:
            points = self._geometry_to_points(drag_state.original_geometry)
            points[drag_state.handle_index] = point
            return tuple(float(value) for vertex in points for value in vertex)

        return drag_state.original_geometry

    def invalid_geometry_reason(self, kind: str, geometry: tuple[float, ...]) -> str | None:
        if kind == "circlepatch":
            if len(geometry) != 3:
                return "circlepatches need center x, center y, and radius."
            if geometry[2] <= 0:
                return "the radius must be greater than 0."
            return None

        points = self._geometry_to_points(geometry)
        if kind == "quadpatch":
            return polygon_invalid_reason(points, minimum_points=4, required_points=4)
        if kind == "polypatch":
            return simple_polygon_invalid_reason(points, minimum_points=3)
        return "unsupported patch type."

    def is_valid_geometry(self, kind: str, geometry: tuple[float, ...]) -> bool:
        return self.invalid_geometry_reason(kind, geometry) is None

    def write_all_patches_to_csv(self) -> bool:
        if self.current_csv_path is None:
            raise RuntimeError("No CSV file is active.")

        with self.state_lock:
            rows = [self.serialize_record(patch) for patch in self.patches]

        try:
            with self.current_csv_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerows(rows)
        except OSError as exc:
            messagebox.showerror(
                "Save Error",
                f"Could not write the updated patch data.\n\n{exc}",
                parent=self.root,
            )
            return False

        return True

    def managed_patch_name_parts(self, name: str) -> tuple[str, int | None] | None:
        if name == "finish":
            return ("finish", None)

        for function_name in PATCH_FUNCTIONS:
            if function_name == "finish":
                continue
            if not name.startswith(function_name):
                continue
            suffix = name[len(function_name):]
            if suffix.isdigit() and int(suffix) > 0:
                return (function_name, int(suffix))
        return None

    def next_patch_name_for_function(self, function_name: str) -> str | None:
        used_numbers: set[int] = set()
        for patch in self.patches:
            parts = self.managed_patch_name_parts(patch.name)
            if parts is None:
                continue
            existing_function, existing_number = parts
            if existing_function != function_name:
                continue
            used_numbers.add(1 if existing_number is None else existing_number)

        if function_name == "finish":
            return None if used_numbers else "finish"

        limit = PATCH_FUNCTION_LIMITS.get(function_name)
        candidate_number = 1
        while candidate_number in used_numbers:
            candidate_number += 1

        if limit is not None and candidate_number > limit:
            return None

        return f"{function_name}{candidate_number}"

    def available_patch_function_options(self) -> list[tuple[str, str]]:
        options: list[tuple[str, str]] = []
        for function_name in PATCH_FUNCTIONS:
            generated_name = self.next_patch_name_for_function(function_name)
            if generated_name is None:
                continue
            if function_name == "finish":
                label = "finish"
            else:
                label = f"{function_name} -> {generated_name}"
            options.append((label, generated_name))
        return options

    def prompt_for_patch_name(self, title: str, prompt: str) -> str | None:
        options = self.available_patch_function_options()
        if not options:
            messagebox.showwarning(
                "No Names Available",
                "No valid patch function names are currently available.",
                parent=self.root,
            )
            return None

        dialog = PatchFunctionPrompt(self.root, title, prompt, options)
        self.root.wait_window(dialog)
        return dialog.result

    def delete_selected_patch(self) -> None:
        with self.state_lock:
            current_mode = self.current_mode
            selected_tag = self.selected_patch_tag
        if current_mode is not None:
            self.status_var.set("Finish or cancel the current patch tool before deleting a patch.")
            return
        if selected_tag is None:
            self.status_var.set("Select a patch in the table to delete it.")
            self.update_reorder_button_states()
            return

        with self.state_lock:
            delete_index = next(
                (index for index, patch in enumerate(self.patches) if patch.table_tag == selected_tag),
                None,
            )
            if delete_index is None:
                return
            deleted_patch = self.patches.pop(delete_index)
            deleted_circle_angle = self.circle_handle_angles.pop(selected_tag, None)
            self.selected_patch_tag = None
            self.selected_patch_original_geometry = None
            self.selected_patch_original_wall_adjacent = None
            self.selected_patch_original_circle_handle_angle = None
            self.selected_patch_has_unsaved_changes = False
            self.selected_vertex_index = None
            self.hovered_handle_key = None
            self.edit_drag_state = None

        if self.table.exists(selected_tag):
            self.table.delete(selected_tag)
        self.apply_table_selection(None)
        self.refresh_table_selection_styles()

        if not self.write_all_patches_to_csv():
            with self.state_lock:
                self.patches.insert(delete_index, deleted_patch)
                if deleted_circle_angle is not None:
                    self.circle_handle_angles[selected_tag] = deleted_circle_angle
            self.table.tag_configure(
                deleted_patch.table_tag,
                background=deleted_patch.outline_hex,
                foreground=deleted_patch.text_hex,
            )
            self.table.insert(
                "",
                delete_index,
                iid=deleted_patch.table_tag,
                values=self.table_row_values(deleted_patch),
                tags=(deleted_patch.table_tag,),
            )
            self.start_selected_patch_session(selected_tag)
            self.apply_table_selection(selected_tag)
            self.refresh_table_selection_styles()
            return

        self.status_var.set(f'Deleted "{deleted_patch.name}".')
        self.update_reorder_button_states()

    def save_patch_record(
        self,
        name: str,
        kind: str,
        geometry: tuple[float, ...],
        circle_handle_angle: float | None = None,
    ) -> None:
        if self.current_csv_path is None:
            raise RuntimeError("No CSV file is active.")

        record = self.make_patch_record(name, kind, geometry)
        if kind == "circlepatch":
            self.circle_handle_angles[record.table_tag] = 0.0 if circle_handle_angle is None else circle_handle_angle
        try:
            with self.current_csv_path.open("a", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow(self.serialize_record(record))
        except OSError as exc:
            messagebox.showerror("Save Error", f"Could not save patch data.\n\n{exc}", parent=self.root)
            return

        self.add_patch_to_ui(record)

    def serialize_record(self, record: PatchRecord) -> list[str]:
        metadata = [f"wall_adjacent={'true' if record.wall_adjacent else 'false'}"]
        if record.kind == "quadpatch":
            return [record.kind, record.name, *metadata, *[format_number(value) for value in record.geometry]]
        if record.kind == "circlepatch":
            return [record.kind, record.name, *metadata, *[format_number(value) for value in record.geometry]]
        if record.kind == "polypatch":
            return [record.kind, record.name, *metadata, *[format_number(value) for value in record.geometry]]
        raise ValueError(f"Unsupported patch type: {record.kind}")

    def make_patch_record(
        self, name: str, kind: str, geometry: tuple[float, ...], wall_adjacent: bool = False
    ) -> PatchRecord:
        self.tag_counter += 1
        return PatchRecord(
            name=name,
            kind=kind,
            geometry=geometry,
            color_rgb=self.generate_patch_color(),
            table_tag=f"patch_{self.tag_counter}",
            wall_adjacent=wall_adjacent,
        )

    def generate_patch_color(self) -> tuple[int, int, int]:
        golden_ratio_conjugate = 0.618033988749895
        self.next_hue = (self.next_hue + golden_ratio_conjugate) % 1.0
        saturation = random.uniform(0.72, 0.95)
        value = random.uniform(0.78, 0.98)
        red, green, blue = colorsys.hsv_to_rgb(self.next_hue, saturation, value)
        return (
            int(round(red * 255)),
            int(round(green * 255)),
            int(round(blue * 255)),
        )

    def add_patch_to_ui(self, record: PatchRecord) -> None:
        with self.state_lock:
            self.patches.append(record)
            if record.kind == "circlepatch" and record.table_tag not in self.circle_handle_angles:
                self.circle_handle_angles[record.table_tag] = 0.0
        self.table.tag_configure(
            record.table_tag,
            background=record.outline_hex,
            foreground=record.text_hex,
        )
        self.table.insert(
            "",
            tk.END,
            iid=record.table_tag,
            values=self.table_row_values(record),
            tags=(record.table_tag,),
        )
        self.select_patch(record.table_tag)

    def update_table_row(self, record: PatchRecord) -> None:
        if self.table.exists(record.table_tag):
            self.table.item(record.table_tag, values=self.table_row_values(record))

    def table_row_values(self, record: PatchRecord) -> tuple[str, str, str]:
        return (checkbox_text(record.wall_adjacent), record.name, record.kind)

    def load_csv(self) -> None:
        if not self.finalize_selected_patch_on_deselect():
            return
        path_text = filedialog.askopenfilename(
            parent=self.root,
            title="Load Patch CSV",
            initialdir=str(SCRIPT_DIR),
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not path_text:
            return

        path = Path(path_text)
        self.clear_patch_data(reset_file=False)
        self.current_csv_path = path
        self.file_var.set(f"CSV: {path}")

        errors = self.load_records_from_file(path)
        if errors:
            messagebox.showwarning(
                "Load Completed With Warnings",
                "Some rows could not be loaded:\n\n" + "\n".join(errors[:10]),
                parent=self.root,
            )
        self.status_var.set(f"Loaded {len(self.patches)} patch(es) from {path.name}. Click a patch to edit it.")
        self.update_button_states()

    def load_records_from_file(self, path: Path) -> list[str]:
        errors: list[str] = []
        try:
            with path.open("r", newline="", encoding="utf-8") as handle:
                reader = csv.reader(handle)
                for line_number, row in enumerate(reader, start=1):
                    if not row or not any(cell.strip() for cell in row):
                        continue
                    try:
                        parsed = parse_csv_row(row)
                        if parsed is None:
                            continue
                        name, kind, geometry, wall_adjacent = parsed
                        record = self.make_patch_record(name, kind, geometry, wall_adjacent=wall_adjacent)
                        self.add_patch_to_ui(record)
                    except ValueError as exc:
                        errors.append(f"Line {line_number}: {exc}")
        except OSError as exc:
            messagebox.showerror("Load Error", f"Could not read the CSV file.\n\n{exc}", parent=self.root)
            self.clear_all()
        return errors

    def reload_csv(self) -> None:
        if not self.finalize_selected_patch_on_deselect():
            return
        if self.current_csv_path is None:
            return
        path = self.current_csv_path
        if not path.exists():
            self.clear_patch_data(reset_file=True)
            self.status_var.set("Create or load a CSV file to begin.")
            self.update_button_states()
            messagebox.showwarning(
                "CSV File Missing",
                f"The active CSV file no longer exists:\n\n{path}\n\nPlease create or load a CSV file.",
                parent=self.root,
            )
            return

        self.clear_patch_data(reset_file=False)
        self.current_csv_path = path
        self.file_var.set(f"CSV: {path}")
        errors = self.load_records_from_file(path)
        if errors:
            messagebox.showwarning(
                "Reload Completed With Warnings",
                "Some rows could not be loaded:\n\n" + "\n".join(errors[:10]),
                parent=self.root,
            )
        self.status_var.set(f"Reloaded {len(self.patches)} patch(es) from {path.name}. Click a patch to edit it.")
        self.update_button_states()

    def export_header(self) -> None:
        if not self.finalize_selected_patch_on_deselect():
            return
        if self.current_csv_path is None:
            messagebox.showwarning(
                "No CSV File",
                "Create or load a patch CSV before exporting a header.",
                parent=self.root,
            )
            return

        csv_path = self.current_csv_path.resolve()
        if not csv_path.exists():
            messagebox.showwarning(
                "CSV File Missing",
                f"The active CSV file no longer exists:\n\n{csv_path}\n\nPlease create or load a CSV file first.",
                parent=self.root,
            )
            return

        try:
            patch_to_header = load_patch_to_header_module()
            patches = patch_to_header.load_patches(csv_path)
            output_path = patch_to_header.resolve_default_output_path(csv_path)
            header_text = patch_to_header.build_header(
                patches=patches,
                source_name=csv_path.name,
                script_name=PATCH_TO_HEADER_PATH.name,
                array_name="patches",
                count_name="PATCH_COUNT",
            )
            output_path.write_text(header_text, encoding="utf-8", newline="\n")
        except Exception as exc:
            messagebox.showerror(
                "Export Error",
                f"Could not export the header file.\n\n{exc}",
                parent=self.root,
            )
            return

        self.status_var.set(f"Exported header: {output_path.name}")
        messagebox.showinfo(
            "Export Complete",
            f"Header exported successfully:\n\n{output_path}",
            parent=self.root,
        )

    def create_new_csv(self) -> None:
        if not self.finalize_selected_patch_on_deselect():
            return
        path_text = filedialog.asksaveasfilename(
            parent=self.root,
            title="Create Patch CSV",
            initialdir=str(SCRIPT_DIR),
            initialfile="patches.csv",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not path_text:
            return

        selected_path = Path(path_text)
        try:
            path = self.resolve_new_csv_path(selected_path)
        except OSError as exc:
            messagebox.showerror(
                "Folder Error",
                f"Could not prepare the output folder for the new CSV file.\n\n{exc}",
                parent=self.root,
            )
            return

        if path.exists():
            overwrite = messagebox.askyesno(
                "Overwrite File",
                f"{path.name} already exists.\n\nDo you want to replace it with a new empty CSV file?",
                parent=self.root,
            )
            if not overwrite:
                return

        try:
            path.write_text("", encoding="utf-8")
        except OSError as exc:
            messagebox.showerror("File Error", f"Could not create the CSV file.\n\n{exc}", parent=self.root)
            return

        self.clear_patch_data(reset_file=False)
        self.current_csv_path = path
        self.file_var.set(f"CSV: {path}")
        self.update_button_states()
        self.status_var.set(f"Created new CSV file: {path.name}. Add or edit patches in the display.")

    def resolve_new_csv_path(self, selected_path: Path) -> Path:
        target_dir = selected_path.parent
        if target_dir.name.casefold() != "output":
            target_dir = target_dir / "output"
            target_dir.mkdir(parents=True, exist_ok=True)

        return target_dir / selected_path.name

    def clear_all(self) -> None:
        if not self.finalize_selected_patch_on_deselect():
            return
        self.clear_patch_data(reset_file=True)
        self.status_var.set("Cleared patches and reset the active CSV file.")
        self.update_button_states()

    def clear_patch_data(self, reset_file: bool) -> None:
        with self.state_lock:
            self.current_mode = None
            self.temp_points = []
            self.circle_creation_drag_active = False
            self.mouse_position = None
            self.display_region = None
            self.selected_patch_tag = None
            self.selected_patch_original_geometry = None
            self.selected_patch_original_wall_adjacent = None
            self.selected_patch_original_circle_handle_angle = None
            self.selected_patch_has_unsaved_changes = False
            self.selected_vertex_index = None
            self.hovered_handle_key = None
            self.edit_drag_state = None
            self.circle_handle_angles.clear()
            self.patches.clear()
            self.latest_display_rgb = None
            self.latest_display_region = None
            self.latest_frame_size = None
        for item in self.table.get_children():
            self.table.delete(item)
        self.update_reorder_button_states()

        if reset_file:
            self.current_csv_path = None
            self.file_var.set("CSV: none")

    def update_button_states(self) -> None:
        if not self.video_ready:
            self.load_button.config(state=tk.DISABLED)
            self.clear_button.config(state=tk.DISABLED)
            self.new_button.config(state=tk.DISABLED)
            self.quad_button.config(state=tk.DISABLED)
            self.circle_button.config(state=tk.DISABLED)
            self.poly_button.config(state=tk.DISABLED)
            self.reload_button.config(state=tk.DISABLED)
            self.export_button.config(state=tk.DISABLED)
            self.calibration_button.config(state=tk.DISABLED)
            self.move_up_button.config(state=tk.DISABLED)
            self.move_down_button.config(state=tk.DISABLED)
            self.delete_patch_button.config(state=tk.DISABLED)
            return

        self.load_button.config(state=tk.NORMAL)
        self.clear_button.config(state=tk.NORMAL)
        self.new_button.config(state=tk.NORMAL)
        self.calibration_button.config(state=tk.NORMAL)

        patch_state = tk.NORMAL if self.current_csv_path is not None else tk.DISABLED
        self.quad_button.config(state=patch_state)
        self.circle_button.config(state=patch_state)
        self.poly_button.config(state=patch_state)
        self.reload_button.config(state=patch_state)
        self.export_button.config(state=patch_state)
        self.update_reorder_button_states()

    def hide_waiting_overlay(self) -> None:
        if self.waiting_text_item is not None:
            self.canvas.itemconfigure(self.waiting_text_item, state="hidden")

    def draw_crosshair(
        self,
        frame_bgr: np.ndarray,
        point: tuple[int, int],
        color_bgr: tuple[int, int, int],
        size: int = CROSSHAIR_SIZE,
        thickness: int = CROSSHAIR_THICKNESS,
    ) -> None:
        x_pos, y_pos = point
        cv2.line(frame_bgr, (x_pos - size, y_pos), (x_pos + size, y_pos), color_bgr, thickness, cv2.LINE_AA)
        cv2.line(frame_bgr, (x_pos, y_pos - size), (x_pos, y_pos + size), color_bgr, thickness, cv2.LINE_AA)

    def draw_translucent_polygon(
        self,
        frame_bgr: np.ndarray,
        points: Iterable[tuple[int, int]],
        fill_bgr: tuple[int, int, int],
        alpha: float,
        outline_bgr: tuple[int, int, int] | None = None,
    ) -> None:
        points_array = np.array(list(points), dtype=np.int32)
        overlay = frame_bgr.copy()
        cv2.fillPoly(overlay, [points_array], fill_bgr)
        cv2.addWeighted(overlay, alpha, frame_bgr, 1.0 - alpha, 0.0, dst=frame_bgr)
        cv2.polylines(
            frame_bgr,
            [points_array],
            True,
            outline_bgr or fill_bgr,
            thickness=2,
            lineType=cv2.LINE_AA,
        )

    def draw_translucent_circle(
        self,
        frame_bgr: np.ndarray,
        center: tuple[int, int],
        radius: int,
        fill_bgr: tuple[int, int, int],
        alpha: float,
        outline_bgr: tuple[int, int, int] | None = None,
    ) -> None:
        if radius <= 0:
            return
        overlay = frame_bgr.copy()
        cv2.circle(overlay, center, radius, fill_bgr, thickness=-1, lineType=cv2.LINE_AA)
        cv2.addWeighted(overlay, alpha, frame_bgr, 1.0 - alpha, 0.0, dst=frame_bgr)
        cv2.circle(
            frame_bgr,
            center,
            radius,
            outline_bgr or fill_bgr,
            thickness=2,
            lineType=cv2.LINE_AA,
        )

    def _geometry_to_points(self, geometry: tuple[float, ...]) -> list[tuple[int, int]]:
        return [
            (int(round(geometry[index])), int(round(geometry[index + 1])))
            for index in range(0, len(geometry), 2)
        ]

    def on_close(self) -> None:
        self.finalize_selected_patch_for_close()
        self.stop_event.set()
        if self.render_thread is not None and self.render_thread.is_alive():
            self.render_thread.join(timeout=1.0)
        if self.instance_listener_socket is not None:
            try:
                self.instance_listener_socket.close()
            except OSError:
                pass
            self.instance_listener_socket = None
        if self.instance_listener_thread is not None and self.instance_listener_thread.is_alive():
            self.instance_listener_thread.join(timeout=1.0)
        if hasattr(self, "capture") and self.capture is not None:
            self.capture.release()
        self.root.destroy()


def distance_between(point_a: tuple[int, int], point_b: tuple[int, int]) -> float:
    return math.hypot(point_b[0] - point_a[0], point_b[1] - point_a[1])


def point_to_segment_distance(
    point: tuple[int, int], segment_start: tuple[int, int], segment_end: tuple[int, int]
) -> float:
    px, py = point
    x1, y1 = segment_start
    x2, y2 = segment_end
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return distance_between(point, segment_start)

    projection = ((px - x1) * dx + (py - y1) * dy) / float(dx * dx + dy * dy)
    projection = max(0.0, min(1.0, projection))
    closest_x = x1 + projection * dx
    closest_y = y1 + projection * dy
    return math.hypot(px - closest_x, py - closest_y)


def complementary_bgr(color_bgr: tuple[int, int, int]) -> tuple[int, int, int]:
    blue, green, red = color_bgr
    red_norm = red / 255.0
    green_norm = green / 255.0
    blue_norm = blue / 255.0

    hue, saturation, value = colorsys.rgb_to_hsv(red_norm, green_norm, blue_norm)
    complementary_hue = (hue + 0.5) % 1.0
    comp_red, comp_green, comp_blue = colorsys.hsv_to_rgb(complementary_hue, saturation, value)
    return (
        int(round(comp_blue * 255)),
        int(round(comp_green * 255)),
        int(round(comp_red * 255)),
    )


def checkbox_text(is_checked: bool) -> str:
    return "[x]" if is_checked else "[ ]"


def load_patch_to_header_module():
    global _PATCH_TO_HEADER_MODULE

    if _PATCH_TO_HEADER_MODULE is not None:
        return _PATCH_TO_HEADER_MODULE

    if not PATCH_TO_HEADER_PATH.exists():
        raise FileNotFoundError(f"Could not find patch-to-header script at {PATCH_TO_HEADER_PATH}")

    spec = importlib.util.spec_from_file_location(PATCH_TO_HEADER_MODULE_NAME, PATCH_TO_HEADER_PATH)
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not load module spec from {PATCH_TO_HEADER_PATH}")

    module = importlib.util.module_from_spec(spec)
    sys.modules[PATCH_TO_HEADER_MODULE_NAME] = module
    spec.loader.exec_module(module)
    _PATCH_TO_HEADER_MODULE = module
    return module


def hide_console_window() -> None:
    if sys.platform != "win32":
        return

    try:
        hwnd = ctypes.windll.kernel32.GetConsoleWindow()
        if hwnd:
            ctypes.windll.user32.ShowWindow(hwnd, 0)
    except Exception:
        return


def configure_windows_app_id() -> None:
    if sys.platform != "win32":
        return

    try:
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(WINDOWS_APP_ID)
    except Exception:
        return


def request_existing_instance_shutdown() -> None:
    try:
        with socket.create_connection((INSTANCE_HOST, INSTANCE_PORT), timeout=0.5) as connection:
            connection.sendall(INSTANCE_SHUTDOWN_MESSAGE)
    except OSError:
        return


def acquire_instance_listener() -> socket.socket:
    deadline = time.time() + INSTANCE_STARTUP_TIMEOUT_S
    first_attempt = True

    while True:
        listener_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                listener_socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            else:
                listener_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener_socket.bind((INSTANCE_HOST, INSTANCE_PORT))
            listener_socket.listen(1)
            listener_socket.settimeout(0.5)
            return listener_socket
        except OSError:
            listener_socket.close()
            request_existing_instance_shutdown()
            if not first_attempt and time.time() >= deadline:
                raise RuntimeError("Could not close the previous Patch Generator instance.")
            first_attempt = False
            time.sleep(0.2)


def ideal_text_color(color_rgb: tuple[int, int, int]) -> str:
    red, green, blue = color_rgb
    luminance = (0.2126 * red) + (0.7152 * green) + (0.0722 * blue)
    return "#000000" if luminance >= 96 else "#FFFFFF"


def format_number(value: float) -> str:
    if float(value).is_integer():
        return str(int(round(value)))
    return f"{value:.4f}".rstrip("0").rstrip(".")


def rectangle_from_three_points(
    first_point: tuple[int, int],
    second_point: tuple[int, int],
    width_point: tuple[int, int],
) -> list[tuple[int, int]] | None:
    dx = second_point[0] - first_point[0]
    dy = second_point[1] - first_point[1]
    side_length = math.hypot(dx, dy)
    if side_length <= 1e-6:
        return None

    normal_x = -dy / side_length
    normal_y = dx / side_length
    signed_width = (
        (width_point[0] - first_point[0]) * normal_x +
        (width_point[1] - first_point[1]) * normal_y
    )
    if abs(signed_width) <= 1e-6:
        return None

    third_point = (
        int(round(second_point[0] + (normal_x * signed_width))),
        int(round(second_point[1] + (normal_y * signed_width))),
    )
    fourth_point = (
        int(round(first_point[0] + (normal_x * signed_width))),
        int(round(first_point[1] + (normal_y * signed_width))),
    )

    rectangle = [first_point, second_point, third_point, fourth_point]
    return rectangle if order_quad_points(rectangle) is not None else None


def order_quad_points(points: list[tuple[int, int]]) -> list[tuple[int, int]] | None:
    if len(points) != 4 or len(set(points)) != 4:
        return None

    centroid_x = sum(point[0] for point in points) / 4.0
    centroid_y = sum(point[1] for point in points) / 4.0
    clockwise = sorted(points, key=lambda point: math.atan2(point[1] - centroid_y, point[0] - centroid_x))

    start_index = min(range(4), key=lambda index: (clockwise[index][1], clockwise[index][0]))
    ordered = clockwise[start_index:] + clockwise[:start_index]

    if not is_convex_quad(ordered):
        return None

    return ordered


def is_convex_quad(points: list[tuple[int, int]]) -> bool:
    return len(points) == 4 and is_convex_polygon(points)


def is_convex_polygon(points: list[tuple[int, int]]) -> bool:
    return polygon_invalid_reason(points) is None


def orientation(
    point_a: tuple[int, int],
    point_b: tuple[int, int],
    point_c: tuple[int, int],
) -> int:
    cross = (
        (point_b[0] - point_a[0]) * (point_c[1] - point_a[1]) -
        (point_b[1] - point_a[1]) * (point_c[0] - point_a[0])
    )
    if abs(cross) < 1e-6:
        return 0
    return 1 if cross > 0 else -1


def on_segment(
    point_a: tuple[int, int],
    point_b: tuple[int, int],
    point_c: tuple[int, int],
) -> bool:
    return (
        min(point_a[0], point_c[0]) <= point_b[0] <= max(point_a[0], point_c[0]) and
        min(point_a[1], point_c[1]) <= point_b[1] <= max(point_a[1], point_c[1])
    )


def segments_intersect(
    start_a: tuple[int, int],
    end_a: tuple[int, int],
    start_b: tuple[int, int],
    end_b: tuple[int, int],
) -> bool:
    o1 = orientation(start_a, end_a, start_b)
    o2 = orientation(start_a, end_a, end_b)
    o3 = orientation(start_b, end_b, start_a)
    o4 = orientation(start_b, end_b, end_a)

    if o1 != o2 and o3 != o4:
        return True

    if o1 == 0 and on_segment(start_a, start_b, end_a):
        return True
    if o2 == 0 and on_segment(start_a, end_b, end_a):
        return True
    if o3 == 0 and on_segment(start_b, start_a, end_b):
        return True
    if o4 == 0 and on_segment(start_b, end_a, end_b):
        return True

    return False


def simple_polygon_invalid_reason(
    points: list[tuple[int, int]],
    minimum_points: int = 3,
    required_points: int | None = None,
) -> str | None:
    if required_points is not None and len(points) != required_points:
        return f"it needs exactly {required_points} points."
    if len(points) < minimum_points:
        return f"it needs at least {minimum_points} points."
    if len(set(points)) != len(points):
        return "two or more points overlap."

    point_count = len(points)
    for index in range(point_count):
        p1 = points[index]
        p2 = points[(index + 1) % point_count]
        p3 = points[(index + 2) % point_count]
        cross = (p2[0] - p1[0]) * (p3[1] - p2[1]) - (p2[1] - p1[1]) * (p3[0] - p2[0])
        if abs(cross) < 1e-6:
            return "at least three neighboring points are collinear."

    for index_a in range(point_count):
        a_start = points[index_a]
        a_end = points[(index_a + 1) % point_count]
        for index_b in range(index_a + 1, point_count):
            if index_b == index_a:
                continue
            if index_b == (index_a + 1) % point_count:
                continue
            if index_a == 0 and index_b == point_count - 1:
                continue

            b_start = points[index_b]
            b_end = points[(index_b + 1) % point_count]
            if segments_intersect(a_start, a_end, b_start, b_end):
                return "the polygon crosses over itself."

    return None


def polygon_invalid_reason(
    points: list[tuple[int, int]], minimum_points: int = 3, required_points: int | None = None
) -> str | None:
    if required_points is not None and len(points) != required_points:
        return f"it needs exactly {required_points} points."
    if len(points) < 3 or len(set(points)) != len(points):
        if len(points) < minimum_points:
            return f"it needs at least {minimum_points} points."
        return "two or more points overlap."

    cross_products: list[float] = []
    point_count = len(points)
    for index in range(point_count):
        p1 = points[index]
        p2 = points[(index + 1) % point_count]
        p3 = points[(index + 2) % point_count]
        vector_a = (p2[0] - p1[0], p2[1] - p1[1])
        vector_b = (p3[0] - p2[0], p3[1] - p2[1])
        cross = vector_a[0] * vector_b[1] - vector_a[1] * vector_b[0]
        if abs(cross) < 1e-6:
            return "at least three neighboring points are collinear."
        cross_products.append(cross)

    has_positive = any(value > 0 for value in cross_products)
    has_negative = any(value < 0 for value in cross_products)
    if has_positive and has_negative:
        return "the polygon is concave."
    return None


def parse_csv_row(row: list[str]) -> tuple[str, str, tuple[float, ...], bool] | None:
    cells = [cell.strip() for cell in row if cell.strip()]
    if not cells:
        return None

    if cells[0].lower() in {"label", "type"}:
        return None

    first_two = [cell.lower() for cell in cells[:2]]
    if "quadpatch" in first_two:
        kind = "quadpatch"
    elif "circlepatch" in first_two:
        kind = "circlepatch"
    elif "polypatch" in first_two:
        kind = "polypatch"
    else:
        raise ValueError("Row does not contain a supported patch label.")

    if len(cells) < 3:
        raise ValueError("Row is missing patch data.")

    if cells[0].lower() == kind:
        name = cells[1]
        remainder_cells = cells[2:]
    else:
        name = cells[0]
        remainder_cells = cells[2:]

    wall_adjacent = False
    number_cells: list[str] = []
    for cell in remainder_cells:
        lowered = cell.lower()
        if lowered.startswith("wall_adjacent="):
            value = lowered.split("=", 1)[1]
            if value in {"1", "true", "yes", "y"}:
                wall_adjacent = True
            elif value in {"0", "false", "no", "n"}:
                wall_adjacent = False
            else:
                raise ValueError("wall_adjacent metadata must be true/false.")
            continue
        number_cells.append(cell)

    try:
        numbers = [float(value) for value in number_cells]
    except ValueError as exc:
        raise ValueError("Patch coordinates must be numeric.") from exc

    if kind == "quadpatch":
        if len(numbers) != 8:
            raise ValueError("Quadpatch rows must contain 8 coordinate values.")
        point_pairs = [
            (int(round(numbers[index])), int(round(numbers[index + 1])))
            for index in range(0, len(numbers), 2)
        ]
        ordered = order_quad_points(point_pairs)
        if ordered is None:
            raise ValueError("Quadpatch rows must define a convex quadrilateral.")
        geometry = tuple(float(value) for point in ordered for value in point)
        return (name, kind, geometry, wall_adjacent)

    if kind == "polypatch":
        if len(numbers) < 6 or len(numbers) % 2 != 0:
            raise ValueError("Polypatch rows must contain at least 3 x/y point pairs.")
        point_pairs = [
            (int(round(numbers[index])), int(round(numbers[index + 1])))
            for index in range(0, len(numbers), 2)
        ]
        invalid_reason = simple_polygon_invalid_reason(point_pairs, minimum_points=3)
        if invalid_reason is not None:
            raise ValueError(f"Polypatch rows are invalid because {invalid_reason}")
        geometry = tuple(float(value) for point in point_pairs for value in point)
        return (name, kind, geometry, wall_adjacent)

    if len(numbers) != 3:
        raise ValueError("Circlepatch rows must contain center x, center y, and radius.")
    if numbers[2] <= 0:
        raise ValueError("Circlepatch radius must be positive.")
    return (name, kind, (numbers[0], numbers[1], numbers[2]), wall_adjacent)


def main() -> None:
    hide_console_window()
    configure_windows_app_id()
    listener_socket = acquire_instance_listener()
    root = tk.Tk()
    root.state("zoomed")
    app = WebcamPatchApp(root)
    app.start_instance_listener(listener_socket)
    root.minsize(1100, 760)
    root.mainloop()
    del app


if __name__ == "__main__":
    main()
