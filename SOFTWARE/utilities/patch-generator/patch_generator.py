from __future__ import annotations

import csv
import colorsys
import math
import random
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import tkinter as tk
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
SCRIPT_DIR = Path(__file__).resolve().parent
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


@dataclass
class PatchRecord:
    name: str
    kind: str
    geometry: tuple[float, ...]
    color_rgb: tuple[int, int, int]
    table_tag: str

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


class NamePrompt(tk.Toplevel):
    def __init__(self, parent: tk.Misc, title: str, prompt: str) -> None:
        super().__init__(parent)
        self.result: str | None = None
        self.title(title)
        self.resizable(False, False)
        self.transient(parent)
        self.grab_set()

        self.columnconfigure(0, weight=1)

        ttk.Label(self, text=prompt).grid(row=0, column=0, padx=16, pady=(16, 8), sticky="w")

        self.value_var = tk.StringVar()
        entry = ttk.Entry(self, textvariable=self.value_var, width=32)
        entry.grid(row=1, column=0, padx=16, pady=(0, 12), sticky="ew")
        entry.focus_set()

        button_row = ttk.Frame(self)
        button_row.grid(row=2, column=0, padx=16, pady=(0, 16), sticky="e")
        ttk.Button(button_row, text="Cancel", command=self.cancel).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(button_row, text="Save", command=self.save).pack(side=tk.LEFT)

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

    def cancel(self) -> None:
        self.result = None
        self.destroy()

    def save(self) -> None:
        value = self.value_var.get().strip()
        if not value:
            messagebox.showwarning("Name Required", "Please enter a name before saving.", parent=self)
            return
        self.result = value
        self.destroy()


class WebcamPatchApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title(APP_TITLE)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.current_csv_path: Path | None = None
        self.patches: list[PatchRecord] = []
        self.current_mode: str | None = None
        self.temp_points: list[tuple[int, int]] = []
        self.mouse_position: tuple[int, int] | None = None
        self.frame_size: tuple[int, int] | None = None
        self.display_region: tuple[int, int, int, int] | None = None
        self.video_photo: ImageTk.PhotoImage | None = None
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

        self.status_var = tk.StringVar(value="Waiting for video feed...")
        self.file_var = tk.StringVar(value="CSV: none")

        self._build_ui()
        self.render_thread = threading.Thread(target=self.render_loop, name="camera-render", daemon=True)
        self.render_thread.start()
        self._schedule_display_refresh()

    def _build_ui(self) -> None:
        style = ttk.Style()
        self.canvas_bg = style.lookup("TFrame", "background") or self.root.cget("bg")

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
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.canvas.bind("<Motion>", self.on_canvas_motion)
        self.canvas.bind("<Leave>", self.on_canvas_leave)
        self.canvas.bind("<Configure>", self.on_canvas_configure)
        self.waiting_text_item = self.canvas.create_text(
            480,
            360,
            text="Waiting for video feed",
            fill="#808080",
            font=("Arial", 28, "bold"),
        )

        side = ttk.Frame(main, width=280)
        side.grid(row=0, column=1, sticky="ns")
        side.columnconfigure(0, weight=1)
        side.rowconfigure(7, weight=1)

        ttk.Label(side, text="Controls", font=("", 12, "bold")).grid(
            row=0, column=0, sticky="w", pady=(0, 10)
        )

        self.quad_button = ttk.Button(side, text="Quadpatch", command=self.start_quadpatch, state=tk.DISABLED)
        self.quad_button.grid(row=1, column=0, sticky="ew", pady=2)

        self.circle_button = ttk.Button(
            side, text="Circlepatch", command=self.start_circlepatch, state=tk.DISABLED
        )
        self.circle_button.grid(row=2, column=0, sticky="ew", pady=2)

        self.load_button = ttk.Button(side, text="Load", command=self.load_csv, state=tk.DISABLED)
        self.load_button.grid(row=3, column=0, sticky="ew", pady=2)

        self.clear_button = ttk.Button(side, text="Clear", command=self.clear_all, state=tk.DISABLED)
        self.clear_button.grid(row=4, column=0, sticky="ew", pady=2)

        self.reload_button = ttk.Button(side, text="Reload", command=self.reload_csv, state=tk.DISABLED)
        self.reload_button.grid(row=5, column=0, sticky="ew", pady=2)

        self.new_button = ttk.Button(side, text="New", command=self.create_new_csv, state=tk.DISABLED)
        self.new_button.grid(row=6, column=0, sticky="ew", pady=2)

        table_frame = ttk.LabelFrame(side, text="Patches", padding=8)
        table_frame.grid(row=7, column=0, sticky="nsew", pady=(12, 0))
        table_frame.columnconfigure(0, weight=1)
        table_frame.rowconfigure(0, weight=1)

        self.table = ttk.Treeview(table_frame, columns=("name", "type"), show="headings", height=18)
        self.table.heading("name", text="Name")
        self.table.heading("type", text="Type")
        self.table.column("name", width=150, anchor="w")
        self.table.column("type", width=100, anchor="w")
        self.table.grid(row=0, column=0, sticky="nsew")

        scrollbar = ttk.Scrollbar(table_frame, orient=tk.VERTICAL, command=self.table.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.table.configure(yscrollcommand=scrollbar.set)

        ttk.Label(side, textvariable=self.file_var, wraplength=250).grid(
            row=8, column=0, sticky="ew", pady=(12, 4)
        )
        ttk.Label(side, textvariable=self.status_var, wraplength=250, justify="left").grid(
            row=9, column=0, sticky="ew"
        )

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

        display = frame_bgr.copy()
        height, width = display.shape[:2]

        for patch in patches:
            if patch.kind == "quadpatch":
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
            for point in temp_points:
                self.draw_crosshair(frame_bgr, point, PREVIEW_RED_BGR)
        elif current_mode == "circlepatch":
            if not temp_points:
                return
            center = temp_points[0]
            self.draw_crosshair(frame_bgr, center, PREVIEW_RED_BGR)
            if mouse_position is None:
                return
            radius = int(round(distance_between(center, mouse_position)))
            self.draw_translucent_circle(
                frame_bgr,
                center,
                radius,
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
        with self.state_lock:
            self.mouse_position = point

    def on_canvas_leave(self, _event: tk.Event) -> None:
        with self.state_lock:
            self.mouse_position = None

    def on_canvas_click(self, event: tk.Event) -> None:
        point = self.clamp_point((event.x, event.y))
        with self.state_lock:
            current_mode = self.current_mode
        if point is None or current_mode is None:
            return

        if current_mode == "quadpatch":
            with self.state_lock:
                self.temp_points.append(point)
                point_count = len(self.temp_points)
            if point_count == 4:
                self.finish_quadpatch()
        elif current_mode == "circlepatch":
            with self.state_lock:
                self.temp_points.append(point)
                point_count = len(self.temp_points)
            if point_count == 2:
                self.finish_circlepatch()

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
        self.status_var.set("Quadpatch mode: click 4 points in the video view.")

    def finish_quadpatch(self) -> None:
        with self.state_lock:
            raw_points = list(self.temp_points)
            self.current_mode = None
            self.mouse_position = None

        ordered = order_quad_points(raw_points)
        if ordered is None:
            messagebox.showerror(
                "Invalid Quadpatch",
                "The selected shape is not a convex quadrilateral. The quadpatch was cancelled.",
                parent=self.root,
            )
            with self.state_lock:
                self.temp_points = []
            self.status_var.set("Quadpatch cancelled.")
            return

        name = self.prompt_for_name("Save Quadpatch", "Enter a name for this quadpatch:")
        if name is None:
            with self.state_lock:
                self.temp_points = []
            self.status_var.set("Quadpatch cancelled.")
            return

        with self.state_lock:
            self.temp_points = []
        geometry = tuple(float(value) for point in ordered for value in point)
        self.save_patch_record(name=name, kind="quadpatch", geometry=geometry)
        self.status_var.set(f'Saved quadpatch "{name}".')

    def start_circlepatch(self) -> None:
        if self.current_csv_path is None:
            return
        with self.state_lock:
            self.current_mode = "circlepatch"
            self.temp_points = []
            self.mouse_position = None
        self.status_var.set("Circlepatch mode: click the center, then click the edge.")

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

        name = self.prompt_for_name("Save Circlepatch", "Enter a name for this circlepatch:")
        if name is None:
            with self.state_lock:
                self.temp_points = []
            self.status_var.set("Circlepatch cancelled.")
            return

        with self.state_lock:
            self.temp_points = []
        geometry = (float(center[0]), float(center[1]), float(radius))
        self.save_patch_record(name=name, kind="circlepatch", geometry=geometry)
        self.status_var.set(f'Saved circlepatch "{name}".')

    def prompt_for_name(self, title: str, prompt: str) -> str | None:
        dialog = NamePrompt(self.root, title, prompt)
        self.root.wait_window(dialog)
        return dialog.result

    def save_patch_record(self, name: str, kind: str, geometry: tuple[float, ...]) -> None:
        if self.current_csv_path is None:
            raise RuntimeError("No CSV file is active.")

        record = self.make_patch_record(name, kind, geometry)
        try:
            with self.current_csv_path.open("a", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow(self.serialize_record(record))
        except OSError as exc:
            messagebox.showerror("Save Error", f"Could not save patch data.\n\n{exc}", parent=self.root)
            return

        self.add_patch_to_ui(record)

    def serialize_record(self, record: PatchRecord) -> list[str]:
        if record.kind == "quadpatch":
            return [record.kind, record.name, *[format_number(value) for value in record.geometry]]
        if record.kind == "circlepatch":
            return [record.kind, record.name, *[format_number(value) for value in record.geometry]]
        raise ValueError(f"Unsupported patch type: {record.kind}")

    def make_patch_record(self, name: str, kind: str, geometry: tuple[float, ...]) -> PatchRecord:
        self.tag_counter += 1
        return PatchRecord(
            name=name,
            kind=kind,
            geometry=geometry,
            color_rgb=self.generate_patch_color(),
            table_tag=f"patch_{self.tag_counter}",
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
        self.table.tag_configure(
            record.table_tag,
            background=record.outline_hex,
            foreground=record.text_hex,
        )
        self.table.insert("", tk.END, values=(record.name, record.kind), tags=(record.table_tag,))

    def load_csv(self) -> None:
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
        self.status_var.set(f"Loaded {len(self.patches)} patch(es) from {path.name}.")
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
                        name, kind, geometry = parsed
                        record = self.make_patch_record(name, kind, geometry)
                        self.add_patch_to_ui(record)
                    except ValueError as exc:
                        errors.append(f"Line {line_number}: {exc}")
        except OSError as exc:
            messagebox.showerror("Load Error", f"Could not read the CSV file.\n\n{exc}", parent=self.root)
            self.clear_all()
        return errors

    def reload_csv(self) -> None:
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
        self.status_var.set(f"Reloaded {len(self.patches)} patch(es) from {path.name}.")
        self.update_button_states()

    def create_new_csv(self) -> None:
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
        self.status_var.set(f"Created new CSV file: {path.name}")

    def resolve_new_csv_path(self, selected_path: Path) -> Path:
        target_dir = selected_path.parent
        if target_dir.name.casefold() != "output":
            target_dir = target_dir / "output"
            target_dir.mkdir(parents=True, exist_ok=True)

        return target_dir / selected_path.name

    def clear_all(self) -> None:
        self.clear_patch_data(reset_file=True)
        self.status_var.set("Cleared patches and reset the active CSV file.")
        self.update_button_states()

    def clear_patch_data(self, reset_file: bool) -> None:
        with self.state_lock:
            self.current_mode = None
            self.temp_points = []
            self.mouse_position = None
            self.display_region = None
            self.patches.clear()
            self.latest_display_rgb = None
            self.latest_display_region = None
            self.latest_frame_size = None
        for item in self.table.get_children():
            self.table.delete(item)

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
            self.reload_button.config(state=tk.DISABLED)
            return

        self.load_button.config(state=tk.NORMAL)
        self.clear_button.config(state=tk.NORMAL)
        self.new_button.config(state=tk.NORMAL)

        patch_state = tk.NORMAL if self.current_csv_path is not None else tk.DISABLED
        self.quad_button.config(state=patch_state)
        self.circle_button.config(state=patch_state)
        self.reload_button.config(state=patch_state)

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
        self.stop_event.set()
        if self.render_thread is not None and self.render_thread.is_alive():
            self.render_thread.join(timeout=1.0)
        if hasattr(self, "capture") and self.capture is not None:
            self.capture.release()
        self.root.destroy()


def distance_between(point_a: tuple[int, int], point_b: tuple[int, int]) -> float:
    return math.hypot(point_b[0] - point_a[0], point_b[1] - point_a[1])


def ideal_text_color(color_rgb: tuple[int, int, int]) -> str:
    red, green, blue = color_rgb
    luminance = (0.2126 * red) + (0.7152 * green) + (0.0722 * blue)
    return "#000000" if luminance >= 96 else "#FFFFFF"


def format_number(value: float) -> str:
    if float(value).is_integer():
        return str(int(round(value)))
    return f"{value:.4f}".rstrip("0").rstrip(".")


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
    if len(points) != 4:
        return False

    cross_products: list[float] = []
    for index in range(4):
        p1 = points[index]
        p2 = points[(index + 1) % 4]
        p3 = points[(index + 2) % 4]
        vector_a = (p2[0] - p1[0], p2[1] - p1[1])
        vector_b = (p3[0] - p2[0], p3[1] - p2[1])
        cross = vector_a[0] * vector_b[1] - vector_a[1] * vector_b[0]
        if abs(cross) < 1e-6:
            return False
        cross_products.append(cross)

    has_positive = any(value > 0 for value in cross_products)
    has_negative = any(value < 0 for value in cross_products)
    return not (has_positive and has_negative)


def parse_csv_row(row: list[str]) -> tuple[str, str, tuple[float, ...]] | None:
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
    else:
        raise ValueError("Row does not contain a supported patch label.")

    if len(cells) < 3:
        raise ValueError("Row is missing patch data.")

    if cells[0].lower() == kind:
        name = cells[1]
        number_cells = cells[2:]
    else:
        name = cells[0]
        number_cells = cells[2:]

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
        return (name, kind, geometry)

    if len(numbers) != 3:
        raise ValueError("Circlepatch rows must contain center x, center y, and radius.")
    if numbers[2] <= 0:
        raise ValueError("Circlepatch radius must be positive.")
    return (name, kind, (numbers[0], numbers[1], numbers[2]))


def main() -> None:
    root = tk.Tk()
    app = WebcamPatchApp(root)
    root.minsize(1100, 760)
    root.mainloop()
    del app


if __name__ == "__main__":
    main()
