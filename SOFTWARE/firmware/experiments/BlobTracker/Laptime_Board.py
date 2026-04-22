from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import queue
import random
import socket
import threading
import time
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk


MAX_CARS = 5
MAX_NAME_LENGTH = 25
ALLOWED_COLORS = {
    "red": "#d7263d",
    "yellow": "#f4d35e",
    "green": "#2a9d8f",
    "blue": "#2b59c3",
    "purple": "#7b2cbf",
    "white": "#f5f5f5",
    "pink": "#ff85a1",
}
BOARD_BACKGROUND = "#15151b"
HEADER_BACKGROUND = "#252531"
HEADER_FOREGROUND = "#f7f7f7"
CELL_BORDER = "#3d3d4c"
SPLASH_BACKGROUND = "#0b0d14"
SPLASH_PROGRESS_BG = "#1d2230"
SPLASH_PROGRESS_FG = "#ef233c"
CONTROL_HOST = "127.0.0.1"
CONTROL_PORT = 39471
ROW_HEIGHT = 44
LEADER_ROW_HEIGHT = 66
ROW_GAP = 3
ANIMATION_SMOOTHING = 0.28
ANIMATION_EPSILON = 0.6
SPLASH_SCALE = 0.75


def parse_time(value: object) -> float | None:
    if value is None:
        return None
    if isinstance(value, str):
        cleaned = value.strip()
        if not cleaned or cleaned == "-":
            return None
        return float(cleaned)
    return float(value)


def format_time(value: float | None) -> str:
    if value is None:
        return "-"
    return f"{value:.3f}"


def format_diff(current: float | None, previous: float | None) -> str:
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


@dataclass
class CarState:
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


class LapTimeBoard(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Formula 1 Style Lap Time Board")
        self.configure(bg=BOARD_BACKGROUND)
        self.protocol("WM_DELETE_WINDOW", self._handle_close)
        self.withdraw()
        self.geometry("1220x700")
        self.minsize(1080, 420)
        self._icon_image: tk.PhotoImage | None = None
        self._background_source: Image.Image | None = None
        self._background_image: ImageTk.PhotoImage | None = None
        self._background_label: tk.Label | None = None
        self._rows_background_image: ImageTk.PhotoImage | None = None
        self._rows_background_item: int | None = None
        self._splash_source: Image.Image | None = None
        self._splash_image: ImageTk.PhotoImage | None = None
        self._splash_image_size: tuple[int, int] = (256, 256)
        self._background_resize_job: str | None = None
        self._splash_window: tk.Toplevel | None = None
        self._splash_canvas: tk.Canvas | None = None
        self._splash_progress_fill: int | None = None
        self._splash_status_label: tk.Label | None = None
        self._splash_started_at = time.perf_counter()
        self._command_queue: queue.Queue[dict[str, object]] = queue.Queue()
        self._control_server_socket: socket.socket | None = None
        self._control_server_thread: threading.Thread | None = None
        self._control_server_stop = threading.Event()
        self._load_window_icon()
        self._load_background_asset()
        self._load_splash_asset()
        self._show_splash_screen()
        self._medal_images = self._load_medal_images()

        self.cars: dict[int, CarState] = {}
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

        self._update_splash_progress(18, "Building header")
        self._build_header()
        self._update_splash_progress(45, "Preparing rows")
        self._build_rows()
        self._update_splash_progress(72, "Loading controls")
        self._build_test_panel()
        self._update_splash_progress(92, "Finalizing board")
        self.refresh_board()
        self.bind("<Configure>", self._handle_window_configure)
        self._update_splash_progress(100, "Ready")
        self._start_control_server()
        self.after(50, self._poll_control_queue)
        minimum_splash_ms = 3000
        elapsed_ms = int((time.perf_counter() - self._splash_started_at) * 1000)
        remaining_ms = max(0, minimum_splash_ms - elapsed_ms)
        self.after(remaining_ms, self._finish_startup)

    def _handle_close(self) -> None:
        self._control_server_stop.set()
        if self._control_server_socket is not None:
            try:
                self._control_server_socket.close()
            except OSError:
                pass
            self._control_server_socket = None
        self.destroy()

    def _start_control_server(self) -> None:
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind((CONTROL_HOST, CONTROL_PORT))
            server_socket.listen()
            server_socket.settimeout(0.5)
        except OSError as exc:
            print(f"Lap board control server unavailable: {exc}")
            return

        self._control_server_socket = server_socket
        self._control_server_thread = threading.Thread(
            target=self._control_server_loop,
            name="LapTimeBoardControlServer",
            daemon=True,
        )
        self._control_server_thread.start()

    def _control_server_loop(self) -> None:
        if self._control_server_socket is None:
            return

        while not self._control_server_stop.is_set():
            try:
                connection, _address = self._control_server_socket.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            with connection:
                connection.settimeout(0.5)
                buffer = b""
                while not self._control_server_stop.is_set():
                    try:
                        chunk = connection.recv(4096)
                    except socket.timeout:
                        continue
                    except OSError:
                        break

                    if not chunk:
                        break
                    buffer += chunk

                for raw_line in buffer.splitlines():
                    line = raw_line.strip()
                    if not line:
                        continue
                    try:
                        payload = json.loads(line.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        continue
                    if isinstance(payload, dict):
                        self._command_queue.put(payload)

    def _poll_control_queue(self) -> None:
        while True:
            try:
                payload = self._command_queue.get_nowait()
            except queue.Empty:
                break
            self._process_remote_command(payload)

        if self.winfo_exists():
            self.after(50, self._poll_control_queue)

    def _process_remote_command(self, payload: dict[str, object]) -> None:
        action = str(payload.get("action", "")).strip().lower()
        if action == "reset":
            laps = payload.get("laps")
            if laps is not None:
                self.run_laps_var.set(str(laps))
            self.reset_board()
            return

        if action == "set_laps":
            laps = payload.get("laps")
            if laps is not None:
                self.run_laps_var.set(str(laps))
            return

        if action != "update":
            return

        try:
            self.process_update(
                car_id=payload["car_id"],
                current_third=int(payload["current_third"]),
                latest_third_time=payload.get("latest_third_time"),
                current_lap=int(payload["current_lap"]),
                latest_lap_time=payload.get("latest_lap_time"),
            )
        except (KeyError, TypeError, ValueError):
            return

    def _get_icon_path(self) -> Path:
        return Path(__file__).resolve().parent / "assets" / "laptime_board.png"

    def _get_asset_path(self, filename: str) -> Path:
        return Path(__file__).resolve().parent / "assets" / filename

    def _get_background_path(self) -> Path:
        return self._get_asset_path("laptime_board_background.png")

    def _get_splash_path(self) -> Path:
        return self._get_asset_path("laptime_board_splash.png")

    def _load_window_icon(self) -> None:
        icon_path = self._get_icon_path()
        if not icon_path.exists():
            return

        try:
            self._icon_image = tk.PhotoImage(file=str(icon_path))
            self.iconphoto(True, self._icon_image)
        except tk.TclError:
            self._icon_image = None

    def _load_splash_asset(self) -> None:
        splash_path = self._get_splash_path()
        if not splash_path.exists():
            self._splash_source = None
            self._splash_image = None
            self._splash_image_size = (256, 256)
            return

        try:
            splash_source = Image.open(splash_path).convert("RGBA")
        except OSError:
            self._splash_source = None
            self._splash_image = None
            self._splash_image_size = (256, 256)
            return

        scaled_size = (
            max(1, int(round(splash_source.width * SPLASH_SCALE))),
            max(1, int(round(splash_source.height * SPLASH_SCALE))),
        )
        self._splash_source = splash_source
        self._splash_image_size = scaled_size
        self._splash_image = ImageTk.PhotoImage(
            splash_source.resize(scaled_size, Image.LANCZOS)
        )

    def _load_background_asset(self) -> None:
        background_path = self._get_background_path()
        if not background_path.exists():
            return

        try:
            self._background_source = Image.open(background_path).convert("RGB")
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
        if self._background_source is None or self._background_label is None:
            return

        width = max(1, self.winfo_width())
        height = max(1, self.winfo_height())
        resized = self._background_source.resize((width, height), Image.LANCZOS)
        self._background_image = ImageTk.PhotoImage(resized)
        self._background_label.configure(image=self._background_image)
        self._background_label.lower()
        self._refresh_rows_background(resized)

    def _refresh_rows_background(self, resized_background: Image.Image) -> None:
        if not hasattr(self, "rows_container") or self._rows_background_item is None:
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
        self._rows_background_image = ImageTk.PhotoImage(rows_background)
        self.rows_container.itemconfigure(self._rows_background_item, image=self._rows_background_image)
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
                image = tk.PhotoImage(file=str(medal_path))
                shrink_factor = max(1, math.ceil(max(image.width(), image.height()) / target_size))
                medal_images[place] = image.subsample(shrink_factor, shrink_factor)
            except tk.TclError:
                continue
        return medal_images

    def _show_splash_screen(self) -> None:
        splash = tk.Toplevel(self)
        splash.overrideredirect(True)
        splash.configure(bg=SPLASH_BACKGROUND)
        try:
            splash.attributes("-topmost", True)
        except tk.TclError:
            pass

        image_width, image_height = self._splash_image_size
        splash_width = image_width + 32
        splash_height = image_height + 88

        screen_width = splash.winfo_screenwidth()
        screen_height = splash.winfo_screenheight()
        pos_x = (screen_width - splash_width) // 2
        pos_y = (screen_height - splash_height) // 2
        splash.geometry(f"{splash_width}x{splash_height}+{pos_x}+{pos_y}")

        canvas = tk.Canvas(
            splash,
            width=image_width,
            height=image_height,
            bg=SPLASH_BACKGROUND,
            highlightthickness=0,
            bd=0,
        )
        canvas.pack(padx=16, pady=(16, 10))
        if self._splash_image is not None:
            canvas.create_image(image_width // 2, image_height // 2, image=self._splash_image)
        else:
            canvas.create_rectangle(0, 0, image_width, image_height, fill=HEADER_BACKGROUND, outline="")
            canvas.create_text(
                image_width // 2,
                image_height // 2,
                text="LAPTIME\nBOARD",
                fill=HEADER_FOREGROUND,
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
        progress_fill = canvas.create_rectangle(
            bar_margin,
            bar_top,
            bar_margin,
            bar_top + bar_height,
            fill=SPLASH_PROGRESS_FG,
            outline="",
        )

        status_label = tk.Label(
            splash,
            text="Starting up",
            bg=SPLASH_BACKGROUND,
            fg="#f8f9fa",
            font=("Bahnschrift", 11),
        )
        status_label.pack(pady=(0, 14))

        self._splash_window = splash
        self._splash_canvas = canvas
        self._splash_progress_fill = progress_fill
        self._splash_status_label = status_label
        self._update_splash_progress(8, "Loading splash")

    def _update_splash_progress(self, percent: int, status: str) -> None:
        if self._splash_window is None or self._splash_canvas is None or self._splash_progress_fill is None:
            return

        image_width, image_height = self._splash_image_size
        bar_margin = 18
        bar_top = image_height - 28
        bar_height = 14
        total_width = image_width - (bar_margin * 2)
        fill_width = bar_margin + int(total_width * max(0, min(percent, 100)) / 100)
        self._splash_canvas.coords(
            self._splash_progress_fill,
            bar_margin,
            bar_top,
            fill_width,
            bar_top + bar_height,
        )
        if self._splash_status_label is not None:
            self._splash_status_label.configure(text=status)
        self._splash_window.update_idletasks()

    def _finish_startup(self) -> None:
        if self._splash_window is not None:
            self._splash_window.destroy()
            self._splash_window = None
            self._splash_canvas = None
            self._splash_progress_fill = None
            self._splash_status_label = None
        self.deiconify()
        self._apply_windowed_fullscreen()
        self._refresh_background_image()
        self.lift()

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
        title = tk.Label(
            self,
            text="LAPTIME BOARD",
            bg=BOARD_BACKGROUND,
            fg="#f8f9fa",
            font=("Bahnschrift SemiBold", 20),
            pady=12,
            cursor="hand2",
        )
        title.grid(row=0, column=0, sticky="ew")
        title.bind("<ButtonPress-1>", self._start_title_hold)
        title.bind("<ButtonRelease-1>", self._cancel_title_hold)
        title.bind("<Leave>", self._cancel_title_hold)
        self.title_label = title

        self.board_frame = tk.Frame(self, bg=BOARD_BACKGROUND, padx=12, pady=6)
        self.board_frame.grid(row=1, column=0, sticky="nsew")
        self.grid_rowconfigure(1, weight=1)
        self.grid_columnconfigure(0, weight=1)
        self.board_frame.grid_columnconfigure(0, weight=1)
        self.board_frame.grid_rowconfigure(1, weight=1)

        header = tk.Frame(self.board_frame, bg=HEADER_BACKGROUND, bd=1, relief="solid")
        header.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        self._configure_table_columns(header)
        for index, (label, _) in enumerate(self.column_specs):
            cell = tk.Label(
                header,
                text=label,
                bg=HEADER_BACKGROUND,
                fg=HEADER_FOREGROUND,
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
        available_colors = list(ALLOWED_COLORS.keys())
        starting_colors = random.sample(available_colors, k=MAX_CARS)
        for car_id in range(1, MAX_CARS + 1):
            name_var = tk.StringVar(value=f"Driver {car_id}")
            color_var = tk.StringVar(value=starting_colors[car_id - 1])
            state = CarState(car_id=car_id, name_var=name_var, color_var=color_var)
            self.cars[car_id] = state

            name_var.trace_add("write", self._limit_name_length(state))
            color_var.trace_add("write", self._refresh_row_from_trace(state))

            row_frame = tk.Frame(self.rows_container, bg=ALLOWED_COLORS["white"], bd=1, relief="solid")
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
        label = tk.Label(
            parent,
            text=text,
            font=("Consolas", 11),
            padx=6,
            pady=7,
            bd=1,
            relief="solid",
        )
        label.grid(row=0, column=column, sticky="nsew")
        return label

    def _build_entry(
        self,
        parent: tk.Widget,
        column: int,
        textvariable: tk.StringVar,
    ) -> tk.Entry:
        entry = tk.Entry(
            parent,
            textvariable=textvariable,
            font=("Bahnschrift", 11),
            justify="center",
            bd=1,
            relief="solid",
        )
        entry.grid(row=0, column=column, sticky="nsew")
        return entry

    def _build_color_menu(
        self,
        parent: tk.Widget,
        column: int,
        variable: tk.StringVar,
    ) -> tk.OptionMenu:
        menu = tk.OptionMenu(parent, variable, *ALLOWED_COLORS.keys())
        menu.config(
            width=1,
            font=("Bahnschrift", 10),
            indicatoron=False,
            bd=1,
            relief="solid",
            highlightthickness=0,
        )
        color_menu = menu["menu"]
        for index, color_name in enumerate(ALLOWED_COLORS.keys()):
            swatch_color = ALLOWED_COLORS[color_name]
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
        self.run_laps_var = tk.StringVar(value="6")
        tk.Entry(
            self.control_panel,
            textvariable=self.run_laps_var,
            width=6,
            font=("Consolas", 11),
            justify="center",
        ).grid(row=0, column=1, sticky="w", padx=(0, 12))
        tk.Button(
            self.control_panel,
            text="Reset",
            command=self.reset_board,
            font=("Bahnschrift SemiBold", 10),
            bg="#6c757d",
            fg="#ffffff",
            activebackground="#5c636a",
            activeforeground="#ffffff",
            padx=12,
            pady=4,
        ).grid(row=0, column=2, sticky="w")

    def _get_run_laps(self) -> int:
        try:
            run_laps = int(self.run_laps_var.get().strip())
        except (AttributeError, ValueError):
            run_laps = 6

        if run_laps < 1:
            run_laps = 1
        return run_laps

    def _limit_name_length(self, state: CarState):
        def callback(*_: object) -> None:
            current = state.name_var.get()
            if len(current) > MAX_NAME_LENGTH:
                state.name_var.set(current[:MAX_NAME_LENGTH])

        return callback

    def _refresh_row_from_trace(self, state: CarState):
        def callback(*_: object) -> None:
            selected = state.color_var.get().strip().lower()
            if selected not in ALLOWED_COLORS:
                state.color_var.set("white")
                return
            self._apply_row_style(state)

        return callback

    def _apply_row_style(self, state: CarState) -> None:
        base_color = ALLOWED_COLORS[state.color_var.get()]
        row_frame = self.row_frames[state.car_id]
        row_frame.configure(bg=BOARD_BACKGROUND, highlightbackground=CELL_BORDER)

        total_cells = len(self.row_widget_order)
        for index, key in enumerate(self.row_widget_order):
            widget = state.frames[key]
            alpha = 1.0 - (0.8 * index / max(1, total_cells - 1))
            bg_color = blend_hex_colors(base_color, BOARD_BACKGROUND, alpha)
            fg_color = best_text_color(bg_color)

            if isinstance(widget, tk.Label):
                widget.configure(bg=bg_color, fg=fg_color, highlightbackground=CELL_BORDER)
            elif isinstance(widget, tk.Entry):
                widget.configure(
                    bg=bg_color,
                    fg=fg_color,
                    insertbackground=fg_color,
                    highlightbackground=CELL_BORDER,
                    disabledbackground=bg_color,
                    disabledforeground=fg_color,
                )
            elif isinstance(widget, tk.OptionMenu):
                widget.configure(
                    bg=bg_color,
                    fg=fg_color,
                    activebackground=bg_color,
                    activeforeground=fg_color,
                    highlightbackground=CELL_BORDER,
                )
                widget["menu"].configure(
                    bg=blend_hex_colors(base_color, BOARD_BACKGROUND, 0.75),
                    fg=best_text_color(blend_hex_colors(base_color, BOARD_BACKGROUND, 0.75)),
                    activebackground=bg_color,
                    activeforeground=fg_color,
                )
                for menu_index, color_name in enumerate(ALLOWED_COLORS.keys()):
                    swatch_color = ALLOWED_COLORS[color_name]
                    swatch_text = best_text_color(swatch_color)
                    widget["menu"].entryconfigure(
                        menu_index,
                        background=swatch_color,
                        foreground=swatch_text,
                        activebackground=swatch_color,
                        activeforeground=swatch_text,
                    )

    def _set_row_emphasis(self, state: CarState, is_leader: bool) -> None:
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

    def _place_row(self, state: CarState) -> None:
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
            target_y, target_height = self._row_targets[car_id]
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

    def reset_board(self) -> None:
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
            raise ValueError(f"Car id must be between 1 and {MAX_CARS}.")
        if current_third not in (1, 2, 3):
            raise ValueError("Current third must be 1, 2, or 3.")
        if current_lap < 1:
            raise ValueError("Current lap must be 1 or higher.")

        state = self.cars[parsed_car_id]
        if state.finished or state.disqualified:
            return

        incoming_progress = ((current_lap - 1) * 3) + current_third
        existing_progress = ((state.current_lap - 1) * 3) + state.current_third

        # Ignore stale packets so a late checkpoint report cannot push a car
        # backward in the displayed lap/third progression.
        if state.has_started and incoming_progress < existing_progress:
            return

        if is_disqualified:
            state.current_third = current_third
            state.current_lap = current_lap
            state.disqualified = True
            self.refresh_board()
            return

        third_time = parse_time(latest_third_time)
        lap_time = parse_time(latest_lap_time)

        if third_time is not None:
            state.previous_third_time = state.last_third_time
            state.last_third_time = third_time
            state.total_elapsed += third_time

        if lap_time is not None:
            state.previous_lap_time = state.last_lap_time
            state.last_lap_time = lap_time

        state.current_third = current_third
        state.current_lap = current_lap
        run_laps = self._get_run_laps()
        if current_lap > run_laps:
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

            state.frames["position"].configure(
                text=str(position) if state.has_started else "-"
            )
            state.frames["car"].configure(text=str(state.car_id))
            state.frames["lap"].configure(
                text="Finished" if state.finished else str(state.current_lap)
            )
            state.frames["third"].configure(text=str(state.current_third))
            state.frames["third_time"].configure(text=format_time(state.last_third_time))
            state.frames["third_diff"].configure(
                text=format_diff(state.last_third_time, state.previous_third_time)
            )
            state.frames["lap_time"].configure(text=format_time(state.last_lap_time))
            state.frames["lap_diff"].configure(
                text=format_diff(state.last_lap_time, state.previous_lap_time)
            )
            if state.disqualified:
                state.frames["medal"].configure(text="DQ", image="", compound="center")
            elif state.finish_place in self._medal_images:
                state.frames["medal"].configure(
                    text="",
                    image=self._medal_images[state.finish_place],
                    compound="center",
                )
            else:
                state.frames["medal"].configure(
                    text=format_place(state.finish_place),
                    image="",
                    compound="center",
                )
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
            1: [
                4.15, 4.10, 4.05,
                4.10, 4.05, 4.00,
                4.05, 4.00, 3.95,
                4.00, 3.95, 3.90,
                3.95, 3.90, 3.85,
                3.90, 3.85, 3.80,
                3.88, 3.83, 3.78,
                3.86, 3.81, 3.76,
                3.84, 3.79, 3.74,
                3.82, 3.77, 3.72,
            ],
            2: [
                4.00, 3.95, 3.90,
                3.95, 3.90, 3.85,
                3.90, 3.85, 3.80,
                4.10, 4.10, 4.10,
                4.15, 4.15, 4.15,
                4.20, 4.20, 4.20,
                4.22, 4.22, 4.22,
                4.24, 4.24, 4.24,
                4.26, 4.26, 4.26,
                4.28, 4.28, 4.28,
            ],
            3: [
                4.70, 4.65, 4.60,
                4.65, 4.60, 4.55,
                4.60, 4.55, 4.50,
                4.55, 4.50, 4.45,
                3.95, 3.90, 3.85,
                3.90, 3.85, 3.80,
                3.88, 3.83, 3.78,
                3.86, 3.81, 3.76,
                3.84, 3.79, 3.74,
                3.82, 3.77, 3.72,
            ],
            4: [
                4.25, 4.20, 4.15,
                4.22, 4.17, 4.12,
                4.20, 4.15, 4.10,
                4.20, 4.18, 4.17,
                4.18, 4.17, 4.15,
                4.16, 4.15, 4.14,
                4.15, 4.14, 4.13,
                4.14, 4.13, 4.12,
                4.13, 4.12, 4.11,
                4.12, 4.11, 4.10,
            ],
            5: [
                4.40, 4.35, 4.30,
                4.35, 4.30, 4.25,
                4.30, 4.25, 4.20,
                4.28, 4.24, 4.20,
                4.26, 4.22, 4.18,
                4.24, 4.20, 4.16,
                4.22, 4.18, 4.14,
                4.20, 4.16, 4.12,
                4.18, 4.14, 4.10,
                4.16, 4.12, 4.08,
            ],
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
                    update = (f"{car_id}DQ", current_third, "-", current_lap, "-")
                    events.append((round(cumulative_time + 0.001, 6), update))
                    break

                cumulative_time += third_time
                lap_running_time += third_time
                current_lap = ((completed_thirds - 1) // 3) + 1
                next_third = (completed_thirds % 3) + 1
                next_lap = current_lap + 1 if completed_thirds % 3 == 0 else current_lap
                lap_time: float | str = round(lap_running_time, 3) if completed_thirds % 3 == 0 else "-"
                if completed_thirds % 3 == 0:
                    lap_running_time = 0.0

                update = (
                    car_id,
                    next_third,
                    round(third_time, 3),
                    next_lap,
                    lap_time,
                )
                events.append((round(cumulative_time, 6), update))

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


def main() -> None:
    app = LapTimeBoard()
    app.mainloop()


if __name__ == "__main__":
    main()
