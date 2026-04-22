#!/usr/bin/env python3
"""
Track up to five red/blue marker pairs from a USB webcam and visualize RC-car motion.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from collections import deque
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
    import tkinter as tk
    from tkinter import filedialog, ttk
except ImportError as exc:  # pragma: no cover
    raise SystemExit("tkinter is required for the tracker GUI.") from exc


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
DEFAULT_CALIBRATION_CSV = Path(__file__).with_name("camera_calibration.csv")
FRAME_INTERVAL_MS = 15
SIDE_PANEL_WIDTH_PX = 360
STATUS_TEXT_HEIGHT = 18
PREVIEW_WIDTH_PX = 1280
PREVIEW_HEIGHT_PX = 720
ROI_MARGIN_PX = 60
PROCESSING_WIDTH_PX = 960
PROCESSING_HEIGHT_PX = 540
FULL_FRAME_RECOVERY_INTERVAL = 6
TRACK_COUNT = 5
WHITE_BGR = (255, 255, 255)
TRACK_LABEL_COLORS = [
    (0, 255, 255),
    (255, 255, 0),
    (255, 0, 255),
    (0, 165, 255),
    (255, 255, 255),
]


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
        default=str(DEFAULT_CALIBRATION_CSV),
        help=(
            "CSV file containing camera settings to apply on startup. "
            f"Default: {DEFAULT_CALIBRATION_CSV.name} beside this script."
        ),
    )
    parser.add_argument(
        "--browse-calibration",
        action="store_true",
        help="Open a file browser to choose the camera calibration CSV at startup.",
    )
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


def apply_camera_calibration(capture: cv2.VideoCapture, csv_path: str) -> None:
    settings = load_camera_calibration(csv_path)
    if not settings:
        return
    print(f"Applying camera calibration from {csv_path}")
    for property_name, property_id, value in settings:
        applied = capture.set(property_id, value)
        actual_value = capture.get(property_id)
        status = "ok" if applied else "ignored"
        print(
            f"  {property_name}: requested {value:.3f}, "
            f"camera reports {actual_value:.3f} ({status})"
        )


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
    pending_pairs = [pair for pair in best_assignment if pair is not None]

    for track_index, previous_position in enumerate(previous_positions):
        if previous_position is not None:
            compacted_assignment[track_index] = best_assignment[track_index]

    for track_index, previous_position in enumerate(previous_positions):
        if previous_position is not None:
            continue
        if not pending_pairs:
            break
        compacted_assignment[track_index] = pending_pairs.pop(0)

    return compacted_assignment


def draw_blob_marker(frame: np.ndarray, blob: Blob, color_bgr: tuple[int, int, int]) -> None:
    cv2.drawContours(frame, [blob.contour], -1, color_bgr, 2, lineType=cv2.LINE_AA)
    point = (int(round(blob.centroid[0])), int(round(blob.centroid[1])))
    cv2.circle(frame, point, 4, color_bgr, thickness=-1, lineType=cv2.LINE_AA)


def draw_pose_overlay(
    frame: np.ndarray,
    samples: deque[PoseSample],
    orientation_full_scale_deg: float,
    speed_reference_px_s: float,
    stop_speed_threshold_px_s: float,
    label: str,
    label_color_bgr: tuple[int, int, int],
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
    cv2.circle(frame, current_point, DOT_RADIUS_PX, BLACK_BGR, thickness=-1, lineType=cv2.LINE_AA)
    cv2.putText(
        frame,
        label,
        (current_point[0] + 8, current_point[1] - 8),
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
        self.last_frame_time_s: float | None = None
        self.frame_counter = 0
        self.processing_scale = min(
            1.0,
            PROCESSING_WIDTH_PX / max(1, actual_width),
            PROCESSING_HEIGHT_PX / max(1, actual_height),
        )
        self.processing_width = max(1, int(round(actual_width * self.processing_scale)))
        self.processing_height = max(1, int(round(actual_height * self.processing_scale)))

        self.root = tk.Tk()
        self.root.title(args.window_name)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.resizable(False, False)
        self.root.geometry(f"{SIDE_PANEL_WIDTH_PX}x520")
        self.root.columnconfigure(0, weight=0)
        self.root.rowconfigure(0, weight=0)
        self.show_history_var = tk.BooleanVar(master=self.root, value=True)
        self.show_travel_arrow_var = tk.BooleanVar(master=self.root, value=True)
        self.show_orientation_arrow_var = tk.BooleanVar(master=self.root, value=True)

        self.side_panel = ttk.Frame(
            self.root,
            padding=12,
            width=SIDE_PANEL_WIDTH_PX,
            height=520,
        )
        self.side_panel.grid(row=0, column=0, sticky="nsew")
        self.side_panel.grid_propagate(False)
        self.side_panel.columnconfigure(0, weight=1)
        self.side_panel.rowconfigure(5, weight=1)

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

        self.status_text = tk.Text(
            self.side_panel,
            width=36,
            height=STATUS_TEXT_HEIGHT,
            wrap="word",
        )
        self.status_text.grid(row=3, column=0, sticky="nsew", pady=(12, 12))
        self.status_text.configure(state="disabled")

        self.controls_frame = ttk.Frame(self.side_panel)
        self.controls_frame.grid(row=4, column=0, sticky="ew", pady=(0, 12))
        self.controls_frame.columnconfigure(0, weight=1)

        ttk.Label(self.controls_frame, text="Overlay Elements").grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(
            self.controls_frame,
            text="History Lines",
            variable=self.show_history_var,
        ).grid(row=1, column=0, sticky="w")
        ttk.Checkbutton(
            self.controls_frame,
            text="Travel Arrow",
            variable=self.show_travel_arrow_var,
        ).grid(row=2, column=0, sticky="w")
        ttk.Checkbutton(
            self.controls_frame,
            text="Orientation Arrow",
            variable=self.show_orientation_arrow_var,
        ).grid(row=3, column=0, sticky="w")

        self.spacer_frame = ttk.Frame(self.side_panel)
        self.spacer_frame.grid(row=5, column=0, sticky="nsew")

        cv2.namedWindow(self.args.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.args.window_name, PREVIEW_WIDTH_PX, PREVIEW_HEIGHT_PX)
        self.root.after(FRAME_INTERVAL_MS, self.update_frame)

    def format_calibration_label(self) -> str:
        calibration_name = Path(self.calibration_path).name if self.calibration_path else "(none)"
        return f"Calibration CSV: {calibration_name}"

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
        self.args.camera_calibration = selected
        self.calibration_label.configure(text=self.format_calibration_label())
        apply_camera_calibration(self.capture, selected)
        self.set_status(
            [
                "Applied camera calibration.",
                f"CSV: {selected}",
            ]
        )

    def set_status(self, lines: list[str]) -> None:
        self.status_text.configure(state="normal")
        self.status_text.delete("1.0", tk.END)
        self.status_text.insert("1.0", "\n".join(lines))
        self.status_text.configure(state="disabled")

    def detect_blob_pairs(
        self,
        processing_frame: np.ndarray,
    ) -> tuple[list[Blob], list[Blob], list[tuple[Blob, Blob] | None]]:
        hsv_frame = preprocess_frame(processing_frame, self.args.blur_kernel)
        scaled_min_area = self.args.min_area * self.processing_scale * self.processing_scale
        scaled_max_area = self.args.max_area * self.processing_scale * self.processing_scale
        scaled_max_pair_distance = self.args.max_pair_distance * self.processing_scale
        red_blobs = extract_blobs(
            make_color_mask(hsv_frame, "red", self.args.morph_kernel),
            scaled_min_area,
            scaled_max_area,
        )
        blue_blobs = extract_blobs(
            make_color_mask(hsv_frame, "blue", self.args.morph_kernel),
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
        self.last_search_mode = f"Full frame ({self.processing_width}x{self.processing_height})"
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
        if cv2.getWindowProperty(self.args.window_name, cv2.WND_PROP_VISIBLE) < 1:
            self.close()
            return

        frame, frame_timestamp_s = read_frame(self.capture)
        if frame is None or frame_timestamp_s is None:
            self.set_status(["Camera frame read failed."])
            self.close()
            return
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

        detected_tracks = 0
        status_lines = [
            f"Detected pairs: {sum(pair is not None for pair in chosen_pairs)}/{TRACK_COUNT}",
            f"Framerate: {fps:.1f} fps",
            f"Search region: {self.last_search_mode}",
            f"Detected red blobs: {len(red_blobs)}",
            f"Detected blue blobs: {len(blue_blobs)}",
            "",
        ]

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
            track.samples.append(pose)
            track.last_pose = pose
            track.previous_position = car_position
            track.previous_timestamp_s = frame_timestamp_s
            if track.motion_reference_position is None or track.motion_reference_timestamp_s is None:
                track.motion_reference_position = car_position
                track.motion_reference_timestamp_s = frame_timestamp_s

            draw_pose_overlay(
                annotated,
                track.samples,
                self.args.orientation_full_scale_deg,
                self.args.speed_reference,
                self.args.stop_speed_threshold,
                str(track_index + 1),
                label_color,
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

        if detected_tracks == 0:
            status_lines.insert(0, "Waiting for two red+blue blob pairs.")
        else:
            status_lines.insert(0, "Tracking active.")

        self.set_status(status_lines)

        preview_width, preview_height = get_preview_size(self.args.window_name)
        preview = fit_frame_to_canvas(annotated, preview_width, preview_height)
        cv2.imshow(self.args.window_name, preview)
        cv2.waitKey(1)

        self.root.after(FRAME_INTERVAL_MS, self.update_frame)

    def close(self) -> None:
        if not self.running:
            return
        self.running = False
        self.capture.release()
        cv2.destroyAllWindows()
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def run(self) -> int:
        self.root.mainloop()
        return 0


def main() -> int:
    args = build_parser().parse_args()
    validate_args(args)
    if args.browse_calibration:
        selected_calibration = choose_calibration_file(args.camera_calibration)
        if selected_calibration is None:
            raise SystemExit("Calibration file selection canceled.")
        args.camera_calibration = selected_calibration

    video_source = parse_camera_source(args.camera)
    capture, backend_name = open_video_capture(video_source, args.backend, args.width, args.height)
    apply_camera_calibration(capture, args.camera_calibration)
    actual_width = int(round(capture.get(cv2.CAP_PROP_FRAME_WIDTH)))
    actual_height = int(round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))

    print(
        f"Video source: {args.camera} (backend={backend_name}, "
        f"requested={args.width}x{args.height}, actual={actual_width}x{actual_height})"
    )

    app = TrackerApp(capture, args, backend_name, actual_width, actual_height)
    return app.run()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        sys.exit(0)
