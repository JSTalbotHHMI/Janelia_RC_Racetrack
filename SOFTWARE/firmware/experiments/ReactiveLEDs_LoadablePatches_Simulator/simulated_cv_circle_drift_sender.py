#!/usr/bin/env python3
"""
Send synthetic computer-vision car updates to ReactiveLEDs_LoadablePatches
while showing the simulated pose overlaid on a video feed.

The generated motion is a clockwise perfect circle in image coordinates
(origin at the top-left, +x to the right, +y downward). The reported travel
direction follows the tangent of that circle. The heading offset changes
smoothly around the orbit: aligned from 12 o'clock to 3 o'clock, gradually
clockwise to a maximum at 6 o'clock, then gradually counter-clockwise to a
maximum at 9 o'clock, and finally back to aligned by 12 o'clock.

Packet format expected by the Teensy sketch:
    CAR,<carId>,<x>,<y>,<travelDeg>,<headingDeg>\n

Example:
    python simulated_cv_circle_drift_sender.py --port COM7 --video-source 0
"""

from __future__ import annotations

import argparse
import math
import sys
import time
import traceback
from collections import deque

try:
    import cv2
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "opencv-python is required. Install it with: pip install opencv-python"
    ) from exc

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "pyserial is required. Install it with: pip install pyserial"
    ) from exc


TRAIL_LENGTH = 30
TRAIL_MIN_ALPHA = 0.10
TRAIL_MAX_ALPHA = 1.00
DOT_RADIUS_PX = 5
FORWARD_ARROW_LENGTH_PX = 42
HEADING_ARROW_LENGTH_PX = 26
TRAIL_THICKNESS_PX = 2
ARROW_THICKNESS_PX = 2
YELLOW_BGR = (0, 255, 255)
BLUE_BGR = (255, 0, 0)
WARNING_RED_BGR = (0, 0, 255)
DRIFT_STATUS_THRESHOLD_DEG = 18.0
HEADING_COLOR_FULL_SCALE_DEG = 45.0


def wrap_angle_deg(angle_deg: float) -> float:
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg <= -180.0:
        angle_deg += 360.0
    return angle_deg


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Stream synthetic CAR packets over serial for the "
            "ReactiveLEDs_LoadablePatches Teensy sketch and overlay them "
            "onto a live video feed."
        )
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Serial port for the Teensy, for example COM7.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate. Default: 115200.",
    )
    parser.add_argument(
        "--video-source",
        default="0",
        help=(
            "OpenCV video source. Use a camera index like 0 or a path to a "
            "video file. Default: 0."
        ),
    )
    parser.add_argument(
        "--video-backend",
        choices=("auto", "any", "dshow", "msmf"),
        default="auto",
        help=(
            "Preferred OpenCV backend for camera sources. On Windows, 'auto' "
            "tries DirectShow and Media Foundation if the default backend fails. "
            "Default: auto."
        ),
    )
    parser.add_argument(
        "--window-name",
        default="ReactiveLEDs CV Simulator",
        help="GUI window title. Default: ReactiveLEDs CV Simulator.",
    )
    parser.add_argument(
        "--car-id",
        type=int,
        default=1,
        help="Car identifier to embed in packets. Default: 1.",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=30.0,
        help="Packet send and redraw rate in Hz. Default: 30.",
    )
    parser.add_argument(
        "--lap-seconds",
        type=float,
        default=8.0,
        help="Seconds per full clockwise lap. Default: 8.0.",
    )
    parser.add_argument(
        "--view-width",
        type=float,
        default=None,
        help=(
            "Coordinate-system width for emitted CV data. Overlay positions are "
            "scaled to the live video frame. Defaults to the video frame width."
        ),
    )
    parser.add_argument(
        "--view-height",
        type=float,
        default=None,
        help=(
            "Coordinate-system height for emitted CV data. Overlay positions are "
            "scaled to the live video frame. Defaults to the video frame height."
        ),
    )
    parser.add_argument(
        "--center-x",
        type=float,
        default=None,
        help="Circle center x in CV coordinates. Defaults to half the view width.",
    )
    parser.add_argument(
        "--center-y",
        type=float,
        default=None,
        help="Circle center y in CV coordinates. Defaults to half the view height.",
    )
    parser.add_argument(
        "--radius",
        type=float,
        default=None,
        help="Circle radius in CV coordinates. Defaults to 36%% of the smaller view dimension.",
    )
    parser.add_argument(
        "--start-angle-deg",
        type=float,
        default=0.0,
        help="Starting orbital angle in degrees. 0 starts on the right. Default: 0.",
    )
    parser.add_argument(
        "--drift-offset-deg",
        type=float,
        default=35.0,
        help=(
            "Heading offset magnitude in drift regions. Values above 18 degrees "
            "will trigger drifting with the current Teensy defaults. Default: 35."
        ),
    )
    parser.add_argument(
        "--drift-side-threshold",
        type=float,
        default=0.72,
        help=(
            "Legacy option retained for backwards compatibility. The current "
            "smooth heading profile ignores this value. Default: 0.72."
        ),
    )
    parser.add_argument(
        "--startup-delay",
        type=float,
        default=2.0,
        help="Seconds to wait after opening the port before sending. Default: 2.0.",
    )
    parser.add_argument(
        "--status-interval",
        type=float,
        default=1.0,
        help="Seconds between console status lines. Set 0 to disable. Default: 1.0.",
    )
    return parser


def parse_video_source(raw_value: str) -> int | str:
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
        return [("any", cap_any), ("dshow", cap_dshow), ("msmf", cap_msmf)]

    return [("any", cap_any)]


def validate_args(args: argparse.Namespace) -> None:
    if args.rate_hz <= 0:
        raise SystemExit("--rate-hz must be greater than 0")
    if args.lap_seconds <= 0:
        raise SystemExit("--lap-seconds must be greater than 0")
    if (
        (args.view_width is not None and args.view_width <= 0) or
        (args.view_height is not None and args.view_height <= 0)
    ):
        raise SystemExit("--view-width and --view-height must be greater than 0")
    if args.radius is not None and args.radius <= 0:
        raise SystemExit("--radius must be greater than 0")
    if args.drift_side_threshold < 0 or args.drift_side_threshold > 1:
        raise SystemExit("--drift-side-threshold must be between 0 and 1")


def lerp(start: float, end: float, amount: float) -> float:
    return start + (end - start) * amount


def heading_offset_for_orbit(theta_rad: float, max_offset_deg: float) -> float:
    if max_offset_deg <= 0.0:
        return 0.0

    full_turn = 2.0 * math.pi
    half_pi = 0.5 * math.pi
    theta_rad = theta_rad % full_turn

    if theta_rad < half_pi:
        return lerp(0.0, max_offset_deg, theta_rad / half_pi)

    if theta_rad < math.pi:
        return lerp(max_offset_deg, -max_offset_deg, (theta_rad - half_pi) / half_pi)

    if theta_rad < 1.5 * math.pi:
        return lerp(-max_offset_deg, 0.0, (theta_rad - math.pi) / half_pi)

    return 0.0


def compute_sample(
    elapsed_s: float,
    center_x: float,
    center_y: float,
    radius: float,
    lap_seconds: float,
    start_angle_rad: float,
    drift_offset_deg: float,
) -> tuple[float, float, float, float, bool]:
    angular_speed = (2.0 * math.pi) / lap_seconds
    theta = start_angle_rad + (elapsed_s * angular_speed)

    x = center_x + radius * math.cos(theta)
    y = center_y + radius * math.sin(theta)

    dx_dt = -radius * angular_speed * math.sin(theta)
    dy_dt = radius * angular_speed * math.cos(theta)
    travel_deg = wrap_angle_deg(math.degrees(math.atan2(dy_dt, dx_dt)))

    heading_offset_deg = heading_offset_for_orbit(theta, drift_offset_deg)
    heading_deg = wrap_angle_deg(travel_deg + heading_offset_deg)
    drifting = abs(heading_offset_deg) >= DRIFT_STATUS_THRESHOLD_DEG

    return x, y, travel_deg, heading_deg, drifting


def cv_point(
    x: float,
    y: float,
    view_width: float,
    view_height: float,
    frame_width: int,
    frame_height: int,
) -> tuple[int, int]:
    scale_x = frame_width / view_width
    scale_y = frame_height / view_height
    px = int(round(x * scale_x))
    py = int(round(y * scale_y))
    return px, py


def endpoint_from_angle(
    origin: tuple[int, int],
    angle_deg: float,
    length_px: int,
) -> tuple[int, int]:
    angle_rad = math.radians(angle_deg)
    end_x = origin[0] + int(round(length_px * math.cos(angle_rad)))
    end_y = origin[1] + int(round(length_px * math.sin(angle_rad)))
    return end_x, end_y


def heading_color_bgr(travel_deg: float, heading_deg: float) -> tuple[int, int, int]:
    delta_deg = wrap_angle_deg(heading_deg - travel_deg)
    color_strength = min(abs(delta_deg) / HEADING_COLOR_FULL_SCALE_DEG, 1.0)

    if delta_deg >= 0.0:
        green = int(round(255.0 * color_strength))
        return 0, green, 0

    red = int(round(255.0 * color_strength))
    return 0, 0, red


def blend_line(
    frame,
    start_point: tuple[int, int],
    end_point: tuple[int, int],
    color_bgr: tuple[int, int, int],
    alpha: float,
    thickness: int,
) -> None:
    overlay = frame.copy()
    cv2.line(overlay, start_point, end_point, color_bgr, thickness, lineType=cv2.LINE_AA)
    cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0.0, dst=frame)


def draw_overlay(
    frame,
    position_history: deque[tuple[float, float]],
    current_x: float,
    current_y: float,
    travel_deg: float,
    heading_deg: float,
    view_width: float,
    view_height: float,
) -> None:
    frame_height, frame_width = frame.shape[:2]

    history_points = [
        cv_point(x, y, view_width, view_height, frame_width, frame_height)
        for x, y in position_history
    ]

    segment_count = max(0, len(history_points) - 1)
    if segment_count > 0:
        for segment_index in range(segment_count):
            alpha_span = TRAIL_MAX_ALPHA - TRAIL_MIN_ALPHA
            alpha = TRAIL_MIN_ALPHA
            if segment_count > 1:
                alpha += alpha_span * (segment_index / (segment_count - 1))
            else:
                alpha = TRAIL_MAX_ALPHA

            blend_line(
                frame=frame,
                start_point=history_points[segment_index],
                end_point=history_points[segment_index + 1],
                color_bgr=YELLOW_BGR,
                alpha=alpha,
                thickness=TRAIL_THICKNESS_PX,
            )

    current_point = cv_point(
        current_x,
        current_y,
        view_width,
        view_height,
        frame_width,
        frame_height,
    )
    forward_tip = endpoint_from_angle(current_point, travel_deg, FORWARD_ARROW_LENGTH_PX)
    heading_tip = endpoint_from_angle(current_point, heading_deg, HEADING_ARROW_LENGTH_PX)

    cv2.circle(frame, current_point, DOT_RADIUS_PX, YELLOW_BGR, thickness=-1, lineType=cv2.LINE_AA)
    cv2.arrowedLine(
        frame,
        current_point,
        forward_tip,
        BLUE_BGR,
        ARROW_THICKNESS_PX,
        line_type=cv2.LINE_AA,
        tipLength=0.25,
    )
    cv2.arrowedLine(
        frame,
        current_point,
        heading_tip,
        heading_color_bgr(travel_deg, heading_deg),
        ARROW_THICKNESS_PX,
        line_type=cv2.LINE_AA,
        tipLength=0.25,
    )


def draw_text_block(frame, lines: list[str], color_bgr: tuple[int, int, int]) -> None:
    y = 28
    for line in lines:
        cv2.putText(
            frame,
            line,
            (16, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color_bgr,
            2,
            cv2.LINE_AA,
        )
        y += 28


def window_is_open(window_name: str) -> bool:
    try:
        return cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) >= 1
    except cv2.error:
        return False


def make_blank_frame(width: int, height: int):
    return cv2.UMat(height, width, cv2.CV_8UC3).get()


def open_video_capture(
    video_source: int | str,
    backend_name: str,
) -> tuple[cv2.VideoCapture, str]:
    for candidate_name, backend_id in backend_candidates(video_source, backend_name):
        capture = cv2.VideoCapture(video_source, backend_id)
        if capture.isOpened():
            return capture, candidate_name
        capture.release()

    raise SystemExit(
        f"Could not open video source: {video_source!r}. "
        "If this is a webcam on Windows, try --video-backend dshow."
    )


def read_frame(capture: cv2.VideoCapture):
    ok, frame = capture.read()
    if ok and frame is not None:
        return frame

    if capture.get(cv2.CAP_PROP_FRAME_COUNT) > 0:
        capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
        ok, frame = capture.read()
        if ok and frame is not None:
            return frame

    return None


def main() -> int:
    args = build_parser().parse_args()
    validate_args(args)

    video_source = parse_video_source(args.video_source)
    capture, opened_backend = open_video_capture(video_source, args.video_backend)
    initial_frame = read_frame(capture)
    if initial_frame is None:
        raise SystemExit(
            "Could not read an initial frame from the selected video source."
        )

    frame_height, frame_width = initial_frame.shape[:2]
    view_width = args.view_width if args.view_width is not None else float(frame_width)
    view_height = args.view_height if args.view_height is not None else float(frame_height)

    center_x = args.center_x if args.center_x is not None else view_width * 0.5
    center_y = args.center_y if args.center_y is not None else view_height * 0.5
    radius = args.radius if args.radius is not None else min(view_width, view_height) * 0.36
    start_angle_rad = math.radians(args.start_angle_deg)
    interval_s = 1.0 / args.rate_hz

    print(f"Opening {args.port} at {args.baud} baud")
    print(
        f"Video source: {args.video_source} "
        f"({frame_width}x{frame_height}, backend={opened_backend})"
    )
    print(f"CV space: {view_width:.1f} x {view_height:.1f}")
    print(
        "Circle:"
        f" center=({center_x:.1f}, {center_y:.1f})"
        f" radius={radius:.1f}"
        f" lap={args.lap_seconds:.2f}s"
    )
    print(
        "Drift:"
        f" offset={args.drift_offset_deg:.1f} deg"
    )
    print(
        "Motion: clockwise, heading aligned from 12->3, rotating clockwise to 6, "
        "counter-clockwise to 9, then returning aligned by 12."
    )
    print("Press q, Esc, or Ctrl+C to stop.")

    cv2.namedWindow(args.window_name, cv2.WINDOW_NORMAL)

    position_history: deque[tuple[float, float]] = deque(maxlen=TRAIL_LENGTH)

    try:
        with serial.serial_for_url(args.port, args.baud, timeout=0.1) as ser:
            time.sleep(args.startup_delay)

            started_at = time.monotonic()
            next_send = started_at
            last_status = started_at - args.status_interval if args.status_interval > 0 else started_at

            frame = initial_frame

            while True:
                if not window_is_open(args.window_name):
                    break

                now = time.monotonic()
                if now < next_send:
                    time.sleep(next_send - now)
                    now = time.monotonic()

                next_frame = read_frame(capture)
                if next_frame is not None:
                    frame = next_frame
                else:
                    warning_frame = frame.copy()
                    draw_text_block(
                        warning_frame,
                        [
                            "Warning: video frame read failed.",
                            "Continuing with the last good frame.",
                        ],
                        WARNING_RED_BGR,
                    )
                    frame = warning_frame
                elapsed_s = now - started_at
                x, y, travel_deg, heading_deg, drifting = compute_sample(
                    elapsed_s=elapsed_s,
                    center_x=center_x,
                    center_y=center_y,
                    radius=radius,
                    lap_seconds=args.lap_seconds,
                    start_angle_rad=start_angle_rad,
                    drift_offset_deg=args.drift_offset_deg,
                )

                position_history.append((x, y))

                packet = (
                    f"CAR,{args.car_id},{x:.2f},{y:.2f},{travel_deg:.2f},{heading_deg:.2f}\n"
                ).encode("ascii")
                ser.write(packet)
                ser.flush()

                annotated_frame = frame.copy()
                draw_overlay(
                    frame=annotated_frame,
                    position_history=position_history,
                    current_x=x,
                    current_y=y,
                    travel_deg=travel_deg,
                    heading_deg=heading_deg,
                    view_width=view_width,
                    view_height=view_height,
                )
                cv2.imshow(args.window_name, annotated_frame)

                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")) or not window_is_open(args.window_name):
                    break

                if args.status_interval > 0 and (now - last_status) >= args.status_interval:
                    state = "DRIFT" if drifting else "GRIP"
                    print(
                        f"{state} "
                        f"x={x:7.2f} y={y:7.2f} "
                        f"travel={travel_deg:7.2f} heading={heading_deg:7.2f}"
                    )
                    last_status = now

                next_send += interval_s
                if next_send < now:
                    next_send = now + interval_s
    except serial.SerialException as exc:
        raise SystemExit(f"Serial error while using {args.port!r}: {exc}") from exc
    except cv2.error as exc:
        raise SystemExit(
            "OpenCV GUI error. If the camera opens but the window fails, "
            "please share the traceback and try a different --video-backend."
        ) from exc
    finally:
        capture.release()
        cv2.destroyAllWindows()

    return 0


def pause_before_exit() -> None:
    try:
        input("Press Enter to close...")
    except EOFError:
        pass


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nStopped.")
        sys.exit(0)
    except SystemExit as exc:
        exit_code = exc.code
        if exit_code not in (None, 0):
            if isinstance(exit_code, str):
                print(exit_code, file=sys.stderr)
            pause_before_exit()
        raise
    except Exception:  # pragma: no cover
        traceback.print_exc()
        pause_before_exit()
        sys.exit(1)
