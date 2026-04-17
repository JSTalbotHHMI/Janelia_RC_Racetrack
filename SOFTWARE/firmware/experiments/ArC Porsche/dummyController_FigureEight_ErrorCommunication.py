#!/usr/bin/env python3
"""
Send synthetic error packets to the Arduino path follower so the car can run
repeatable open-loop patterns without the camera stack.

Expected Arduino packet format:
    ERR,<speed_error_mps>,<heading_error_rad>\n

Example:
    python dummy_circle_error_sender.py --port COM7 --mode figure8
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "pyserial is required. Install it with: pip install pyserial"
    ) from exc


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Stream synthetic speed/heading error packets to the Arduino."
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Serial port for the Arduino, for example COM7.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate. Default: 115200.",
    )
    parser.add_argument(
        "--speed-error",
        type=float,
        default=-0.60,
        help=(
            "Constant speed error in m/s. Positive means the car is too fast, "
            "negative means too slow. More negative usually makes it drive faster. "
            "Default: -0.60."
        ),
    )
    parser.add_argument(
        "--heading-error",
        type=float,
        default=-0.55,
        help=(
            "Base heading error magnitude in rad. Negative should bias left with "
            "the current Arduino sign convention; positive should bias right. "
            "Default: -0.55."
        ),
    )
    parser.add_argument(
        "--right-heading-error",
        type=float,
        default=0.95,
        help=(
            "Heading error to use for the right-hand lobe in figure8 mode. "
            "This is separate because the car may need a stronger command to turn "
            "right cleanly. Default: 0.95."
        ),
    )
    parser.add_argument(
        "--mode",
        choices=("circle", "figure8"),
        default="figure8",
        help="Pattern to send. Default: figure8.",
    )
    parser.add_argument(
        "--lobe-seconds",
        type=float,
        default=2.4,
        help=(
            "For figure8 mode, how long to hold each circle direction before "
            "switching. Default: 2.4."
        ),
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=20.0,
        help="Packet send rate in Hz. Default: 20.",
    )
    parser.add_argument(
        "--startup-delay",
        type=float,
        default=2.0,
        help="Seconds to wait after opening the port before sending. Default: 2.0.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()

    if args.rate_hz <= 0:
        raise SystemExit("--rate-hz must be greater than 0")
    if args.lobe_seconds <= 0:
        raise SystemExit("--lobe-seconds must be greater than 0")

    interval_s = 1.0 / args.rate_hz

    print(f"Opening {args.port} at {args.baud} baud")
    print(f"Mode: {args.mode}")
    print(f"Base speed error: {args.speed_error:.4f} m/s")
    print(f"Left heading error: {args.heading_error:.4f} rad")
    if args.mode == "figure8":
        print(f"Right heading error: {args.right_heading_error:.4f} rad")
    if args.mode == "figure8":
        print(f"Switching turn direction every {args.lobe_seconds:.2f} s")
    print("Press Ctrl+C to stop.")

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        time.sleep(args.startup_delay)

        pattern_started = time.monotonic()
        next_send = time.monotonic()
        while True:
            now = time.monotonic()
            if now < next_send:
                time.sleep(next_send - now)

            if args.mode == "figure8":
                elapsed = now - pattern_started
                lobe_index = int(elapsed / args.lobe_seconds)
                heading_error = args.heading_error if (lobe_index % 2 == 0) else args.right_heading_error
            else:
                heading_error = args.heading_error

            packet = f"ERR,{args.speed_error:.4f},{heading_error:.4f}\n".encode("ascii")
            ser.write(packet)
            ser.flush()
            next_send += interval_s


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nStopped.")
        sys.exit(0)
