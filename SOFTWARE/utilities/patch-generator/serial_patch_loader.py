#!/usr/bin/env python3
"""Watch a patch CSV and stream the patch set to the controller over serial."""

from __future__ import annotations

import argparse
import importlib.util
import sys
import time
from pathlib import Path

try:
    import serial
    from serial import SerialException
except ModuleNotFoundError as exc:
    print("This script requires pyserial. Install it with: pip install pyserial", file=sys.stderr)
    raise SystemExit(1) from exc


SCRIPT_DIR = Path(__file__).resolve().parent
PATCH_TO_HEADER_PATH = SCRIPT_DIR.parent / "patch-to-header" / "patch-to-header.py"
PATCH_TO_HEADER_MODULE_NAME = "patch_to_header_serial_loader"
DEFAULT_AUTO_SEGMENT_LENGTH = 3


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send patch CSV updates to ReactiveLEDs_LoadablePatches over serial."
    )
    parser.add_argument("port", help="Serial port, for example COM5")
    parser.add_argument(
        "csv_path",
        type=Path,
        help="Patch CSV to stream to the controller",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate. Default: 115200",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.25,
        help="Watch polling interval in seconds. Default: 0.25",
    )
    parser.add_argument(
        "--settle-seconds",
        type=float,
        default=2.0,
        help="How long to wait after opening the port before the first upload. Default: 2.0",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Send the current CSV one time, then exit",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=4.0,
        help="Seconds to wait for a PATCHES_OK or PATCHES_ERROR response. Default: 4.0",
    )
    parser.add_argument(
        "--segment-lengths",
        help="Comma-separated LED counts for runtime segments, for example 12,8,15. "
             "When omitted, the controller keeps its current/default segment layout.",
    )
    return parser.parse_args()


def load_patch_to_header_module():
    spec = importlib.util.spec_from_file_location(PATCH_TO_HEADER_MODULE_NAME, PATCH_TO_HEADER_PATH)
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not load patch parser from {PATCH_TO_HEADER_PATH}")

    module = importlib.util.module_from_spec(spec)
    sys.modules[PATCH_TO_HEADER_MODULE_NAME] = module
    spec.loader.exec_module(module)
    return module


def format_number(value: float) -> str:
    if float(value).is_integer():
        return str(int(round(value)))
    return f"{value:.4f}".rstrip("0").rstrip(".")


def shape_token(kind: str) -> str:
    if kind == "circlepatch":
        return "CIRCLE"
    if kind == "quadpatch":
        return "QUAD"
    if kind == "polypatch":
        return "POLY"
    raise ValueError(f"Unsupported patch kind: {kind}")


def sanitize_patch_name(name: str) -> str:
    cleaned = name.strip()
    if not cleaned:
        raise ValueError("Patch names cannot be blank.")
    if any(character in cleaned for character in ",\r\n\t"):
        raise ValueError(f"Patch name {name!r} contains a comma or control character.")
    return cleaned


def build_patch_lines(patches: list[object]) -> list[str]:
    lines = [f"PATCHES_BEGIN,{len(patches)}"]
    for index, patch in enumerate(patches):
        name = sanitize_patch_name(patch.name)
        numbers = ",".join(format_number(value) for value in patch.numbers)
        wall_reactive = "1" if patch.wall_adjacent else "0"
        line = ",".join(
            (
                "PATCH",
                str(index),
                name,
                shape_token(patch.kind),
                str(patch.segment_index),
                wall_reactive,
                numbers,
            )
        )
        lines.append(line)
    lines.append("PATCHES_END")
    return lines


def parse_segment_lengths(raw_value: str | None) -> list[int] | None:
    if raw_value is None:
        return None

    lengths: list[int] = []
    for token in raw_value.split(","):
        cleaned = token.strip()
        if not cleaned:
            continue
        length = int(cleaned)
        if length <= 0:
            raise ValueError("Segment lengths must be positive integers.")
        lengths.append(length)

    if not lengths:
        raise ValueError("At least one segment length is required when using --segment-lengths.")

    return lengths


def build_segment_lines(segment_lengths: list[int] | None) -> list[str]:
    if segment_lengths is None:
        return []

    lines = [f"SEGMENTS_BEGIN,{len(segment_lengths)}"]
    for index, length in enumerate(segment_lengths):
        lines.append(f"SEGMENT,{index},{length}")
    lines.append("SEGMENTS_END")
    return lines


def infer_segment_lengths(patches: list[object], segment_lengths: list[int] | None) -> list[int] | None:
    if segment_lengths is not None:
        return segment_lengths

    needed_segment_count = 0
    for patch in patches:
        segment_index = int(getattr(patch, "segment_index", -1))
        if segment_index >= 0:
            needed_segment_count = max(needed_segment_count, segment_index + 1)

    if needed_segment_count <= 4:
        return None

    return [DEFAULT_AUTO_SEGMENT_LENGTH] * needed_segment_count


def file_signature(path: Path) -> tuple[int, int]:
    stat = path.stat()
    return (stat.st_mtime_ns, stat.st_size)


def drain_serial(serial_port: serial.Serial) -> None:
    deadline = time.monotonic() + 0.25
    while time.monotonic() < deadline:
        line = serial_port.readline()
        if not line:
            break
        message = line.decode("utf-8", errors="replace").strip()
        if message:
            print(f"[controller] {message}")


def read_controller_message(serial_port: serial.Serial, deadline: float) -> str:
    while time.monotonic() < deadline:
        raw_line = serial_port.readline()
        if not raw_line:
            continue

        message = raw_line.decode("utf-8", errors="replace").strip()
        if not message:
            continue

        print(f"[controller] {message}")
        return message

    raise TimeoutError("Timed out waiting for a controller response.")


def wait_for_prefixes(
    serial_port: serial.Serial,
    deadline: float,
    accepted_prefixes: tuple[str, ...],
) -> str:
    while True:
        message = read_controller_message(serial_port, deadline)
        if message.startswith("PATCHES_ERROR,"):
            raise RuntimeError(message)
        if any(message.startswith(prefix) for prefix in accepted_prefixes):
            return message


def send_patch_set(
    serial_port: serial.Serial,
    patch_parser,
    csv_path: Path,
    response_timeout: float,
    segment_lengths: list[int] | None,
) -> None:
    patches = patch_parser.load_patches(csv_path)
    segment_lengths = infer_segment_lengths(patches, segment_lengths)
    segment_lines = build_segment_lines(segment_lengths)
    patch_lines = build_patch_lines(patches)

    serial_port.reset_input_buffer()
    deadline = time.monotonic() + response_timeout

    if segment_lines:
        serial_port.write(f"{segment_lines[0]}\n".encode("utf-8"))
        serial_port.flush()
        wait_for_prefixes(serial_port, deadline, ("SEGMENTS_BEGIN_OK,",))

        for index, line in enumerate(segment_lines[1:-1]):
            serial_port.write(f"{line}\n".encode("utf-8"))
            serial_port.flush()
            wait_for_prefixes(serial_port, deadline, (f"SEGMENT_OK,{index}",))

        serial_port.write(f"{segment_lines[-1]}\n".encode("utf-8"))
        serial_port.flush()
        wait_for_prefixes(serial_port, deadline, ("SEGMENTS_OK,",))

    serial_port.write(f"{patch_lines[0]}\n".encode("utf-8"))
    serial_port.flush()
    wait_for_prefixes(serial_port, deadline, ("PATCHES_BEGIN_OK,",))

    for index, line in enumerate(patch_lines[1:-1]):
        serial_port.write(f"{line}\n".encode("utf-8"))
        serial_port.flush()
        wait_for_prefixes(serial_port, deadline, (f"PATCH_OK,{index}",))

    serial_port.write(f"{patch_lines[-1]}\n".encode("utf-8"))
    serial_port.flush()
    wait_for_prefixes(serial_port, deadline, ("PATCHES_OK,",))


def main() -> int:
    args = parse_args()
    csv_path = args.csv_path.resolve()
    patch_parser = load_patch_to_header_module()
    segment_lengths = parse_segment_lengths(args.segment_lengths)

    if not csv_path.exists():
        print(f"CSV file not found: {csv_path}", file=sys.stderr)
        return 1

    try:
        with serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=0.2,
            write_timeout=2.0,
        ) as serial_port:
            print(f"Opened {args.port} at {args.baud} baud.")
            if args.settle_seconds > 0:
                time.sleep(args.settle_seconds)
            drain_serial(serial_port)

            last_signature: tuple[int, int] | None = None
            while True:
                current_signature = file_signature(csv_path)
                if current_signature != last_signature:
                    print(f"Uploading patches from {csv_path} ...")
                    send_patch_set(
                        serial_port,
                        patch_parser,
                        csv_path,
                        args.timeout,
                        segment_lengths,
                    )
                    print("Patch upload complete.")
                    last_signature = current_signature

                if args.once:
                    return 0

                time.sleep(max(args.interval, 0.05))
    except (OSError, SerialException, ValueError, RuntimeError, TimeoutError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
