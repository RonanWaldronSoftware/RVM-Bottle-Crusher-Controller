#!/usr/bin/env python3
"""Low-latency barcode scanner for NVIDIA Jetson.

The BB01 scanner supports both raw CSI sensors in the dual-camera build.
It auto-detects a V4L2 node by sensor card name (``imx708`` or ``imx219``),
streams RG10 frames, converts Bayer intensity to adaptive 8-bit grayscale, and
decodes the newest frame with ZXing-C++. Argus remains available as an optional
backend for cameras with NVIDIA ISP support.
"""
from __future__ import annotations

import argparse
from collections import deque
import csv
import json
import os
import re
import shutil
import subprocess
import signal
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, Sequence

import cv2
import numpy as np

try:
    import zxingcpp
except ImportError as exc:
    raise SystemExit(
        "zxing-cpp is not installed. Run ./install.sh in this project."
    ) from exc

PREVIEW_HEADER = struct.Struct("!HH")

FORMAT_PRESETS: dict[str, tuple[str, ...]] = {
    "retail": ("EAN13", "EAN8", "UPCA", "UPCE", "Code128"),
    "qr": ("QRCode",),
    "1d": ("Code128", "Code39", "Code93", "EAN13", "EAN8", "UPCA", "UPCE", "ITF", "Codabar"),
    "shipping": ("Code128", "Code39", "ITF", "DataMatrix", "QRCode", "PDF417"),
}
FORMAT_ALIASES = {
    "QR": "QRCode", "QRCODE": "QRCode", "EAN13": "EAN13", "EAN8": "EAN8",
    "UPCA": "UPCA", "UPCE": "UPCE", "CODE128": "Code128", "CODE39": "Code39",
    "CODE93": "Code93", "ITF": "ITF", "CODABAR": "Codabar",
    "DATAMATRIX": "DataMatrix", "PDF417": "PDF417", "AZTEC": "Aztec",
}


def eprint(*values: object) -> None:
    print(*values, file=sys.stderr, flush=True)


def parse_roi(value: str) -> tuple[float, float, float, float]:
    try:
        values = tuple(float(item.strip()) for item in value.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("ROI must be x0,y0,x1,y1") from exc
    if len(values) != 4:
        raise argparse.ArgumentTypeError("ROI must contain four values")
    x0, y0, x1, y1 = values
    if not (0 <= x0 < x1 <= 1 and 0 <= y0 < y1 <= 1):
        raise argparse.ArgumentTypeError("ROI values must satisfy 0 <= x0 < x1 <= 1")
    return x0, y0, x1, y1


def normalize_format_name(name: str) -> str:
    compact = name.strip().replace("-", "").replace("_", "").upper()
    return FORMAT_ALIASES.get(compact, name.strip())


def build_formats(specification: str):
    lowered = specification.strip().lower()
    if lowered == "all":
        return None
    names: Sequence[str]
    if lowered in FORMAT_PRESETS:
        names = FORMAT_PRESETS[lowered]
    else:
        names = tuple(normalize_format_name(x) for x in specification.split(",") if x.strip())
    if not names:
        raise ValueError("No barcode formats selected")
    values = []
    missing = []
    for name in names:
        try:
            values.append(getattr(zxingcpp.BarcodeFormat, name))
        except AttributeError:
            missing.append(name)
    if missing:
        raise ValueError("Unsupported ZXing formats: " + ", ".join(missing))
    try:
        return zxingcpp.BarcodeFormats(values)
    except (AttributeError, TypeError):
        combined = values[0]
        for value in values[1:]:
            combined = combined | value
        return combined


def make_decoder(args: argparse.Namespace, formats):
    kwargs: dict[str, object] = {
        "try_rotate": bool(args.rotate),
        "try_downscale": bool(args.downscale),
        "return_errors": False,
    }
    if formats is not None:
        kwargs["formats"] = formats
    if hasattr(zxingcpp, "Binarizer"):
        kwargs["binarizer"] = (
            zxingcpp.Binarizer.GlobalHistogram
            if args.binarizer == "global"
            else zxingcpp.Binarizer.LocalAverage
        )
    if args.invert:
        kwargs["try_invert"] = True

    reader = getattr(zxingcpp, "read_barcodes", None)
    if reader is None:
        single_reader = getattr(zxingcpp, "read_barcode")
        def decode_single(image: np.ndarray):
            frame = np.ascontiguousarray(image, dtype=np.uint8)
            active = dict(kwargs)
            for removable in (None, "try_invert", "return_errors", "binarizer"):
                if removable:
                    active.pop(removable, None)
                try:
                    return single_reader(frame, **active)
                except TypeError as exc:
                    if "incompatible function arguments" not in str(exc):
                        raise
            return None
        return decode_single

    def decode(image: np.ndarray):
        frame = np.ascontiguousarray(image, dtype=np.uint8)
        active = dict(kwargs)
        attempts = [None, "try_invert", "return_errors", "binarizer"]
        last_error: Optional[TypeError] = None
        for removable in attempts:
            if removable:
                active.pop(removable, None)
            try:
                results = reader(frame, **active)
                return results[0] if results else None
            except TypeError as exc:
                if "incompatible function arguments" not in str(exc):
                    raise
                last_error = exc
        if last_error:
            raise last_error
        return None
    return decode


@dataclass(frozen=True)
class FramePacket:
    image: np.ndarray
    sequence: int
    captured_ns: int


class LatestFrameBuffer:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._packet: Optional[FramePacket] = None
        self._closed = False

    def publish(self, packet: FramePacket) -> None:
        with self._condition:
            self._packet = packet
            self._condition.notify_all()

    def wait_for_newer(self, last_sequence: int, timeout: float = 0.5) -> Optional[FramePacket]:
        deadline = time.monotonic() + timeout
        with self._condition:
            while not self._closed:
                if self._packet is not None and self._packet.sequence > last_sequence:
                    return self._packet
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)
        return None

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()


@dataclass
class RuntimeStats:
    started: float
    captured: int = 0
    decoded: int = 0
    skipped: int = 0
    found: int = 0
    decode_ms_ema: float = 0.0

    def add_decode(self, elapsed_ms: float, skipped: int) -> None:
        self.decoded += 1
        self.skipped += max(skipped, 0)
        self.decode_ms_ema = elapsed_ms if self.decoded == 1 else self.decode_ms_ema * 0.9 + elapsed_ms * 0.1


class CameraSource:
    """Open Argus, ordinary V4L2, or the BB01 raw IMX708 stream."""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.backend = args.camera_backend
        self.raw_sensor_name = str(getattr(args, "raw_sensor_name", "imx708")).strip().lower()
        self.pipeline: Optional[str] = None
        self.capture: Optional[cv2.VideoCapture] = None
        self.raw_process: Optional[subprocess.Popen[bytes]] = None
        self.raw_device: Optional[str] = None
        self.raw_buffer: Optional[bytearray] = None
        self.raw_frame_bytes = 0
        self.raw_bytes_per_line = 0
        self.raw_stderr: deque[str] = deque(maxlen=20)
        self.raw_stderr_thread: Optional[threading.Thread] = None

    def _argus_pipeline(self) -> str:
        if self.args.gstreamer_pipeline:
            return self.args.gstreamer_pipeline
        source = f"nvarguscamerasrc sensor-id={self.args.sensor_id}"
        if self.args.exposure_us is not None:
            exposure_ns = int(self.args.exposure_us) * 1000
            source += (
                f' exposuretimerange="{exposure_ns} {exposure_ns}"'
                f' gainrange="{self.args.gain} {self.args.gain}"'
            )
        source_caps = (
            "video/x-raw(memory:NVMM),"
            f"width=(int){self.args.sensor_width},"
            f"height=(int){self.args.sensor_height},"
            "format=(string)NV12,"
            f"framerate=(fraction){self.args.sensor_fps}/1"
        )
        output_caps = (
            "video/x-raw,"
            f"width=(int){self.args.width},"
            f"height=(int){self.args.height},"
            "format=(string)BGRx"
        )
        return (
            f"{source} ! {source_caps} ! queue leaky=downstream max-size-buffers=1 "
            f"! nvvidconv ! {output_caps} ! videoconvert "
            "! video/x-raw,format=(string)BGR "
            "! appsink drop=true max-buffers=1 sync=false"
        )

    @staticmethod
    def _video_sort_key(path: Path) -> tuple[int, str]:
        suffix = path.name.removeprefix("video")
        return (int(suffix) if suffix.isdigit() else 10_000, path.name)

    @staticmethod
    def _v4l2_info(device: str) -> str:
        try:
            result = subprocess.run(
                ["v4l2-ctl", "-d", device, "--info"],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=4,
            )
        except (OSError, subprocess.TimeoutExpired):
            return ""
        return result.stdout

    def _find_raw_device(self) -> str:
        sensor = self.raw_sensor_name
        if sensor not in {"imx708", "imx219"}:
            raise RuntimeError(f"Unsupported raw sensor name: {sensor}")

        requested = str(self.args.v4l2_device).strip()
        auto_index = 0
        if requested.lower().startswith("auto:"):
            try:
                auto_index = int(requested.split(":", 1)[1])
            except ValueError as exc:
                raise RuntimeError(f"Invalid --v4l2-device value: {requested}") from exc
            requested = "auto"

        if requested and requested.lower() != "auto":
            if not Path(requested).exists():
                raise RuntimeError(f"Requested V4L2 device does not exist: {requested}")
            return requested

        devices = sorted(Path("/dev").glob("video*"), key=self._video_sort_key)
        if not devices:
            raise RuntimeError(
                f"No /dev/video* V4L2 camera was found. Verify the active DTB and CSI ribbons."
            )

        # Prefer devices whose card/driver info mentions the requested sensor, but
        # fall back to /dev/video order. Some Jetson raw V4L2 nodes have generic
        # card names even when the device-tree sensor is correct.
        matching = [
            path for path in devices
            if sensor in self._v4l2_info(str(path)).lower()
        ]
        candidates = matching if len(matching) > auto_index else devices
        if len(candidates) <= auto_index:
            raise RuntimeError(
                f"Only {len(candidates)} V4L2 device(s) available; cannot select index {auto_index}. "
                f"Detected devices: {', '.join(str(p) for p in devices)}"
            )
        selected = str(candidates[auto_index])
        if not matching:
            eprint(
                f"WARNING: no V4L2 card name contained {sensor}; "
                f"falling back to {selected}."
            )
        return selected

    def _run_v4l2_setting(self, arguments: list[str]) -> str:
        if self.raw_device is None:
            return ""
        result = subprocess.run(
            ["v4l2-ctl", "-d", self.raw_device, *arguments],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        if result.returncode and result.stdout.strip():
            eprint("V4L2 setting warning:", result.stdout.strip())
        return result.stdout

    def _query_raw_layout(self) -> None:
        if self.raw_device is None:
            raise RuntimeError(f"{self.raw_sensor_name.upper()} device has not been selected")
        result = subprocess.run(
            ["v4l2-ctl", "-d", self.raw_device, "--get-fmt-video"],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        if result.returncode:
            details = result.stdout.strip() or f"v4l2-ctl exited with {result.returncode}"
            raise RuntimeError(
                f"Could not query the negotiated raw V4L2 format: {details}"
            )
        output = result.stdout
        line_match = re.search(r"Bytes per Line\s*:\s*(\d+)", output, re.IGNORECASE)
        size_match = re.search(r"Size Image\s*:\s*(\d+)", output, re.IGNORECASE)
        minimum_line = self.args.sensor_width * 2
        self.raw_bytes_per_line = int(line_match.group(1)) if line_match else minimum_line
        self.raw_frame_bytes = (
            int(size_match.group(1))
            if size_match
            else self.raw_bytes_per_line * self.args.sensor_height
        )
        if self.raw_bytes_per_line < minimum_line or self.raw_bytes_per_line % 2:
            raise RuntimeError(
                f"Unexpected raw-camera bytes-per-line value: {self.raw_bytes_per_line}"
            )
        minimum_frame = self.raw_bytes_per_line * self.args.sensor_height
        if self.raw_frame_bytes < minimum_frame:
            raise RuntimeError(
                f"Unexpected raw-camera frame size: {self.raw_frame_bytes}; "
                f"minimum is {minimum_frame}"
            )

    def _drain_raw_stderr(self) -> None:
        process = self.raw_process
        if process is None or process.stderr is None:
            return
        for payload in process.stderr:
            line = payload.decode("utf-8", errors="replace").strip()
            if line:
                self.raw_stderr.append(line)

    def _open_raw(self) -> None:
        if shutil.which("v4l2-ctl") is None:
            raise RuntimeError("v4l2-ctl is missing. Run ./install.sh first.")

        self.raw_device = self._find_raw_device()

        # The public IMX708 driver exposes frame-rate, exposure and gain as
        # ordinary V4L2 controls. Unsupported settings are non-fatal so the
        # driver's safe defaults can still be used.
        self._run_v4l2_setting(
            [
                "--set-fmt-video="
                f"width={self.args.sensor_width},height={self.args.sensor_height},"
                "pixelformat=RG10"
            ]
        )
        self._query_raw_layout()
        self.raw_buffer = bytearray(self.raw_frame_bytes)
        self._run_v4l2_setting([f"--set-parm={self.args.sensor_fps}"])
        if self.args.exposure_us is not None:
            self._run_v4l2_setting(
                [
                    "--set-ctrl",
                    f"exposure={int(self.args.exposure_us)},gain={int(round(self.args.gain))}",
                ]
            )

        # Reassert the capture format on the same V4L2 file handle that
        # performs streaming. Some Tegra capture drivers do not reliably retain
        # a format selected by an earlier v4l2-ctl process.
        command = [
            "v4l2-ctl",
            "-d",
            self.raw_device,
            (
                "--set-fmt-video="
                f"width={self.args.sensor_width},height={self.args.sensor_height},"
                "pixelformat=RG10"
            ),
            "--stream-mmap=4",
            "--stream-skip=3",
            "--stream-to=-",
        ]
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        self.raw_process = process
        self.raw_stderr_thread = threading.Thread(
            target=self._drain_raw_stderr,
            name=f"{self.raw_sensor_name}-v4l2-stderr",
            daemon=True,
        )
        self.raw_stderr_thread.start()
        time.sleep(0.15)
        if process.poll() is not None:
            details = "; ".join(self.raw_stderr) or "v4l2-ctl exited during startup"
            raise RuntimeError(f"Could not start {self.raw_sensor_name.upper()} raw stream: {details}")

        self.backend = f"raw-{self.raw_sensor_name}"
        eprint(
            f"Camera backend: {self.raw_sensor_name.upper()} raw V4L2 "
            f"{self.raw_device}"
        )
        eprint(
            f"Camera: RG10 {self.args.sensor_width}x{self.args.sensor_height} "
            f"at up to {self.args.sensor_fps:g} fps; processing "
            f"{self.args.width}x{self.args.height} grayscale"
        )

    def open(self) -> None:
        if self.backend == "auto":
            backends = ["imx708", "argus", "v4l2"]
        else:
            backends = [self.backend]

        errors = []
        for backend in backends:
            try:
                if backend == "imx708":
                    self.raw_sensor_name = "imx708"
                    self._open_raw()
                    return
                if backend == "raw":
                    self._open_raw()
                    return
                if backend == "argus":
                    pipeline = self._argus_pipeline()
                    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                    if cap.isOpened():
                        self.backend = backend
                        self.pipeline = pipeline
                        self.capture = cap
                        eprint(f"Camera backend: Argus sensor-id={self.args.sensor_id}")
                        eprint(
                            f"Camera: sensor {self.args.sensor_width}x{self.args.sensor_height} "
                            f"@ {self.args.sensor_fps} fps; processing "
                            f"{self.args.width}x{self.args.height}"
                        )
                        return
                    cap.release()
                    raise RuntimeError("Argus/GStreamer could not open the CSI camera")
                if backend == "v4l2":
                    requested = str(self.args.v4l2_device)
                    if requested.lower() == "auto":
                        raise RuntimeError(
                            "Ordinary V4L2 mode requires an explicit --v4l2-device"
                        )
                    device: object = requested
                    if requested.isdigit():
                        device = int(requested)
                    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.args.width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.args.height)
                    cap.set(cv2.CAP_PROP_FPS, self.args.fps)
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                    if cap.isOpened():
                        self.backend = backend
                        self.capture = cap
                        eprint(f"Camera backend: V4L2 {requested}")
                        return
                    cap.release()
                    raise RuntimeError(f"V4L2 could not open {requested}")
            except Exception as exc:
                self.close()
                errors.append(f"{backend}: {exc}")

        raise RuntimeError("; ".join(errors) or "No camera backend could be opened")

    def _read_exact_raw_frame(self) -> memoryview:
        process = self.raw_process
        buffer = self.raw_buffer
        if process is None or process.stdout is None or buffer is None:
            raise RuntimeError(f"{self.raw_sensor_name.upper()} raw stream is not open")

        view = memoryview(buffer)
        offset = 0
        while offset < self.raw_frame_bytes:
            count = process.stdout.readinto(view[offset:])
            if not count:
                return_code = process.poll()
                details = "; ".join(self.raw_stderr)
                message = f"{self.raw_sensor_name.upper()} stopped returning raw frames"
                if return_code is not None:
                    message += f" (v4l2-ctl exited with {return_code})"
                if details:
                    message += f": {details}"
                raise RuntimeError(message)
            offset += count
        return view

    def _read_imx708(self) -> np.ndarray:
        view = self._read_exact_raw_frame()
        words_per_line = self.raw_bytes_per_line // 2
        raw_with_stride = np.frombuffer(
            view,
            dtype="<u2",
            count=words_per_line * self.args.sensor_height,
        ).reshape((self.args.sensor_height, words_per_line))
        raw = raw_with_stride[:, : self.args.sensor_width]

        # Barcode decoding needs luminance, not colour. Resize the Bayer
        # intensity directly, then apply robust per-frame contrast scaling.
        reduced = cv2.resize(
            raw,
            (self.args.width, self.args.height),
            interpolation=cv2.INTER_AREA,
        )
        sample = reduced[::8, ::8]
        low, high = np.percentile(sample, (1.0, 99.5))
        if high <= low + 1.0:
            return np.clip(reduced >> 2, 0, 255).astype(np.uint8)
        scale = 255.0 / float(high - low)
        gray = np.clip(
            (reduced.astype(np.float32) - float(low)) * scale,
            0,
            255,
        ).astype(np.uint8)
        return np.ascontiguousarray(gray)

    def read(self) -> np.ndarray:
        if self.backend.startswith("raw-"):
            return self._read_imx708()
        if self.capture is None:
            raise RuntimeError("Camera is not open")
        ok, frame = self.capture.read()
        if not ok or frame is None:
            raise RuntimeError("Camera stopped returning frames")
        if frame.ndim == 2:
            return np.ascontiguousarray(frame, dtype=np.uint8)
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def close(self) -> None:
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        process = self.raw_process
        self.raw_process = None
        if process is not None:
            try:
                process.terminate()
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1)
            except OSError:
                pass
            for stream in (process.stdout, process.stderr):
                if stream is not None:
                    try:
                        stream.close()
                    except OSError:
                        pass
        self.raw_buffer = None


class CameraCapture(threading.Thread):
    def __init__(self, source: CameraSource, output: LatestFrameBuffer, args: argparse.Namespace,
                 stop_event: threading.Event, stats: RuntimeStats,
                 preview_socket: Optional[socket.socket]) -> None:
        super().__init__(name="camera-capture", daemon=True)
        self.source = source
        self.output = output
        self.args = args
        self.stop_event = stop_event
        self.stats = stats
        self.preview_socket = preview_socket
        self.error: Optional[BaseException] = None
        x0, y0, x1, y1 = args.roi
        self.left = int(round(args.width * x0))
        self.top = int(round(args.height * y0))
        self.right = int(round(args.width * x1))
        self.bottom = int(round(args.height * y1))
        self.preview_interval = 1.0 / args.embedded_preview_fps
        self.next_preview = 0.0

    def run(self) -> None:
        sequence = 0
        try:
            while not self.stop_event.is_set():
                gray = self.source.read()
                captured_ns = time.monotonic_ns()
                actual_h, actual_w = gray.shape[:2]
                if (actual_w, actual_h) != (self.args.width, self.args.height):
                    gray = cv2.resize(gray, (self.args.width, self.args.height), interpolation=cv2.INTER_AREA)
                now = time.monotonic()
                if self.preview_socket is not None and now >= self.next_preview:
                    self.next_preview = now + self.preview_interval
                    preview = cv2.resize(
                        gray,
                        (self.args.embedded_preview_width, self.args.embedded_preview_height),
                        interpolation=cv2.INTER_AREA,
                    )
                    packet = PREVIEW_HEADER.pack(
                        self.args.embedded_preview_width,
                        self.args.embedded_preview_height,
                    ) + preview.tobytes(order="C")
                    try:
                        self.preview_socket.send(packet)
                    except (BlockingIOError, BrokenPipeError, OSError):
                        pass
                roi = np.ascontiguousarray(
                    gray[self.top:self.bottom, self.left:self.right], dtype=np.uint8
                )
                sequence += 1
                self.stats.captured = sequence
                self.output.publish(FramePacket(roi, sequence, captured_ns))
        except BaseException as exc:
            self.error = exc
            self.stop_event.set()
            self.output.close()


class CsvLogger:
    def __init__(self, path: Optional[str]) -> None:
        self.file = None
        self.writer = None
        if not path:
            return
        target = Path(path).expanduser()
        target.parent.mkdir(parents=True, exist_ok=True)
        needs_header = not target.exists() or target.stat().st_size == 0
        self.file = target.open("a", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        if needs_header:
            self.writer.writerow(["timestamp_utc", "format", "text", "decode_ms", "frame_age_ms"])
            self.file.flush()

    def write(self, timestamp: str, fmt: str, text: str, decode_ms: float, age_ms: float) -> None:
        if self.writer is not None and self.file is not None:
            self.writer.writerow([timestamp, fmt, text, f"{decode_ms:.3f}", f"{age_ms:.3f}"])
            self.file.flush()

    def close(self) -> None:
        if self.file is not None:
            self.file.close()


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def result_format_name(barcode: object) -> str:
    return str(getattr(barcode, "format", "unknown"))


def result_text(barcode: object) -> str:
    return str(getattr(barcode, "text", ""))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="NVIDIA Jetson OpenCV/GStreamer barcode scanner")
    parser.add_argument("--camera-backend", choices=("imx708", "raw", "argus", "v4l2", "auto"), default="imx708")
    parser.add_argument("--sensor-id", type=int, default=0)
    parser.add_argument("--sensor-width", type=int, default=4608)
    parser.add_argument("--sensor-height", type=int, default=2592)
    parser.add_argument("--sensor-fps", type=int, default=8)
    parser.add_argument("--v4l2-device", default="auto")
    parser.add_argument(
        "--raw-sensor-name",
        choices=("imx708", "imx219"),
        default="imx708",
    )
    parser.add_argument("--gstreamer-pipeline", default=None)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=8.0)
    parser.add_argument("--preview", default="none", help=argparse.SUPPRESS)
    parser.add_argument("--preview-width", type=int, default=400, help=argparse.SUPPRESS)
    parser.add_argument("--preview-height", type=int, default=225, help=argparse.SUPPRESS)
    parser.add_argument("--embedded-preview-fd", type=int, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--embedded-preview-width", type=int, default=400, help=argparse.SUPPRESS)
    parser.add_argument("--embedded-preview-height", type=int, default=225, help=argparse.SUPPRESS)
    parser.add_argument("--embedded-preview-fps", type=float, default=8.0, help=argparse.SUPPRESS)
    parser.add_argument("--formats", default="retail")
    parser.add_argument("--roi", type=parse_roi, default=parse_roi("0,0,1,1"))
    parser.add_argument("--rotate", action="store_true")
    parser.add_argument("--downscale", action="store_true")
    parser.add_argument("--invert", action="store_true")
    parser.add_argument("--binarizer", choices=("local", "global"), default="local")
    parser.add_argument("--exposure-us", type=int, default=None)
    parser.add_argument("--gain", type=float, default=4.0)
    parser.add_argument("--cooldown", type=float, default=0.75)
    parser.add_argument("--csv", default=None)
    parser.add_argument("--output-mode", choices=("message", "plain", "json"), default="message")
    parser.add_argument("--plain", action="store_true")
    parser.add_argument("--beep", action="store_true")
    parser.add_argument("--stats-interval", type=float, default=2.0)
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if args.width < 320 or args.height < 240:
        raise ValueError("Processing resolution is too small")
    if args.sensor_width <= 0 or args.sensor_height <= 0 or args.sensor_fps <= 0:
        raise ValueError("Sensor mode dimensions and FPS must be positive")
    if args.embedded_preview_width * args.embedded_preview_height > 180000:
        raise ValueError("Embedded preview is too large")
    if args.embedded_preview_fps <= 0 or args.cooldown < 0:
        raise ValueError("Invalid preview FPS or cooldown")


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        validate_args(args)
        formats = build_formats(args.formats)
    except ValueError as exc:
        parser.error(str(exc))

    stop_event = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop_event.set())
    signal.signal(signal.SIGTERM, lambda *_: stop_event.set())

    preview_socket: Optional[socket.socket] = None
    source = CameraSource(args)
    frame_buffer = LatestFrameBuffer()
    stats = RuntimeStats(time.monotonic())
    capture_thread: Optional[CameraCapture] = None
    csv_logger = CsvLogger(args.csv)

    try:
        if args.embedded_preview_fd is not None:
            preview_socket = socket.socket(fileno=args.embedded_preview_fd)
            preview_socket.setblocking(False)
            eprint(
                f"Embedded preview: {args.embedded_preview_width}x{args.embedded_preview_height} "
                f"at {args.embedded_preview_fps:g} fps"
            )
        source.open()
        decode = make_decoder(args, formats)
        roi_width = int(args.width * (args.roi[2] - args.roi[0]))
        roi_height = int(args.height * (args.roi[3] - args.roi[1]))
        eprint(f"Decode ROI: {roi_width}x{roi_height}; formats: {args.formats}")
        if source.backend.startswith("raw-"):
            eprint(
                f"{source.raw_sensor_name.upper()} mode: raw Bayer luminance; "
                "barcode decoding uses grayscale intensity."
            )

        capture_thread = CameraCapture(source, frame_buffer, args, stop_event, stats, preview_socket)
        capture_thread.start()
        last_sequence = 0
        last_report = time.monotonic()
        last_accepted: dict[str, float] = {}

        while not stop_event.is_set():
            packet = frame_buffer.wait_for_newer(last_sequence, 0.5)
            if packet is None:
                if capture_thread.error is not None:
                    raise RuntimeError(
                        f"Camera capture failed: {capture_thread.error}"
                    ) from capture_thread.error
                continue
            skipped = packet.sequence - last_sequence - 1 if last_sequence else 0
            last_sequence = packet.sequence
            started_ns = time.monotonic_ns()
            barcode = decode(packet.image)
            ended_ns = time.monotonic_ns()
            decode_ms = (ended_ns - started_ns) / 1_000_000
            age_ms = (ended_ns - packet.captured_ns) / 1_000_000
            stats.add_decode(decode_ms, skipped)

            if barcode is not None:
                text = result_text(barcode)
                fmt = result_format_name(barcode)
                now = time.monotonic()
                key = f"{fmt}\0{text}"
                if now - last_accepted.get(key, float("-inf")) >= args.cooldown:
                    last_accepted[key] = now
                    stats.found += 1
                    timestamp = utc_timestamp()
                    mode = "plain" if args.plain else args.output_mode
                    if mode == "plain":
                        print(text, flush=True)
                    elif mode == "message":
                        print(f"BARCODE SCANNED: {text}", flush=True)
                    else:
                        print(json.dumps({
                            "timestamp": timestamp, "format": fmt, "text": text,
                            "decode_ms": round(decode_ms, 3), "frame_age_ms": round(age_ms, 3),
                        }, ensure_ascii=False), flush=True)
                    csv_logger.write(timestamp, fmt, text, decode_ms, age_ms)
                    if args.beep:
                        print("\a", file=sys.stderr, end="", flush=True)

            now = time.monotonic()
            if args.stats_interval > 0 and now - last_report >= args.stats_interval:
                runtime = max(now - stats.started, 1e-9)
                eprint(
                    f"stats capture={stats.captured/runtime:.1f}fps "
                    f"decode={stats.decoded/runtime:.1f}fps "
                    f"decode_ema={stats.decode_ms_ema:.1f}ms "
                    f"stale_frames_discarded={stats.skipped} accepted={stats.found}"
                )
                last_report = now
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        eprint(f"ERROR: {exc}")
        if source.pipeline:
            eprint("GStreamer pipeline:", source.pipeline)
        return 1
    finally:
        stop_event.set()
        frame_buffer.close()
        if capture_thread is not None:
            capture_thread.join(timeout=2)
        source.close()
        if preview_socket is not None:
            try:
                preview_socket.close()
            except OSError:
                pass
        csv_logger.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
