"""Read-only Wuji Glove skeleton recorder.

This module subscribes only to the glove's ``hand_skeleton`` data stream.
It contains no ROS client and never commands a robot hand.
"""

from __future__ import annotations

import json
import math
import queue
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from tkinter import ttk
from typing import Any, Sequence

import numpy as np


DEFAULT_ENDPOINT = "192.168.10.151:50001"
RECORDINGS_DIR = Path(__file__).resolve().parent / "recordings"


def extract_keypoints(frame: object) -> tuple[tuple[float, float, float], ...]:
    """Return the 21 finite XYZ points contained in an SDK skeleton frame."""
    try:
        joints = frame.joints  # type: ignore[attr-defined]
    except AttributeError as error:
        raise ValueError("frame has no joints") from error

    points: list[tuple[float, float, float]] = []
    for joint in joints:
        try:
            xyz = tuple(float(value) for value in joint.pose.position)
        except (AttributeError, TypeError, ValueError) as error:
            raise ValueError("invalid keypoint") from error
        if len(xyz) != 3 or not all(math.isfinite(value) for value in xyz):
            raise ValueError("invalid keypoint")
        points.append(xyz)

    if len(points) != 21:
        raise ValueError("expected 21 keypoints")
    return tuple(points)


@dataclass
class Recording:
    endpoint: str
    serial_number: str
    hand_side: str
    timestamps_ns: list[int] = field(default_factory=list)
    frames: list[tuple[tuple[float, float, float], ...]] = field(default_factory=list)

    def append(self, timestamp_ns: int, points: Sequence[Sequence[float]]) -> None:
        if len(points) != 21:
            raise ValueError("expected 21 keypoints")

        validated: list[tuple[float, float, float]] = []
        for point in points:
            try:
                xyz = tuple(float(value) for value in point)
            except (TypeError, ValueError) as error:
                raise ValueError("invalid keypoint") from error
            if len(xyz) != 3 or not all(math.isfinite(value) for value in xyz):
                raise ValueError("invalid keypoint")
            validated.append(xyz)

        self.timestamps_ns.append(int(timestamp_ns))
        self.frames.append(tuple(validated))

    def save(self, output_dir: Path) -> Path:
        if not self.frames:
            raise ValueError("no valid frames to save")

        output_dir.mkdir(parents=True, exist_ok=True)
        path = output_dir / f"glove_{datetime.now():%Y%m%d_%H%M%S}.npz"
        metadata = {
            "endpoint": self.endpoint,
            "serial_number": self.serial_number,
            "hand_side": self.hand_side,
            "frame_count": len(self.frames),
            "coordinate_unit": "m",
        }
        np.savez_compressed(
            path,
            timestamps_ns=np.asarray(self.timestamps_ns, dtype=np.int64),
            keypoints_m=np.asarray(self.frames, dtype=np.float64),
            metadata=np.asarray(json.dumps(metadata, ensure_ascii=False)),
        )
        return path


def _resource_value(resource: Any, fallback: str) -> str:
    """Read an SDK resource that may expose either ``get`` or a direct value."""
    try:
        value = resource.get() if hasattr(resource, "get") else resource
    except Exception:
        return fallback
    return str(value) if value is not None else fallback


class GloveSession:
    """Own the SDK connection and a read-only skeleton subscription."""

    def __init__(self, endpoint: str) -> None:
        self.endpoint = endpoint
        self.manager: Any | None = None
        self.glove: Any | None = None
        self.subscriber: Any | None = None
        self.serial_number = "unknown"
        self.hand_side = "unknown"

    def connect(self) -> None:
        from wuji_sdk import SdkManager

        self.manager = SdkManager.instance()
        self.glove = self.manager.connect(
            address=self.endpoint, device_name="glove_recorder"
        )
        self.serial_number = _resource_value(self.glove.sn(), "unknown")
        self.hand_side = _resource_value(self.glove.hand_side(), "unknown")
        self.subscriber = self.glove.hand_skeleton().subscribe()

    def recv_points(self) -> tuple[tuple[float, float, float], ...] | None:
        if self.subscriber is None:
            raise RuntimeError("glove is not connected")
        frame = self.subscriber.recv()
        return None if frame is None else extract_keypoints(frame)

    def close(self) -> None:
        if self.manager is not None and self.glove is not None:
            try:
                self.manager.disconnect(self.glove)
            finally:
                self.glove = None
                self.subscriber = None


class RecorderApp:
    """Small Tkinter UI for starting and saving one recording at a time."""

    def __init__(self, root: tk.Tk, endpoint: str = DEFAULT_ENDPOINT) -> None:
        self.root = root
        self.endpoint = endpoint
        self.session = GloveSession(endpoint)
        self.recording: Recording | None = None
        self.stop_event = threading.Event()
        self.worker: threading.Thread | None = None
        self.events: queue.SimpleQueue[tuple[str, str]] = queue.SimpleQueue()
        self.started_ns: int | None = None

        root.title("Wuji Glove 骨骼采集")
        root.resizable(False, False)
        frame = ttk.Frame(root, padding=16)
        frame.grid()

        self.status = tk.StringVar(value="正在连接手套…")
        self.count = tk.StringVar(value="已记录：0 帧")
        self.duration = tk.StringVar(value="时长：0.0 秒")
        ttk.Label(frame, textvariable=self.status, width=64).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 8)
        )
        ttk.Label(frame, textvariable=self.count).grid(row=1, column=0, sticky="w")
        ttk.Label(frame, textvariable=self.duration).grid(row=1, column=1, sticky="e")
        self.start_button = ttk.Button(
            frame, text="开始记录", command=self.start_recording, state="disabled"
        )
        self.start_button.grid(row=2, column=0, padx=(0, 8), pady=(16, 0), sticky="ew")
        self.stop_button = ttk.Button(
            frame, text="停止并保存", command=self.stop_recording, state="disabled"
        )
        self.stop_button.grid(row=2, column=1, pady=(16, 0), sticky="ew")
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(100, self._refresh)

    def connect(self) -> None:
        try:
            self.session.connect()
        except Exception as error:
            self.status.set(f"连接失败：{error}")
            return
        self.status.set(
            f"已连接：{self.session.hand_side} glove（{self.session.serial_number}）"
        )
        self.start_button.configure(state="normal")

    def start_recording(self) -> None:
        if self.session.subscriber is None:
            self.status.set("手套未连接，不能开始记录")
            return
        self.recording = Recording(
            self.endpoint, self.session.serial_number, self.session.hand_side
        )
        self.started_ns = time.monotonic_ns()
        self.stop_event.clear()
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.status.set("正在记录手套骨骼数据…")
        self.worker = threading.Thread(target=self._capture_loop, daemon=True)
        self.worker.start()

    def _capture_loop(self) -> None:
        try:
            while self.stop_event.is_set() is False:
                points = self.session.recv_points()
                if points is None or self.stop_event.is_set():
                    continue
                if self.recording is not None:
                    self.recording.append(time.monotonic_ns(), points)
        except Exception as error:
            self.events.put(("error", str(error)))

    def stop_recording(self) -> None:
        if self.recording is None:
            return
        self.stop_event.set()
        if self.worker is not None:
            self.worker.join(timeout=1.0)
        try:
            path = self.recording.save(RECORDINGS_DIR)
        except ValueError as error:
            self.status.set(f"未保存：{error}")
        else:
            self.status.set(f"已保存：{path}")
        self.recording = None
        self.started_ns = None
        self.start_button.configure(state="normal")
        self.stop_button.configure(state="disabled")

    def _refresh(self) -> None:
        while not self.events.empty():
            kind, message = self.events.get()
            if kind == "error":
                self.stop_event.set()
                self.status.set(f"采集停止：{message}")
                self.stop_button.configure(state="disabled")
                self.start_button.configure(state="normal")

        if self.recording is not None:
            self.count.set(f"已记录：{len(self.recording.frames)} 帧")
            if self.started_ns is not None:
                elapsed = (time.monotonic_ns() - self.started_ns) / 1_000_000_000
                self.duration.set(f"时长：{elapsed:.1f} 秒")
        self.root.after(100, self._refresh)

    def close(self) -> None:
        if self.recording is not None:
            self.stop_event.set()
        self.session.close()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    app = RecorderApp(root)
    app.connect()
    root.mainloop()


if __name__ == "__main__":
    main()
