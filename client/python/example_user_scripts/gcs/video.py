"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

VideoStream: reads an RTSP stream on a thread and keeps only the newest frame,
reconnecting when the stream drops. Used for X's tracker video and, on real
drones, each drone's gimbal camera.
"""

import os
import threading
import time

# Newest frame only: TCP, no buffering, no reordering delay. Set before OpenCV
# opens its first stream.
os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS",
                      "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|"
                      "max_delay;0|reorder_queue_size;0")
# A stream that isn't up yet (e.g. a companion still starting) is retried every
# 2 s; keep FFmpeg's and OpenCV's per-attempt complaints out of the terminal
os.environ.setdefault("OPENCV_FFMPEG_LOGLEVEL", "-8")
os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")

import cv2  # noqa: E402

from .log import log  # noqa: E402


class VideoStream:
    def __init__(self, url, name):
        self.url = url
        self.name = name
        self.frame = None  # newest frame (BGR)
        self.frame_time = 0.0  # time.monotonic() it arrived
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name=f"video {name}", daemon=True)
        self._thread.start()

    @property
    def ok(self):
        return self.frame is not None and time.monotonic() - self.frame_time < 2.0

    def close(self):
        self._stop.set()

    def _loop(self):
        while not self._stop.is_set():
            cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
            if not cap.isOpened():
                self._stop.wait(2.0)
                continue
            log.info(f"{self.name}: receiving {self.url}")
            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    break
                self.frame, self.frame_time = frame, time.monotonic()
            cap.release()
            if not self._stop.is_set():
                log.warning(f"{self.name}: video lost; retrying")
                self._stop.wait(2.0)
