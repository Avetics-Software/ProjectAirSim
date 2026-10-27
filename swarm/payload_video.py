"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Video side of the simulated gimbal payload: RtspStreamer encodes GimbalCam
frames to H.264 with ffmpeg and publishes them to an RTSP server (MediaMTX,
started by run_cesium_sim.sh). The dashboard window is gcs.dashboard.
"""

import shutil
import subprocess
import threading
import time

import cv2
import numpy as np

from projectairsim.utils import projectairsim_log


def image_to_bgr(image_msg):
    data = np.frombuffer(image_msg["data"], dtype=np.uint8)
    return data.reshape(image_msg["height"], image_msg["width"], -1)[:, :, :3]


class RtspStreamer:
    """Publishes the latest frame to `url` at a steady frame rate.

    Frames from the sim arrive unevenly, so a writer thread repeats the newest
    one at `fps` to keep players (QGC, OpenCV, VLC) happy.

    `bitrate_kbps` caps the H.264 bitrate; 0 leaves it to the encoder
    (about 2 Mbit/s with h264_nvenc at 1080p, which blurs small vehicles).
    """

    def __init__(self, url, resolution, fps=30, bitrate_kbps=0):
        if shutil.which("ffmpeg") is None:
            raise RuntimeError("ffmpeg not found; install it or run with --no-rtsp")
        self.url = url
        self.width, self.height = resolution
        self.fps = fps
        self.bitrate_kbps = bitrate_kbps
        self._frame = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._proc = self._start_ffmpeg()
        self._thread = threading.Thread(target=self._write_loop, daemon=True)
        self._thread.start()
        rate = f"{bitrate_kbps} kbit/s" if bitrate_kbps else "encoder's default bitrate"
        projectairsim_log().info(f"GimbalCam RTSP stream: {url} ({rate})")

    def _start_ffmpeg(self):
        encoders = subprocess.run(
            ["ffmpeg", "-hide_banner", "-encoders"], capture_output=True, text=True
        ).stdout
        if "h264_nvenc" in encoders:
            codec = ["-c:v", "h264_nvenc", "-preset", "p1", "-tune", "ll", "-zerolatency", "1"]
        else:
            codec = ["-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency"]
        if self.bitrate_kbps:
            b = self.bitrate_kbps
            codec += ["-b:v", f"{b}k", "-maxrate", f"{b}k", "-bufsize", f"{2 * b}k"]
        cmd = [
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-f", "rawvideo", "-pix_fmt", "bgr24",
            "-s", f"{self.width}x{self.height}", "-framerate", str(self.fps), "-i", "-",
            *codec, "-g", str(self.fps), "-bf", "0", "-pix_fmt", "yuv420p",
            "-f", "rtsp", "-rtsp_transport", "tcp", self.url,
        ]
        return subprocess.Popen(cmd, stdin=subprocess.PIPE)

    def push(self, bgr):
        if bgr.shape[1] != self.width or bgr.shape[0] != self.height:
            bgr = cv2.resize(bgr, (self.width, self.height))
        with self._lock:
            self._frame = np.ascontiguousarray(bgr)

    def _write_loop(self):
        period = 1.0 / self.fps
        next_t = time.monotonic()
        while not self._stop.is_set():
            with self._lock:
                frame = self._frame
            if frame is not None:
                try:
                    self._proc.stdin.write(frame.tobytes())
                except (BrokenPipeError, ValueError):
                    projectairsim_log().warning("RTSP encoder stopped; is MediaMTX running?")
                    return
            next_t += period
            time.sleep(max(0.0, next_t - time.monotonic()))

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2)
        try:
            self._proc.stdin.close()
        except Exception:
            pass
        self._proc.terminate()
