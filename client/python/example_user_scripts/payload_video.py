"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Video side of the simulated gimbal payload:

- RtspStreamer: encodes GimbalCam frames to H.264 with ffmpeg and publishes them
  to an RTSP server (MediaMTX, started by run_cesium_sim.sh)
- CameraWindows: one dashboard window with the gimbal and chase cameras, a
  minimap and a status panel. Each pane zooms on its own with the mouse wheel;
  the keyboard steers the gimbal
"""

import shutil
import subprocess
import threading
import time

import tkinter as tk

import cv2
import numpy as np
from PIL import Image, ImageTk

from projectairsim.utils import projectairsim_log


def image_to_bgr(image_msg):
    data = np.frombuffer(image_msg["data"], dtype=np.uint8)
    return data.reshape(image_msg["height"], image_msg["width"], -1)[:, :, :3]


class RtspStreamer:
    """Publishes the latest frame to `url` at a steady frame rate.

    Frames from the sim arrive unevenly, so a writer thread repeats the newest
    one at `fps` to keep players (QGC, OpenCV, VLC) happy.
    """

    def __init__(self, url, resolution, fps=30):
        if shutil.which("ffmpeg") is None:
            raise RuntimeError("ffmpeg not found; install it or run with --no-rtsp")
        self.url = url
        self.width, self.height = resolution
        self.fps = fps
        self._frame = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._proc = self._start_ffmpeg()
        self._thread = threading.Thread(target=self._write_loop, daemon=True)
        self._thread.start()
        projectairsim_log().info(f"GimbalCam RTSP stream: {url}")

    def _start_ffmpeg(self):
        encoders = subprocess.run(
            ["ffmpeg", "-hide_banner", "-encoders"], capture_output=True, text=True
        ).stdout
        if "h264_nvenc" in encoders:
            codec = ["-c:v", "h264_nvenc", "-preset", "p1", "-tune", "ll", "-zerolatency", "1"]
        else:
            codec = ["-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency"]
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


class CameraWindows:
    """One dashboard window with tiled panes: GimbalCam, ChaseCam, a minimap
    and a status panel. All OpenCV calls stay on the caller's thread.

      +----------------------------+-------------+
      |                            |  ChaseCam   |
      |         GimbalCam          +-------------+
      |                            |             |
      +----------------------------+   Minimap   |
      |          Status            |             |
      +----------------------------+-------------+

    Controls (with the window focused):
      W / S          tilt up / down          A / D   pan left / right
      wheel          zooms the pane under the pointer: optical zoom on
                     GimbalCam, digital zoom on ChaseCam, scale on Minimap
      + / -          zoom the camera         [ / ]   zoom the map out / in
      C              centre (level, ahead)   X       look straight down
      L              map labels on/off

    It is a Tk window: OpenCV's Qt windows zoom the whole image on the mouse
    wheel, which can't be turned off.
    """

    WINDOW = "ProjectAirSim"
    PAN_STEP_DEG = 3.0
    ZOOM_STEP_PERCENT = 5.0
    # Pane rectangles (x, y, w, h) on a 1440x810 canvas
    SIZE = (1440, 810)
    PANES = {
        "GimbalCam": (0, 0, 960, 540),
        "ChaseCam": (960, 0, 480, 270),
        "Minimap": (960, 270, 480, 540),
        "Status": (0, 540, 960, 270),
    }
    BORDER = (90, 90, 90)
    CHASE_ZOOM_MAX = 8.0
    MAX_FPS = 30.0
    IDLE_REDRAW_S = 0.5  # redraw this often anyway, for the status and minimap

    def __init__(self, on_gimbal, on_zoom, initial_pitch=-45.0, minimap=None):
        self.on_gimbal = on_gimbal  # called with (pitch_deg, yaw_deg)
        self.on_zoom = on_zoom  # called with a zoom delta in percent
        self.minimap = minimap
        self.pitch = initial_pitch
        self.yaw = 0.0
        self.zoom_text = ""
        self.status_lines = []  # set by the caller
        self.chase_zoom = 1.0  # digital zoom of the ChaseCam pane
        self._frames = {}
        self._lock = threading.Lock()
        self._view = (1.0, 0, 0)  # scale and offset from canvas to the window
        self._seq = 0  # bumped by new frames and input; redraw when it changes
        self._drawn_seq = -1
        self._last_draw = 0.0

        self.root = tk.Tk()
        self.root.title(self.WINDOW)
        self.root.geometry(f"{self.SIZE[0]}x{self.SIZE[1]}")
        self.root.configure(bg="black")
        self.root.protocol("WM_DELETE_WINDOW", self.root.iconify)  # stop with Ctrl+C
        self.label = tk.Label(self.root, bg="black", bd=0, highlightthickness=0)
        self.label.pack(fill=tk.BOTH, expand=True)
        self._photo = None
        self.root.bind("<KeyPress>", self._on_key)
        for button, step in (("<Button-4>", 1), ("<Button-5>", -1)):  # X11 wheel
            self.label.bind(button, lambda e, s=step: self._on_wheel(e.x, e.y, s))
        self.label.bind(
            "<MouseWheel>", lambda e: self._on_wheel(e.x, e.y, 1 if e.delta > 0 else -1)
        )

    def receive(self, name, bgr):
        """Store a frame (any thread)."""
        with self._lock:
            self._frames[name] = bgr
            self._seq += 1

    def _pane_at(self, x, y):
        for name, (px, py, pw, ph) in self.PANES.items():
            if px <= x < px + pw and py <= y < py + ph:
                return name
        return None

    def _on_wheel(self, wx, wy, step):
        self._seq += 1
        scale, ox, oy = self._view
        pane = self._pane_at((wx - ox) / scale, (wy - oy) / scale)
        if pane == "GimbalCam":
            self.on_zoom(step * self.ZOOM_STEP_PERCENT)
        elif pane == "ChaseCam":
            self.chase_zoom = min(self.CHASE_ZOOM_MAX, max(1.0, self.chase_zoom * 1.25 ** step))
        elif pane == "Minimap" and self.minimap:
            self.minimap.zoom(step)

    def _move(self, d_pitch, d_yaw):
        self.pitch = max(-90.0, min(30.0, self.pitch + d_pitch))
        self.yaw = (self.yaw + d_yaw + 180.0) % 360.0 - 180.0
        self.on_gimbal(self.pitch, self.yaw)

    @staticmethod
    def _fit(canvas, img, rect):
        """Draw img into rect, keeping its aspect ratio."""
        x, y, w, h = rect
        ih, iw = img.shape[:2]
        s = min(w / iw, h / ih)
        nw, nh = max(1, int(iw * s)), max(1, int(ih * s))
        ox, oy = x + (w - nw) // 2, y + (h - nh) // 2
        canvas[oy:oy + nh, ox:ox + nw] = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)

    @staticmethod
    def _crop_centre(img, zoom):
        h, w = img.shape[:2]
        ch, cw = int(h / zoom), int(w / zoom)
        y, x = (h - ch) // 2, (w - cw) // 2
        return img[y:y + ch, x:x + cw]

    @staticmethod
    def _text(canvas, text, org, scale=0.5, colour=(255, 255, 255), box=True):
        """Text on a dark box, so it reads over any image."""
        font = cv2.FONT_HERSHEY_SIMPLEX
        if box:
            (w, h), base = cv2.getTextSize(text, font, scale, 1)
            x, y = org
            cv2.rectangle(canvas, (x - 4, y - h - 4), (x + w + 4, y + base + 2), (0, 0, 0), -1)
        cv2.putText(canvas, text, org, font, scale, colour, 1, cv2.LINE_AA)

    def _compose(self, frames):
        canvas = np.zeros((self.SIZE[1], self.SIZE[0], 3), np.uint8)
        for name in ("GimbalCam", "ChaseCam"):
            rect = self.PANES[name]
            if name in frames:
                frame = frames[name]
                if name == "ChaseCam" and self.chase_zoom > 1.0:
                    frame = self._crop_centre(frame, self.chase_zoom)
                self._fit(canvas, frame, rect)
            else:
                self._text(canvas, "waiting for camera...", (rect[0] + 20, rect[1] + rect[3] // 2))
        g = self.PANES["GimbalCam"]
        self._text(canvas, f"GimbalCam  pitch {self.pitch:+.0f}  yaw {self.yaw:+.0f}  {self.zoom_text}",
                   (g[0] + 10, g[1] + 22), 0.6)
        c = self.PANES["ChaseCam"]
        chase_title = "ChaseCam" + (f"  {self.chase_zoom:.1f}x digital" if self.chase_zoom > 1 else "")
        self._text(canvas, chase_title, (c[0] + 8, c[1] + 18))

        m = self.PANES["Minimap"]
        if self.minimap:
            canvas[m[1]:m[1] + m[3], m[0]:m[0] + m[2]] = self.minimap.render(m[2], m[3])

        x, y, w, h = self.PANES["Status"]
        canvas[y:y + h, x:x + w] = (24, 24, 24)
        help_lines = [
            "Gimbal: W/S tilt  A/D pan  C centre  X down     Camera zoom: +/-     "
            "Map: [ ]  L labels",
            "Mouse wheel zooms the pane under the pointer",
        ]
        for i, line in enumerate(self.status_lines + [""] + help_lines):
            self._text(canvas, line, (x + 12, y + 26 + 22 * i), 0.5,
                       (170, 170, 170) if line in help_lines else (235, 235, 235), box=False)

        for px, py, pw, ph in self.PANES.values():
            cv2.rectangle(canvas, (px, py), (px + pw - 1, py + ph - 1), self.BORDER, 1)
        return canvas

    def update(self):
        """Draw the dashboard and handle input; call every few ms."""
        now = time.monotonic()
        changed = self._seq != self._drawn_seq or now - self._last_draw > self.IDLE_REDRAW_S
        if not changed or now - self._last_draw < 1.0 / self.MAX_FPS:
            self.root.update()  # input only
            return
        with self._lock:
            frames = dict(self._frames)
            self._drawn_seq = self._seq
        self._last_draw = now
        canvas = self._compose(frames)

        # Fit the canvas to the window, keeping its aspect ratio
        ww, wh = max(self.label.winfo_width(), 2), max(self.label.winfo_height(), 2)
        scale = min(ww / self.SIZE[0], wh / self.SIZE[1])
        size = (max(1, int(self.SIZE[0] * scale)), max(1, int(self.SIZE[1] * scale)))
        self._view = (scale, (ww - size[0]) // 2, (wh - size[1]) // 2)
        if size != self.SIZE:
            canvas = cv2.resize(canvas, size, interpolation=cv2.INTER_AREA)
        image = Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
        if self._photo is None or (self._photo.width(), self._photo.height()) != size:
            self._photo = ImageTk.PhotoImage(image)
            self.label.configure(image=self._photo)
        else:
            self._photo.paste(image)  # much cheaper than a new PhotoImage
        self.root.update()

    def _on_key(self, event):
        self._seq += 1
        ch = event.char.lower() if event.char else ""
        step = self.PAN_STEP_DEG
        if ch == "w":
            self._move(step, 0)
        elif ch == "s":
            self._move(-step, 0)
        elif ch == "a":
            self._move(0, -step)
        elif ch == "d":
            self._move(0, step)
        elif ch == "c":
            self._move(-self.pitch, -self.yaw)
        elif ch == "x":
            self._move(-90.0 - self.pitch, -self.yaw)
        elif ch in ("+", "="):
            self.on_zoom(self.ZOOM_STEP_PERCENT)
        elif ch in ("-", "_"):
            self.on_zoom(-self.ZOOM_STEP_PERCENT)
        elif self.minimap and ch == "]":
            self.minimap.zoom(1)
        elif self.minimap and ch == "[":
            self.minimap.zoom(-1)
        elif self.minimap and ch == "l":
            self.minimap.show_labels = not self.minimap.show_labels

    def close(self):
        try:
            self.root.destroy()
        except tk.TclError:
            pass
