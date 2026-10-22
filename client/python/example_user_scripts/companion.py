"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Link to a companion computer running the X vision pipeline (A5X_RPI_Release,
e.g. a Radxa with the rknn-sim profile) on one of the sim's drones.

The companion reads the drone's GimbalCam over RTSP and its PX4 over MAVLink
(run_cesium_sim.sh --pi-ip), detects and tracks targets, and steers the
gimbal itself. This side does what X's ground station
(map-plot/laptop_client.py) does, over the same UDP text protocol:

  - shows the companion's annotated video (rtsp://<ip>:8554/tracker)
  - CLICK:x,y (in that video's pixels) to pick a target, after BOX:<size>
    for a dragged box (X has no DRAG), CANCEL to drop it
  - PITCH:/YAW: jog, ZOOM:IN/OUT/STOP and GIMBAL:CENTER, since the companion
    owns the gimbal while it runs
  - HEARTBEAT every 0.2 s, so the companion knows a ground station is up
"""

import os
import socket
import threading
import time

import cv2

from projectairsim.utils import projectairsim_log

COMMAND_PORT = 5556  # X's GCS_INPUT_PORT: tracking, control and heartbeat
TRACKER_PORT = 8554  # the companion's MediaMTX
HEARTBEAT_PERIOD_S = 0.2
ZOOM_PULSE_S = 0.4  # one wheel notch zooms this long, then ZOOM:STOP


class CompanionLink:
    def __init__(self, ip, name="companion", port=COMMAND_PORT):
        self.ip = ip
        self.name = name
        self.addr = (ip, port)
        self.tracker_url = f"rtsp://{ip}:{TRACKER_PORT}/tracker"
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.frame = None  # newest tracker frame (BGR)
        self.frame_time = 0.0
        self.tracking = False  # a target has been sent and not cancelled
        self.track_point = None  # (x, y) in tracker pixels
        self._stop = threading.Event()
        self._zoom_timer = None
        self._threads = [
            threading.Thread(target=self._heartbeat_loop, daemon=True),
            threading.Thread(target=self._video_loop, daemon=True),
        ]
        for t in self._threads:
            t.start()
        projectairsim_log().info(f"{name}: X pipeline at {ip}, video {self.tracker_url}")

    # ------------------------------------------------------------------
    # Commands

    def send(self, text):
        try:
            self.sock.sendto(text.encode(), self.addr)
        except OSError as err:
            projectairsim_log().debug(f"{self.name}: send {text!r} failed: {err}")

    def click(self, x, y):
        self.send(f"CLICK:{int(x)},{int(y)}")
        self.tracking, self.track_point = True, (int(x), int(y))

    def drag(self, x1, y1, x2, y2):
        # X tracks a square: set its size from the box, then click the centre.
        # The size sticks for later clicks, as with X's own GCS box slider.
        self.send(f"BOX:{int(max(abs(x2 - x1), abs(y2 - y1)))}")
        self.click((x1 + x2) / 2, (y1 + y2) / 2)

    def cancel(self):
        self.send("CANCEL")
        self.tracking, self.track_point = False, None

    def nudge(self, dx, dy):
        """Move the tracked point by (dx, dy) tracker pixels."""
        if not self.tracking or not self.track_point:
            return
        size = self.frame.shape[1::-1] if self.frame is not None else (640, 368)
        x = min(size[0], max(0, self.track_point[0] + dx))
        y = min(size[1], max(0, self.track_point[1] + dy))
        self.click(x, y)

    def jog(self, axis, direction):
        """axis PITCH (UP/DOWN/STOP) or YAW (LEFT/RIGHT/STOP)."""
        self.send(f"{axis}:{direction}")

    def zoom(self, step):
        """+1 zoom in, -1 out, for a short pulse."""
        self.send("ZOOM:IN" if step > 0 else "ZOOM:OUT")
        if self._zoom_timer:
            self._zoom_timer.cancel()
        self._zoom_timer = threading.Timer(ZOOM_PULSE_S, lambda: self.send("ZOOM:STOP"))
        self._zoom_timer.daemon = True
        self._zoom_timer.start()

    def center_gimbal(self):
        self.send("GIMBAL:CENTER")

    # ------------------------------------------------------------------
    # Background

    def _heartbeat_loop(self):
        while not self._stop.wait(HEARTBEAT_PERIOD_S):
            self.send(f"HEARTBEAT:{int(time.time_ns() / 1e7)}")

    def _video_loop(self):
        # Newest frame only: TCP, no buffering, no reordering delay
        os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS",
                              "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|"
                              "max_delay;0|reorder_queue_size;0")
        while not self._stop.is_set():
            cap = cv2.VideoCapture(self.tracker_url, cv2.CAP_FFMPEG)
            if not cap.isOpened():
                self._stop.wait(2.0)
                continue
            projectairsim_log().info(f"{self.name}: receiving the tracker video")
            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    break
                self.frame, self.frame_time = frame, time.monotonic()
            cap.release()
            if not self._stop.is_set():
                projectairsim_log().warning(f"{self.name}: tracker video lost; retrying")
                self._stop.wait(2.0)

    @property
    def video_ok(self):
        return self.frame is not None and time.monotonic() - self.frame_time < 2.0

    def close(self):
        self._stop.set()
        self.sock.close()
