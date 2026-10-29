"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Link to a companion computer running the X vision pipeline (A5X_RPI_Release,
e.g. a Radxa) on one of the drones: in the sim an rknn-sim-* profile, on a real
drone its flight profile.

The companion reads the drone's gimbal camera and its PX4 over MAVLink,
detects and tracks targets, and steers the gimbal itself. This side does what X's ground station
(map-plot/laptop_client.py) does, over the same UDP text protocol:

  - shows the companion's annotated video (rtsp://<ip>:8554/tracker)
  - CLICK:x,y (in that video's pixels) to pick a target, after BOX:<size>
    for a dragged box (X has no DRAG), CANCEL to drop it
  - PITCH:/YAW: jog, ZOOM:IN/OUT/STOP and GIMBAL:CENTER, since the companion
    owns the gimbal while it runs
  - STRIKE:START / STRIKE:STOP to arm and release X's strike, in which X flies
    the drone at the tracked target (if its profile enables drone control)
  - HEARTBEAT every 0.2 s, so the companion knows a ground station is up
"""

import socket
import threading
import time

from .log import log
from .video import VideoStream

COMMAND_PORT = 5556  # X's GCS_INPUT_PORT: tracking, control and heartbeat
TRACKER_PORT = 8554  # the companion's MediaMTX
HEARTBEAT_PERIOD_S = 0.2
ZOOM_PULSE_S = 0.4  # one wheel notch zooms this long, then ZOOM:STOP (controls: setting)
# X's STRIKE_COMMAND. Commands go to one companion's IP, so the plain keyword
# never arms another drone's X.
STRIKE_COMMAND = "STRIKE"


class CompanionLink:
    def __init__(self, ip, name="companion", port=COMMAND_PORT, zoom_pulse_s=ZOOM_PULSE_S,
                 strike_command=STRIKE_COMMAND):
        self.ip = ip
        self.strike_command = strike_command
        self.zoom_pulse_s = zoom_pulse_s
        self.name = name
        self.addr = (ip, port)
        self.tracker_url = f"rtsp://{ip}:{TRACKER_PORT}/tracker"
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.video = VideoStream(self.tracker_url, f"{name} tracker")
        self.tracking = False  # a target has been sent and not cancelled
        self.mission_active = False  # block operator gimbal commands during a mission
        self.track_point = None  # (x, y) in tracker pixels
        self.striking = False  # strike armed and not released
        self._stop = threading.Event()
        self._zoom_timer = None
        threading.Thread(target=self._heartbeat_loop, daemon=True).start()
        log.info(f"{name}: X pipeline at {ip}, video {self.tracker_url}")

    # ------------------------------------------------------------------
    # Commands

    def send(self, text):
        if self.mission_active and text.startswith(("CLICK:", "BOX:", "PITCH:", "YAW:", "GIMBAL:")):
            return
        if not text.startswith("HEARTBEAT"):
            log.debug(f"{self.name} <- {text}")  # what the operator sent, for the log
        try:
            self.sock.sendto(text.encode(), self.addr)
        except OSError as err:
            log.debug(f"{self.name}: send {text!r} failed: {err}")

    def click(self, x, y):
        if self.mission_active:
            return
        self.tracking, self.track_point = True, (int(x), int(y))
        # One UDP datagram is not reliable (the 15:19 auto-lock lost its CLICK
        # and X never initialised the tracker). Repeats are idempotent in X:
        # the same coordinates are deduplicated after the first init.
        for _ in range(3):
            self.send(f"CLICK:{int(x)},{int(y)}")
            time.sleep(0.05)

    def drag(self, x1, y1, x2, y2):
        # X tracks a square: set its size from the box, then click the centre.
        # The size sticks for later clicks, as with X's own GCS box slider.
        box = int(max(abs(x2 - x1), abs(y2 - y1)))
        for _ in range(2):
            self.send(f"BOX:{box}")
            time.sleep(0.03)
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
        self._zoom_timer = threading.Timer(self.zoom_pulse_s, lambda: self.send("ZOOM:STOP"))
        self._zoom_timer.daemon = True
        self._zoom_timer.start()

    def center_gimbal(self):
        self.send("GIMBAL:CENTER")

    def strike(self, enabled):
        """Arm (True) or release (False) X's strike; False if refused.

        Arming is refused while a mission holds the camera; releasing always
        goes out, even if this side thinks it isn't armed.
        """
        if enabled and self.mission_active:
            log.info(f"{self.name}: not striking while a mission controls the camera")
            return False
        self.send(f"{self.strike_command}:{'START' if enabled else 'STOP'}")
        self.striking = bool(enabled)
        log.info(f"{self.name}: strike {'ARMED' if enabled else 'released'}")
        return True

    def set_mission_active(self, active):
        """Release a strike, stop a selected track and held jog before handing the
        camera to PX4.

        X itself must use MAVLINK_STAND_DOWN_IN_MISSION so autonomous tracking
        and re-claims stop even if the ground station disconnects.
        """
        if active == self.mission_active:
            return
        if active:
            if self.striking:
                self.strike(False)
            self.cancel()
            self.jog("PITCH", "STOP")
            self.jog("YAW", "STOP")
        self.mission_active = active

    # ------------------------------------------------------------------
    # Background

    def _heartbeat_loop(self):
        while not self._stop.wait(HEARTBEAT_PERIOD_S):
            self.send(f"HEARTBEAT:{int(time.time_ns() / 1e7)}")

    @property
    def frame(self):
        return self.video.frame

    @property
    def frame_time(self):
        return self.video.frame_time

    @property
    def video_ok(self):
        return self.video.ok

    def close(self):
        if self.striking:
            self.strike(False)
        self._stop.set()
        self.video.close()
        self.sock.close()
