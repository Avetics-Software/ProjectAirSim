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

import collections
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
    """One dashboard window: camera panes, a minimap and a status panel drawn
    with OpenCV, and (with add_controls) a control bar for the swarm below.

      +----------------------------+-------------+
      |                            | small camera|
      |  main camera (GimbalCam    +-------------+
      |  or ChaseCam)              |             |
      +----------------------------+   Minimap   |
      |          Status            |             |
      +----------------------------+-------------+
      | drones, view, flight buttons, command box, messages |

    Controls (with the camera/map area focused):
      W / S          tilt up / down          A / D   pan left / right
      wheel          zooms the pane under the pointer: optical zoom on the
                     GimbalCam, digital zoom on the ChaseCam, scale on the map
      + / -          zoom the camera         [ / ]   zoom the map out / in
      C              centre (level, ahead)   X       look straight down
      V or click the small camera            swap the two cameras
      click the map  set the selected drone's target for "Go to target", or
                     with "Draw area" on, add a corner of the search area
      L              map labels on/off       1-9     show and steer that drone

    With a companion computer (the X pipeline) on the selected drone the main
    pane is always Tracker, its annotated video, and View (or V) picks the
    camera in the small pane. In Tracker, click a target to track
    it or drag a box around it; right-click or Esc cancels; the arrow keys
    nudge the tracked point; G or C centres the gimbal (X has no look-down).
    W/A/S/D and the zoom keys and wheel then go to the companion, which owns
    the gimbal. X ignores them while it tracks, so W/A/S/D, G and C drop the
    track first.

    It is a Tk window: OpenCV's Qt windows zoom the whole image on the mouse
    wheel, which can't be turned off.
    """

    WINDOW = "ProjectAirSim"
    PAN_STEP_DEG = 3.0
    ZOOM_STEP_PERCENT = 5.0
    # Pane rectangles (x, y, w, h) on a 1440x810 canvas
    SIZE = (1440, 810)
    PANES = {
        "Main": (0, 0, 960, 540),
        "Small": (960, 0, 480, 270),
        "Minimap": (960, 270, 480, 540),
        "Status": (0, 540, 960, 270),
    }
    BORDER = (90, 90, 90)
    CHASE_ZOOM_MAX = 8.0
    MAX_FPS = 30.0
    IDLE_REDRAW_S = 0.5  # redraw this often anyway, for the status and minimap
    LOG_LINES = 200
    # Control bar colours
    BG, FG, FIELD, ACCENT = "#1e1f22", "#e6e6e6", "#2b2d31", "#3d6fb6"

    def __init__(self, on_gimbal, on_zoom, initial_pitch=-45.0, minimap=None,
                 on_select=None):
        self.on_gimbal = on_gimbal  # called with (pitch_deg, yaw_deg)
        self.on_zoom = on_zoom  # called with a zoom delta in percent
        self.on_select = on_select  # called with a drone number for keys 1-9
        self.on_command = None  # called with a console command line (controls)
        self.title = ""  # which drone the camera panes show
        self.minimap = minimap
        self.pitch = initial_pitch
        self.yaw = 0.0
        self.zoom_text = ""
        self.status_lines = []  # set by the caller
        self.main_cam = "GimbalCam"  # shown in the big pane; the other is small
        # With a companion the big pane is always its Tracker video and this one
        # of GimbalCam/ChaseCam goes in the small pane
        self.side_cam = "GimbalCam"
        self.companion = None  # CompanionLink of the selected drone, if it has one
        self._fits = {}  # pane -> (x0, y0, scale, frame w, frame h) of the last draw
        self._drag = None  # (x0, y0, x1, y1) canvas px while dragging a track box
        self._jog = {}  # axis -> key held, for jogging the companion's gimbal
        self._jog_release = {}  # axis -> pending Tk after() id
        self.chase_zoom = 1.0  # digital zoom of the ChaseCam
        self.selected = 1  # drone number shown and steered
        self.targets = {}  # drone number -> (lat, lon) clicked on the map
        self.area = []  # corners (lat, lon) of the search area being drawn
        self._frames = {}
        self._lock = threading.Lock()
        self._messages = collections.deque()  # log lines waiting to be shown
        self._view = (1.0, 0, 0)  # scale and offset from canvas to the label
        self._seq = 0  # bumped by new frames and input; redraw when it changes
        self._drawn_seq = -1
        self._last_draw = 0.0
        self._log = None

        self.root = tk.Tk()
        self.root.title(self.WINDOW)
        self.root.geometry(f"{self.SIZE[0]}x{self.SIZE[1]}")
        self.root.configure(bg="black")
        self.root.option_add("*Font", ("DejaVu Sans", 10))
        self.root.protocol("WM_DELETE_WINDOW", self.root.iconify)  # stop with Ctrl+C
        self.label = tk.Label(self.root, bg="black", bd=0, highlightthickness=0,
                              takefocus=True)
        self.label.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self._photo = None
        self.root.bind("<KeyPress>", self._on_key)
        self.root.bind("<KeyRelease>", self._on_key_release)
        self.label.bind("<Button-1>", self._on_click)
        self.label.bind("<B1-Motion>", self._on_drag)
        self.label.bind("<ButtonRelease-1>", self._on_release)
        self.label.bind("<Button-3>", lambda e: self._cancel_track())
        for button, step in (("<Button-4>", 1), ("<Button-5>", -1)):  # X11 wheel
            self.label.bind(button, lambda e, s=step: self._on_wheel(e.x, e.y, s))
        self.label.bind(
            "<MouseWheel>", lambda e: self._on_wheel(e.x, e.y, 1 if e.delta > 0 else -1)
        )

    # ------------------------------------------------------------------
    # Control bar

    def add_controls(self, drone_names, on_command, default_alt=30.0):
        """Add the control bar: drone and camera choice, flight buttons for the
        selected drone or all of them, a command box and recent messages."""
        self.on_command = on_command
        self.root.geometry(f"{self.SIZE[0]}x{self.SIZE[1] + 222}")
        bar = tk.Frame(self.root, bg=self.BG)
        bar.pack(side=tk.BOTTOM, fill=tk.X, before=self.label)

        def label(parent, text):
            return tk.Label(parent, text=text, bg=self.BG, fg=self.FG)

        def button(parent, text, command):
            return tk.Button(parent, text=text, command=command, bg=self.FIELD, fg=self.FG,
                             activebackground=self.ACCENT, activeforeground="white",
                             relief=tk.FLAT, padx=10, pady=3, highlightthickness=0)

        def radio(parent, text, var, value, command):
            return tk.Radiobutton(parent, text=text, variable=var, value=value,
                                  command=command, indicatoron=False, bg=self.FIELD,
                                  fg=self.FG, selectcolor=self.ACCENT, activebackground=self.ACCENT,
                                  relief=tk.FLAT, padx=10, pady=3, highlightthickness=0)

        # Row 1: which drone and camera to show, and who flight buttons command
        row = tk.Frame(bar, bg=self.BG)
        row.pack(fill=tk.X, padx=8, pady=(6, 2))
        label(row, "Drone").pack(side=tk.LEFT)
        self._drone_var = tk.IntVar(value=1)
        for n, name in enumerate(drone_names, start=1):
            radio(row, name, self._drone_var, n,
                  lambda n=n: self.on_select and self.on_select(n)).pack(side=tk.LEFT, padx=2)
        label(row, "   View").pack(side=tk.LEFT)
        self._view_var = tk.StringVar(value=self.main_cam)
        self._view_buttons = {}
        for cam in ("GimbalCam", "ChaseCam"):
            b = radio(row, cam, self._view_var, cam, lambda cam=cam: self._set_main(cam))
            b.pack(side=tk.LEFT, padx=2)
            self._view_buttons[cam] = b
        self._all_var = tk.BooleanVar(value=False)
        if len(drone_names) > 1:
            tk.Checkbutton(row, text="Buttons command all drones", variable=self._all_var,
                           bg=self.BG, fg=self.FG, selectcolor=self.FIELD,
                           activebackground=self.BG, activeforeground=self.FG,
                           highlightthickness=0).pack(side=tk.LEFT, padx=16)

        # Row 2: flight buttons
        row = tk.Frame(bar, bg=self.BG)
        row.pack(fill=tk.X, padx=8, pady=2)
        label(row, "Height (m)").pack(side=tk.LEFT)
        self._alt_var = tk.StringVar(value=f"{default_alt:g}")
        tk.Entry(row, textvariable=self._alt_var, width=5, bg=self.FIELD, fg=self.FG,
                 insertbackground=self.FG, relief=tk.FLAT).pack(side=tk.LEFT, padx=(4, 10))
        label(row, "Heading (deg)").pack(side=tk.LEFT)
        self._heading_var = tk.StringVar(value="")
        tk.Entry(row, textvariable=self._heading_var, width=5, bg=self.FIELD, fg=self.FG,
                 insertbackground=self.FG, relief=tk.FLAT).pack(side=tk.LEFT, padx=(4, 10))
        for text, action in (("Take off", "takeoff"), ("Go to target", "goto"),
                             ("Set heading", "heading"),
                             ("Hold", "hold"), ("Land", "land"), ("Return home", "rtl")):
            button(row, text, lambda a=action: self._fly(a)).pack(side=tk.LEFT, padx=2)
        self._target_text = tk.StringVar()
        label(row, "").pack(side=tk.LEFT, padx=6)
        tk.Label(row, textvariable=self._target_text, bg=self.BG, fg="#ff8080").pack(side=tk.LEFT)
        self._show_target()

        # Row 3: grid search of an area drawn on the map
        row = tk.Frame(bar, bg=self.BG)
        row.pack(fill=tk.X, padx=8, pady=2)
        label(row, "Search area").pack(side=tk.LEFT)
        self._drawing = tk.BooleanVar(value=False)
        tk.Checkbutton(row, text="Draw area (click corners on the map)", variable=self._drawing,
                       indicatoron=False, bg=self.FIELD, fg=self.FG, selectcolor=self.ACCENT,
                       activebackground=self.ACCENT, relief=tk.FLAT, padx=10, pady=3,
                       highlightthickness=0).pack(side=tk.LEFT, padx=(6, 2))
        button(row, "Undo corner", self._undo_corner).pack(side=tk.LEFT, padx=2)
        button(row, "Clear area", self._clear_area).pack(side=tk.LEFT, padx=2)
        label(row, "  Pass spacing (m)").pack(side=tk.LEFT)
        self._spacing_var = tk.StringVar(value="40")
        tk.Entry(row, textvariable=self._spacing_var, width=5, bg=self.FIELD, fg=self.FG,
                 insertbackground=self.FG, relief=tk.FLAT).pack(side=tk.LEFT, padx=4)
        label(row, "Drones").pack(side=tk.LEFT)
        self._search_drones = tk.StringVar(value="all")
        tk.Entry(row, textvariable=self._search_drones, width=8, bg=self.FIELD, fg=self.FG,
                 insertbackground=self.FG, relief=tk.FLAT).pack(side=tk.LEFT, padx=4)
        button(row, "Search area", self._search_area).pack(side=tk.LEFT, padx=6)
        self._area_text = tk.StringVar(value="no area")
        tk.Label(row, textvariable=self._area_text, bg=self.BG, fg="#e0e070").pack(side=tk.LEFT)

        # Row 4: command box, same commands as the terminal
        row = tk.Frame(bar, bg=self.BG)
        row.pack(fill=tk.X, padx=8, pady=2)
        label(row, "Command").pack(side=tk.LEFT)
        self._entry = tk.Entry(row, bg=self.FIELD, fg=self.FG, insertbackground=self.FG,
                               relief=tk.FLAT)
        self._entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        self._entry.bind("<Return>", lambda e: self._send_entry())
        self._entry.bind("<Up>", lambda e: self._history_step(-1))
        self._entry.bind("<Down>", lambda e: self._history_step(1))
        self._history, self._history_pos = [], 0
        button(row, "Send", self._send_entry).pack(side=tk.LEFT)
        button(row, "Help", lambda: self.on_command("help")).pack(side=tk.LEFT, padx=2)

        # Row 5: recent messages
        self._log = tk.Text(bar, height=5, bg="#141517", fg="#c8c8c8", relief=tk.FLAT,
                            font=("DejaVu Sans Mono", 9), state=tk.DISABLED, takefocus=False)
        self._log.pack(fill=tk.X, padx=8, pady=(2, 8))

    @staticmethod
    def drone_name(number):
        return f"Drone{number}"

    def set_selected(self, number):
        """Show drone `number`'s choice in the control bar and its target."""
        self.selected = number
        if hasattr(self, "_drone_var"):
            self._drone_var.set(number)
        if self.minimap:
            self.minimap.select(self.drone_name(number))
        self._show_target()
        self._seq += 1

    def _show_target(self):
        if not hasattr(self, "_target_text"):
            return
        target = self.targets.get(self.selected)
        name = self.drone_name(self.selected)
        self._target_text.set(f"{name} target: {target[0]:.6f}, {target[1]:.6f}" if target
                              else f"{name} target: click the map")

    def log(self, text):
        """Show a message in the control bar (any thread)."""
        self._messages.append(text)

    def _fly(self, action):
        who = "all" if self._all_var.get() else str(self.selected)
        alt = self._alt_var.get().strip() or "30"
        if action == "takeoff":
            self.on_command(f"{who} takeoff {alt}")
        elif action == "goto":
            target = self.targets.get(self.selected)
            if not target:
                self.log(f"Click the map to set {self.drone_name(self.selected)}'s target first")
                return
            heading = self._heading_var.get().strip()  # blank: keep the heading
            self.on_command(f"{who} goto {target[0]:.6f} {target[1]:.6f} {alt} {heading}".strip())
        elif action == "heading":
            heading = self._heading_var.get().strip()
            if not heading:
                self.log("Type a heading in degrees from north (0 = N, 90 = E) first")
                return
            self.on_command(f"{who} heading {heading}")
        else:
            self.on_command(f"{who} {action}")

    def _update_area(self):
        if self.minimap:
            self.minimap.set_area(self.area)
        n = len(self.area)
        self._area_text.set("no area" if n == 0 else f"{n} corner{'s' if n != 1 else ''}"
                            + ("" if n >= 3 else " (need 3)"))
        self._seq += 1

    def _undo_corner(self):
        if self.area:
            self.area.pop()
            self._update_area()

    def _clear_area(self):
        self.area.clear()
        self._update_area()

    def _search_area(self):
        if len(self.area) < 3:
            self.log("Draw the search area first: turn on Draw area and click 3+ corners")
            return
        drones = self._search_drones.get().strip().replace(" ", "") or "all"
        alt = self._alt_var.get().strip() or "30"
        spacing = self._spacing_var.get().strip() or "40"
        corners = ";".join(f"{lat:.6f},{lon:.6f}" for lat, lon in self.area)
        self._drawing.set(False)
        self.on_command(f"{drones} searcharea {alt} {spacing} {corners}")

    def _send_entry(self):
        line = self._entry.get().strip()
        if not line:
            return
        self._history.append(line)
        self._history_pos = len(self._history)
        self._entry.delete(0, tk.END)
        self.log(f"> {line}")
        self.on_command(line)

    def _history_step(self, step):
        if not self._history:
            return
        self._history_pos = max(0, min(len(self._history), self._history_pos + step))
        self._entry.delete(0, tk.END)
        if self._history_pos < len(self._history):
            self._entry.insert(0, self._history[self._history_pos])

    def _flush_log(self):
        if not self._log or not self._messages:
            return
        self._log.configure(state=tk.NORMAL)
        while self._messages:
            self._log.insert(tk.END, self._messages.popleft().rstrip() + "\n")
        lines = int(self._log.index("end-1c").split(".")[0])
        if lines > self.LOG_LINES:
            self._log.delete("1.0", f"{lines - self.LOG_LINES}.0")
        self._log.see(tk.END)
        self._log.configure(state=tk.DISABLED)

    # ------------------------------------------------------------------
    # Panes and input

    def receive(self, name, bgr):
        """Store a frame (any thread)."""
        with self._lock:
            self._frames[name] = bgr
            self._seq += 1

    def set_companion(self, companion):
        """The selected drone's CompanionLink, or None."""
        self.companion = companion
        if hasattr(self, "_view_var"):
            self._view_var.set(self.side_cam if companion else self.main_cam)
        with self._lock:
            self._frames.pop("Tracker", None)
        self._seq += 1

    def _set_main(self, cam):
        """The View choice: the big pane, or with a companion the small one."""
        if self.companion:
            self.side_cam = cam
        else:
            self.main_cam = cam
        if hasattr(self, "_view_var"):
            self._view_var.set(cam)
        self._seq += 1

    def _pane_at(self, x, y):
        for name, (px, py, pw, ph) in self.PANES.items():
            if px <= x < px + pw and py <= y < py + ph:
                return name
        return None

    def _canvas_xy(self, wx, wy):
        scale, ox, oy = self._view
        return (wx - ox) / scale, (wy - oy) / scale

    def _cam_in(self, pane):
        if self.companion:
            return {"Main": "Tracker", "Small": self.side_cam}.get(pane)
        if pane == "Main":
            return self.main_cam
        if pane == "Small":
            return "ChaseCam" if self.main_cam == "GimbalCam" else "GimbalCam"
        return None

    def _swap_cameras(self):
        """V or a click on the small pane: swap GimbalCam and ChaseCam."""
        shown = self.side_cam if self.companion else self.main_cam
        self._set_main("ChaseCam" if shown == "GimbalCam" else "GimbalCam")

    # ------------------------------------------------------------------
    # Tracking with a companion

    def _tracker_xy(self, x, y):
        """Canvas px over the Tracker pane -> tracker video px, or None."""
        pane = next((p for p in ("Main", "Small") if self._cam_in(p) == "Tracker"), None)
        if pane is None or pane not in self._fits or self._pane_at(x, y) != pane:
            return None
        x0, y0, s, fw, fh = self._fits[pane]
        fx, fy = (x - x0) / s, (y - y0) / s
        if not (0 <= fx < fw and 0 <= fy < fh):
            return None
        return int(fx), int(fy)

    def _on_drag(self, event):
        if self._drag:
            x, y = self._canvas_xy(event.x, event.y)
            self._drag = (self._drag[0], self._drag[1], x, y)
            self._seq += 1

    def _on_release(self, event):
        if not self._drag or not self.companion:
            self._drag = None
            return
        x0, y0, x1, y1 = self._drag
        self._drag = None
        self._seq += 1
        a, b = self._tracker_xy(x0, y0), self._tracker_xy(x1, y1)
        if a is None:
            return
        if b is None or abs(b[0] - a[0]) < 5 or abs(b[1] - a[1]) < 5:
            self.companion.click(*a)
            self.log(f"Track: CLICK {a[0]},{a[1]}")
        else:
            (ax, bx), (ay, by) = sorted((a[0], b[0])), sorted((a[1], b[1]))
            self.companion.drag(ax, ay, bx, by)
            self.log(f"Track: box {ax},{ay} to {bx},{by}")

    def _cancel_track(self):
        if self.companion:
            self.companion.cancel()
            self.log("Track cancelled")
            self._seq += 1

    def _jog_press(self, axis, direction):
        """Hold-to-jog the companion's gimbal; X11 key repeat sends release/press
        pairs, so a release only stops the jog if no press follows quickly."""
        pending = self._jog_release.pop(axis, None)
        if pending:
            self.root.after_cancel(pending)
        self._take_over()
        if self._jog.get(axis) != direction:
            self._jog[axis] = direction
            self.companion.jog(axis, direction)

    def _take_over(self):
        """X ignores manual gimbal commands while it tracks, so steering by hand
        drops the track first."""
        if self.companion and self.companion.tracking:
            self._cancel_track()

    def _on_key_release(self, event):
        if not self.companion or isinstance(event.widget, tk.Entry):
            return
        axis = {"w": "PITCH", "s": "PITCH", "a": "YAW", "d": "YAW"}.get(
            (event.keysym or "").lower())
        if axis and axis in self._jog:
            def stop(axis=axis):
                self._jog_release.pop(axis, None)
                if self._jog.pop(axis, None) and self.companion:
                    self.companion.jog(axis, "STOP")
            self._jog_release[axis] = self.root.after(120, stop)

    def _on_click(self, event):
        self.label.focus_set()  # keys steer the gimbal again after typing
        self._seq += 1
        x, y = self._canvas_xy(event.x, event.y)
        pane = self._pane_at(x, y)
        if self.companion and self._tracker_xy(x, y) is not None:
            self._drag = (x, y, x, y)  # a click or the start of a box
        elif pane == "Small":
            self._swap_cameras()
        elif pane == "Minimap" and self.minimap:
            px, py, pw, ph = self.PANES["Minimap"]
            geo = self.minimap.pixel_to_geo(x - px, y - py, pw, ph)
            if geo and hasattr(self, "_drawing") and self._drawing.get():
                self.area.append(geo)
                self._update_area()
            elif geo:
                self.targets[self.selected] = geo
                self.minimap.set_picked(self.drone_name(self.selected), geo)
                self._show_target()

    def _on_wheel(self, wx, wy, step):
        self._seq += 1
        pane = self._pane_at(*self._canvas_xy(wx, wy))
        cam = self._cam_in(pane)
        if cam in ("GimbalCam", "Tracker") and self.companion:
            self.companion.zoom(step)
        elif cam == "GimbalCam":
            self.on_zoom(step * self.ZOOM_STEP_PERCENT)
        elif cam == "ChaseCam":
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
        return ox, oy, s, iw, ih

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

    def _cam_title(self, cam):
        if cam == "Tracker":
            state = "tracking" if self.companion and self.companion.tracking else "click or drag to track"
            return f"{self.title} Tracker (companion {self.companion.ip if self.companion else ''})  {state}"
        if cam == "GimbalCam":
            return (f"{self.title} GimbalCam  pitch {self.pitch:+.0f}  yaw {self.yaw:+.0f}  "
                    f"{self.zoom_text}")
        return f"{self.title} ChaseCam" + (
            f"  {self.chase_zoom:.1f}x digital" if self.chase_zoom > 1 else "")

    def _compose(self, frames):
        canvas = np.zeros((self.SIZE[1], self.SIZE[0], 3), np.uint8)
        for pane, scale in (("Main", 0.6), ("Small", 0.5)):
            cam = self._cam_in(pane)
            rect = self.PANES[pane]
            if cam in frames:
                frame = frames[cam]
                if cam == "ChaseCam" and self.chase_zoom > 1.0:
                    frame = self._crop_centre(frame, self.chase_zoom)
                self._fits[pane] = self._fit(canvas, frame, rect)
            else:
                self._fits.pop(pane, None)
                waiting = ("waiting for the companion's video..." if cam == "Tracker"
                           else "waiting for camera...")
                self._text(canvas, waiting, (rect[0] + 20, rect[1] + rect[3] // 2))
            self._text(canvas, self._cam_title(cam), (rect[0] + 10, rect[1] + 22), scale)

        if self._drag:
            x0, y0, x1, y1 = (int(v) for v in self._drag)
            if abs(x1 - x0) >= 5 or abs(y1 - y0) >= 5:
                cv2.rectangle(canvas, (x0, y0), (x1, y1), (0, 200, 255), 2)

        m = self.PANES["Minimap"]
        if self.minimap:
            canvas[m[1]:m[1] + m[3], m[0]:m[0] + m[2]] = self.minimap.render(m[2], m[3])

        x, y, w, h = self.PANES["Status"]
        canvas[y:y + h, x:x + w] = (24, 24, 24)
        help_lines = [
            "Gimbal: W/S tilt  A/D pan  C centre  X down   Camera zoom: +/-   "
            "Map: [ ]  L labels  click = target",
            "Mouse wheel zooms the pane under the pointer.  V or click the small camera: swap",
        ]
        if self.companion:
            help_lines.append("Companion: in Tracker click / drag a box to track, right-click or "
                              "Esc cancel, arrows nudge, G centre gimbal")
        for i, line in enumerate(self.status_lines + [""] + help_lines):
            self._text(canvas, line, (x + 12, y + 26 + 22 * i), 0.5,
                       (170, 170, 170) if line in help_lines else (235, 235, 235), box=False)

        for px, py, pw, ph in self.PANES.values():
            cv2.rectangle(canvas, (px, py), (px + pw - 1, py + ph - 1), self.BORDER, 1)
        return canvas

    def update(self):
        """Draw the dashboard and handle input; call every few ms."""
        self._flush_log()
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

        # Fit the canvas to the label, keeping its aspect ratio
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
        if isinstance(event.widget, tk.Entry):
            return  # typing a command or height, not steering
        self._seq += 1
        ch = event.char.lower() if event.char else ""
        step = self.PAN_STEP_DEG
        if self.companion and self._companion_key(ch or (event.keysym or "").lower(), event.keysym):
            return
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
        elif ch == "v":
            self._swap_cameras()
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
        elif self.on_select and ch.isdigit() and ch != "0":
            self.on_select(int(ch))

    def _companion_key(self, ch, keysym):
        """Keys that go to the companion when the drone has one; True if used."""
        c = self.companion
        jogs = {"w": ("PITCH", "UP"), "s": ("PITCH", "DOWN"),
                "a": ("YAW", "LEFT"), "d": ("YAW", "RIGHT")}
        nudges = {"Up": (0, -8), "Down": (0, 8), "Left": (-8, 0), "Right": (8, 0)}
        if ch in jogs:
            self._jog_press(*jogs[ch])
        elif ch in ("+", "="):
            c.zoom(1)
        elif ch in ("-", "_"):
            c.zoom(-1)
        elif ch in ("g", "c"):
            self._take_over()
            c.center_gimbal()
        elif ch == "x":
            # Steering the gimbal from here would take PX4's gimbal control
            # away from X, which only claims it when it starts
            self.log("Look-down isn't available while the companion owns the gimbal")
        elif keysym == "Escape":
            self._cancel_track()
        elif keysym in nudges:
            c.nudge(*nudges[keysym])
        else:
            return False
        return True

    def close(self):
        try:
            self.root.destroy()
        except tk.TclError:
            pass
