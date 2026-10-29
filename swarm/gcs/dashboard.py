"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

CameraWindows: the ground station's dashboard window. Camera panes (the
selected drone's gimbal camera and chase view, or with a companion computer
its tracker video), a minimap and a status panel, drawn with OpenCV into a Tk
window, and a control bar for the swarm. Each pane zooms on its own with the
mouse wheel; the keyboard steers the gimbal.
"""

import collections
import glob
import os
import threading
import time

import tkinter as tk
from tkinter import messagebox, ttk

import cv2
import numpy as np
from PIL import Image, ImageTk


class HoverTip:
    """A small note shown under a widget after the pointer rests on it."""

    DELAY_MS = 500

    def __init__(self, widget, text):
        self.widget, self.text = widget, text
        self._after = None
        self._tip = None
        widget.bind("<Enter>", self._schedule, add="+")
        widget.bind("<Leave>", self._hide, add="+")
        widget.bind("<ButtonPress>", self._hide, add="+")

    def _schedule(self, _event):
        self._after = self.widget.after(self.DELAY_MS, self._show)

    def _show(self):
        x = self.widget.winfo_rootx()
        y = self.widget.winfo_rooty() - 28  # above: the control bar is at the bottom
        self._tip = tk.Toplevel(self.widget)
        self._tip.wm_overrideredirect(True)
        self._tip.wm_geometry(f"+{x}+{y}")
        tk.Label(self._tip, text=self.text, bg="#2b2d31", fg="#e6e6e6", relief=tk.SOLID,
                 borderwidth=1, padx=6, pady=2, font=("DejaVu Sans", 9)).pack()

    def _hide(self, _event=None):
        if self._after:
            self.widget.after_cancel(self._after)
            self._after = None
        if self._tip:
            self._tip.destroy()
            self._tip = None


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
    DANGER = "#a52a2a"
    DRAWER_WIDTH = 360

    def __init__(self, on_gimbal, on_zoom, initial_pitch=-45.0, minimap=None,
                 on_select=None):
        self.on_gimbal = on_gimbal  # called with (pitch_deg, yaw_deg)
        self.on_zoom = on_zoom  # called with a zoom delta in percent
        self.on_select = on_select  # called with a drone number for keys 1-9
        self.on_command = None  # called with a console command line (controls)
        self.title = ""  # which drone the camera panes show
        self.autopilot = "PX4"  # the selected drone's autopilot
        self.minimap = minimap
        self.pitch = initial_pitch
        self.yaw = 0.0
        self.zoom_text = ""
        # Set by the caller: each a str, or (str, BGR colour) to colour the line
        self.status_lines = []
        self.alert_line = None  # (text, BGR colour) shown at the top of the status panel
        self.show_help = False  # F1: the shortcuts sheet over the main camera
        self.fence = []  # geofence corners (lat, lon) being drawn
        self.fence_drawing = False  # map clicks add fence corners (safety panel)
        self.on_fence_changed = None  # called after a fence corner is added
        self.on_f9 = None  # F9: the sim's fault controls, if any
        self.gimbal_actual = None  # (pitch, yaw) the gimbal reports, for the title
        self.yaw_lock = False  # the selected drone's gimbal holds a compass heading
        self.on_yaw_lock = None  # Y: toggle yaw lock (drones without a companion)
        self.on_key_log = None  # called with "key X down/up", for the debug log
        self.on_target_picked = None  # called with (lat, lon) when a map target is set
        self.on_fence_drawing_changed = None  # called when fence drawing starts or stops
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

    def add_controls(self, drone_names, on_command, default_alt=30.0, missions_dir=None):
        """Add the control bar: drone and camera choice, flight buttons for the
        selected drone or all of them, a command box and recent messages."""
        self.on_command = on_command
        self._drone_names = list(drone_names)
        self.root.geometry(f"{self.SIZE[0]}x{self.SIZE[1] + 256}")
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
            b = radio(row, name, self._drone_var, n,
                      lambda n=n: self.on_select and self.on_select(n))
            b.pack(side=tk.LEFT, padx=2)
            if n <= 9:
                HoverTip(b, f"Show and steer {name} (key {n})")
        label(row, "   View").pack(side=tk.LEFT)
        self._view_var = tk.StringVar(value=self.main_cam)
        self._view_buttons = {}
        for cam in ("GimbalCam", "ChaseCam"):
            b = radio(row, cam, self._view_var, cam, lambda cam=cam: self._set_main(cam))
            b.pack(side=tk.LEFT, padx=2)
            HoverTip(b, "V swaps GimbalCam and ChaseCam;  F1 lists every shortcut")
            self._view_buttons[cam] = b
        self._all_var = tk.BooleanVar(value=False)
        if len(drone_names) > 1:
            tk.Checkbutton(row, text="Buttons command all drones", variable=self._all_var,
                           bg=self.BG, fg=self.FG, selectcolor=self.FIELD,
                           activebackground=self.BG, activeforeground=self.FG,
                           highlightthickness=0).pack(side=tk.LEFT, padx=16)
        # Emergency buttons, always visible at the right, and the safety drawer
        for text, command, tip in (
                ("RTL ALL", self._rtl_all, "Send every drone home; asks first (Ctrl+R)"),
                ("HOLD ALL", self._hold_all, "Hold every drone where it is (Ctrl+H)")):
            b = tk.Button(row, text=text, command=command, bg=self.DANGER, fg="white",
                          activebackground="#d04040", activeforeground="white", relief=tk.FLAT,
                          padx=12, pady=3, highlightthickness=0,
                          font=("DejaVu Sans", 10, "bold"))
            b.pack(side=tk.RIGHT, padx=2)
            HoverTip(b, tip)
        self._safety_button = button(row, "Safety (F2)", self.toggle_drawer)
        self._safety_button.pack(side=tk.RIGHT, padx=(2, 12))
        HoverTip(self._safety_button, "Alerts, drone health, responses, geofence (F2)")
        # Shown by add_settings_drawer() (the sim's settings)
        self._settings_button = button(row, "Sim (F3)", self.toggle_settings_drawer)
        HoverTip(self._settings_button, "Sim settings: every GimbalCam's resolution and image "
                                        "effects, the time of day (F3)")
        # The drawer docks at the right of the camera canvas when open
        self.drawer = tk.Frame(self.root, bg=self.BG, width=self.DRAWER_WIDTH)
        self.drawer.pack_propagate(False)
        self.drawer_open = False
        # The sim settings drawer (F3) docks in the same place; one is open at a time
        self.settings_drawer = tk.Frame(self.root, bg=self.BG, width=self.DRAWER_WIDTH)
        self.settings_drawer.pack_propagate(False)
        self.settings_drawer_open = False
        self.on_settings_drawer = None  # called when the settings drawer opens
        self.root.bind_all("<Control-h>", lambda e: self._hold_all())
        self.root.bind_all("<Control-r>", lambda e: self._rtl_all())
        self.root.bind_all("<F1>", lambda e: self._toggle_help())
        self.root.bind_all("<F2>", lambda e: self.toggle_drawer())
        self.root.bind_all("<F3>", lambda e: self.on_settings_drawer and self.toggle_settings_drawer())
        self.root.bind_all("<F9>", lambda e: self.on_f9 and self.on_f9())

        # Row 2: flight buttons
        row = tk.Frame(bar, bg=self.BG)
        row.pack(fill=tk.X, padx=8, pady=2)
        label(row, "Height (m)").pack(side=tk.LEFT)
        self._alt_var = tk.StringVar(value=f"{default_alt:g}")
        tk.Entry(row, textvariable=self._alt_var, width=5, bg=self.FIELD, fg=self.FG,
                 insertbackground=self.FG, relief=tk.FLAT).pack(side=tk.LEFT, padx=(4, 10))
        label(row, "Heading (deg)").pack(side=tk.LEFT)
        self._heading_var = tk.StringVar(value="")
        # Typed or picked on the compass, the heading shows on the compass
        self._heading_var.trace_add("write", lambda *_: self._show_heading_pick())
        tk.Entry(row, textvariable=self._heading_var, width=5, bg=self.FIELD, fg=self.FG,
                 insertbackground=self.FG, relief=tk.FLAT).pack(side=tk.LEFT, padx=(4, 10))
        for text, action in (("Take off", "takeoff"), ("Go to target", "goto"),
                             ("Set heading", "heading"),
                             ("Hold", "hold"), ("Land", "land"), ("Return home", "rtl"),
                             ("Disarm", "disarm")):
            button(row, text, lambda a=action: self._fly(a)).pack(side=tk.LEFT, padx=2)
        self._target_text = tk.StringVar()
        label(row, "").pack(side=tk.LEFT, padx=6)
        tk.Label(row, textvariable=self._target_text, bg=self.BG, fg="#ff8080").pack(side=tk.LEFT)
        self._show_target()

        # Row 2b: a mission file, loaded, uploaded and started (the console's `mission` commands)
        row = tk.Frame(bar, bg=self.BG)
        row.pack(fill=tk.X, padx=8, pady=2)
        label(row, "Mission").pack(side=tk.LEFT)
        self._missions_dir = missions_dir
        self._mission_var = tk.StringVar()
        self._mission_box = ttk.Combobox(row, textvariable=self._mission_var, width=24,
                                         postcommand=self._refresh_missions)
        self._mission_box.pack(side=tk.LEFT, padx=(6, 4))
        self._refresh_missions()
        for text, sub, tip in (("Load", "load", "Read the mission file (mission load NAME)"),
                               ("Upload", "upload", "Send it to every drone in the file"),
                               ("Start", "start", "Arm and fly it"),
                               ("Stop", "stop", "Stop the mission; the drones hold")):
            b = button(row, text, lambda s=sub: self._mission_command(s))
            b.pack(side=tk.LEFT, padx=2)
            HoverTip(b, tip)

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
        self._entry.bind("<Control-h>", lambda e: self._hold_all())  # not Tk's backspace
        self._entry.bind("<Control-r>", lambda e: self._rtl_all())
        self._history, self._history_pos = [], 0
        button(row, "Send", self._send_entry).pack(side=tk.LEFT)
        button(row, "Help", lambda: self.on_command("help")).pack(side=tk.LEFT, padx=2)

        # Row 5: recent messages
        self._log = tk.Text(bar, height=5, bg="#141517", fg="#c8c8c8", relief=tk.FLAT,
                            font=("DejaVu Sans Mono", 9), state=tk.DISABLED, takefocus=False)
        self._log.pack(fill=tk.X, padx=8, pady=(2, 8))

    # ------------------------------------------------------------------
    # Missions

    def _refresh_missions(self):
        names = []
        if self._missions_dir:
            names = sorted(os.path.splitext(os.path.basename(f))[0]
                           for f in glob.glob(os.path.join(self._missions_dir, "*.y*ml")))
        self._mission_box["values"] = names

    def _mission_command(self, sub):
        name = self._mission_var.get().strip()
        if sub == "load":
            if not name:
                return self.log("mission: choose a mission to load")
            line = f"mission load {name}"
        else:
            line = f"mission {sub}"
        self.log(line)
        self.on_command(line)

    # ------------------------------------------------------------------
    # Safety

    def _hold_all(self):
        self.log("HOLD ALL")
        self.on_command("all hold")
        return "break"  # Ctrl+H: not a backspace in the command box

    def _rtl_all(self):
        if messagebox.askyesno("Return all", "Send every drone home (RTL)?", parent=self.root):
            self.log("RTL ALL")
            self.on_command("all rtl")
        return "break"

    def toggle_drawer(self, show=None):
        """Open or close the safety drawer at the right of the cameras."""
        show = not self.drawer_open if show is None else show
        if show and not self.drawer_open:
            self.toggle_settings_drawer(False)
            self.drawer.pack(side=tk.RIGHT, fill=tk.Y, before=self.label)
        elif not show and self.drawer_open:
            self.drawer.pack_forget()
            self.set_fence_drawing(False)  # map clicks set targets again
        self.drawer_open = show
        self._safety_button.configure(bg=self.ACCENT if show else self.FIELD)
        self._seq += 1

    def add_settings_drawer(self, on_open):
        """Offer the sim settings drawer (F3): its button, and on_open() called
        each time it opens, to show the current settings."""
        self.on_settings_drawer = on_open
        self._settings_button.pack(side=tk.RIGHT, padx=2, after=self._safety_button)

    def toggle_settings_drawer(self, show=None):
        """Open or close the sim settings drawer at the right of the cameras."""
        if not self.on_settings_drawer:
            return
        show = not self.settings_drawer_open if show is None else show
        if show and not self.settings_drawer_open:
            self.toggle_drawer(False)
            self.settings_drawer.pack(side=tk.RIGHT, fill=tk.Y, before=self.label)
            self.settings_drawer_open = True
            self.on_settings_drawer()
        elif not show and self.settings_drawer_open:
            self.settings_drawer.pack_forget()
        self.settings_drawer_open = show
        self._settings_button.configure(bg=self.ACCENT if show else self.FIELD)
        self._seq += 1

    def _toggle_help(self):
        self.show_help = not self.show_help
        self._seq += 1

    def set_fence_drawing(self, on):
        """Map clicks add geofence corners (on) or set targets (off)."""
        if on == self.fence_drawing:
            return
        self.fence_drawing = on
        if on and hasattr(self, "_drawing"):
            self._drawing.set(False)  # not the search area at the same time
        self.log("Geofence drawing: click corners on the map; Esc when done" if on
                 else "Geofence drawing off: map clicks set targets")
        if self.on_fence_drawing_changed:
            self.on_fence_drawing_changed()
        self._seq += 1

    def _drawing_mode(self):
        """What a map click adds now, if not a target."""
        if self.fence_drawing:
            return "geofence"
        if hasattr(self, "_drawing") and self._drawing.get():
            return "search area"
        return None

    def set_fence(self, corners):
        self.fence = list(corners)
        if self.minimap:
            self.minimap.set_fence(self.fence)
        self._seq += 1

    def drone_name(self, number):
        names = getattr(self, "_drone_names", [])
        return names[number - 1] if 0 < number <= len(names) else f"Drone{number}"

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

    def _show_heading_pick(self):
        if not self.minimap:
            return
        try:
            text = self._heading_var.get().strip()
            self.minimap.set_heading_pick(float(text) % 360.0 if text else None)
        except ValueError:
            self.minimap.set_heading_pick(None)
        self._seq += 1

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
                self.log("Click the compass, or type a heading in degrees from north "
                         "(0 = N, 90 = E), first")
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
        if self.on_key_log:
            self.on_key_log(f"key {event.keysym} up")
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
            bearing = self.minimap.compass_heading(x - px, y - py)
            if bearing is not None and hasattr(self, "_heading_var"):
                # On the compass: pick a heading (to the nearest 5 deg); it is
                # sent with Set heading or Go to target
                heading = round(bearing / 5.0) * 5 % 360
                self._heading_var.set(f"{heading:.0f}")
                self.log(f"Heading {heading:03.0f} deg picked: press Set heading, or Go to "
                         "target to turn there on arrival")
                return
            geo = self.minimap.pixel_to_geo(x - px, y - py, pw, ph)
            if geo and self.fence_drawing:
                self.set_fence(self.fence + [geo])
                n = len(self.fence)
                self.log(f"Geofence corner {n}" + (" (need 3)" if n < 3 else
                         ": Esc when done, then Upload in the Geofence tab"))
                if self.on_fence_changed:
                    self.on_fence_changed()
            elif geo and hasattr(self, "_drawing") and self._drawing.get():
                self.area.append(geo)
                self._update_area()
            elif geo:
                self.targets[self.selected] = geo
                self.minimap.set_picked(self.drone_name(self.selected), geo)
                self._show_target()
                if self.on_target_picked:
                    self.on_target_picked(geo)

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
            state = ("mission controls camera" if self.companion and self.companion.mission_active
                     else "tracking" if self.companion and self.companion.tracking
                     else "click or drag to track")
            return f"{self.title} Tracker (companion {self.companion.ip if self.companion else ''})  {state}"
        if cam == "GimbalCam":
            via = "X" if self.companion else self.autopilot  # who steers this camera
            pitch, yaw = self.gimbal_actual or (self.pitch, self.yaw)
            mode = "" if self.companion else (
                "  yaw lock (Y)" if self.yaw_lock else "  yaw follow (Y)")
            return (f"{self.title} GimbalCam  pitch {pitch:+.0f}  yaw {yaw:+.0f}{mode}  "
                    f"{self.zoom_text}  (via {via})")
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
            mode = self._drawing_mode()
            if mode:
                # Map clicks don't set targets now: say so, on the map itself
                text = f"Drawing {mode}: click corners, Esc when done"
                cv2.rectangle(canvas, (m[0] + 1, m[1] + 1), (m[0] + m[2] - 2, m[1] + 26),
                              (0, 90, 160) if mode == "geofence" else (90, 90, 0), -1)
                self._text(canvas, text, (m[0] + 10, m[1] + 19), 0.5, box=False)

        x, y, w, h = self.PANES["Status"]
        canvas[y:y + h, x:x + w] = (24, 24, 24)
        lines = []  # (text, colour)
        if self.alert_line:
            text, colour = self.alert_line
            cv2.rectangle(canvas, (x + 1, y + 1), (x + w - 2, y + 30), (30, 30, 50), -1)
            self._text(canvas, text, (x + 12, y + 22), 0.55, colour, box=False)
            top = y + 56
        else:
            top = y + 26
        for line in self.status_lines:
            lines.append(line if isinstance(line, tuple) else (line, (235, 235, 235)))
        grey = (150, 150, 150)
        lines += [("", grey), ("F1 shortcuts   F2 safety and alerts   "
                               + ("F3 sim settings   " if self.on_settings_drawer else "")
                               + "Ctrl+H hold all   Ctrl+R return all", grey)]
        for i, (line, colour) in enumerate(lines):
            if top + 22 * i > y + h - 6:
                break
            self._text(canvas, line, (x + 12, top + 22 * i), 0.5, colour, box=False)

        for px, py, pw, ph in self.PANES.values():
            cv2.rectangle(canvas, (px, py), (px + pw - 1, py + ph - 1), self.BORDER, 1)
        if self.show_help:
            self._draw_shortcuts(canvas)
        return canvas

    def _shortcut_groups(self):
        """The keys that work now, for the selected drone, grouped."""
        if self.companion:
            camera = [("Click / drag in Tracker", "track a target / a box"),
                      ("Esc, right-click", "stop tracking"),
                      ("Arrow keys", "nudge the tracked point"),
                      ("W/S  A/D (hold)", "tilt / pan the gimbal (through X)"),
                      ("G or C", "centre the gimbal"),
                      ("Wheel, + / -", "zoom"),
                      ("V, click small view", "GimbalCam / ChaseCam beside Tracker")]
        else:
            camera = [("W/S  A/D", "tilt / pan the gimbal"),
                      ("C", "centre (level, ahead)"),
                      ("X", "look straight down"),
                      ("Y", "yaw lock / follow" + ("  (lock on)" if self.yaw_lock else "")),
                      ("Wheel, + / -", "zoom the pane under the pointer"),
                      ("V, click small view", "swap GimbalCam / ChaseCam")]
        if self.on_settings_drawer:
            camera.append(("F3", "sim settings: camera, time of day"))
        groups = [
            ("Camera", camera),
            ("Map", [("Click", "set the selected drone's target"),
                     ("Click the compass", "pick a heading"),
                     ("[  /  ]", "zoom out / in"),
                     ("L", "labels on / off"),
                     ("Esc", "finish drawing an area or fence")]),
            ("Drones", [("1-9", "show and steer that drone")]),
            ("Safety", [("F2", "safety panel: alerts, health, geofence"),
                        ("Ctrl+H", "hold every drone"),
                        ("Ctrl+R", "return every drone home (asks)")]
             + ([("F9", "sim faults")] if self.on_f9 else [])),
        ]
        return groups

    def _draw_shortcuts(self, canvas):
        """F1: the shortcuts, over the main camera pane."""
        x, y, w, h = self.PANES["Main"]
        region = canvas[y:y + h, x:x + w]
        region[:] = (22, 22, 24)  # solid, so the pane's own title doesn't show through
        self._text(canvas, "Shortcuts", (x + 24, y + 34), 0.7, (235, 235, 235), box=False)
        self._text(canvas, "F1 or Esc to close", (x + w - 190, y + 34), 0.5, (150, 150, 150),
                   box=False)
        col_x = [x + 24, x + w // 2 + 8]
        col_y = [y + 72, y + 72]
        for i, (title, keys) in enumerate(self._shortcut_groups()):
            col = 0 if i in (0, 2) else 1  # Camera, Drones left; Map, Safety right
            cy = col_y[col]
            self._text(canvas, title, (col_x[col], cy), 0.55, (120, 200, 255), box=False)
            cy += 24
            for key, what in keys:
                self._text(canvas, key, (col_x[col], cy), 0.47, (235, 235, 235), box=False)
                self._text(canvas, what, (col_x[col] + 170, cy), 0.47, (175, 175, 175),
                           box=False)
                cy += 21
            col_y[col] = cy + 16

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
        if self.on_key_log:
            self.on_key_log(f"key {event.keysym} down")
        if isinstance(event.widget, tk.Entry):
            return  # typing a command or height, not steering
        self._seq += 1
        if event.keysym == "Escape" and self.show_help:
            self._toggle_help()
            return
        if event.keysym == "Escape" and self._drawing_mode():
            # Finish drawing (before Esc's other use, cancelling a track)
            if self.fence_drawing:
                self.set_fence_drawing(False)
            else:
                self._drawing.set(False)
                self.log("Search area drawing off: map clicks set targets")
            return
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
        elif ch == "y" and self.on_yaw_lock:
            self.on_yaw_lock()
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
