"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

SimPanel: the dashboard's sim settings drawer (F3, sim only), docked at the
right of the cameras where the safety drawer goes. Tabs:

  Camera       every drone's GimbalCam: resolution, aperture, sensor noise,
               lens distortion and vibration
  Time of day  the time and date at the site, which place the sun

Each control sends the console's `sim camera ...` or `sim time ...` command, so
the reply shows in the messages and the terminal does the same thing. On the
Camera tab, ticking an effect turns it on or off with its current settings;
Apply sends the typed settings and turns it on.
"""

import tkinter as tk
from tkinter import ttk

GREY = "#9a9a9a"
RESOLUTIONS = ("360p", "480p", "720p", "1080p", "2k", "4k")
RESOLUTION_NAMES = {(640, 360): "360p", (854, 480): "480p", (1280, 720): "720p",
                    (1920, 1080): "1080p", (2560, 1440): "2k", (3840, 2160): "4k"}
# Quick times for the Time of day tab, at the site
TIME_PRESETS = (("Dawn", "06:45"), ("Morning", "09:00"), ("Noon", "13:00"),
                ("Afternoon", "16:00"), ("Dusk", "19:00"), ("Night", "21:00"))


class SimPanel:
    def __init__(self, windows, settings):
        """settings(): what the sim is set to now, a dict: the camera's
        (resolution, aperture_f, noise..., distortion..., vibration...), and
        time (a datetime at the site, or None for the map's own sun), sun (a
        line saying where it is) and timezone."""
        self.windows = windows
        self.settings = settings
        w = windows
        self.BG, self.FG, self.FIELD, self.ACCENT = w.BG, w.FG, w.FIELD, w.ACCENT
        root = w.settings_drawer
        self._shown = None

        head = tk.Frame(root, bg=self.BG)
        head.pack(fill=tk.X, padx=8, pady=(8, 4))
        tk.Label(head, text="Sim settings", bg=self.BG, fg=self.FG,
                 font=("DejaVu Sans", 12, "bold")).pack(side=tk.LEFT)
        self._button(head, "x", lambda: w.toggle_settings_drawer(False)).pack(side=tk.RIGHT)

        tabs = tk.Frame(root, bg=self.BG)
        tabs.pack(fill=tk.X, padx=8, pady=(0, 6))
        self._tab = tk.StringVar(value="Camera")
        self.frames = {}
        for name in ("Camera", "Time of day"):
            tk.Radiobutton(tabs, text=name, variable=self._tab, value=name,
                           command=lambda n=name: self.show_tab(n), indicatoron=False,
                           bg=self.FIELD, fg=self.FG, selectcolor=self.ACCENT,
                           activebackground=self.ACCENT, relief=tk.FLAT, padx=6, pady=2,
                           highlightthickness=0).pack(side=tk.LEFT, padx=1)
            self.frames[name] = tk.Frame(root, bg=self.BG)
        self._build_camera(self.frames["Camera"])
        self._build_time(self.frames["Time of day"])
        self.show_tab("Camera")
        self.load()

    def show_tab(self, name):
        self._tab.set(name)
        for frame in self.frames.values():
            frame.pack_forget()
        self.frames[name].pack(fill=tk.BOTH, expand=True, padx=8)

    def open(self, tab=None):
        self.windows.toggle_settings_drawer(True)
        if tab:
            self.show_tab(tab)

    # ------------------------------------------------------------------
    # Widgets

    def _button(self, parent, text, command):
        return tk.Button(parent, text=text, command=command, bg=self.FIELD, fg=self.FG,
                         activebackground=self.ACCENT, activeforeground="white", relief=tk.FLAT,
                         padx=8, pady=2, highlightthickness=0)

    def _label(self, parent, text="", fg=None, **kw):
        return tk.Label(parent, text=text, bg=self.BG, fg=fg or self.FG, anchor="w",
                        justify=tk.LEFT, **kw)

    def _entry(self, parent, width):
        var = tk.StringVar()
        tk.Entry(parent, textvariable=var, width=width, bg=self.FIELD, fg=self.FG,
                 insertbackground=self.FG, relief=tk.FLAT).pack(side=tk.LEFT, padx=(2, 6))
        return var

    def _field(self, parent, text, width):
        self._label(parent, text).pack(side=tk.LEFT)
        return self._entry(parent, width)

    def _row(self, parent):
        row = tk.Frame(parent, bg=self.BG)
        row.pack(fill=tk.X, pady=2)
        return row

    def _section(self, parent, text):
        self._label(parent, text, font=("DejaVu Sans", 10, "bold")).pack(fill=tk.X, pady=(10, 2))

    def _effect(self, parent, text, word):
        """A section whose heading is the effect's on/off tick box."""
        var = tk.BooleanVar()
        tk.Checkbutton(parent, text=text, variable=var, font=("DejaVu Sans", 10, "bold"),
                       command=lambda: self._send(f"camera {word} {'on' if var.get() else 'off'}"),
                       bg=self.BG, fg=self.FG, selectcolor=self.FIELD, activebackground=self.BG,
                       activeforeground=self.FG, highlightthickness=0, anchor="w").pack(
            fill=tk.X, pady=(10, 2))
        return var

    # ------------------------------------------------------------------
    # Camera tab

    def _build_camera(self, f):
        self._label(f, "Every drone's GimbalCam", fg=GREY).pack(fill=tk.X)

        self._section(f, "Resolution")
        row = self._row(f)
        self._res = tk.StringVar()
        ttk.Combobox(row, textvariable=self._res, values=RESOLUTIONS, width=10).pack(side=tk.LEFT)
        self._button(row, "Apply", self._apply_resolution).pack(side=tk.LEFT, padx=6)
        self._label(f, "The stream restarts; recordings in progress stop", fg=GREY).pack(fill=tk.X)

        self._section(f, "Aperture")
        row = self._row(f)
        self._label(row, "f/").pack(side=tk.LEFT)
        self._aperture = self._entry(row, 6)
        self._button(row, "Apply", self._apply_aperture).pack(side=tk.LEFT, padx=6)
        self._label(f, "Depth of field only; exposure stays automatic", fg=GREY).pack(fill=tk.X)

        self._noise_on = self._effect(f, "Sensor noise", "noise")
        row = self._row(f)
        self._gain = self._field(row, "Gain (dB)", 5)
        self._temperature = self._field(row, "Temp (C)", 5)
        self._button(row, "Apply", self._apply_noise).pack(side=tk.LEFT, padx=6)

        self._distortion_on = self._effect(f, "Lens distortion", "distortion")
        row = self._row(f)
        self._coeffs = [self._field(row, name, 6) for name in ("k1", "k2", "k3")]
        row = self._row(f)
        self._coeffs[2:2] = [self._field(row, name, 6) for name in ("p1", "p2")]
        self._button(row, "Apply", self._apply_distortion).pack(side=tk.LEFT, padx=6)
        self._label(f, "OpenCV coefficients; negative k1 is barrel", fg=GREY).pack(fill=tk.X)

        self._vibration_on = self._effect(f, "Vibration", "vibration")
        row = self._row(f)
        self._amplitude = self._field(row, "Amplitude (deg)", 6)
        self._rate = self._field(row, "Rate (Hz)", 5)
        self._button(row, "Apply", self._apply_vibration).pack(side=tk.LEFT, padx=6)

        self._section(f, "Now")
        self._camera_now = self._label(f, fg=GREY, font=("DejaVu Sans Mono", 9))
        self._camera_now.pack(fill=tk.X)

    # ------------------------------------------------------------------
    # Time of day tab

    def _build_time(self, f):
        self._time_zone = self._label(f, fg=GREY, wraplength=330)
        self._time_zone.pack(fill=tk.X)

        self._section(f, "Time and date")
        row = self._row(f)
        self._time = self._field(row, "Time", 8)
        self._date = self._field(row, "Date", 11)
        row = self._row(f)
        self._button(row, "Apply", self._apply_time).pack(side=tk.LEFT)
        self._button(row, "Now", lambda: self._send("time now")).pack(side=tk.LEFT, padx=6)
        self._label(f, "HH:MM, and YYYY-MM-DD or blank for today", fg=GREY).pack(fill=tk.X)

        self._section(f, "Quick times (on the date above)")
        for i in range(0, len(TIME_PRESETS), 3):
            row = self._row(f)
            for name, hhmm in TIME_PRESETS[i:i + 3]:
                self._button(row, f"{name} {hhmm}", lambda t=hhmm: self._set_time(t)).pack(
                    side=tk.LEFT, padx=(0, 4))

        self._section(f, "Now")
        self._time_now = self._label(f, fg=GREY, wraplength=330)
        self._time_now.pack(fill=tk.X)
        self._label(f, "The map's photographed shadows don't move; the light, the sky "
                       "and the shadows of what the sim draws do", fg=GREY,
                    wraplength=330).pack(fill=tk.X, pady=(10, 0))

    # ------------------------------------------------------------------
    # Showing the settings

    def load(self):
        """Fill the controls with what the sim is set to now (on opening, so
        typing isn't overwritten while the panel is open)."""
        s = self.settings()
        self._res.set(RESOLUTION_NAMES.get(tuple(s["resolution"]),
                                           "%dx%d" % tuple(s["resolution"])))
        self._aperture.set(f"{s['aperture_f']:g}")
        self._gain.set(f"{s['noise_gain_db']:g}")
        self._temperature.set(f"{s['noise_temperature_c']:g}")
        for var, value in zip(self._coeffs, s["distortion_coeffs"]):
            var.set(f"{value:g}")
        self._amplitude.set(f"{s['vibration_amplitude_deg']:g}")
        self._rate.set(f"{s['vibration_hz']:g}")
        when = s.get("time")
        self._time.set(f"{when:%H:%M}" if when else "")
        self._date.set(f"{when:%Y-%m-%d}" if when else "")
        self._shown = None
        self.refresh()

    def refresh(self):
        """Update the summaries (twice a second, from the ground station)."""
        if not self.windows.settings_drawer_open:
            return
        s = self.settings()
        lines = [f"{s['resolution'][0]}x{s['resolution'][1]}, f/{s['aperture_f']:g}",
                 f"noise      {'%.1f levels' % s['noise_sigma'] if s['noise'] else 'off'}",
                 "distortion " + ("k1=%g k2=%g p1=%g p2=%g k3=%g" % tuple(s["distortion_coeffs"])
                                  if s["distortion"] else "off"),
                 "vibration  " + (f"{s['vibration_amplitude_deg']:g} deg at "
                                  f"{s['vibration_hz']:g} Hz" if s["vibration"] else "off")]
        time_text = s.get("sun") or "The map's own sun: no time set"
        shown = ("\n".join(lines), time_text, s.get("timezone", ""))
        if shown != self._shown:
            self._shown = shown
            self._camera_now.configure(text=shown[0])
            self._time_now.configure(text=shown[1])
            self._time_zone.configure(text=f"Site time zone {shown[2]} (--timezone)")
            # The tick boxes follow commands typed in the console too
            self._noise_on.set(s["noise"])
            self._distortion_on.set(s["distortion"])
            self._vibration_on.set(s["vibration"])

    # ------------------------------------------------------------------
    # Applying

    def _send(self, args):
        line = f"sim {args}"
        self.windows.log(line)
        self.windows.on_command(line)

    @staticmethod
    def _numbers(*vars_):
        """The fields as numbers, or None if one isn't."""
        try:
            return [float(v.get()) for v in vars_]
        except ValueError:
            return None

    def _apply(self, args, *vars_):
        values = self._numbers(*vars_)
        if values is None:
            self.windows.log("Sim settings: type numbers in the fields")
            return
        self._send(args + " " + " ".join(f"{v:g}" for v in values))

    def _apply_resolution(self):
        value = self._res.get().strip()
        if value:
            self._send(f"camera res {value}")

    def _apply_aperture(self):
        self._apply("camera aperture", self._aperture)

    def _apply_noise(self):
        self._apply("camera noise", self._gain, self._temperature)

    def _apply_distortion(self):
        # The command takes them in OpenCV's order: k1 k2 p1 p2 k3
        self._apply("camera distortion", *self._coeffs)

    def _apply_vibration(self):
        self._apply("camera vibration", self._amplitude, self._rate)

    def _set_time(self, hhmm):
        self._time.set(hhmm)
        self._apply_time()

    def _apply_time(self):
        hhmm, date = self._time.get().strip(), self._date.get().strip()
        if not hhmm:
            self.windows.log("Sim settings: type a time (HH:MM)")
            return
        self._send(f"time {hhmm}" + (f" {date}" if date else ""))
