"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

CameraPanel: the dashboard's camera drawer (F3, sim only), docked at the right
of the cameras where the safety drawer goes. It changes every drone's GimbalCam
while the sim runs: resolution, aperture, sensor noise, lens distortion and
vibration. Each control sends the console's `sim camera ...` command, so the
reply shows in the messages and the terminal does the same thing.

Ticking an effect turns it on or off with its current settings; Apply sends the
typed settings and turns it on.
"""

import tkinter as tk
from tkinter import ttk

GREY = "#9a9a9a"
RESOLUTIONS = ("360p", "480p", "720p", "1080p", "2k", "4k")


class CameraPanel:
    def __init__(self, windows, camera):
        """camera: the sim's camera properties; `camera.settings()` is what the
        GimbalCams are set to now."""
        self.windows = windows
        self.camera = camera
        w = windows
        self.BG, self.FG, self.FIELD, self.ACCENT = w.BG, w.FG, w.FIELD, w.ACCENT
        root = w.camera_drawer
        self._shown = None

        head = tk.Frame(root, bg=self.BG)
        head.pack(fill=tk.X, padx=8, pady=(8, 0))
        tk.Label(head, text="Camera", bg=self.BG, fg=self.FG,
                 font=("DejaVu Sans", 12, "bold")).pack(side=tk.LEFT)
        self._button(head, "x", lambda: w.toggle_camera_drawer(False)).pack(side=tk.RIGHT)
        self._label(root, "Every drone's GimbalCam, while the sim runs", fg=GREY).pack(
            fill=tk.X, padx=8, pady=(0, 6))

        f = tk.Frame(root, bg=self.BG)
        f.pack(fill=tk.BOTH, expand=True, padx=8)

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
        self._now = self._label(f, fg=GREY, font=("DejaVu Sans Mono", 9))
        self._now.pack(fill=tk.X)
        self.load()

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
                       command=lambda: self._send(f"{word} {'on' if var.get() else 'off'}"),
                       bg=self.BG, fg=self.FG, selectcolor=self.FIELD, activebackground=self.BG,
                       activeforeground=self.FG, highlightthickness=0, anchor="w").pack(
            fill=tk.X, pady=(10, 2))
        return var

    # ------------------------------------------------------------------
    # Showing the settings

    def load(self):
        """Fill the controls with what the camera is set to now (on opening,
        so typing isn't overwritten while the panel is open)."""
        s = self.camera.settings()
        width, height = s["resolution"]
        self._res.set({(640, 360): "360p", (854, 480): "480p", (1280, 720): "720p",
                       (1920, 1080): "1080p", (2560, 1440): "2k", (3840, 2160): "4k"}.get(
            (width, height), f"{width}x{height}"))
        self._aperture.set(f"{s['aperture_f']:g}")
        self._noise_on.set(s["noise"])
        self._gain.set(f"{s['noise_gain_db']:g}")
        self._temperature.set(f"{s['noise_temperature_c']:g}")
        self._distortion_on.set(s["distortion"])
        for var, value in zip(self._coeffs, s["distortion_coeffs"]):
            var.set(f"{value:g}")
        self._vibration_on.set(s["vibration"])
        self._amplitude.set(f"{s['vibration_amplitude_deg']:g}")
        self._rate.set(f"{s['vibration_hz']:g}")
        self._shown = None
        self.refresh()

    def refresh(self):
        """Update the summary (twice a second, from the ground station)."""
        if not self.windows.camera_drawer_open:
            return
        s = self.camera.settings()
        lines = [f"{s['resolution'][0]}x{s['resolution'][1]}, f/{s['aperture_f']:g}",
                 f"noise      {'%.1f levels' % s['noise_sigma'] if s['noise'] else 'off'}",
                 "distortion " + ("k1=%g k2=%g p1=%g p2=%g k3=%g" % s["distortion_coeffs"]
                                  if s["distortion"] else "off"),
                 "vibration  " + (f"{s['vibration_amplitude_deg']:g} deg at "
                                  f"{s['vibration_hz']:g} Hz" if s["vibration"] else "off")]
        text = "\n".join(lines)
        if text != self._shown:
            self._shown = text
            self._now.configure(text=text)
            # The tick boxes follow commands typed in the console too
            self._noise_on.set(s["noise"])
            self._distortion_on.set(s["distortion"])
            self._vibration_on.set(s["vibration"])

    # ------------------------------------------------------------------
    # Applying

    def _send(self, args):
        line = f"sim camera {args}"
        self.windows.log(line)
        self.windows.on_command(line)

    @staticmethod
    def _numbers(*vars_):
        """The fields as numbers, or None (with a message) if one isn't."""
        try:
            return [float(v.get()) for v in vars_]
        except ValueError:
            return None

    def _apply(self, args, *vars_):
        values = self._numbers(*vars_)
        if values is None:
            self.windows.log("Camera panel: type numbers in the fields")
            return
        self._send(args + " " + " ".join(f"{v:g}" for v in values))

    def _apply_resolution(self):
        value = self._res.get().strip()
        if value:
            self._send(f"res {value}")

    def _apply_aperture(self):
        self._apply("aperture", self._aperture)

    def _apply_noise(self):
        self._apply("noise", self._gain, self._temperature)

    def _apply_distortion(self):
        # The command takes them in OpenCV's order: k1 k2 p1 p2 k3
        self._apply("distortion", *self._coeffs)

    def _apply_vibration(self):
        self._apply("vibration", self._amplitude, self._rate)
