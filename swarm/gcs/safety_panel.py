"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

SafetyPanel: the dashboard's safety drawer (F2), docked at the right of the
cameras. Tabs:

  Alerts     active alerts, most urgent first, to acknowledge; recent history
  Health     each drone's battery, GPS, link and what its autopilot does on link loss
  Responses  thresholds and what the ground station does about them
  Geofence   draw a fence on the map and upload it to every drone
  Sim        (sim only, F9) cut a drone's link, drain its battery

Hidden, it costs nothing; open, it refreshes about twice a second.
"""

import time
import tkinter as tk

from .mavlink_link import GPS_FIX_NAMES
from .safety import CRITICAL, FENCE_ACTIONS, SEVERITY_NAMES, WARNING

RED, AMBER, GREY = "#ff7070", "#ffbf40", "#9a9a9a"
COLOURS = {CRITICAL: RED, WARNING: AMBER}


class SafetyPanel:
    def __init__(self, windows, station, sim_faults=None):
        self.windows = windows
        self.station = station
        self.safety = station.safety
        self.sim_faults = sim_faults
        w = windows
        self.BG, self.FG, self.FIELD, self.ACCENT = w.BG, w.FG, w.FIELD, w.ACCENT
        root = w.drawer
        self._shown_alerts = None
        self._shown_history = None

        head = tk.Frame(root, bg=self.BG)
        head.pack(fill=tk.X, padx=8, pady=(8, 4))
        tk.Label(head, text="Safety", bg=self.BG, fg=self.FG,
                 font=("DejaVu Sans", 12, "bold")).pack(side=tk.LEFT)
        self._button(head, "x", lambda: w.toggle_drawer(False)).pack(side=tk.RIGHT)

        tabs = tk.Frame(root, bg=self.BG)
        tabs.pack(fill=tk.X, padx=8, pady=(0, 6))
        self._tab = tk.StringVar(value="Alerts")
        self.frames = {}
        names = ["Alerts", "Health", "Responses", "Geofence"] + (["Sim"] if sim_faults else [])
        for name in names:
            tk.Radiobutton(tabs, text=name, variable=self._tab, value=name,
                           command=lambda n=name: self.show_tab(n), indicatoron=False,
                           bg=self.FIELD, fg=self.FG, selectcolor=self.ACCENT,
                           activebackground=self.ACCENT, relief=tk.FLAT, padx=6, pady=2,
                           highlightthickness=0).pack(side=tk.LEFT, padx=1)
            self.frames[name] = tk.Frame(root, bg=self.BG)
        self._build_alerts(self.frames["Alerts"])
        self._build_health(self.frames["Health"])
        self._build_responses(self.frames["Responses"])
        self._build_geofence(self.frames["Geofence"])
        if sim_faults:
            self._build_sim(self.frames["Sim"])
        self.show_tab("Alerts")

    # ------------------------------------------------------------------
    # Widgets

    def _button(self, parent, text, command, **kw):
        return tk.Button(parent, text=text, command=command, bg=self.FIELD, fg=self.FG,
                         activebackground=self.ACCENT, activeforeground="white", relief=tk.FLAT,
                         padx=8, pady=2, highlightthickness=0, **kw)

    def _label(self, parent, text="", fg=None, **kw):
        return tk.Label(parent, text=text, bg=self.BG, fg=fg or self.FG, anchor="w",
                        justify=tk.LEFT, **kw)

    def _text(self, parent, height):
        t = tk.Text(parent, height=height, bg="#141517", fg="#d0d0d0", relief=tk.FLAT,
                    font=("DejaVu Sans Mono", 9), wrap=tk.WORD, takefocus=False)
        for severity, colour in COLOURS.items():
            t.tag_configure(str(severity), foreground=colour)
        t.tag_configure("grey", foreground=GREY)
        t.tag_configure("bold", font=("DejaVu Sans Mono", 9, "bold"))
        return t

    def show_tab(self, name):
        if name != "Geofence":
            self.windows.set_fence_drawing(False)  # only while its tab is showing
        self._tab.set(name)
        for frame in self.frames.values():
            frame.pack_forget()
        self.frames[name].pack(fill=tk.BOTH, expand=True, padx=8)
        self.refresh()

    def open(self, tab=None):
        self.windows.toggle_drawer(True)
        if tab:
            self.show_tab(tab)

    # ------------------------------------------------------------------
    # Alerts

    def _build_alerts(self, f):
        self._label(f, "Active (most urgent first)").pack(fill=tk.X)
        self._alerts = tk.Listbox(f, height=9, bg="#141517", fg=self.FG, relief=tk.FLAT,
                                  font=("DejaVu Sans Mono", 9), selectbackground=self.ACCENT,
                                  highlightthickness=0, activestyle="none")
        self._alerts.pack(fill=tk.X, pady=(2, 4))
        row = tk.Frame(f, bg=self.BG)
        row.pack(fill=tk.X)
        self._button(row, "Ack selected", self._ack_selected).pack(side=tk.LEFT)
        self._button(row, "Ack all", lambda: self._ack(None)).pack(side=tk.LEFT, padx=4)
        self._label(f, "History", fg=GREY).pack(fill=tk.X, pady=(10, 0))
        self._history = self._text(f, 14)
        self._history.pack(fill=tk.BOTH, expand=True, pady=(2, 8))
        self._alert_keys = []

    def _ack_selected(self):
        sel = self._alerts.curselection()
        if sel and sel[0] < len(self._alert_keys):
            self._ack(self._alert_keys[sel[0]])

    def _ack(self, key):
        self.safety.ack(key)
        self._shown_alerts = None
        self.refresh()

    def _refresh_alerts(self):
        alerts = self.safety.ordered()
        shown = [(a.key, a.severity, a.text, a.acked) for a in alerts]
        if shown != self._shown_alerts:
            self._shown_alerts = shown
            self._alerts.delete(0, tk.END)
            self._alert_keys = [a.key for a in alerts]
            for a in alerts:
                when = time.strftime("%H:%M:%S", time.localtime(a.since))
                mark = " " if a.acked else "!"
                self._alerts.insert(tk.END, f"{mark}{when} {a.text}")
                self._alerts.itemconfig(tk.END, fg=GREY if a.acked else COLOURS[a.severity])
            if not alerts:
                self._alerts.insert(tk.END, " no active alerts")
                self._alerts.itemconfig(tk.END, fg=GREY)
        history = list(self.safety.history)[-60:]
        if len(history) != self._shown_history:
            self._shown_history = len(history)
            t = self._history
            t.configure(state=tk.NORMAL)
            t.delete("1.0", tk.END)
            for when, severity, text, what in reversed(history):
                stamp = time.strftime("%H:%M:%S", time.localtime(when))
                tag = "grey" if what == "cleared" else str(severity)
                t.insert(tk.END, f"{stamp} {'cleared: ' if what == 'cleared' else ''}{text}\n", tag)
            t.configure(state=tk.DISABLED)

    # ------------------------------------------------------------------
    # Health

    def _build_health(self, f):
        self._health = self._text(f, 30)
        self._health.pack(fill=tk.BOTH, expand=True, pady=(0, 8))

    def _refresh_health(self):
        t = self._health
        t.configure(state=tk.NORMAL)
        t.delete("1.0", tk.END)
        s = self.safety.settings
        for i, link in enumerate(self.station.links):
            severity = self.safety.drone_severity(i)
            t.insert(tk.END, f"{link.name}  {link.mode_name}  {'armed' if link.armed else 'disarmed'}\n",
                     ("bold", str(severity)) if severity else "bold")
            if i in self.station.swarm.lost:
                t.insert(tk.END, "  LINK LOST: values below are its last\n", str(CRITICAL))
            pct = link.battery_pct
            bat = "?" if pct is None else f"{pct:.0f}%"
            if link.battery_v:
                bat += f"  {link.battery_v:.1f} V"
            if link.battery_time_s:
                bat += f"  {link.battery_time_s / 60:.0f} min left"
            tag = "" if pct is None or pct > s.battery_warn_pct else (
                str(CRITICAL) if pct <= s.battery_act_pct else str(WARNING))
            t.insert(tk.END, f"  battery  {bat}\n", tag)
            if link.gps_fix is None:
                gps, tag = "?", ""
            else:
                gps = f"{GPS_FIX_NAMES.get(link.gps_fix, link.gps_fix)}  {link.gps_sats} sats"
                if link.gps_eph_m is not None:
                    gps += f"  ±{link.gps_eph_m:.1f} m"
                tag = str(CRITICAL) if link.gps_fix < 3 else (
                    str(WARNING) if (link.gps_sats or 0) < s.gps_min_sats
                    or (link.gps_eph_m or 0) > s.gps_max_eph_m else "")
            t.insert(tk.END, f"  GPS      {gps}\n", tag)
            age = link.link_age
            rate = self.safety.position_rate(i)
            link_text = "no messages yet" if age is None else f"{age:.1f} s ago, {rate:.1f} pos/s"
            t.insert(tk.END, f"  link     {link_text}\n",
                     str(CRITICAL) if age is not None and age > s.link_lost_s else "")
            dll = self.safety.dll_info.get(i, "?")
            t.insert(tk.END, f"  on link loss {link.ap.label} will {dll}\n", "grey")
            if link.status_text:
                t.insert(tk.END, f"  {link.ap.label}: {link.status_text}\n", "grey")
            t.insert(tk.END, "\n")
        t.configure(state=tk.DISABLED)

    # ------------------------------------------------------------------
    # Responses

    RESPONSE_FIELDS = [
        ("battery_warn_pct", "Battery warning at (%)"),
        ("battery_act_pct", "Battery action at (%)"),
        ("battery_action", "Battery action"),
        ("link_lost_s", "Link lost after (s)"),
        ("min_position_rate", "Weak link below (positions/s)"),
        ("gps_min_sats", "GPS: fewest satellites"),
        ("gps_max_eph_m", "GPS: worst accuracy (m)"),
        ("lost_margin_m", "Keep clear of a lost drone (m)"),
    ]
    CHOICES = {"battery_action": ("rtl", "land", "warn"), "fence_action": tuple(FENCE_ACTIONS)}

    def _build_responses(self, f):
        self._vars = {}
        grid = tk.Frame(f, bg=self.BG)
        grid.pack(fill=tk.X)
        for r, (key, text) in enumerate(self.RESPONSE_FIELDS):
            self._label(grid, text).grid(row=r, column=0, sticky="w", pady=2)
            var = tk.StringVar(value=str(getattr(self.safety.settings, key)))
            self._vars[key] = var
            self._field(grid, key, var).grid(row=r, column=1, sticky="e", padx=(6, 0))
        self._button(f, "Apply", self._apply_responses).pack(anchor="w", pady=8)
        self._label(f, "Battery action: rtl returns home, land lands where it is, warn only "
                    "alerts (a fixed-wing returns home instead of landing). The autopilot's own "
                    "low-battery failsafe still applies.", fg=GREY,
                    wraplength=self.windows.DRAWER_WIDTH - 24).pack(fill=tk.X)

    def _field(self, parent, key, var):
        if key in self.CHOICES:
            menu = tk.OptionMenu(parent, var, *self.CHOICES[key])
            menu.configure(bg=self.FIELD, fg=self.FG, activebackground=self.ACCENT,
                           relief=tk.FLAT, highlightthickness=0, width=6)
            menu["menu"].configure(bg=self.FIELD, fg=self.FG)
            return menu
        return tk.Entry(parent, textvariable=var, width=7, bg=self.FIELD, fg=self.FG,
                        insertbackground=self.FG, relief=tk.FLAT)

    def _apply_responses(self, keys=None):
        for key in keys or [k for k, _ in self.RESPONSE_FIELDS]:
            try:
                self.safety.settings.set(key, self._vars[key].get())
            except ValueError as err:
                self.windows.log(f"Safety setting {key}: {err}")
                self._vars[key].set(str(getattr(self.safety.settings, key)))
        self.windows.log("Safety settings applied")

    # ------------------------------------------------------------------
    # Geofence

    def _build_geofence(self, f):
        self._drawing_fence = tk.BooleanVar(value=False)
        tk.Checkbutton(f, text="Draw fence (click corners on the map)",
                       variable=self._drawing_fence, command=self._toggle_fence_drawing,
                       indicatoron=False, bg=self.FIELD, fg=self.FG, selectcolor=self.ACCENT,
                       activebackground=self.ACCENT, relief=tk.FLAT, padx=8, pady=3,
                       highlightthickness=0).pack(anchor="w")
        row = tk.Frame(f, bg=self.BG)
        row.pack(fill=tk.X, pady=4)
        self._button(row, "Undo corner",
                     lambda: self._set_fence(self.windows.fence[:-1])).pack(side=tk.LEFT)
        self._button(row, "Clear", lambda: self._set_fence([])).pack(side=tk.LEFT, padx=4)
        grid = tk.Frame(f, bg=self.BG)
        grid.pack(fill=tk.X, pady=4)
        for r, (key, text) in enumerate((("fence_max_alt_m", "Ceiling above home (m)"),
                                         ("fence_margin_m", "Warn this close (m)"),
                                         ("fence_action", "Autopilot's action at the fence"))):
            self._label(grid, text).grid(row=r, column=0, sticky="w", pady=2)
            var = tk.StringVar(value=str(getattr(self.safety.settings, key)))
            self._vars[key] = var
            self._field(grid, key, var).grid(row=r, column=1, sticky="e", padx=(6, 0))
        self._button(f, "Upload to all drones", self._upload_fence).pack(anchor="w", pady=6)
        self._fence_status = self._label(f, "", fg=GREY,
                                         wraplength=self.windows.DRAWER_WIDTH - 24)
        self._fence_status.pack(fill=tk.X)
        self._label(f, "The fence is uploaded to each autopilot, which enforces it itself (even "
                    "without this link). Uploading an empty fence removes it. The ground "
                    "station also warns near it and alerts outside it.", fg=GREY,
                    wraplength=self.windows.DRAWER_WIDTH - 24).pack(fill=tk.X, pady=(8, 0))
        self.windows.on_fence_changed = self.refresh
        self.windows.on_fence_drawing_changed = self._sync_fence_drawing

    def _toggle_fence_drawing(self):
        self.windows.set_fence_drawing(self._drawing_fence.get())

    def _sync_fence_drawing(self):
        """The dashboard turned drawing on or off (Esc, drawer closed)."""
        self._drawing_fence.set(self.windows.fence_drawing)

    def _set_fence(self, corners):
        self.windows.set_fence(corners)
        self.refresh()

    def _upload_fence(self):
        self._apply_responses(["fence_max_alt_m", "fence_margin_m", "fence_action"])
        self.windows.set_fence_drawing(False)
        self.station.upload_fence(self.windows.fence)

    def _refresh_geofence(self):
        n = len(self.windows.fence)
        drawn = "no fence drawn" if n == 0 else f"{n} corner{'s' if n != 1 else ''}" + (
            "" if n >= 3 else " (need 3)")
        uploaded = self.station.fence_status or "not uploaded"
        self._fence_status.configure(text=f"{drawn}; {uploaded}")

    # ------------------------------------------------------------------
    # Sim faults

    def _build_sim(self, f):
        self._label(f, "Simulate faults on the sim's drones.", fg=GREY).pack(fill=tk.X)
        self._cut_vars = []
        self._bat_vars = []
        for i, link in enumerate(self.station.links):
            box = tk.Frame(f, bg=self.BG)
            box.pack(fill=tk.X, pady=(8, 0))
            self._label(box, link.name, font=("DejaVu Sans", 10, "bold")).pack(anchor="w")
            row = tk.Frame(box, bg=self.BG)
            row.pack(fill=tk.X)
            cut = tk.BooleanVar(value=False)
            self._cut_vars.append(cut)
            tk.Checkbutton(row, text="Cut link", variable=cut,
                           command=lambda i=i, v=cut: self.sim_faults.cut_link(i, v.get()),
                           indicatoron=False, bg=self.FIELD, fg=self.FG, selectcolor="#a52a2a",
                           activebackground=self.ACCENT, relief=tk.FLAT, padx=8, pady=2,
                           highlightthickness=0).pack(side=tk.LEFT)
            self._button(row, "Reset", lambda i=i: self._reset(i)).pack(side=tk.RIGHT)
            row = tk.Frame(box, bg=self.BG)
            row.pack(fill=tk.X)
            self._label(row, "Battery %").pack(side=tk.LEFT)
            bat = tk.IntVar(value=50)
            self._bat_vars.append(bat)
            tk.Scale(row, from_=0, to=100, orient=tk.HORIZONTAL, variable=bat, length=170,
                     bg=self.BG, fg=self.FG, troughcolor=self.FIELD, highlightthickness=0,
                     showvalue=True).pack(side=tk.LEFT, padx=4)
            self._button(row, "Set",
                         lambda i=i, v=bat: self.sim_faults.set_battery(i, v.get())
                         ).pack(side=tk.LEFT)
        self._label(f, "Cut link stops all MAVLink between this station and that drone. "
                    "PX4: after COM_DL_LOSS_T the drone does its link-loss action "
                    "(NAV_DLL_ACT; the sim sends it on PX4's behalf, since the simulator "
                    "itself keeps PX4's link alive). ArduPilot notices by itself and runs "
                    "FS_LONG_ACTN after FS_LONG_TIMEOUT.\n\nBattery: PX4's falls to the level "
                    "set within seconds and stays there while armed (full again once "
                    "disarmed); ArduPilot's is set to it and drains on from there.\n\n"
                    "Reset reconnects, returns the battery to normal and, on PX4, restarts "
                    "the drone's simulated hardware (battery and power simulators): use it "
                    "if PX4 refuses to arm with \"Battery unhealthy\".", fg=GREY,
                    wraplength=self.windows.DRAWER_WIDTH - 24).pack(fill=tk.X, pady=(12, 0))

    def _reset(self, i):
        self._cut_vars[i].set(False)
        self.sim_faults.reset(i)

    # ------------------------------------------------------------------

    def refresh(self):
        if not self.windows.drawer_open:
            return
        tab = self._tab.get()
        if tab == "Alerts":
            self._refresh_alerts()
        elif tab == "Health":
            self._refresh_health()
        elif tab == "Geofence":
            self._refresh_geofence()
        elif tab == "Sim":
            for i, var in enumerate(self._cut_vars):
                if var.get() != self.sim_faults.is_cut(i):
                    var.set(self.sim_faults.is_cut(i))


def alert_line(safety):
    """(text, BGR colour) for the status panel's top line, or None."""
    alert, others = safety.most_urgent()
    if alert is None:
        return None
    more = f"   (+{others} more, F2)" if others else "   (F2 for details)"
    colour = (90, 90, 255) if alert.severity == CRITICAL else (40, 190, 255)
    return f"! {SEVERITY_NAMES[alert.severity]}: {alert.text}{more}", colour
