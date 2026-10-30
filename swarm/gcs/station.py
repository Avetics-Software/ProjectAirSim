"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

GroundStation: the dashboard, swarm control and companion links for a fleet
of PX4 drones, driven only by MAVLink, video streams and the companions' UDP.

Whoever runs it supplies each drone's camera (see DroneCamera) and may add
console commands, map objects and status lines through the hooks; the sim
launcher uses them for its spawned objects, moving actors and `goto`.
"""

import asyncio
import contextlib
import io
import logging
import math
import os
import sys
import threading
import time

from .companion import CompanionLink
from .dashboard import CameraWindows
from .log import log
from .mavlink_link import GPS_FIX_NAMES
from .minimap import MiniMap, road_segments
from .mission import MISSION_HELP, MissionControl
from .fleet import Fleet
from .safety import CRITICAL, WARNING, SafetyMonitor, fence_check
from .sim_panel import SimPanel
from .safety_panel import SafetyPanel, alert_line
from .swarm import SWARM_HELP, SwarmControl
from .video import VideoStream

GIMBAL_INITIAL_PITCH = -45.0
ROAD_RADIUS_M = 1500.0  # OpenStreetMap roads loaded around home for the map

STATION_HELP = """  select N                              show and steer drone N (or keys 1-9)
  gimbal PITCH YAW [N]                  point drone N's gimbal (degrees, yaw relative to
                                        the drone) [the selected drone]
  gimbal lock on|off [N]                yaw lock: hold the camera's compass heading while
                                        the drone turns (or Y); off: follow the drone
  zoom PERCENT [N]                      set the zoom (0 = widest, 100 = longest focal length)
  strike on|off [N]                     arm / release X's strike on drone N (needs its
                                        companion) [the selected drone]
Safety (F2 opens the safety panel; Ctrl+H holds every drone):
  alerts                                list active alerts
  ack [all]                             acknowledge the most urgent alert, or all
  fence upload | fence clear            send the fence drawn on the map to every drone /
                                        remove it
  fence set LAT,LON;LAT,LON;...         set the fence's corners (then fence upload)
  safety KEY VALUE                      change a threshold or response, e.g.
                                        safety battery_act_pct 25 (safety lists them)"""
LINE_COLOURS = {0: (235, 235, 235), WARNING: (40, 190, 255), CRITICAL: (90, 90, 255)}


class DroneCamera:
    """A drone's cameras as the dashboard sees them, for a real drone: video
    from RTSP, the gimbal steered through PX4's gimbal manager and the zoom
    through the drone's MAVLink camera. The sim's camera subclasses this."""

    def __init__(self, link, windows, video_url=None, chase_url=None):
        self.link = link
        self.name = link.name
        self.windows = windows
        self.selected = False
        self.pitch, self.yaw = GIMBAL_INITIAL_PITCH, 0.0  # commanded; yaw from the nose
        # Yaw lock: the gimbal holds a compass heading while the drone turns
        # (else it follows the drone's heading)
        self.yaw_lock = False
        self.last_control_claim = 0.0
        self.streams = {}  # pane name -> VideoStream
        if video_url:
            self.streams["GimbalCam"] = VideoStream(video_url, f"{self.name} GimbalCam")
        if chase_url:
            self.streams["ChaseCam"] = VideoStream(chase_url, f"{self.name} ChaseCam")
        self._shown = {}  # pane name -> frame_time last passed to the dashboard

    def steer(self, pitch_deg, yaw_deg):
        # QGC can take gimbal control at any time; re-claim it when the user
        # starts steering from the keyboard again
        self.pitch, self.yaw = pitch_deg, yaw_deg
        now = time.monotonic()
        if now - self.last_control_claim > 2.0:
            self.link.take_gimbal_control()
        self.last_control_claim = now
        if self.yaw_lock and self.link.heading is not None:
            # A compass heading: where the nose-relative yaw points right now
            heading = (self.link.heading + yaw_deg + 180.0) % 360.0 - 180.0
            self.link.set_gimbal_attitude(pitch_deg, heading, yaw_lock=True)
        else:
            self.link.set_gimbal_attitude(pitch_deg, yaw_deg)

    def set_yaw_lock(self, on):
        """Hold the camera's compass heading (on) or follow the drone (off),
        keeping it pointed where it is."""
        self.yaw_lock = on
        self.steer(self.pitch, self.yaw)

    def set_zoom(self, percent):
        self.link.set_zoom(percent)

    def nudge_zoom(self, delta_percent):
        self.set_zoom(self.link.zoom_percent + delta_percent)

    # The lens's optical zoom (longest / widest focal length), for zoom
    # factors ("2x"); unknown for a real camera, which then takes percentages
    max_zoom = None

    def zoom_to(self, spec):
        """Zoom to a mission step's {"factor": F} or {"percent": P}; returns the
        percentage set. Assumes the focal length moves linearly with the zoom
        percentage, as the sim's lens does."""
        if "percent" in spec:
            percent = spec["percent"]
        elif not self.max_zoom or self.max_zoom <= 1:
            raise ValueError(f"the camera's zoom range is unknown: give the zoom as a "
                             f"percentage, not {spec['factor']:g}x")
        else:
            percent = (spec["factor"] - 1.0) / (self.max_zoom - 1.0) * 100.0
            if percent > 100.0:
                log.warning(f"{self.link.name}: {spec['factor']:g}x is beyond the lens's "
                            f"{self.max_zoom:g}x; zooming to {self.max_zoom:g}x")
        percent = max(0.0, min(100.0, percent))
        self.set_zoom(percent)
        log.info(f"{self.link.name}: zoom {percent:.1f}%" + (
            f" ({spec['factor']:g}x)" if "factor" in spec else ""))
        return percent

    def zoom_text(self):
        return f"zoom {self.link.zoom_percent:.0f}%"

    def tick(self, dt):
        if not self.selected:
            return
        for pane, stream in self.streams.items():
            if stream.frame is not None and stream.frame_time != self._shown.get(pane):
                self._shown[pane] = stream.frame_time
                self.windows.receive(pane, stream.frame)

    def close(self):
        for stream in self.streams.values():
            stream.close()


class DashboardLog(logging.Handler):
    """Shows info messages and warnings in the dashboard's message box."""

    def __init__(self, windows):
        super().__init__(logging.INFO)
        self.windows = windows
        self.setFormatter(logging.Formatter("%(asctime)s %(message)s", datefmt="%H:%M:%S"))

    def emit(self, record):
        message = record.getMessage()
        # Skip the topic listings printed for each sim drone as it loads
        if message.startswith(" ") or "can be subscribed" in message or message.startswith("-"):
            return
        self.windows.log(self.format(record))


class GroundStation:
    """The dashboard window and everything behind it.

    links: VehicleLink per drone (drone N = links[N - 1]).
    make_camera(index, link, windows): that drone's DroneCamera.
    companions: {drone index: companion IP}.
    Hooks, all optional:
      extra_commands(line, parts) -> True if it handled the console command
      extra_help: text added to `help`
      extra_objects() -> [(name, kind, north, east, heading)] for the map
      extra_status() -> lines added to the status panel
    config: settings, with .swarm, .deconfliction, .safety and .controls
      (gcs.fleet.Fleet from the config file); defaults if None.
    sim_faults: the sim's fault injection (cut_link, is_cut, set_battery,
      reset), for the safety panel's Sim tab.
    sim_settings() -> dict of the sim's camera and time-of-day settings, for
      the sim settings panel (F3), which sends `sim camera|time ...` commands.
    before_mission: callable (sync or async) run before each `mission start`,
      e.g. the sim resetting its moving actors so runs repeat.
    run_dir: where mission runs are logged [runs/ next to the gcs package].
    on_mission_track(i, spec, run_id): called when drone i's mission hands its
      camera to a tracker (a `track` step) and with spec None when it takes
      it back; the sim locks the tracker onto the target and scores it.
    on_mission_run(event, mission, run_id, indices): called when a mission run
      starts ("start") and when it has ended ("end"), e.g. to record it.
    """

    def __init__(self, links, make_camera, companions=None, home=None, config=None,
                 extra_commands=None, extra_help="", extra_objects=None, extra_status=None,
                 sim_faults=None, before_mission=None, run_dir=None, on_mission_track=None,
                 on_mission_run=None, sim_settings=None):
        self.links = links
        self.extra_commands = extra_commands
        self.extra_help = extra_help
        self.extra_objects = extra_objects
        self.extra_status = extra_status
        self.commands = None  # asyncio.Queue of (line, from the dashboard?), made by start()
        self.windows = CameraWindows(
            on_gimbal=self.steer, on_zoom=self.nudge_zoom, on_select=self.select,
            initial_pitch=GIMBAL_INITIAL_PITCH, minimap=MiniMap(),
        )
        self.windows.add_controls(
            [link.name for link in links], self._dashboard_command,
            missions_dir=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                      "missions"))
        self.windows.on_key_log = log.debug  # key presses in the debug log
        self.windows.on_target_picked = self._check_target
        self.windows.on_yaw_lock = self._toggle_yaw_lock
        config = config or Fleet(drones=[])
        self.config = config
        self.windows.PAN_STEP_DEG = config.controls.gimbal_step_deg
        self.windows.ZOOM_STEP_PERCENT = config.controls.zoom_step_percent
        self.dashboard_log = DashboardLog(self.windows)
        log.addHandler(self.dashboard_log)
        self.cameras = [make_camera(i, link, self.windows) for i, link in enumerate(links)]
        self.companions = {
            i: CompanionLink(ip, name=f"{links[i].name} companion",
                             zoom_pulse_s=config.controls.companion_zoom_pulse_s)
            for i, ip in (companions or {}).items()
        }
        self.swarm = SwarmControl(links, config.swarm, config.deconfliction)
        self.safety = SafetyMonitor(links, self.swarm, self.companions, config.safety)
        base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.missions = MissionControl(
            links, self.swarm, lambda: self.home, run_dir or os.path.join(base, "runs"),
            search_dirs=[os.path.join(base, "missions")], before_start=before_mission,
            default_alt=config.swarm.default_alt_m, companions=self.companions,
            on_track=on_mission_track, on_run=on_mission_run,
            on_zoom=lambda i, spec: self.cameras[i].zoom_to(spec),
            on_fence=self._mission_fence)
        self.fence_status = ""  # last geofence upload, for the panel
        self.panel = SafetyPanel(self.windows, self, sim_faults)
        if sim_faults:
            self.windows.on_f9 = lambda: self.panel.open("Sim")
        self.sim_panel = None
        if sim_settings:
            self.sim_panel = SimPanel(self.windows, sim_settings)
            self.windows.add_settings_drawer(self.sim_panel.load)
        self.home = None
        self._fixed_home = home  # (lat, lon) from the fleet file, or None
        self._tracker_frame_time = 0.0
        self._tasks = []
        self.selected = 0
        self.select(1)

    @property
    def current(self):
        return self.cameras[self.selected]

    # ------------------------------------------------------------------
    # Selection, gimbal and zoom (from the dashboard)

    def select(self, number):
        """Show and steer drone `number` (1-based)."""
        if not 1 <= number <= len(self.cameras):
            return
        self.selected = number - 1
        for i, camera in enumerate(self.cameras):
            camera.selected = i == self.selected
        self.windows.title = self.current.name if len(self.cameras) > 1 else ""
        self.windows.autopilot = self.current.link.ap.label  # who steers a camera with no companion
        self.windows.set_selected(number)
        self.windows.set_companion(self.companions.get(self.selected))
        self._tracker_frame_time = 0.0
        self.windows.pitch, self.windows.yaw = self.current.pitch, self.current.yaw
        with self.windows._lock:
            self.windows._frames.clear()

    # Who steers a drone's camera: its companion's X if it has one (the
    # dashboard's keys go to X, see CameraWindows._companion_key), otherwise
    # the ground station through PX4's gimbal manager. Every direct gimbal or
    # zoom command passes here, so none can take the camera away from X.

    def x_controls(self, index, what="gimbal"):
        """True (and says so) if drone `index`'s camera belongs to its companion."""
        companion = self.companions.get(index)
        if companion:
            print(f"{self.links[index].name}'s {what} is controlled by its companion (X at "
                  f"{companion.ip}): use W/A/S/D, G and the zoom keys or wheel with it selected")
        return bool(companion)

    def mission_has_camera(self, index):
        """True while drone `index` flies a mission that holds its gimbal: in
        MISSION mode, with PX4's autopilot in control of the gimbal (or not yet
        reported). A `track` step hands the gimbal back. X uses the same rule
        to stand down (MAVLINK_STAND_DOWN_IN_MISSION)."""
        link = self.links[index]
        return link.mode == "MISSION" and link.autopilot_has_gimbal is not False

    def mission_controls(self, index):
        """True (and says so) if drone `index`'s mission steers its gimbal."""
        if self.mission_has_camera(index):
            self.windows.log(f"{self.links[index].name} is flying a mission, which steers "
                             "its gimbal: hold it (or mission stop) to steer by hand")
            return True
        return False

    def steer(self, pitch_deg, yaw_deg):
        if not self.x_controls(self.selected) and not self.mission_controls(self.selected):
            self.current.steer(pitch_deg, yaw_deg)

    def nudge_zoom(self, delta_percent):
        if not self.x_controls(self.selected, "zoom"):
            self.current.nudge_zoom(delta_percent)

    def tick(self, dt):
        for i, companion in self.companions.items():
            # An armed strike owns the aircraft: the mission ending (X switches
            # to POSITION/FBWA) must not hand the camera back, cancel the track
            # or release the strike. Without this the companion's mission
            # hand-over releases a strike the moment it starts.
            companion.set_mission_active(self.mission_has_camera(i)
                                         and not companion.striking)
        for camera in self.cameras:
            camera.tick(dt)
        self.windows.zoom_text = self.current.zoom_text()
        companion = self.companions.get(self.selected)
        link = self.current.link
        fresh = link.gimbal_attitude and time.monotonic() - link.gimbal_attitude_time < 1.0
        # The title shows where the gimbal actually points (it takes time to get
        # there, and holds a heading under yaw lock)
        self.windows.gimbal_actual = link.gimbal_attitude if fresh else None
        self.windows.yaw_lock = self.current.yaw_lock
        if companion and fresh:
            # The companion steers the gimbal: steps start from where it is
            self.current.pitch, self.current.yaw = link.gimbal_attitude
            self.windows.pitch, self.windows.yaw = link.gimbal_attitude
        if companion and companion.frame is not None and companion.frame_time != self._tracker_frame_time:
            self._tracker_frame_time = companion.frame_time
            self.windows.receive("Tracker", companion.frame)
        self.windows.update()

    # ------------------------------------------------------------------
    # Map

    def set_home(self, lat, lon, roads=None):
        """Centre the map's coordinates on lat/lon and forget targets and plans
        made around the old home. `roads`: (north1, east1, north2, east2)
        segments; None loads OpenStreetMap's in the background."""
        self.home = (lat, lon)
        minimap = self.windows.minimap
        minimap.set_home(lat, lon)
        minimap.picked.clear()
        self.windows.targets.clear()
        self.swarm.destinations.clear()
        self.swarm.plans.clear()
        self.swarm.crossings.clear()
        minimap.set_roads(roads or [])
        if roads is None:
            threading.Thread(target=self._load_roads, args=(lat, lon), daemon=True).start()

    def _load_roads(self, lat, lon):
        try:
            from .road_routes import RoadNetwork
            network = RoadNetwork(lat, lon, ROAD_RADIUS_M, "vehicle")
            if self.home == (lat, lon):
                self.windows.minimap.set_roads(road_segments(network))
        except Exception as err:
            log.warning(f"Minimap has no roads: {err}")

    def update_map(self):
        """Refresh the minimap and status panel from the drones' telemetry."""
        if self.home is None:
            first_fix = next((link.position for link in self.links if link.position), None)
            if self._fixed_home:
                self.set_home(*self._fixed_home)
            elif first_fix:
                self.set_home(*first_fix[:2])
        minimap = self.windows.minimap
        objects = list(self.extra_objects()) if self.extra_objects else []

        # Every drone: the selected one centres the map, the others are markers
        drone_lines = []
        centre = None
        for i, link in enumerate(self.links):
            mark = ">" if i == self.selected and len(self.links) > 1 else " "
            colour = LINE_COLOURS[self.safety.drone_severity(i)]
            if link.position is None or self.home is None:
                drone_lines.append((f"{mark}{link.name}  no MAVLink yet", colour))
                continue
            lat, lon, _, rel = link.position
            north, east = minimap.to_ned(lat, lon)
            heading = link.heading or 0.0
            lost = i in self.swarm.lost
            if i == self.selected:
                centre = (north, east, heading)
            else:
                objects.append((link.name, "lost" if lost else "drone", north, east, heading))
            drone_lines.append((
                f"{mark}{link.name}  {link.mode_name:<8} {'armed' if link.armed else 'disarmed':<8} "
                f"{rel:5.1f} m  hdg {heading:3.0f}  {math.hypot(*link.velocity[:2]):4.1f} m/s  "
                f"{self._health_tokens(link, lost)}{self._mission_progress(i)}", colour))
        minimap.update(centre, objects, selected_name=self.links[self.selected].name)
        swarm = self.swarm
        name = lambda i: self.links[i].name  # noqa: E731
        minimap.set_heading_to({name(i): dest for i, dest in swarm.destinations.items() if dest})
        minimap.set_routes({name(i): swarm.route(i) for i in swarm.plans})
        minimap.set_crossings(swarm.crossings)
        minimap.set_heading_goals({name(i): h for i, h in swarm.headings.items() if h is not None})
        mission = self.missions.mission
        minimap.set_mission_cameras([
            (lat, lon, f"{name(i)} {text}")
            for i in sorted(self.missions.uploaded)
            for lat, lon, text in (mission.drones[i].camera_steps if mission else [])
        ])

        for i, companion in self.companions.items():
            drone_lines.append(
                f" {name(i)} companion {companion.ip}: "
                f"{'video ok' if companion.video_ok else 'no video yet'}"
                f"{', tracking' if companion.tracking else ''}"
                f"{', STRIKE ARMED' if companion.striking else ''}"
            )
        minimap.set_fence(self.windows.fence)
        extra = list(self.extra_status()) if self.extra_status else []
        self.windows.status_lines = drone_lines + extra

    def _mission_progress(self, i):
        missions = self.missions
        if i not in missions.flying:
            return ""
        return (f"  {missions.mission.name} {self.links[i].mission_seq + 1}/"
                f"{len(missions.mission.drones[i].items)}")

    @staticmethod
    def _health_tokens(link, lost):
        """Battery, GPS and (if late) link age, compact, for the status line."""
        if lost:
            return f"LINK LOST {link.link_age:.0f}s"
        bat = "bat ?" if link.battery_pct is None else f"bat {link.battery_pct:.0f}%"
        gps = "" if link.gps_fix is None else (
            f"  GPS {GPS_FIX_NAMES.get(link.gps_fix, link.gps_fix)} {link.gps_sats}")
        age = link.link_age
        late = f"  link {age:.1f}s" if age is not None and age > 1.0 else ""
        return bat + gps + late

    # ------------------------------------------------------------------
    # Safety

    def _check_safety(self):
        self.safety.evaluate()
        self.windows.alert_line = alert_line(self.safety)
        self.panel.refresh()
        if self.sim_panel:
            self.sim_panel.refresh()

    async def _run_safety(self, period=0.5):
        while True:
            await asyncio.sleep(period)
            try:
                self._check_safety()
            except Exception as err:
                log.warning(f"Safety check failed: {err}")

    def _toggle_yaw_lock(self):
        if self.companions.get(self.selected):
            self.windows.log(f"{self.current.name}'s gimbal is X's: its yaw lock is X's "
                             "MAVLINK_YAW_LOCK setting")
            return
        if self.mission_controls(self.selected):
            return
        camera = self.current
        camera.set_yaw_lock(not camera.yaw_lock)
        self.windows.log(f"{camera.name} gimbal: " + (
            "yaw lock, holding its compass heading while the drone turns" if camera.yaw_lock
            else "yaw follow, turning with the drone"))

    def _check_target(self, geo):
        """Warn at once about a map target PX4 will refuse: outside the fence."""
        if len(self.safety.fence) >= 3 and not fence_check(self.safety.fence, *geo)[0]:
            self.windows.log("That target is outside the uploaded geofence: PX4 will refuse "
                             "to fly there")

    def upload_fence(self, corners):
        """Send the geofence to every drone, with the ceiling and the autopilot's
        action from the safety settings; an empty fence removes it. Runs in the
        background."""
        corners = list(corners)
        if 0 < len(corners) < 3:
            log.warning("A geofence needs at least 3 corners")
            return
        asyncio.get_running_loop().create_task(self.send_fence(corners))

    async def send_fence(self, corners, ceiling=None):
        """upload_fence, awaited; ceiling (m above home) replaces the safety
        settings' fence_max_alt_m. Returns the names of the drones that took it."""
        s = self.safety.settings
        if ceiling is not None and ceiling != s.fence_max_alt_m:
            log.info(f"Fence ceiling {s.fence_max_alt_m:g} m -> {ceiling:g} m")
            s.fence_max_alt_m = float(ceiling)
        self.safety.fence = corners
        self.windows.set_fence(corners)
        self.fence_status = "uploading..."
        done = []
        for link in self.links:
            if link.position is None or link.link_age is None or link.link_age > 3.0:
                log.warning(f"{link.name}: no link; geofence not uploaded")
                continue
            try:
                await asyncio.to_thread(link.upload_fence, corners)
                ap = link.ap
                if corners:
                    await asyncio.to_thread(link.set_param, ap.fence_action_param,
                                            ap.FENCE_ACTIONS[s.fence_action])
                    await asyncio.to_thread(link.set_param, ap.fence_ceiling_param,
                                            float(s.fence_max_alt_m))
                for name, value in (ap.FENCE_ON if corners else ap.FENCE_OFF).items():
                    await asyncio.to_thread(link.set_param, name, value)
                done.append(link.name)
            except Exception as err:
                log.warning(f"{link.name}: geofence upload failed: {err}")
        what = "removed from" if not corners else "uploaded to"
        self.fence_status = (f"{what} {', '.join(done)} at {time.strftime('%H:%M:%S')}"
                             if done else "upload failed")
        log.info(f"Geofence {what} {len(done)} of {len(self.links)} drones")
        return done

    async def _mission_fence(self, corners, ceiling):
        """A mission's fence: every drone must take it before the mission goes up."""
        done = await self.send_fence(corners, ceiling)
        missing = [link.name for link in self.links if link.name not in done]
        if missing:
            raise RuntimeError(f"the mission's geofence didn't reach {', '.join(missing)}")

    async def _safety_command(self, word, parts):
        """alerts, ack, fence, safety: True if it was one."""
        if word == "alerts":
            alerts = self.safety.ordered()
            for a in alerts:
                ack = " (acknowledged)" if a.acked else ""
                print(f"{'CRITICAL' if a.severity == CRITICAL else 'warning '} {a.text}{ack}")
            if not alerts:
                print("No active alerts")
        elif word == "ack":
            if len(parts) > 1 and parts[1].lower() == "all":
                self.safety.ack()
            else:
                alert, _ = self.safety.most_urgent()
                if alert:
                    self.safety.ack(alert.key)
            self.windows.alert_line = alert_line(self.safety)
        elif word == "fence" and len(parts) == 2 and parts[1].lower() in ("upload", "clear"):
            self.upload_fence(self.windows.fence if parts[1].lower() == "upload" else [])
        elif word == "fence" and len(parts) >= 3 and parts[1].lower() == "set":
            try:
                corners = [tuple(float(v) for v in p.split(",")) for p in
                           "".join(parts[2:]).split(";") if p]
                self.windows.set_fence(corners)
                print(f"Fence: {len(corners)} corners; 'fence upload' sends it")
            except ValueError:
                print("Usage: fence set LAT,LON;LAT,LON;...")
        elif word == "safety":
            s = self.safety.settings
            if len(parts) == 3:
                try:
                    s.set(parts[1], parts[2])
                except (AttributeError, ValueError) as err:
                    print(f"safety {parts[1]}: {err}")
            print("  ".join(f"{k}={v}" for k, v in vars(s).items()))
        else:
            return False
        return True

    # ------------------------------------------------------------------
    # Console

    def help_text(self):
        return "\n".join(t for t in (self.extra_help, STATION_HELP, MISSION_HELP, SWARM_HELP)
                         if t)

    def _camera_arg(self, parts, index):
        """The camera of the drone number at parts[index], else the selected one."""
        return self.cameras[int(parts[index]) - 1] if len(parts) > index else self.current

    async def handle_command(self, line):
        """Run one console command (from the terminal or the dashboard)."""
        parts = line.split()
        if not parts:
            return
        word = parts[0].lower()
        if word == "help":
            print(self.help_text())
        elif word == "select" and len(parts) == 2 and parts[1].isdigit():
            self.select(int(parts[1]))
        elif word == "gimbal" and len(parts) in (3, 4) and parts[1].lower() == "lock":
            try:
                camera = self._camera_arg(parts, 3)
                on = {"on": True, "off": False}[parts[2].lower()]
            except (KeyError, ValueError, IndexError):
                print("Usage: gimbal lock on|off [DRONE]")
                return
            index = self.cameras.index(camera)
            if not self.x_controls(index) and not self.mission_controls(index):
                camera.set_yaw_lock(on)
                print(f"{camera.name} gimbal: yaw {'lock' if on else 'follow'}")
        elif word == "gimbal" and len(parts) in (3, 4):
            try:
                pitch, yaw = float(parts[1]), float(parts[2])
                camera = self._camera_arg(parts, 3)
            except (ValueError, IndexError):
                print("Usage: gimbal PITCH YAW [DRONE]")
                return
            index = self.cameras.index(camera)
            if self.x_controls(index) or self.mission_controls(index):
                return
            camera.steer(pitch, yaw)
            if camera.selected:
                self.windows.pitch, self.windows.yaw = pitch, yaw
        elif word == "zoom" and len(parts) in (2, 3):
            try:
                camera = self._camera_arg(parts, 2)
                percent = max(0.0, min(100.0, float(parts[1])))
            except (ValueError, IndexError):
                print("Usage: zoom PERCENT [DRONE]")
                return
            if not self.x_controls(self.cameras.index(camera), "zoom"):
                camera.set_zoom(percent)
        elif word == "strike" and len(parts) in (2, 3):
            try:
                on = {"on": True, "off": False}[parts[1].lower()]
                index = int(parts[2]) - 1 if len(parts) == 3 else self.selected
                if index < 0:
                    raise IndexError
                name = self.links[index].name
            except (KeyError, ValueError, IndexError):
                print("Usage: strike on|off [DRONE]")
                return
            companion = self.companions.get(index)
            if not companion:
                print(f"{name} has no companion computer to strike with")
            elif companion.strike(on):
                print(f"{name} strike {'ARMED' if on else 'released'}")
            else:
                print(f"{name} is flying a mission that holds the camera: not striking")
        elif await self._safety_command(word, parts):
            pass
        elif await self.missions.run_command(parts):
            pass
        elif await self.swarm.run_command(parts):
            pass
        elif self.extra_commands and await self.extra_commands(line, parts):
            pass
        else:
            print(f"Unknown command: {line.strip()} (try help)")

    def _dashboard_command(self, line):
        if self.commands:
            self.commands.put_nowait((line, True))

    async def _run_console(self):
        """Run commands typed in the terminal or in the dashboard's command box.
        Replies to dashboard commands are shown in the dashboard."""
        loop = asyncio.get_running_loop()

        def read_stdin():
            # A daemon thread, so a blocked read never holds up exiting
            for line in sys.stdin:
                loop.call_soon_threadsafe(self.commands.put_nowait, (line, False))

        threading.Thread(target=read_stdin, daemon=True).start()
        print(self.help_text())
        while True:
            line, from_dashboard = await self.commands.get()
            if not from_dashboard:
                await self.handle_command(line)
                continue
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                await self.handle_command(line)
            for text in out.getvalue().splitlines():
                self.windows.log(text)

    # ------------------------------------------------------------------
    # Running

    async def _run_ui(self):
        """Keep the dashboard, zoom and video serviced."""
        last = time.monotonic()
        while True:
            now = time.monotonic()
            try:
                self.tick(now - last)
            except Exception as err:  # e.g. the sim's scene is reloading under us
                log.debug(f"UI tick skipped: {err}")
            last = now
            await asyncio.sleep(0.015)

    async def _run_map(self, period=1.0):
        while True:
            await asyncio.sleep(period)
            try:
                self.update_map()
            except Exception as err:  # e.g. the sim's scene is reloading under us
                log.debug(f"Map update skipped: {err}")

    def start_ui(self):
        """Start drawing the dashboard (needs the running event loop)."""
        self._tasks.append(asyncio.create_task(self._run_ui()))

    def start(self):
        """Start the console, map and swarm's separation monitor too."""
        self.commands = asyncio.Queue()
        if not self._tasks:
            self.start_ui()
        self._tasks.append(asyncio.create_task(self._run_console()))
        self._tasks.append(asyncio.create_task(self._run_map()))
        self._tasks.append(asyncio.create_task(self._run_safety()))
        self.swarm.start()

    def close(self):
        self.missions.close()
        for task in self._tasks:
            task.cancel()
        self.swarm.stop()
        log.removeHandler(self.dashboard_log)
        self.windows.close()
        for companion in self.companions.values():
            companion.close()
        for camera in self.cameras:
            camera.close()
