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
import sys
import threading
import time

from .companion import CompanionLink
from .dashboard import CameraWindows
from .log import log
from .minimap import MiniMap, road_segments
from .swarm import SWARM_HELP, SwarmControl
from .video import VideoStream

GIMBAL_INITIAL_PITCH = -45.0
ROAD_RADIUS_M = 1500.0  # OpenStreetMap roads loaded around home for the map

STATION_HELP = """  select N                              show and steer drone N (or keys 1-9)
  gimbal PITCH YAW [N]                  point drone N's gimbal (degrees, yaw relative to
                                        the drone) [the selected drone]
  zoom PERCENT [N]                      set the zoom (0 = widest, 100 = longest focal length)"""


class DroneCamera:
    """A drone's cameras as the dashboard sees them, for a real drone: video
    from RTSP, the gimbal steered through PX4's gimbal manager and the zoom
    through the drone's MAVLink camera. The sim's camera subclasses this."""

    def __init__(self, link, windows, video_url=None, chase_url=None):
        self.link = link
        self.name = link.name
        self.windows = windows
        self.selected = False
        self.pitch, self.yaw = GIMBAL_INITIAL_PITCH, 0.0
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
        self.link.set_gimbal_attitude(pitch_deg, yaw_deg)

    def set_zoom(self, percent):
        self.link.set_zoom(percent)

    def nudge_zoom(self, delta_percent):
        self.set_zoom(self.link.zoom_percent + delta_percent)

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
    """

    def __init__(self, links, make_camera, companions=None, layer_step=5.0, home=None,
                 extra_commands=None, extra_help="", extra_objects=None, extra_status=None):
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
        self.windows.add_controls([link.name for link in links], self._dashboard_command)
        self.dashboard_log = DashboardLog(self.windows)
        log.addHandler(self.dashboard_log)
        self.cameras = [make_camera(i, link, self.windows) for i, link in enumerate(links)]
        self.companions = {
            i: CompanionLink(ip, name=f"{links[i].name} companion")
            for i, ip in (companions or {}).items()
        }
        self.swarm = SwarmControl(links, layer_step)
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
        self.windows.set_selected(number)
        self.windows.set_companion(self.companions.get(self.selected))
        self._tracker_frame_time = 0.0
        self.windows.pitch, self.windows.yaw = self.current.pitch, self.current.yaw
        with self.windows._lock:
            self.windows._frames.clear()

    def steer(self, pitch_deg, yaw_deg):
        self.current.steer(pitch_deg, yaw_deg)

    def nudge_zoom(self, delta_percent):
        self.current.nudge_zoom(delta_percent)

    def tick(self, dt):
        for camera in self.cameras:
            camera.tick(dt)
        self.windows.zoom_text = self.current.zoom_text()
        companion = self.companions.get(self.selected)
        link = self.current.link
        if companion and link.gimbal_attitude and time.monotonic() - link.gimbal_attitude_time < 1.0:
            # The companion steers the gimbal, so show where it actually points
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
            if link.position is None or self.home is None:
                drone_lines.append(f"{mark}{link.name}  no MAVLink yet")
                continue
            lat, lon, _, rel = link.position
            north, east = minimap.to_ned(lat, lon)
            heading = link.heading or 0.0
            if i == self.selected:
                centre = (north, east, heading)
            else:
                objects.append((link.name, "drone", north, east, heading))
            stale = "" if link.connected else "   LINK LOST"
            drone_lines.append(
                f"{mark}{link.name}  {link.mode:<8} {'armed' if link.armed else 'disarmed':<8} "
                f"height {rel:5.1f} m   heading {heading:3.0f}   "
                f"speed {math.hypot(*link.velocity[:2]):4.1f} m/s{stale}"
            )
        minimap.update(centre, objects, selected_name=self.links[self.selected].name)
        swarm = self.swarm
        name = lambda i: self.links[i].name  # noqa: E731
        minimap.set_heading_to({name(i): dest for i, dest in swarm.destinations.items() if dest})
        minimap.set_routes({name(i): swarm.route(i) for i in swarm.plans})
        minimap.set_crossings(swarm.crossings)
        minimap.set_heading_goals({name(i): h for i, h in swarm.headings.items() if h is not None})

        for i, companion in self.companions.items():
            drone_lines.append(
                f" {name(i)} companion {companion.ip}: "
                f"{'video ok' if companion.video_ok else 'no video yet'}"
                f"{', tracking' if companion.tracking else ''}"
            )
        extra = list(self.extra_status()) if self.extra_status else []
        self.windows.status_lines = drone_lines + extra

    # ------------------------------------------------------------------
    # Console

    def help_text(self):
        return "\n".join(t for t in (self.extra_help, STATION_HELP, SWARM_HELP) if t)

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
        elif word == "gimbal" and len(parts) in (3, 4):
            try:
                pitch, yaw = float(parts[1]), float(parts[2])
                camera = self._camera_arg(parts, 3)
            except (ValueError, IndexError):
                print("Usage: gimbal PITCH YAW [DRONE]")
                return
            camera.steer(pitch, yaw)
            if camera.selected:
                self.windows.pitch, self.windows.yaw = pitch, yaw
        elif word == "zoom" and len(parts) in (2, 3):
            try:
                self._camera_arg(parts, 2).set_zoom(max(0.0, min(100.0, float(parts[1]))))
            except (ValueError, IndexError):
                print("Usage: zoom PERCENT [DRONE]")
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
        self.swarm.start()

    def close(self):
        for task in self._tasks:
            task.cancel()
        self.swarm.stop()
        log.removeHandler(self.dashboard_log)
        self.windows.close()
        for companion in self.companions.values():
            companion.close()
        for camera in self.cameras:
            camera.close()
