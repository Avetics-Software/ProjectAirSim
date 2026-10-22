"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Flies several PX4 drones as a swarm from the run console. Commands go to each
drone's PX4 over its MAVLink payload link (payload_mavlink.PayloadLink), the
way a companion computer would send them, so QGroundControl shows what the
drones are doing and can take over at any time.

Commands name the drones first: a number, a list like 1,3, or all.
  all takeoff 30                         arm and climb to 30 m above home
  2 goto 1.4150 103.8660 40              fly drone 2 there, 40 m above home
  all goto 1.4150 103.8660 40            fly there in a line, 15 m apart
  1,3 hold | land | rtl                  hover / land here / return home and land
  all search LAT1 LON1 LAT2 LON2 [ALT] [SPACING]
                                         split the box into strips and sweep
                                         it, one strip per drone
  status                                 each drone's mode, height and position
A new command for a drone replaces what it was doing.
"""

import asyncio
import math

from projectairsim.utils import projectairsim_log

from payload_mavlink import M

EARTH_M_PER_DEG = 111_320.0
DEFAULT_ALT_M = 30.0
FORMATION_SPACING_M = 15.0
SEARCH_SPACING_M = 40.0
ARRIVE_M = 4.0
WAYPOINT_TIMEOUT_S = 180.0

SWARM_HELP = """Swarm (name drones first: 2, 1,3 or all):
  all takeoff [ALT]                     arm and climb to ALT m above home [30]
  N goto LAT LON [ALT]                  fly there; several drones form a line 15 m apart
  N hold | land | rtl                   hover / land here / return home and land
  N search LAT1 LON1 LAT2 LON2 [ALT] [SPACING]
                                        sweep the box, one strip per drone
  select N                              show drone N in the dashboard (or keys 1-9)
  status                                every drone's mode, height and position"""


def offset_geo(lat, lon, north_m, east_m):
    return (lat + north_m / EARTH_M_PER_DEG,
            lon + east_m / (EARTH_M_PER_DEG * math.cos(math.radians(lat))))


def lawnmower(lat1, lon1, lat2, lon2, spacing_m):
    """Back-and-forth north/south passes over a lat/lon box, spacing_m apart."""
    south, north = sorted((lat1, lat2))
    west, east = sorted((lon1, lon2))
    width_m = (east - west) * EARTH_M_PER_DEG * math.cos(math.radians(south))
    passes = max(1, math.ceil(width_m / spacing_m))
    step = (east - west) / passes
    points = []
    for k in range(passes):
        lon = west + (k + 0.5) * step
        ends = [(south, lon), (north, lon)]
        points += ends if k % 2 == 0 else ends[::-1]
    return points


class SwarmControl:
    def __init__(self, links):
        self.links = links  # PayloadLink per drone, drone N = links[N - 1]
        self.tasks = {}  # drone index -> asyncio task running its command
        self.destinations = {}  # drone index -> (lat, lon) it was last sent to

    # ------------------------------------------------------------------
    # Console

    def parse_targets(self, word):
        if word.lower() == "all":
            return list(range(len(self.links)))
        indices = [int(w) - 1 for w in word.split(",")]
        bad = [i + 1 for i in indices if not 0 <= i < len(self.links)]
        if bad:
            raise ValueError(f"no drone {bad[0]} (drones 1-{len(self.links)})")
        return indices

    async def run_command(self, parts):
        """Handle a swarm command; returns False if `parts` isn't one."""
        if parts and parts[0].lower() == "status":
            print("\n".join(self.status_lines()))
            return True
        if len(parts) < 2 or not (parts[0].lower() == "all" or parts[0][0].isdigit()):
            return False
        try:
            targets = self.parse_targets(parts[0])
        except ValueError as err:
            print(err)
            return True
        action, args = parts[1].lower(), parts[2:]
        try:
            if action == "takeoff":
                alt = float(args[0]) if args else DEFAULT_ALT_M
                for i in targets:
                    self._start(i, self._takeoff(i, alt))
            elif action == "goto" and len(args) >= 2:
                lat, lon = float(args[0]), float(args[1])
                alt = float(args[2]) if len(args) > 2 else None
                for k, i in enumerate(targets):
                    east = (k - (len(targets) - 1) / 2) * FORMATION_SPACING_M
                    self._start(i, self._fly_to(i, *offset_geo(lat, lon, 0.0, east), alt))
            elif action in ("hold", "land", "rtl"):
                for i in targets:
                    self._start(i, self._simple(i, action))
            elif action == "search" and len(args) >= 4:
                box = [float(a) for a in args[:4]]
                alt = float(args[4]) if len(args) > 4 else DEFAULT_ALT_M
                spacing = float(args[5]) if len(args) > 5 else SEARCH_SPACING_M
                self.search(targets, box, alt, spacing)
            else:
                print(SWARM_HELP)
        except ValueError:
            print(SWARM_HELP)
        return True

    def status_lines(self):
        lines = []
        for i, link in enumerate(self.links):
            task = self.tasks.get(i)
            doing = task.get_name() if task and not task.done() else ""
            if link.position:
                lat, lon, _, rel = link.position
                where = f"{rel:5.1f} m  {lat:.6f}, {lon:.6f}"
            else:
                where = "no position yet"
            lines.append(f"Drone{i + 1}  {link.mode:<8} {'armed' if link.armed else 'disarmed':<8} "
                         f"{where}  {doing}")
        return lines

    # ------------------------------------------------------------------
    # Commands (each runs as a task per drone)

    def _start(self, i, coro):
        old = self.tasks.get(i)
        if old and not old.done():
            old.cancel()
        task = asyncio.create_task(self._report(i, coro))
        task.set_name(coro.__name__.strip("_"))
        self.tasks[i] = task

    async def _report(self, i, coro):
        try:
            await coro
        except asyncio.CancelledError:
            raise
        except Exception as err:
            projectairsim_log().warning(f"Drone{i + 1}: {err}")

    async def _call(self, func, *args):
        # Link calls wait for PX4's acknowledgement, so keep them off the UI loop
        return await asyncio.to_thread(func, *args)

    async def _takeoff(self, i, alt):
        link = self.links[i]
        # PX4 refuses to arm until its estimator has converged
        for attempt in range(60):
            result = await self._call(link.takeoff, alt)
            if result == M.MAV_RESULT_ACCEPTED:
                projectairsim_log().info(f"Drone{i + 1}: takeoff to {alt:.0f} m")
                return
            await asyncio.sleep(1.0)
        raise RuntimeError(f"PX4 would not take off ({link.status_text or 'no reason given'})")

    async def _ensure_airborne(self, i, alt):
        link = self.links[i]
        if link.position and link.position[3] > 2.0 and link.armed:
            return
        await self._takeoff(i, alt)
        while not link.position or link.position[3] < alt - 2.0:
            await asyncio.sleep(0.5)

    async def _fly_to(self, i, lat, lon, alt=None, wait=False):
        link = self.links[i]
        if alt is None:
            alt = max(link.position[3], DEFAULT_ALT_M) if link.position else DEFAULT_ALT_M
        await self._ensure_airborne(i, alt)
        await self._call(link.fly_to, lat, lon, alt)
        self.destinations[i] = (lat, lon)
        if wait:
            loop = asyncio.get_running_loop()
            deadline = loop.time() + WAYPOINT_TIMEOUT_S
            while (link.distance_to(lat, lon) or 1e9) > ARRIVE_M:
                if loop.time() > deadline:
                    raise RuntimeError(f"didn't reach {lat:.6f}, {lon:.6f}")
                await asyncio.sleep(0.5)

    async def _simple(self, i, action):
        link = self.links[i]
        func = {"hold": link.hold, "land": link.land, "rtl": link.return_home}[action]
        self.destinations.pop(i, None)
        result = await self._call(func)
        if result not in (None, M.MAV_RESULT_ACCEPTED):
            raise RuntimeError(f"PX4 refused {action} (result {result})")

    # ------------------------------------------------------------------
    # Search

    def search(self, targets, box, alt, spacing):
        """Split the box into one strip per drone (west to east) and sweep each."""
        lat1, lon1, lat2, lon2 = box
        west, east = sorted((lon1, lon2))
        width = (east - west) / len(targets)
        for k, i in enumerate(targets):
            strip = (lat1, west + k * width, lat2, west + (k + 1) * width)
            self._start(i, self._sweep(i, strip, alt, spacing))
        print(f"Searching with {len(targets)} drones at {alt:.0f} m, passes {spacing:.0f} m apart")

    async def _sweep(self, i, strip, alt, spacing):
        points = lawnmower(*strip, spacing)
        for n, (lat, lon) in enumerate(points):
            await self._fly_to(i, lat, lon, alt, wait=True)
            projectairsim_log().debug(f"Drone{i + 1}: search waypoint {n + 1}/{len(points)}")
        projectairsim_log().info(f"Drone{i + 1}: search strip done; holding")
        await self._simple(i, "hold")

    async def wait_idle(self):
        """Wait until every drone has finished its command."""
        tasks = [t for t in self.tasks.values() if not t.done()]
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)


async def search_demo(swarm, home_lat, home_lon, heading_deg, alt=DEFAULT_ALT_M):
    """Hands-off check that the swarm flies: take off together, sweep a box
    ahead of the spawn point, then return home and land."""
    h = math.radians(heading_deg)
    centre = offset_geo(home_lat, home_lon, 150.0 * math.cos(h), 150.0 * math.sin(h))
    lat1, lon1 = offset_geo(*centre, -100.0, -60.0 * len(swarm.links))
    lat2, lon2 = offset_geo(*centre, 100.0, 60.0 * len(swarm.links))
    targets = list(range(len(swarm.links)))
    projectairsim_log().info("Swarm demo: takeoff")
    for i in targets:
        swarm._start(i, swarm._ensure_airborne(i, alt))
    await swarm.wait_idle()
    projectairsim_log().info("Swarm demo: searching")
    swarm.search(targets, (lat1, lon1, lat2, lon2), alt, SEARCH_SPACING_M)
    await swarm.wait_idle()
    projectairsim_log().info("Swarm demo: return home")
    for i in targets:
        swarm._start(i, swarm._simple(i, "rtl"))
    await swarm.wait_idle()
