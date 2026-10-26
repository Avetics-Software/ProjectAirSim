"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Flies several PX4 drones as a swarm from the run console or the dashboard.
Commands go to each drone's PX4 over MAVLink (mavlink_link.VehicleLink), the
way a companion computer or ground station would send them, so QGroundControl
shows what the drones are doing and can take over at any time.

Commands name the drones first: a number, a list like 1,3, or all.
  all takeoff 30                         arm and climb (each to its own height)
  2 goto 1.4150 103.8660 40 [HEADING]    fly drone 2 there and hover; with HEADING
                                         (degrees from north) it turns to face that
                                         way once it arrives
  2 heading 90                           turn drone 2 in place to face east
  all goto 1.4150 103.8660 40            fly there in a line, 15 m apart
  1,3 hold | land | rtl                  hover / land here / return home and land
  1,3 searcharea ALT SPACING LAT,LON;LAT,LON;...
                                         grid-search a polygon: its passes are
                                         split between the drones and each gets
                                         a PX4 mission
  all search LAT1 LON1 LAT2 LON2 [ALT] [SPACING]
                                         the same for a lat/lon box
  status                                 each drone's mode, height and position
A new command for a drone replaces what it was doing.

Deconfliction:
  - Height layers: drone N flies at the height asked for + (N - 1) x the layer
    step (5 m by default), so drones whose paths cross pass at different heights.
  - Every new path is checked against the other drones' paths; crossings closer
    than the separation distance are reported (and drawn on the map), with whether the
    height layers keep them apart.
  - A separation monitor watches every pair of airborne drones and predicts,
    from their positions and velocities, how close they will get in the next
    look-ahead seconds. If they would come within the separation horizontally
    and vertically, the higher-numbered drone gives way: it
    holds and climbs, and resumes once the other is clear_h_m away and no longer
    closing. This also covers flights commanded from QGC.
"""

import asyncio
import itertools
import math
import time

from .log import log
from .mavlink_link import M
from .search_planner import find_crossings, plan_area_search
from .settings import DeconflictionSettings, SwarmSettings

EARTH_M_PER_DEG = 111_320.0
MONITOR_PERIOD_S = 0.25
# Heights, spacings and separations are settings (gcs/settings.py), from the
# config file's swarm: and deconfliction: sections

SWARM_HELP = """Swarm (name drones first: 2, 1,3 or all):
  all takeoff [ALT]                     arm and climb to ALT m above home [swarm:
                                        default_alt_m, 30]
  N goto LAT LON [ALT] [HEADING]        fly there; on arrival turn to HEADING deg from
                                        north if given; several drones form a line
  N heading DEG                         turn in place to face DEG from north
  N hold | land | rtl                   hover / land here / return home and land
  N searcharea ALT SPACING LAT,LON;LAT,LON;...
                                        grid-search a polygon, passes split between drones
  N search LAT1 LON1 LAT2 LON2 [ALT] [SPACING]
                                        the same for a box
  select N                              show drone N in the dashboard (or keys 1-9)
  status                                every drone's mode, height and position
  Drone N flies (N-1) x the layer step higher than the height asked for."""


def offset_geo(lat, lon, north_m, east_m):
    return (lat + north_m / EARTH_M_PER_DEG,
            lon + east_m / (EARTH_M_PER_DEG * math.cos(math.radians(lat))))


def horizontal_m(a, b):
    return math.hypot((b[0] - a[0]) * EARTH_M_PER_DEG,
                      (b[1] - a[1]) * EARTH_M_PER_DEG * math.cos(math.radians(a[0])))


class SwarmControl:
    def __init__(self, links, swarm=None, deconfliction=None):
        self.links = links  # VehicleLink per drone, drone N = links[N - 1]
        self.cfg = swarm or SwarmSettings()
        self.dc = deconfliction or DeconflictionSettings()
        self.tasks = {}  # drone index -> asyncio task running its command
        # What each drone is doing, for the map and deconfliction:
        self.destinations = {}  # index -> (lat, lon) it is flying to now
        self.plans = {}  # index -> [(lat, lon), ...] its remaining route
        self.plan_alts = {}  # index -> height (m above home) of that route
        self.plan_max_alts = {}  # scripted routes may change height between waypoints
        # index -> (item, "RTL" or "LAND"): a mission's own last step, so its
        # mode change there isn't taken for a failsafe
        self.mission_ends = {}
        self.crossings = []  # [(lat, lon, text, safe)] predicted crossings
        self.giving_way = {}  # index -> what to resume once clear
        self.headings = {}  # index -> heading asked for with its last goto
        # The last command sent to each drone (takeoff, fly_to, fly_mission, hold,
        # land, rtl, ...), so PX4 changing mode on its own (a failsafe) shows
        self.last_action = {}
        # Drones whose link is lost: index -> (lat, lon, alt AMSL) last known.
        # They can't be commanded, so others give way around that spot, by
        # lost_margin_m rather than the usual separation.
        self.lost = {}
        self.lost_margin_m = 15.0
        self._monitor = None
        self._rate_mark = (time.monotonic(), [0] * len(links))  # position rate

    def start(self):
        """Start the separation monitor (needs the running event loop)."""
        if self._monitor is None or self._monitor.done():
            self._monitor = asyncio.create_task(self._separation_monitor())

    def stop(self):
        if self._monitor:
            self._monitor.cancel()

    def current_alt(self, i):
        """Drone i's height if airborne, else its layer's default height."""
        pos = self.links[i].position
        if pos and pos[3] > 2.0 and self.links[i].armed:
            return pos[3]
        return self.layered(i, self.cfg.default_alt_m)

    def layered(self, i, alt):
        """Drone i's height for a requested height: its own layer."""
        return alt + i * self.cfg.layer_step_m

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
                alt = float(args[0]) if args else self.cfg.default_alt_m
                for i in targets:
                    self._start(i, self._takeoff(i, self.layered(i, alt)))
            elif action == "heading" and len(args) == 1:
                heading = float(args[0]) % 360.0
                for i in targets:
                    self._start(i, self._turn(i, heading))
            elif action == "goto" and len(args) >= 2:
                lat, lon = float(args[0]), float(args[1])
                alt = float(args[2]) if len(args) > 2 and args[2] != "-" else None
                heading = float(args[3]) % 360.0 if len(args) > 3 else None
                for k, i in enumerate(targets):
                    east = (k - (len(targets) - 1) / 2) * self.cfg.formation_spacing_m
                    point = offset_geo(lat, lon, 0.0, east)
                    drone_alt = self.layered(i, alt) if alt is not None else self.current_alt(i)
                    self._set_plan(i, [point], drone_alt)
                    self._start(i, self._fly_to(i, *point, drone_alt, heading=heading))
                self.check_crossings(targets)
            elif action in ("hold", "land", "rtl"):
                self.act(targets, action)
            elif action == "searcharea" and len(args) >= 3:
                alt, spacing = float(args[0]), float(args[1])
                polygon = [tuple(float(v) for v in p.split(",")) for p in
                           " ".join(args[2:]).replace(" ", "").split(";") if p]
                self.search_area(targets, polygon, alt, spacing)
            elif action == "search" and len(args) >= 4:
                lat1, lon1, lat2, lon2 = (float(a) for a in args[:4])
                alt = float(args[4]) if len(args) > 4 else self.cfg.default_alt_m
                spacing = float(args[5]) if len(args) > 5 else self.cfg.search_spacing_m
                box = [(lat1, lon1), (lat1, lon2), (lat2, lon2), (lat2, lon1)]
                self.search_area(targets, box, alt, spacing)
            else:
                print(SWARM_HELP)
        except ValueError as err:
            print(f"{err}\n{SWARM_HELP}" if str(err) else SWARM_HELP)
        return True

    def status_lines(self):
        now = time.monotonic()
        then, counts = self._rate_mark
        rates = [(link.position_count - c) / max(now - then, 1e-3)
                 for link, c in zip(self.links, counts)]
        self._rate_mark = (now, [link.position_count for link in self.links])
        lines = []
        for i, link in enumerate(self.links):
            task = self.tasks.get(i)
            doing = task.get_name() if task and not task.done() else ""
            if i in self.giving_way:
                doing += " (giving way)"
            if link.position:
                lat, lon, _, rel = link.position
                where = f"{rel:5.1f} m  {lat:.6f}, {lon:.6f}"
            else:
                where = "no position yet"
            lines.append(f"{self.links[i].name}  {link.mode:<8} {'armed' if link.armed else 'disarmed':<8} "
                         f"{where}  position {rates[i]:.0f}/s  {doing}")
        return lines

    # ------------------------------------------------------------------
    # Commands (each runs as a task per drone)

    def _start(self, i, coro):
        old = self.tasks.get(i)
        if old and not old.done():
            old.cancel()
        self.giving_way.pop(i, None)
        task = asyncio.create_task(self._report(i, coro))
        task.set_name(coro.__name__.strip("_"))
        # A command replaced before it started never runs; close it quietly
        task.add_done_callback(lambda _, c=coro: c.close())
        self.tasks[i] = task
        self.last_action[i] = task.get_name()

    def act(self, indices, action):
        """hold, land or rtl for these drones (e.g. a safety response)."""
        for i in indices:
            self._start(i, self._simple(i, action))
            self.last_action[i] = action

    def hold_all(self):
        self.act(range(len(self.links)), "hold")

    def rtl_all(self):
        self.act(range(len(self.links)), "rtl")

    async def _report(self, i, coro):
        try:
            await coro
        except asyncio.CancelledError:
            raise
        except Exception as err:
            log.warning(f"{self.links[i].name}: {err}")

    async def _call(self, func, *args):
        # Link calls wait for PX4's acknowledgement, so keep them off the UI loop
        return await asyncio.to_thread(func, *args)

    def _set_plan(self, i, points, alt):
        self.plans[i] = list(points)
        self.plan_alts[i] = alt
        self.plan_max_alts[i] = alt
        self.destinations[i] = points[0] if points else None

    def _clear_plan(self, i):
        self.plans.pop(i, None)
        self.plan_alts.pop(i, None)
        self.plan_max_alts.pop(i, None)
        self.destinations.pop(i, None)

    async def _arm_and(self, i, action, *args):
        """Retry an action that needs PX4 armed and its estimator settled."""
        link = self.links[i]
        for _ in range(60):
            if not link.armed:
                # PX4 won't arm in a manual mode without a radio; it can boot
                # into one when its position estimate isn't ready yet
                if link.mode in ("MANUAL", "STABILIZED", "ALTCTL", "POSCTL", "ACRO", "?"):
                    await self._call(link.hold)
                await self._call(link.arm)
            result = await self._call(action, *args)
            if result == M.MAV_RESULT_ACCEPTED:
                return
            await asyncio.sleep(1.0)
        raise RuntimeError(f"PX4 refused ({link.status_text or 'no reason given'})")

    async def _takeoff(self, i, alt):
        await self._arm_and(i, self.links[i].takeoff, alt)
        log.info(f"{self.links[i].name}: takeoff to {alt:.0f} m")

    async def _ensure_airborne(self, i, alt):
        link = self.links[i]
        if link.position and link.position[3] > 2.0 and link.armed:
            return
        await self._takeoff(i, alt)
        while not link.position or link.position[3] < alt - 2.0:
            await asyncio.sleep(0.5)

    async def _turn(self, i, heading):
        link = self.links[i]
        if not (link.position and link.position[3] > 2.0 and link.armed):
            raise RuntimeError("take off before turning")
        await self._call(link.turn_to, heading)
        self.headings[i] = heading
        log.info(f"{self.links[i].name}: turning to {heading:.0f} deg")

    async def _fly_to(self, i, lat, lon, alt=None, wait=False, heading=None):
        link = self.links[i]
        if alt is None:
            default = self.cfg.default_alt_m
            alt = max(link.position[3], default) if link.position else default
            self.plan_alts[i] = alt
            self.plan_max_alts[i] = alt
        await self._ensure_airborne(i, alt)
        # Fly there keeping the current heading (said explicitly: without a yaw
        # PX4 turns the drone to face the way it flies); a heading asked for is
        # only applied on arrival
        await self._call(link.fly_to, lat, lon, alt, -1.0, link.heading)
        self.destinations[i] = (lat, lon)
        self.headings[i] = heading
        if wait or heading is not None:
            loop = asyncio.get_running_loop()
            distance = link.distance_to(lat, lon) or 0.0
            deadline = loop.time() + self.cfg.waypoint_timeout_s + distance / 2.0
            while (link.distance_to(lat, lon) or 1e9) > self.cfg.arrive_m or i in self.giving_way:
                if loop.time() > deadline:
                    raise RuntimeError(f"didn't reach {lat:.6f}, {lon:.6f}")
                await asyncio.sleep(0.5)
        if heading is not None:
            await self._call(link.fly_to, lat, lon, alt, -1.0, heading)
            log.info(f"{self.links[i].name}: arrived; turning to {heading:.0f} deg")

    async def _simple(self, i, action):
        link = self.links[i]
        func = {"hold": link.hold, "land": link.land, "rtl": link.return_home}[action]
        self._clear_plan(i)
        result = await self._call(func)
        if result not in (None, M.MAV_RESULT_ACCEPTED):
            raise RuntimeError(f"PX4 refused {action} (result {result})")

    # ------------------------------------------------------------------
    # Area search (PX4 missions)

    def search_area(self, targets, polygon, alt, spacing):
        """Grid-search a [(lat, lon), ...] polygon with the target drones."""
        paths, heading = plan_area_search(polygon, len(targets), spacing)
        used = 0
        for i, path in zip(targets, paths):
            if not path:
                log.info(f"{self.links[i].name}: area too small to need it")
                continue
            used += 1
            drone_alt = self.layered(i, alt)
            self._set_plan(i, path, drone_alt)
            self._start(i, self._fly_mission(i, path, drone_alt))
        log.info(
            f"Searching the area with {used} drone{'s' if used != 1 else ''}: passes "
            f"{spacing:.0f} m apart, heading {heading:.0f} deg"
        )
        self.check_crossings(targets)

    async def _fly_mission(self, i, path, alt):
        link = self.links[i]
        airborne = bool(link.position and link.position[3] > 2.0 and link.armed)
        first, count = await self._call(link.upload_mission, path, alt, not airborne)
        log.info(f"{self.links[i].name}: mission of {len(path)} waypoints uploaded")
        await self._arm_and(i, link.start_mission)
        # Follow progress for the map until the last waypoint is reached
        while link.mission_reached < count - 1:
            k = link.mission_seq - first
            if 0 <= k < len(path):
                self.plans[i] = path[k:]
                self.destinations[i] = path[k]
            await asyncio.sleep(0.5)
        self._clear_plan(i)
        log.info(f"{self.links[i].name}: search done; holding at the last waypoint")

    async def wait_idle(self):
        """Wait until every drone has finished its command."""
        tasks = [t for t in self.tasks.values() if not t.done()]
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

    # ------------------------------------------------------------------
    # Deconfliction

    def route(self, i):
        """Drone i's remaining route from where it is now, or []."""
        plan = self.plans.get(i)
        link = self.links[i]
        if not plan or not link.position:
            return []
        return [link.position[:2]] + list(plan)

    def check_crossings(self, changed):
        """Report where the changed drones' routes come close to other drones'
        routes, and whether the height layers keep them apart."""
        crossings = []
        for i, j in itertools.combinations(range(len(self.links)), 2):
            if i not in changed and j not in changed:
                continue
            a, b = self.route(i), self.route(j)
            found = []  # one report per place where the paths meet
            for lat, lon, d in find_crossings(a, b, self.dc.separation_h_m):
                if all(horizontal_m((lat, lon), f) > 20.0 for f in found):
                    found.append((lat, lon))
            for lat, lon in found[:3]:
                alt_i, alt_j = self.plan_alts.get(i), self.plan_alts.get(j)
                gap = abs(alt_i - alt_j) if alt_i is not None and alt_j is not None else 0.0
                safe = gap >= self.dc.separation_v_m
                text = (f"{self.links[i].name} and {self.links[j].name} paths cross at {lat:.6f}, {lon:.6f}: "
                        + (f"{gap:.0f} m apart in height, OK" if safe else
                           "same height; the separation monitor will make one give way"))
                crossings.append((lat, lon, f"{i + 1}x{j + 1}", safe))
                (log.info if safe else log.warning)(text)
        # Keep crossings involving drones that didn't change
        self.crossings = [c for c in self.crossings
                          if not any(str(k + 1) in c[2].split("x") for k in changed)] + crossings

    async def _separation_monitor(self):
        while True:
            await asyncio.sleep(MONITOR_PERIOD_S)
            try:
                await self._check_separation()
            except Exception as err:
                log.debug(f"Separation check skipped: {err}")

    def _state(self, i, now):
        """Drone i's (north, east, up) metres from a common origin and velocity,
        extrapolated to now. A lost drone stays where it was last seen."""
        link = self.links[i]
        if i in self.lost:
            lat, lon, amsl = self.lost[i]
            vn = ve = vd = dt = 0.0
        else:
            lat, lon, amsl, _ = link.position
            vn, ve, vd = link.velocity
            dt = max(0.0, min(1.0, now - link.position_time))
        north = (lat - self.links[0].position[0]) * EARTH_M_PER_DEG + vn * dt
        east = ((lon - self.links[0].position[1]) * EARTH_M_PER_DEG
                * math.cos(math.radians(lat)) + ve * dt)
        return (north, east, amsl - vd * dt), (vn, ve, -vd)

    def predict(self, i, j, now):
        """Closest approach of drones i and j within the look-ahead, assuming they
        keep their velocities: (horizontal m, vertical m, seconds from now)."""
        (pi, vi), (pj, vj) = self._state(i, now), self._state(j, now)
        r = [b - a for a, b in zip(pi, pj)]
        v = [b - a for a, b in zip(vi, vj)]
        vv = v[0] ** 2 + v[1] ** 2
        t = 0.0 if vv < 1e-6 else max(0.0, min(self.dc.lookahead_s,
                                                -(r[0] * v[0] + r[1] * v[1]) / vv))
        dh = math.hypot(r[0] + v[0] * t, r[1] + v[1] * t)
        dv = abs(r[2] + v[2] * t)
        return dh, dv, t

    def separation_h(self, i, j):
        """Horizontal separation to keep between drones i and j: wider around a
        lost drone, and widened by both drones' GPS inaccuracy."""
        if i in self.lost or j in self.lost:
            return self.lost_margin_m
        eph = sum(self.links[k].gps_eph_m or 0.0 for k in (i, j))
        return self.dc.separation_h_m + min(eph, self.dc.max_gps_margin_m)

    async def _check_separation(self):
        if not self.links[0].position:
            return
        now = time.monotonic()
        airborne = [i for i, link in enumerate(self.links)
                    if link.position and link.position[3] > 2.0 and link.armed]
        for i, j in itertools.combinations(airborne, 2):
            if i in self.lost and j in self.lost:
                continue
            dh, dv, t = self.predict(i, j, now)
            if dh < self.separation_h(i, j) and dv < self.dc.separation_v_m \
                    and j not in self.giving_way and i not in self.giving_way:
                # The higher-numbered drone gives way, unless it can't be reached
                yielder, other = (i, j) if j in self.lost else (j, i)
                await self._give_way(yielder, other, dh, dv, t)
        # Resume drones that are clear of everyone again
        for j, state in list(self.giving_way.items()):
            pj = self.links[j].position
            if not pj:
                continue
            clear = True
            for k in airborne:
                if k == j:
                    continue
                pk = self.lost[k][:2] if k in self.lost else self.links[k].position[:2]
                dh_now = horizontal_m(pj[:2], pk)
                dh, dv, _ = self.predict(j, k, now)
                clear_h = max(self.dc.clear_h_m, self.separation_h(j, k) + 4.0)
                if dh_now < clear_h or (dh < self.separation_h(j, k)
                                        and dv < self.dc.separation_v_m + 2):
                    clear = False
            if clear:
                del self.giving_way[j]
                await self._resume(j, state)

    async def _give_way(self, j, i, dh, dv, t):
        link = self.links[j]
        lat, lon, _, rel = link.position
        state = {"mode": link.mode, "destination": self.destinations.get(j),
                 "alt": self.plan_alts.get(j, rel), "heading": self.headings.get(j)}
        self.giving_way[j] = state
        when = "now" if t < 0.5 else f"in {t:.1f} s"
        log.warning(
            f"{self.links[j].name} would pass {dh:.1f} m from {self.links[i].name} ({dv:.1f} m in height) "
            f"{when}: giving way, climbing {self.dc.give_way_climb_m:.0f} m"
        )
        await self._call(link.fly_to, lat, lon, rel + self.dc.give_way_climb_m)

    async def _resume(self, j, state):
        link = self.links[j]
        log.info(f"{self.links[j].name}: clear again, resuming")
        if state["mode"] == "MISSION":
            await self._call(link.resume_mission)
        elif state["destination"]:
            # A heading asked for is applied by the goto itself, on arrival
            await self._call(link.fly_to, *state["destination"], state["alt"])
        else:
            await self._call(link.hold)


async def search_demo(swarm, home_lat, home_lon, heading_deg, alt=None):
    """Hands-off check that the swarm flies: search a box ahead of the spawn
    point together, then return home and land."""
    h = math.radians(heading_deg)
    centre = offset_geo(home_lat, home_lon, 150.0 * math.cos(h), 150.0 * math.sin(h))
    lat1, lon1 = offset_geo(*centre, -100.0, -60.0 * len(swarm.links))
    lat2, lon2 = offset_geo(*centre, 100.0, 60.0 * len(swarm.links))
    targets = list(range(len(swarm.links)))
    alt = swarm.cfg.default_alt_m if alt is None else alt
    log.info("Swarm demo: searching")
    box = [(lat1, lon1), (lat1, lon2), (lat2, lon2), (lat2, lon1)]
    swarm.search_area(targets, box, alt, swarm.cfg.search_spacing_m)
    await swarm.wait_idle()
    log.info("Swarm demo: return home")
    for i in targets:
        swarm._start(i, swarm._simple(i, "rtl"))
    await swarm.wait_idle()
