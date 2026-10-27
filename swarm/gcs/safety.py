"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

SafetyMonitor: watches every drone's health and raises alerts, and responds
where the ground station can:

  link lost            the drone is kept out of the others' way at its last
                       known position; the alert says what PX4 will do itself
  low battery          warn, then return home (or land) at the action level
  GPS                  no 3D fix, too few satellites, poor accuracy
  PX4 failsafe         PX4 switched to RTL or LAND without being told to
  geofence             outside, or close to, the fence drawn on the map
  giving way           the separation monitor is moving a drone aside
  PX4 errors           STATUSTEXT of severity ERROR or worse
  companion video      a companion's tracker video stopped

An alert is raised once, updated while it lasts and cleared when its
condition ends; each is logged. A response runs once per alert.
"""

import collections
import math
import threading
import time
from dataclasses import dataclass

from .log import log
from .mavlink_link import GPS_FIX_NAMES, M
from .settings import Settings

CRITICAL, WARNING = 2, 1
SEVERITY_NAMES = {CRITICAL: "CRITICAL", WARNING: "warning"}
EARTH_M_PER_DEG = 111_320.0
# What the autopilot does at the fence, by name; each autopilot's own values
# are its dialect's FENCE_ACTIONS (gcs/autopilot.py)
FENCE_ACTIONS = ("warn", "hold", "rtl", "land")


@dataclass
class SafetySettings(Settings):
    """Thresholds and responses; the config file's `safety:` block, or the
    dashboard's safety panel."""
    battery_warn_pct: float = 30.0
    battery_act_pct: float = 20.0
    battery_action: str = "rtl"  # rtl | land | warn
    link_lost_s: float = 3.0
    min_position_rate: float = 2.0  # positions per second; fewer is a warning
    gps_min_sats: int = 8
    gps_max_eph_m: float = 2.5
    fence_margin_m: float = 10.0
    fence_max_alt_m: float = 120.0  # above home
    fence_action: str = "rtl"  # what the autopilot does at the fence: warn | hold | rtl | land
    lost_margin_m: float = 15.0  # kept clear around a lost drone's last position

    CHOICES = {"battery_action": ("rtl", "land", "warn"), "fence_action": FENCE_ACTIONS}


@dataclass
class Alert:
    key: str  # what it is about, e.g. "battery:2"
    drone: int  # index, or -1 for the whole swarm
    severity: int
    text: str
    since: float  # time.time() it was raised
    acked: bool = False


class SafetyMonitor:
    def __init__(self, links, swarm, companions, settings=None):
        self.links = links
        self.swarm = swarm
        self.companions = companions  # drone index -> CompanionLink
        self.settings = settings or SafetySettings()
        self.active = {}  # key -> Alert
        self.history = collections.deque(maxlen=200)  # (time, severity, text, "raised"/"cleared")
        self.fence = []  # [(lat, lon)]: the geofence drawn and uploaded
        self.dll_info = {}  # drone index -> what PX4 does on link loss, e.g. "return home after 10 s"
        self._acted = set()  # alert keys whose response has run
        self._rate_mark = None  # (time, position counts), from the first check
        self._rates = [0.0] * len(links)
        self._video_seen = set()  # companions whose video has worked
        self._dll_reading = set()
        self._dll_read_time = {}  # drone index -> time.monotonic() of the last read

    # ------------------------------------------------------------------
    # Alerts

    def ordered(self):
        """Active alerts, most urgent first."""
        return sorted(self.active.values(), key=lambda a: (-a.severity, a.since))

    def most_urgent(self):
        """The most urgent unacknowledged alert and how many others are
        unacknowledged, or (None, 0)."""
        open_alerts = [a for a in self.ordered() if not a.acked]
        return (open_alerts[0], len(open_alerts) - 1) if open_alerts else (None, 0)

    def ack(self, key=None):
        """Acknowledge one alert, or all (key None). It shows again if it gets worse."""
        for alert in self.active.values():
            if key is None or alert.key == key:
                alert.acked = True

    def drone_severity(self, i):
        return max((a.severity for a in self.active.values() if a.drone == i), default=0)

    def _raise(self, key, drone, severity, text):
        alert = self.active.get(key)
        if alert is None:
            self.active[key] = Alert(key, drone, severity, text, time.time())
            self.history.append((time.time(), severity, text, "raised"))
            (log.warning if severity == CRITICAL else log.info)(f"! {text}")
        else:
            if severity > alert.severity:
                alert.acked = False  # worse: show it again
                self.history.append((time.time(), severity, text, "raised"))
                log.warning(f"! {text}")
            alert.severity, alert.text = severity, text

    def _clear(self, key):
        alert = self.active.pop(key, None)
        self._acted.discard(key)
        if alert:
            self.history.append((time.time(), alert.severity, alert.text, "cleared"))
            log.info(f"Cleared: {alert.text}")

    # ------------------------------------------------------------------
    # Evaluation (call about twice a second from the event loop)

    def evaluate(self):
        now = time.monotonic()
        self._update_rates(now)
        self.swarm.lost_margin_m = self.settings.lost_margin_m
        found = {}  # key -> (drone, severity, text)
        for i, link in enumerate(self.links):
            self._check_link(i, link, found)
            if i in self.swarm.lost:
                continue  # its telemetry is stale
            self._check_battery(i, link, found)
            self._check_gps(i, link, found)
            self._check_failsafe(i, link, found)
            self._check_fence(i, link, found)
            self._check_px4_errors(i, link, found, now)
        self._check_routes(found)
        for j, _ in self.swarm.giving_way.items():
            found[f"giving_way:{j}"] = (j, WARNING, f"{self.links[j].name} is giving way to "
                                        "another drone (climbing aside)")
        for i, companion in self.companions.items():
            if companion.video_ok:
                self._video_seen.add(i)
            elif i in self._video_seen:
                found[f"video:{i}"] = (i, WARNING, f"{self.links[i].name} companion video lost")
        for key, (drone, severity, text) in found.items():
            self._raise(key, drone, severity, text)
        for key in [k for k in self.active if k not in found]:
            self._clear(key)
        self._respond()

    def _update_rates(self, now):
        if self._rate_mark is None:
            self._rate_mark = (now, [link.position_count for link in self.links])
            return
        then, counts = self._rate_mark
        if now - then >= 2.0:
            self._rates = [(link.position_count - c) / (now - then)
                           for link, c in zip(self.links, counts)]
            self._rate_mark = (now, [link.position_count for link in self.links])

    def position_rate(self, i):
        return self._rates[i]

    def _check_link(self, i, link, found):
        s = self.settings
        age = link.link_age
        name = link.name
        if age is not None and age > s.link_lost_s:
            if i not in self.swarm.lost and link.position:
                lat, lon, amsl, _ = link.position
                self.swarm.lost[i] = (lat, lon, amsl)
            own = self.dll_info.get(i)
            found[f"link:{i}"] = (i, CRITICAL, f"{name} link lost {age:.0f} s ago"
                                  + (f"; {link.ap.label} will {own}" if own else "")
                                  + f"; others keep {self.swarm.separation_h(i, i):.0f} m from "
                                  "where it was")
            return
        if i in self.swarm.lost:
            del self.swarm.lost[i]
            log.info(f"{name} link back")
        # Re-read now and then: QGC can change it, and a restarted autopilot resets it
        stale = time.monotonic() - self._dll_read_time.get(i, -1e9) > 60.0
        if age is not None and stale and i not in self._dll_reading:
            self._read_dll(i, link)
        # (a link about to be declared lost isn't also "weak")
        if link.connected and link.armed and age is not None and age < 1.0 \
                and 0 < self._rates[i] < s.min_position_rate:
            found[f"rate:{i}"] = (i, WARNING, f"{name} link weak: {self._rates[i]:.1f} "
                                  "positions/s")

    def _read_dll(self, i, link):
        """Read what the autopilot does on link loss, in the background."""
        self._dll_reading.add(i)
        self._dll_read_time[i] = time.monotonic()

        def read():
            ap = link.ap
            action = link.get_param(ap.link_loss_action_param)
            delay = link.get_param(ap.link_loss_timeout_param)
            if action is not None:
                text = ap.LINK_LOSS_ACTIONS.get(int(action), f"action {int(action)}")
                self.dll_info[i] = text if int(action) == 0 else (
                    f"{text} after {delay:.0f} s" if delay is not None else text)
            self._dll_reading.discard(i)

        threading.Thread(target=read, daemon=True).start()

    def _check_battery(self, i, link, found):
        s, pct = self.settings, link.battery_pct
        if pct is None:
            return
        if pct <= s.battery_act_pct:
            response = {"rtl": "returning home", "land": "landing", "warn": "no action set"}[
                self.battery_action(i)]
            left = len(self.swarm.plans.get(i) or [])
            unflown = f" ({left} search waypoints left unflown)" if left > 1 else ""
            found[f"battery:{i}"] = (i, CRITICAL, f"{link.name} battery {pct:.0f}%: "
                                     f"{response}{unflown}")
        elif pct <= s.battery_warn_pct:
            found[f"battery:{i}"] = (i, WARNING, f"{link.name} battery {pct:.0f}%")

    def _check_gps(self, i, link, found):
        s = self.settings
        if link.gps_fix is None:
            return
        if link.gps_fix < M.GPS_FIX_TYPE_3D_FIX:
            found[f"gps:{i}"] = (i, CRITICAL, f"{link.name} GPS: no 3D fix "
                                 f"({GPS_FIX_NAMES.get(link.gps_fix, link.gps_fix)})")
        elif (link.gps_sats or 0) < s.gps_min_sats or (link.gps_eph_m or 0) > s.gps_max_eph_m:
            eph = f", accuracy {link.gps_eph_m:.1f} m" if link.gps_eph_m is not None else ""
            found[f"gps:{i}"] = (i, WARNING, f"{link.name} GPS weak: {link.gps_sats} satellites"
                                 f"{eph}")

    def _check_failsafe(self, i, link, found):
        commanded = self.swarm.last_action.get(i)
        end = self.swarm.mission_ends.get(i)
        if commanded == "mission" and end and link.mission_seq >= end[0] and link.mode == end[1]:
            return  # the mission's own ending
        if link.armed and link.mode in ("RTL", "LAND") and commanded not in ("rtl", "land"):
            found[f"failsafe:{i}"] = (i, WARNING, f"{link.name}: {link.ap.label} switched to "
                                      f"{link.mode_name} on its own (a failsafe?)")

    def _check_fence(self, i, link, found):
        s = self.settings
        if len(self.fence) < 3 or not link.position or not link.armed:
            return
        lat, lon, _, rel = link.position
        inside, edge_m = fence_check(self.fence, lat, lon)
        if not inside or rel > s.fence_max_alt_m:
            where = "above the fence ceiling" if inside else "outside the geofence"
            found[f"fence:{i}"] = (i, CRITICAL, f"{link.name} is {where}")
        elif edge_m < s.fence_margin_m or rel > s.fence_max_alt_m - s.fence_margin_m:
            found[f"fence:{i}"] = (i, WARNING, f"{link.name} is near the geofence "
                                   f"({edge_m:.0f} m)")

    def _check_routes(self, found):
        """A planned route (goto or search) that leaves the geofence: PX4 will
        refuse it or stop at the fence."""
        if len(self.fence) < 3:
            return
        for i, plan in self.swarm.plans.items():
            alt = self.swarm.plan_max_alts.get(i, self.swarm.plan_alts.get(i))
            outside = [p for p in plan or [] if not fence_check(self.fence, *p[:2])[0]]
            if outside or (alt is not None and alt > self.settings.fence_max_alt_m):
                what = (f"{len(outside)} of its waypoints are outside the geofence" if outside
                        else f"it flies at {alt:.0f} m, above the fence ceiling")
                found[f"route:{i}"] = (i, WARNING, f"{self.links[i].name}'s route leaves the "
                                       f"geofence: {what}; {self.links[i].ap.label} will refuse "
                                       "or stop at it")

    def _check_px4_errors(self, i, link, found, now):
        if link.status_time and now - link.status_time < 30.0 \
                and link.status_severity <= M.MAV_SEVERITY_ERROR:
            found[f"px4:{i}"] = (i, WARNING, f"{link.name} {link.ap.label}: {link.status_text}")

    # ------------------------------------------------------------------
    # Responses

    def battery_action(self, i):
        """The battery response for drone i: a fixed-wing can't land where it
        is, so it returns home instead."""
        action = self.settings.battery_action
        return "rtl" if action == "land" and not self.links[i].airframe.hovers else action

    def _respond(self):
        s = self.settings
        for key, alert in list(self.active.items()):
            if key in self._acted:
                continue
            kind, _, index = key.partition(":")
            if kind == "battery" and alert.severity == CRITICAL and s.battery_action != "warn":
                i = int(index)
                link = self.links[i]
                if link.armed and link.position and link.position[3] > 2.0:
                    self._acted.add(key)
                    self.swarm.act([i], self.battery_action(i))


def fence_check(polygon, lat, lon):
    """(inside, distance in m to the nearest edge) for a lat/lon polygon."""
    k = EARTH_M_PER_DEG * math.cos(math.radians(lat))
    pts = [((p_lon - lon) * k, (p_lat - lat) * EARTH_M_PER_DEG) for p_lat, p_lon in polygon]
    inside = False
    nearest = math.inf
    for (x1, y1), (x2, y2) in zip(pts, pts[1:] + pts[:1]):
        if (y1 > 0) != (y2 > 0) and 0 < (x2 - x1) * (0 - y1) / (y2 - y1) + x1:
            inside = not inside
        dx, dy = x2 - x1, y2 - y1
        t = max(0.0, min(1.0, -(x1 * dx + y1 * dy) / (dx * dx + dy * dy or 1e-9)))
        nearest = min(nearest, math.hypot(x1 + t * dx, y1 + t * dy))
    return inside, nearest
