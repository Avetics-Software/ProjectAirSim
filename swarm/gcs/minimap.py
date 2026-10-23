"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

A top-down map of the area around the drone: OpenStreetMap roads, the drone
(yellow; other drones of a swarm in blue) and any other objects (in the sim,
every spawned or moving object), north up. Positions are pushed in by the
caller (about once a second); the map is drawn into a pane of the dashboard.
"""

import math
import threading

import cv2
import numpy as np

BACKGROUND = (34, 34, 34)
GRID = (52, 52, 52)
ROAD = (95, 95, 95)
TEXT = (220, 220, 220)
DRONE = (0, 215, 255)
KIND_COLOURS = {
    "vehicle": (80, 160, 255),
    "person": (120, 230, 120),
    "object": (170, 170, 170),
    "drone": (235, 170, 60),  # other drones of a swarm (blue)
    "lost": (60, 60, 255),  # a drone whose link is lost, at its last position (red)
}
TARGET = (60, 60, 255)
AREA = (200, 200, 0)  # search area being drawn
FENCE = (0, 140, 255)  # geofence (orange)
PICK = (255, 220, 120)  # heading picked on the compass, not yet sent (light blue)
SAFE, UNSAFE = (120, 220, 120), (40, 40, 255)  # path crossings
MIN_M_PER_PX, MAX_M_PER_PX = 0.25, 16.0
EARTH_M_PER_DEG = 111_320.0


class MiniMap:
    def __init__(self, m_per_px=2.0):
        self.m_per_px = m_per_px
        self.show_labels = True
        self._roads = []  # [(n1, e1, n2, e2)] metres from home
        self._drone = None  # (north, east, heading_deg)
        self._objects = []  # [(name, kind, north, east, heading_deg)]
        self.home = None  # (lat, lon) of the north/east origin, for clicks
        # Targets per drone name, (north, east): picked by clicking (not yet
        # sent) and where the drone is actually flying to
        self.picked = {}
        self.heading_to = {}
        self.selected_name = None  # the drone at the centre
        self.routes = {}  # drone name -> [(north, east)] its remaining route
        self.area = []  # [(north, east)] corners of the search area being drawn
        self.fence = []  # [(north, east)] corners of the geofence
        self.crossings = []  # [(north, east, label, safe)]
        self.heading_goals = {}  # drone name -> heading (deg) it was told to face
        self.heading_pick = None  # heading (deg) picked on the compass, not yet sent
        self.compass = None  # (cx, cy, r) in render pixels, once drawn
        self._lock = threading.Lock()
        self._cache = None
        self._cache_key = None
        self._centre = (0.0, 0.0)  # north/east at the centre of the last render

    def set_roads(self, segments):
        """Road centrelines as (north1, east1, north2, east2) metres from home."""
        with self._lock:
            self._roads = list(segments)
            self._cache_key = None

    def update(self, drone, objects, selected_name=None):
        """drone: (north, east, heading_deg) of the selected drone, or None.
        objects: [(name, kind, north, east, heading_deg)], other drones as kind
        "drone"."""
        with self._lock:
            self._drone = drone
            self._objects = list(objects)
            if selected_name:
                self.selected_name = selected_name
            self._cache_key = None

    def set_home(self, lat, lon):
        self.home = (lat, lon)

    def pixel_to_geo(self, x, y, width, height):
        """lat/lon under pixel (x, y) of a width x height render, or None."""
        if not self.home:
            return None
        north = self._centre[0] - (y - height / 2) * self.m_per_px
        east = self._centre[1] + (x - width / 2) * self.m_per_px
        lat = self.home[0] + north / EARTH_M_PER_DEG
        lon = self.home[1] + east / (EARTH_M_PER_DEG * math.cos(math.radians(self.home[0])))
        return lat, lon

    def to_ned(self, lat, lon):
        """Metres north and east of home."""
        north = (lat - self.home[0]) * EARTH_M_PER_DEG
        east = (lon - self.home[1]) * EARTH_M_PER_DEG * math.cos(math.radians(self.home[0]))
        return north, east

    def set_picked(self, name, lat_lon):
        """The target clicked for drone `name` (lat, lon), or None to clear."""
        if not self.home:
            return
        with self._lock:
            if lat_lon:
                self.picked[name] = self.to_ned(*lat_lon)
            else:
                self.picked.pop(name, None)
            self._cache_key = None

    def set_heading_to(self, destinations):
        """Where each drone is flying: {name: (lat, lon)}."""
        if not self.home:
            return
        with self._lock:
            self.heading_to = {name: self.to_ned(*ll) for name, ll in destinations.items()}
            self._cache_key = None

    def set_routes(self, routes):
        """Each drone's remaining route: {name: [(lat, lon), ...]}."""
        if not self.home:
            return
        with self._lock:
            self.routes = {n: [self.to_ned(*p[:2]) for p in r] for n, r in routes.items()}
            self._cache_key = None

    def set_area(self, corners):
        """Corners [(lat, lon), ...] of the search area being drawn."""
        if not self.home:
            return
        with self._lock:
            self.area = [self.to_ned(*p) for p in corners]
            self._cache_key = None

    def set_fence(self, corners):
        """Corners [(lat, lon), ...] of the geofence."""
        if not self.home:
            return
        with self._lock:
            self.fence = [self.to_ned(*p) for p in corners]
            self._cache_key = None

    def set_crossings(self, crossings):
        """Predicted path crossings [(lat, lon, label, safe)]."""
        if not self.home:
            return
        with self._lock:
            self.crossings = [(*self.to_ned(lat, lon), label, safe)
                              for lat, lon, label, safe in crossings]
            self._cache_key = None

    def set_heading_pick(self, heading):
        """The heading picked (compass click or typed), shown until sent; None hides it."""
        with self._lock:
            self.heading_pick = heading
            self._cache_key = None

    def compass_heading(self, x, y):
        """Degrees from north of a click at render pixel (x, y) on the compass,
        or None if the click is not on it."""
        if not self.compass:
            return None
        cx, cy, r = self.compass
        dx, dy = x - cx, y - cy
        if math.hypot(dx, dy) > r + 6:
            return None
        return math.degrees(math.atan2(dx, -dy)) % 360.0

    def set_heading_goals(self, goals):
        """Headings the drones were told to face: {name: degrees from north}."""
        with self._lock:
            self.heading_goals = dict(goals)
            self._cache_key = None

    def select(self, name):
        with self._lock:
            self.selected_name = name
            self._cache_key = None

    def zoom(self, steps):
        """Positive steps zoom in."""
        self.m_per_px = min(MAX_M_PER_PX, max(MIN_M_PER_PX, self.m_per_px * 0.8 ** steps))

    def render(self, width, height):
        key = (width, height, self.m_per_px, self.show_labels)
        if not self._drone:
            self.compass = None  # drawn only with a selected drone
        with self._lock:
            if self._cache is not None and self._cache_key == key:
                return self._cache
            roads, drone, objects = self._roads, self._drone, self._objects
        img = np.full((height, width, 3), BACKGROUND, np.uint8)
        cn, ce = (drone[0], drone[1]) if drone else (0.0, 0.0)
        self._centre = (cn, ce)
        scale = 1.0 / self.m_per_px

        def px(north, east):
            return (int(width / 2 + (east - ce) * scale), int(height / 2 - (north - cn) * scale))

        self._draw_grid(img, cn, ce, scale)
        for n1, e1, n2, e2 in roads:
            cv2.line(img, px(n1, e1), px(n2, e2), ROAD, max(1, int(6 * scale)), cv2.LINE_AA)

        visible = 0
        for name, kind, n, e, heading in objects:
            x, y = px(n, e)
            if not (0 <= x < width and 0 <= y < height):
                continue
            visible += 1
            colour = KIND_COLOURS.get(kind, KIND_COLOURS["object"])
            if kind in ("drone", "lost"):
                self._draw_drone(img, x, y, heading, colour, scale=0.75)
            elif kind == "object":
                cv2.rectangle(img, (x - 3, y - 3), (x + 3, y + 3), colour, -1)
            else:
                h = math.radians(heading)
                tip = (int(x + 9 * math.sin(h)), int(y - 9 * math.cos(h)))
                cv2.line(img, (x, y), tip, colour, 2, cv2.LINE_AA)
                cv2.circle(img, (x, y), 4, colour, -1, cv2.LINE_AA)
            if self.show_labels:
                cv2.putText(img, name, (x + 6, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                            colour, 1, cv2.LINE_AA)

        self._draw_fence(img, px)
        self._draw_area(img, px)
        self._draw_routes(img, px)
        self._draw_targets(img, px, drone, objects)
        self._draw_crossings(img, px)
        if drone:
            self._draw_drone(img, width // 2, height // 2, drone[2])
            self._draw_compass(img, width, height, drone[2])
        self._draw_legend(img, width, height, visible, len(objects))
        with self._lock:
            self._cache, self._cache_key = img, key
        return img

    def _draw_compass(self, img, width, height, heading):
        """The selected drone's heading against North, bottom right."""
        r = 38
        cx, cy = width - r - 14, height - r - 34
        self.compass = (cx, cy, r)
        cv2.circle(img, (cx, cy), r + 4, (20, 20, 20), -1, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), r, (110, 110, 110), 1, cv2.LINE_AA)
        for deg in range(0, 360, 30):
            a = math.radians(deg)
            inner = r - (7 if deg % 90 == 0 else 4)
            cv2.line(img, (int(cx + inner * math.sin(a)), int(cy - inner * math.cos(a))),
                     (int(cx + r * math.sin(a)), int(cy - r * math.cos(a))), (150, 150, 150), 1,
                     cv2.LINE_AA)
        for text, deg in (("N", 0), ("E", 90), ("S", 180), ("W", 270)):
            a = math.radians(deg)
            tx, ty = cx + (r - 15) * math.sin(a), cy - (r - 15) * math.cos(a)
            colour = (60, 60, 255) if text == "N" else TEXT
            cv2.putText(img, text, (int(tx - 4), int(ty + 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                        colour, 1, cv2.LINE_AA)
        goal = self.heading_goals.get(self.selected_name)
        if goal is not None:
            a = math.radians(goal)
            cv2.line(img, (int(cx + (r - 9) * math.sin(a)), int(cy - (r - 9) * math.cos(a))),
                     (int(cx + (r + 4) * math.sin(a)), int(cy - (r + 4) * math.cos(a))),
                     TARGET, 3, cv2.LINE_AA)
        if self.heading_pick is not None:
            # Picked but not sent: a hollow pointer from the rim
            a = math.radians(self.heading_pick)
            pts = [(cx + (r + 4) * math.sin(a), cy - (r + 4) * math.cos(a)),
                   (cx + (r - 10) * math.sin(a - 0.18), cy - (r - 10) * math.cos(a - 0.18)),
                   (cx + (r - 10) * math.sin(a + 0.18), cy - (r - 10) * math.cos(a + 0.18))]
            cv2.polylines(img, [np.array(pts, np.int32)], True, PICK, 2, cv2.LINE_AA)
        a = math.radians(heading)
        tip = (int(cx + (r - 6) * math.sin(a)), int(cy - (r - 6) * math.cos(a)))
        tail = (int(cx - 10 * math.sin(a)), int(cy + 10 * math.cos(a)))
        cv2.arrowedLine(img, tail, tip, DRONE, 2, cv2.LINE_AA, tipLength=0.3)
        label = f"{self.selected_name or ''} {heading % 360:03.0f} deg"
        if self.heading_pick is not None:
            label += f" > {self.heading_pick:03.0f}"
        (tw, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
        cv2.putText(img, label, (cx - tw // 2, cy + r + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    TEXT, 1, cv2.LINE_AA)

    def _draw_area(self, img, px):
        if not self.area:
            return
        pts = np.array([px(*p) for p in self.area], np.int32)
        if len(pts) >= 3:
            overlay = img.copy()
            cv2.fillPoly(overlay, [pts], AREA)
            cv2.addWeighted(overlay, 0.18, img, 0.82, 0, img)
        cv2.polylines(img, [pts], len(pts) >= 3, AREA, 1, cv2.LINE_AA)
        for p in pts:
            cv2.circle(img, tuple(int(v) for v in p), 3, AREA, -1, cv2.LINE_AA)

    def _draw_fence(self, img, px):
        """Dashed orange outline, so it reads apart from the search area."""
        if not self.fence:
            return
        pts = [np.array(px(*p), float) for p in self.fence]
        edges = list(zip(pts, pts[1:] + pts[:1])) if len(pts) >= 3 else list(zip(pts, pts[1:]))
        for a, b in edges:
            length = np.linalg.norm(b - a)
            for k in range(0, int(length), 12):
                p1 = a + (b - a) * (k / max(length, 1e-6))
                p2 = a + (b - a) * (min(k + 7, length) / max(length, 1e-6))
                cv2.line(img, tuple(int(v) for v in p1), tuple(int(v) for v in p2), FENCE, 2,
                         cv2.LINE_AA)
        for p in pts:
            cv2.circle(img, tuple(int(v) for v in p), 3, FENCE, -1, cv2.LINE_AA)

    def _draw_routes(self, img, px):
        """Each drone's remaining route, thin, in its colour."""
        for name, route in self.routes.items():
            if len(route) < 2:
                continue
            colour = TARGET if name == self.selected_name else KIND_COLOURS["drone"]
            pts = np.array([px(*p) for p in route], np.int32)
            cv2.polylines(img, [pts], False, colour, 1, cv2.LINE_AA)

    def _draw_crossings(self, img, px):
        for north, east, label, safe in self.crossings:
            x, y = px(north, east)
            colour = SAFE if safe else UNSAFE
            cv2.line(img, (x - 6, y - 6), (x + 6, y + 6), colour, 2, cv2.LINE_AA)
            cv2.line(img, (x - 6, y + 6), (x + 6, y - 6), colour, 2, cv2.LINE_AA)
            cv2.putText(img, label, (x + 8, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.35, colour, 1,
                        cv2.LINE_AA)

    def _draw_targets(self, img, px, drone, objects):
        """Each drone's target: where it is flying (a line from the drone) or,
        failing that, the one clicked for it. The selected drone's in red,
        the others' in blue and labelled."""
        positions = {n: (north, east) for n, kind, north, east, _ in objects if kind == "drone"}
        if drone and self.selected_name:
            positions[self.selected_name] = (drone[0], drone[1])
        for name in set(self.heading_to) | set(self.picked):
            mine = name == self.selected_name
            colour = TARGET if mine else KIND_COLOURS["drone"]
            flying = name in self.heading_to
            tx, ty = px(*(self.heading_to[name] if flying else self.picked[name]))
            if flying and name in positions:
                cv2.line(img, px(*positions[name]), (tx, ty), colour, 2 if mine else 1,
                         cv2.LINE_AA)
            size = 9 if mine else 6
            cv2.circle(img, (tx, ty), size, colour, 2 if mine else 1, cv2.LINE_AA)
            cv2.line(img, (tx - size - 5, ty), (tx + size + 5, ty), colour, 1, cv2.LINE_AA)
            cv2.line(img, (tx, ty - size - 5), (tx, ty + size + 5), colour, 1, cv2.LINE_AA)
            if not mine:
                cv2.putText(img, f"{name} target", (tx + 8, ty + 14), cv2.FONT_HERSHEY_SIMPLEX,
                            0.35, colour, 1, cv2.LINE_AA)
            # A clicked target not sent yet, for the selected drone
            if mine and flying and name in self.picked and self.picked[name] != self.heading_to[name]:
                ux, uy = px(*self.picked[name])
                cv2.circle(img, (ux, uy), 7, TARGET, 1, cv2.LINE_AA)

    def _draw_grid(self, img, cn, ce, scale):
        h, w = img.shape[:2]
        spacing = 100.0 if self.m_per_px <= 4 else 500.0
        e0 = math.floor((ce - w / 2 / scale) / spacing) * spacing
        n0 = math.floor((cn - h / 2 / scale) / spacing) * spacing
        e = e0
        while (e - ce) * scale < w / 2:
            x = int(w / 2 + (e - ce) * scale)
            cv2.line(img, (x, 0), (x, h), GRID, 1)
            e += spacing
        n = n0
        while (n - cn) * scale < h / 2:
            y = int(h / 2 - (n - cn) * scale)
            cv2.line(img, (0, y), (w, y), GRID, 1)
            n += spacing

    @staticmethod
    def _draw_drone(img, x, y, heading, colour=DRONE, scale=1.0):
        h = math.radians(heading)

        def rot(dx, dy):  # dx right, dy forward
            dx, dy = dx * scale, dy * scale
            return (int(x + dx * math.cos(h) + dy * math.sin(h)),
                    int(y + dx * math.sin(h) - dy * math.cos(h)))

        pts = np.array([rot(0, 12), rot(-7, -7), rot(0, -3), rot(7, -7)], np.int32)
        cv2.fillPoly(img, [pts], colour, cv2.LINE_AA)

    def _draw_legend(self, img, width, height, visible, total):
        # North arrow
        cv2.arrowedLine(img, (width - 20, 40), (width - 20, 14), TEXT, 2, cv2.LINE_AA, tipLength=0.4)
        cv2.putText(img, "N", (width - 26, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT, 1, cv2.LINE_AA)
        # Scale bar: a round length about a quarter of the width
        target = width / 4 * self.m_per_px
        length = next((v for v in (10, 20, 50, 100, 200, 500, 1000, 2000) if v >= target), 5000)
        bar = int(length / self.m_per_px)
        cv2.line(img, (12, height - 14), (12 + bar, height - 14), TEXT, 2)
        cv2.putText(img, f"{length} m", (12, height - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.4, TEXT, 1,
                    cv2.LINE_AA)
        cv2.putText(img, f"{visible}/{total} objects   wheel: zoom  L: labels  click: target",
                    (12, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.4, TEXT, 1, cv2.LINE_AA)


def road_segments(network):
    """Road centrelines of a road_routes.RoadNetwork whose origin is the home point."""
    seen = set()
    segments = []
    for a, edges in network.edges.items():
        for b, *_ in edges:
            if (b, a) in seen:
                continue
            seen.add((a, b))
            (n1, e1), (n2, e2) = network.nodes[a], network.nodes[b]
            segments.append((n1, e1, n2, e2))
    return segments
