"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

A top-down map of the area around the drone: OpenStreetMap roads, the drone
and every spawned or moving object, north up. Positions are pushed in by the
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
}
MIN_M_PER_PX, MAX_M_PER_PX = 0.25, 16.0


class MiniMap:
    def __init__(self, m_per_px=2.0):
        self.m_per_px = m_per_px
        self.show_labels = True
        self._roads = []  # [(n1, e1, n2, e2)] metres from home
        self._drone = None  # (north, east, heading_deg)
        self._objects = []  # [(name, kind, north, east, heading_deg)]
        self._lock = threading.Lock()
        self._cache = None
        self._cache_key = None

    def set_roads(self, segments):
        """Road centrelines as (north1, east1, north2, east2) metres from home."""
        with self._lock:
            self._roads = list(segments)
            self._cache_key = None

    def update(self, drone, objects):
        """drone: (north, east, heading_deg) or None.
        objects: [(name, kind, north, east, heading_deg)]."""
        with self._lock:
            self._drone = drone
            self._objects = list(objects)
            self._cache_key = None

    def zoom(self, steps):
        """Positive steps zoom in."""
        self.m_per_px = min(MAX_M_PER_PX, max(MIN_M_PER_PX, self.m_per_px * 0.8 ** steps))

    def render(self, width, height):
        key = (width, height, self.m_per_px, self.show_labels)
        with self._lock:
            if self._cache is not None and self._cache_key == key:
                return self._cache
            roads, drone, objects = self._roads, self._drone, self._objects
        img = np.full((height, width, 3), BACKGROUND, np.uint8)
        cn, ce = (drone[0], drone[1]) if drone else (0.0, 0.0)
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
            if kind == "object":
                cv2.rectangle(img, (x - 3, y - 3), (x + 3, y + 3), colour, -1)
            else:
                h = math.radians(heading)
                tip = (int(x + 9 * math.sin(h)), int(y - 9 * math.cos(h)))
                cv2.line(img, (x, y), tip, colour, 2, cv2.LINE_AA)
                cv2.circle(img, (x, y), 4, colour, -1, cv2.LINE_AA)
            if self.show_labels:
                cv2.putText(img, name, (x + 6, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                            colour, 1, cv2.LINE_AA)

        if drone:
            self._draw_drone(img, width // 2, height // 2, drone[2])
        self._draw_legend(img, width, height, visible, len(objects))
        with self._lock:
            self._cache, self._cache_key = img, key
        return img

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
    def _draw_drone(img, x, y, heading):
        h = math.radians(heading)

        def rot(dx, dy):  # dx right, dy forward
            return (int(x + dx * math.cos(h) + dy * math.sin(h)),
                    int(y + dx * math.sin(h) - dy * math.cos(h)))

        pts = np.array([rot(0, 12), rot(-7, -7), rot(0, -3), rot(7, -7)], np.int32)
        cv2.fillPoly(img, [pts], DRONE, cv2.LINE_AA)

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
        cv2.putText(img, f"{visible}/{total} objects in view  (wheel: zoom, L: labels)",
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
