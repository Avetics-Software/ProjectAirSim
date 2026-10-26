"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Plans a grid (lawnmower) search of a polygon for one or more drones.

Parallel passes `spacing` metres apart are laid across the polygon, running
along its longest edge so the drones turn as rarely as possible. The passes
are split into one contiguous block per drone, balanced by length, and each
block is flown back and forth. Concave polygons work: a pass that crosses the
polygon more than once is flown as several pieces.

Also checks planned paths against each other for crossings, for deconfliction.
"""

import math

EARTH_M_PER_DEG = 111_320.0


class LocalFrame:
    """Flat east/north metres around a reference point."""

    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0
        self.k = EARTH_M_PER_DEG * math.cos(math.radians(lat0))

    def to_xy(self, lat, lon):
        return ((lon - self.lon0) * self.k, (lat - self.lat0) * EARTH_M_PER_DEG)

    def to_geo(self, x, y):
        return (self.lat0 + y / EARTH_M_PER_DEG, self.lon0 + x / self.k)


def polygon_area_m2(xy):
    return abs(sum(x1 * y2 - x2 * y1 for (x1, y1), (x2, y2) in zip(xy, xy[1:] + xy[:1]))) / 2


def _rotate(points, angle):
    c, s = math.cos(angle), math.sin(angle)
    return [(x * c - y * s, x * s + y * c) for x, y in points]


def _lanes(poly, spacing):
    """Horizontal passes across a polygon (in its own frame): [(y, x_start,
    x_end)], bottom to top, a pass split where it leaves the polygon."""
    ys = [p[1] for p in poly]
    lanes = []
    y = min(ys) + spacing / 2
    while y < max(ys):
        xs = []
        for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
            if (y1 <= y < y2) or (y2 <= y < y1):
                xs.append(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
        xs.sort()
        for a, b in zip(xs[0::2], xs[1::2]):
            if b - a > 1.0:
                lanes.append((y, a, b))
        y += spacing
    return lanes


def _partition(costs, k):
    """Split costs into k contiguous runs minimising the largest run's sum;
    returns [(start, end)] index ranges."""
    n = len(costs)
    prefix = [0.0]
    for c in costs:
        prefix.append(prefix[-1] + c)
    inf = float("inf")
    # best[j][i]: smallest possible max-run splitting costs[:i] into j runs
    best = [[inf] * (n + 1) for _ in range(k + 1)]
    cut = [[0] * (n + 1) for _ in range(k + 1)]
    best[0][0] = 0.0
    for j in range(1, k + 1):
        for i in range(j, n + 1):
            for m in range(j - 1, i):
                worst = max(best[j - 1][m], prefix[i] - prefix[m])
                if worst < best[j][i]:
                    best[j][i], cut[j][i] = worst, m
    ranges, i = [], n
    for j in range(k, 0, -1):
        ranges.append((cut[j][i], i))
        i = cut[j][i]
    return ranges[::-1]


def plan_area_search(polygon, drones, spacing):
    """Split a [(lat, lon), ...] polygon between `drones` drones.

    Returns one waypoint list [(lat, lon), ...] per drone (some may be empty
    for a small area) and the sweep heading in degrees.
    """
    if len(polygon) < 3:
        raise ValueError("an area needs at least 3 corners")
    lat0 = sum(p[0] for p in polygon) / len(polygon)
    lon0 = sum(p[1] for p in polygon) / len(polygon)
    frame = LocalFrame(lat0, lon0)
    xy = [frame.to_xy(*p) for p in polygon]

    # Sweep along the longest edge
    edges = list(zip(xy, xy[1:] + xy[:1]))
    (x1, y1), (x2, y2) = max(edges, key=lambda e: math.dist(*e))
    angle = math.atan2(y2 - y1, x2 - x1)
    poly = _rotate(xy, -angle)  # passes become horizontal

    lanes = _lanes(poly, spacing)
    if not lanes:
        raise ValueError("the area is narrower than the pass spacing")

    # Contiguous blocks of passes, one per drone, so the longest drone's
    # flight (passes plus a turn of `spacing` between them) is as short as
    # possible
    costs = [b - a + spacing for _, a, b in lanes]
    blocks = [lanes[i:j] for i, j in _partition(costs, min(drones, len(lanes)))]
    blocks += [[] for _ in range(drones - len(blocks))]

    paths = []
    for block in blocks:
        pts = []
        for k, (y, a, b) in enumerate(block):
            ends = [(a, y), (b, y)]
            pts += ends if k % 2 == 0 else ends[::-1]
        geo = [frame.to_geo(x, y) for x, y in _rotate(pts, angle)]
        paths.append(geo)
    heading = (90.0 - math.degrees(angle)) % 180.0
    return paths, heading


def path_length_m(points):
    total = 0.0
    for (la1, lo1), (la2, lo2) in zip(points, points[1:]):
        total += math.hypot((la2 - la1) * EARTH_M_PER_DEG,
                            (lo2 - lo1) * EARTH_M_PER_DEG * math.cos(math.radians(la1)))
    return total


# ----------------------------------------------------------------------
# Path crossings


def _segment_distance(p1, p2, q1, q2):
    """Closest distance between segments p1-p2 and q1-q2 (xy) and the point on
    p1-p2 where it happens."""
    def point_seg(p, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = dx * dx + dy * dy
        t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L))
        c = (a[0] + t * dx, a[1] + t * dy)
        return math.dist(p, c), c

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    d1, d2 = cross(q1, q2, p1), cross(q1, q2, p2)
    d3, d4 = cross(p1, p2, q1), cross(p1, p2, q2)
    if d1 * d2 < 0 and d3 * d4 < 0:  # proper intersection
        t = d1 / (d1 - d2)
        return 0.0, (p1[0] + t * (p2[0] - p1[0]), p1[1] + t * (p2[1] - p1[1]))
    # Otherwise the closest approach involves an endpoint; report the point
    # halfway between the two closest points
    candidates = []
    for p, a, b in ((p1, q1, q2), (p2, q1, q2), (q1, p1, p2), (q2, p1, p2)):
        d, c = point_seg(p, a, b)
        candidates.append((d, ((p[0] + c[0]) / 2, (p[1] + c[1]) / 2)))
    return min(candidates, key=lambda c: c[0])


def find_crossings(path_a, path_b, min_sep_m):
    """Places where two [(lat, lon), ...] paths pass within min_sep_m of each
    other: [(lat, lon, distance_m)], one per pair of segments."""
    if len(path_a) < 2 or len(path_b) < 2:
        return []
    frame = LocalFrame(*path_a[0])
    a = [frame.to_xy(*p) for p in path_a]
    b = [frame.to_xy(*p) for p in path_b]
    found = []
    for p1, p2 in zip(a, a[1:]):
        for q1, q2 in zip(b, b[1:]):
            d, where = _segment_distance(p1, p2, q1, q2)
            if d < min_sep_m:
                found.append((*frame.to_geo(*where), d))
    return found
