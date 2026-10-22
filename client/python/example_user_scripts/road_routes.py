"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Paths along real roads and footpaths from OpenStreetMap, for moving actors.

The photorealistic Cesium tiles have no road data, so the road network around
a point is downloaded from the Overpass API once and cached in osm_cache/.
Vehicles keep to the left lane of two-way roads (Singapore drives on the left).

Preview routes before using them in config_moving_actors.yml:
  python road_routes.py --lat 1.4134 --lon 103.8650 --kind vehicle --random 1500 \\
      --count 5 --geojson routes.geojson
then drop routes.geojson onto https://geojson.io to see them on a map.
"""

import argparse
import heapq
import json
import math
import os
import random
import time
import urllib.error
import urllib.parse
import urllib.request

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CACHE_DIR = os.path.join(SCRIPT_DIR, "osm_cache")
# The main Overpass server, then a mirror; both get busy at times
OVERPASS_URLS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
]
USER_AGENT = "ProjectAirSim-road-routes/1.0"  # Overpass refuses requests without one
EARTH_M_PER_DEG = 111_320.0

HIGHWAYS = {
    "vehicle": [
        "motorway", "trunk", "primary", "secondary", "tertiary", "unclassified",
        "residential", "living_street", "service", "motorway_link", "trunk_link",
        "primary_link", "secondary_link", "tertiary_link",
    ],
    "person": [
        "footway", "path", "pedestrian", "living_street", "residential", "service",
        "unclassified", "track", "cycleway",
    ],
}
# Airport runways and taxiways (OSM aeroway=*), which vehicles can drive along
# when given waypoints or a route; random traffic stays off them
AEROWAYS = {"vehicle": ["runway", "taxiway"], "person": []}
CACHE_VERSION = 2  # bump when the query changes, so old caches are not reused
# Vehicles drive in the centre of the leftmost (kerb-side) lane for their
# direction (Singapore drives on the left); the lane count comes from OSM's
# `lanes` tag, else these defaults
LANE_WIDTH_M = 3.5
DEFAULT_LANES_TWO_WAY = 2
DEFAULT_LANES_ONE_WAY = {"motorway": 2, "trunk": 2}  # others 1
# A route's from/to point must be this close to the road network
MAX_SNAP_M = 50.0
# How strongly random routes prefer each road type; others count 1
ROAD_WEIGHTS = {
    "motorway": 4, "trunk": 4, "primary": 4, "secondary": 4, "tertiary": 3,
    "unclassified": 2, "residential": 2, "footway": 3, "pedestrian": 3, "path": 2,
}


def load_ways(lat, lon, radius_m, kind):
    """OSM ways of `kind` within radius_m of lat/lon (downloaded or cached)."""
    dlat = radius_m / EARTH_M_PER_DEG
    dlon = radius_m / (EARTH_M_PER_DEG * math.cos(math.radians(lat)))
    # Round the box so nearby spawn points share a cache file
    south, west = round(lat - dlat, 3), round(lon - dlon, 3)
    north, east = round(lat + dlat, 3), round(lon + dlon, 3)
    cache = os.path.join(
        CACHE_DIR, f"{kind}_v{CACHE_VERSION}_{south}_{west}_{north}_{east}.json"
    )
    if os.path.exists(cache):
        with open(cache) as f:
            return json.load(f)

    bbox = f"({south},{west},{north},{east})"
    highways = "|".join(HIGHWAYS[kind])
    ways = f'way["highway"~"^({highways})$"]["access"!~"^(no|private)$"]{bbox};'
    if AEROWAYS[kind]:
        ways += f'way["aeroway"~"^({"|".join(AEROWAYS[kind])})$"]{bbox};'
    query = f"[out:json][timeout:90];({ways});out body;>;out skel qt;"
    data = _overpass(query)
    os.makedirs(CACHE_DIR, exist_ok=True)
    with open(cache, "w") as f:
        json.dump(data, f)
    return data


def _overpass(query, attempts=3):
    errors = []
    for attempt in range(attempts):
        for url in OVERPASS_URLS:
            request = urllib.request.Request(
                url,
                data=urllib.parse.urlencode({"data": query}).encode(),
                headers={"User-Agent": USER_AGENT},
            )
            try:
                with urllib.request.urlopen(request, timeout=120) as resp:
                    return json.load(resp)
            except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as err:
                errors.append(f"{url}: {err}")
        time.sleep(5 * (attempt + 1))
    raise RuntimeError("OpenStreetMap download failed: " + "; ".join(errors[-2:]))


class RoadNetwork:
    """A graph of OSM ways that finds routes as lists of [lat, lon]."""

    def __init__(self, lat, lon, radius_m=1500.0, kind="vehicle"):
        self.kind = kind
        self.lat0, self.lon0 = lat, lon
        self.nodes = {}  # id -> (north, east) metres from lat0/lon0
        self.edges = {}  # id -> [(next id, length m, lane offset m, weight)]
        self.bridges = set()  # (id, id) edges on bridges and viaducts
        data = load_ways(lat, lon, radius_m, kind)
        for el in data["elements"]:
            if el["type"] == "node":
                self.nodes[el["id"]] = self._to_local(el["lat"], el["lon"])
        for el in data["elements"]:
            if el["type"] == "way":
                self._add_way(el)
        if not self.edges:
            raise RuntimeError(f"no OSM {kind} paths within {radius_m:.0f} m of {lat}, {lon}")

    # Coordinates --------------------------------------------------------
    def _to_local(self, lat, lon):
        return (
            (lat - self.lat0) * EARTH_M_PER_DEG,
            (lon - self.lon0) * EARTH_M_PER_DEG * math.cos(math.radians(self.lat0)),
        )

    def _to_geo(self, north, east):
        return [
            self.lat0 + north / EARTH_M_PER_DEG,
            self.lon0 + east / (EARTH_M_PER_DEG * math.cos(math.radians(self.lat0))),
        ]

    # Graph --------------------------------------------------------------
    def _add_way(self, way):
        tags = way.get("tags", {})
        ids = [n for n in way["nodes"] if n in self.nodes]
        oneway = tags.get("oneway", "no")
        if self.kind == "vehicle" and tags.get("junction") in ("roundabout", "circular"):
            oneway = tags.get("oneway", "yes")
        if self.kind == "person" or oneway in ("no", "false", "0"):
            forward, backward = True, True
        elif oneway == "-1":
            forward, backward = False, True
        else:
            forward, backward = True, False
        offset = self._lane_offset(tags, forward and backward)
        # Weight 0 keeps random routes off runways and taxiways
        weight = 0 if "aeroway" in tags else ROAD_WEIGHTS.get(tags.get("highway"), 1)
        on_bridge = tags.get("bridge", "no") != "no"
        for a, b in zip(ids, ids[1:]):
            length = math.dist(self.nodes[a], self.nodes[b])
            if on_bridge:
                self.bridges.update({(a, b), (b, a)})
            if forward:
                self.edges.setdefault(a, []).append((b, length, offset, weight))
            if backward:
                self.edges.setdefault(b, []).append((a, length, offset, weight))

    def _lane_offset(self, tags, two_way):
        """Metres left of the way's centreline to the centre of the kerb-side
        lane. OSM draws a carriageway along its middle, so with N lanes the
        leftmost lane's centre is (N - 1) / 2 lane widths left of it."""
        if self.kind != "vehicle":
            return 0.0
        try:
            lanes = int(str(tags.get("lanes", "")).split(";")[0])
        except ValueError:
            lanes = 0
        if lanes < 1:
            lanes = (DEFAULT_LANES_TWO_WAY if two_way
                     else DEFAULT_LANES_ONE_WAY.get(tags.get("highway"), 1))
        return (lanes - 1) / 2 * LANE_WIDTH_M

    def nearest_node(self, lat, lon, roads_only=False):
        point = self._to_local(lat, lon)
        candidates = self._road_nodes() if roads_only else self.edges
        return min(candidates, key=lambda n: math.dist(self.nodes[n], point))

    def random_node(self, lat, lon, radius_m, rng):
        point = self._to_local(lat, lon)
        # Sorted, so the same seed picks the same start whatever order OSM sent
        near = sorted(n for n in self._road_nodes() if math.dist(self.nodes[n], point) <= radius_m)
        return rng.choice(near) if near else self.nearest_node(lat, lon, roads_only=True)

    def _road_nodes(self):
        """Nodes on roads random routes may use (not runways or taxiways)."""
        return [n for n, edges in self.edges.items() if any(e[3] > 0 for e in edges)]

    # Routes -------------------------------------------------------------
    def _snap(self, lat, lon, label):
        """The network point nearest lat/lon, refusing points far off the roads
        (the route would quietly start or end somewhere else)."""
        node = self.nearest_node(lat, lon)
        off = math.dist(self.nodes[node], self._to_local(lat, lon))
        if off > MAX_SNAP_M:
            raise ValueError(
                f"route '{label}' point {lat:.6f}, {lon:.6f} is {off:.0f} m from the "
                f"nearest {self.kind} path (limit {MAX_SNAP_M:.0f} m); move it onto a "
                "road or footpath"
            )
        return node

    def route(self, start, end):
        """Shortest path between the network points nearest two [lat, lon]."""
        source, target = self._snap(*start[:2], "from"), self._snap(*end[:2], "to")
        path, offsets = self._shortest(source, target)
        return self._finish(path, offsets)

    def route_via(self, points, closed=False):
        """Paths along the network through [lat, lon] waypoints in order, and
        back to the first one if `closed`. A leg whose ends aren't both within
        MAX_SNAP_M of the network, or that the network doesn't connect (e.g. a
        public road and an airside taxiway), is a straight line instead."""
        points = [list(p[:2]) for p in points]
        points += [points[0]] if closed else []
        nodes = [self._node_within(lat, lon, MAX_SNAP_M) for lat, lon in points]
        out = []
        run, run_offsets = [], []  # consecutive legs along the network

        def add(geo_points):
            for p in geo_points:
                if not out or path_length_m([out[-1], p]) > 0.5:
                    out.append(p)

        def flush():
            if len(run) > 1:
                add(self._finish(run, run_offsets))
            run.clear()
            run_offsets.clear()

        for i in range(len(points) - 1):
            a, b = nodes[i], nodes[i + 1]
            leg = None
            if a is not None and b is not None:
                leg = ([a], []) if a == b else self._shortest_or_none(a, b)
            if leg is None:
                flush()
                add([points[i], points[i + 1]])
                continue
            leg_nodes, leg_offsets = leg
            if run and run[-1] != leg_nodes[0]:
                flush()
            if not run:
                run.append(leg_nodes[0])
            run.extend(leg_nodes[1:])
            run_offsets.extend(leg_offsets)
        flush()
        if len(out) < 2:
            raise ValueError("all waypoints are at the same point")
        return out

    def _node_within(self, lat, lon, max_m):
        node = self.nearest_node(lat, lon)
        return node if math.dist(self.nodes[node], self._to_local(lat, lon)) <= max_m else None

    def _shortest_or_none(self, source, target):
        try:
            return self._shortest(source, target)
        except RuntimeError:
            return None

    def _shortest(self, source, target):
        """Dijkstra between two node ids: (node ids, lane offset per edge)."""
        dist, prev = {source: 0.0}, {}
        queue = [(0.0, source)]
        while queue:
            d, node = heapq.heappop(queue)
            if node == target:
                break
            if d > dist[node]:
                continue
            for nxt, length, offset, _ in self.edges.get(node, []):
                if d + length < dist.get(nxt, math.inf):
                    dist[nxt] = d + length
                    prev[nxt] = (node, offset)
                    heapq.heappush(queue, (d + length, nxt))
        if target not in dist:
            raise RuntimeError("no route between those points on the OSM network")
        path, offsets = [target], []
        while path[-1] != source:
            node, offset = prev[path[-1]]
            path.append(node)
            offsets.append(offset)
        return path[::-1], offsets[::-1]

    def random_route(self, length_m, rng, start=None, start_radius_m=400.0):
        """A random drive/walk of about length_m, from `start` ([lat, lon]) or
        from a random point within start_radius_m of the network's centre."""
        node = (
            self.nearest_node(*start[:2], roads_only=True)
            if start
            else self.random_node(self.lat0, self.lon0, start_radius_m, rng)
        )
        path, offsets, travelled, previous = [node], [], 0.0, None
        visited = {node}
        while travelled < length_m:
            options = [e for e in self.edges.get(node, []) if e[3] > 0]
            if not options:
                break
            # Prefer roads not driven yet, then anything but straight back
            # (unless it's a dead end), favouring bigger roads
            onward = (
                [e for e in options if e[0] not in visited]
                or [e for e in options if e[0] != previous]
                or options
            )
            nxt, length, offset, _ = rng.choices(onward, weights=[e[3] for e in onward])[0]
            previous, node = node, nxt
            visited.add(node)
            path.append(node)
            offsets.append(offset)
            travelled += length
        if len(path) < 2:
            raise RuntimeError("random route start has no onward path")
        return self._finish(path, offsets)

    def _finish(self, path, offsets):
        """Node ids to path points, shifted left into each edge's lane: [lat, lon],
        or [lat, lon, 1] where the point is on a bridge (the sim keeps the
        object on the deck there instead of under it)."""
        points = [self.nodes[n] for n in path]
        shifted = []
        for i, (n, e) in enumerate(points):
            a = points[max(i - 1, 0)]
            b = points[min(i + 1, len(points) - 1)]
            dn, de = b[0] - a[0], b[1] - a[1]
            norm = math.hypot(dn, de)
            offset = offsets[min(i, len(offsets) - 1)]
            if norm > 0 and offset:
                # Left of the direction of travel is (de, -dn) in north/east
                n, e = n + offset * de / norm, e - offset * dn / norm
            on_bridge = any(
                (path[j], path[j + 1]) in self.bridges
                for j in (i - 1, i) if 0 <= j < len(path) - 1
            )
            if not shifted or math.dist(shifted[-1][:2], (n, e)) > 0.5:
                shifted.append((n, e, on_bridge))
        return [self._to_geo(n, e) + ([1] if bridge else []) for n, e, bridge in shifted]


def path_length_m(points):
    """Length of [lat, lon, ...] points in metres."""
    total = 0.0
    for p, q in zip(points, points[1:]):
        (lat1, lon1), (lat2, lon2) = p[:2], q[:2]
        dn = (lat2 - lat1) * EARTH_M_PER_DEG
        de = (lon2 - lon1) * EARTH_M_PER_DEG * math.cos(math.radians(lat1))
        total += math.hypot(dn, de)
    return total


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--lat", type=float, default=1.413425)
    parser.add_argument("--lon", type=float, default=103.865)
    parser.add_argument("--radius", type=float, default=1500.0, help="network radius (m)")
    parser.add_argument("--kind", choices=["vehicle", "person"], default="vehicle")
    parser.add_argument("--random", type=float, default=1000.0, help="random route length (m)")
    parser.add_argument("--count", type=int, default=5, help="number of random routes")
    parser.add_argument("--to", nargs=2, type=float, metavar=("LAT", "LON"),
                        help="instead, the shortest route from --lat/--lon to here")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--geojson", default="routes.geojson")
    args = parser.parse_args()

    net = RoadNetwork(args.lat, args.lon, args.radius, args.kind)
    rng = random.Random(args.seed)
    if args.to:
        routes = [net.route([args.lat, args.lon], args.to)]
    else:
        routes = [net.random_route(args.random, rng) for _ in range(args.count)]
    features = [
        {
            "type": "Feature",
            "properties": {"name": f"route{i + 1}", "length_m": round(path_length_m(r))},
            "geometry": {"type": "LineString", "coordinates": [[lon, lat] for lat, lon in r]},
        }
        for i, r in enumerate(routes)
    ]
    with open(args.geojson, "w") as f:
        json.dump({"type": "FeatureCollection", "features": features}, f)
    for feat in features:
        p = feat["properties"]
        print(f"{p['name']}: {p['length_m']} m, {len(feat['geometry']['coordinates'])} points")
    print(f"Wrote {args.geojson}; view it at https://geojson.io")


if __name__ == "__main__":
    main()
