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
LANE_OFFSET_M = {"vehicle": 1.8, "person": 0.0}
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
    cache = os.path.join(CACHE_DIR, f"{kind}_{south}_{west}_{north}_{east}.json")
    if os.path.exists(cache):
        with open(cache) as f:
            return json.load(f)

    highways = "|".join(HIGHWAYS[kind])
    query = (
        f'[out:json][timeout:90];way["highway"~"^({highways})$"]'
        f'["access"!~"^(no|private)$"]({south},{west},{north},{east});'
        "out body;>;out skel qt;"
    )
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
        self.edges = {}  # id -> [(next id, length m, two_way, weight)]
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
        two_way = forward and backward
        weight = ROAD_WEIGHTS.get(tags.get("highway"), 1)
        for a, b in zip(ids, ids[1:]):
            length = math.dist(self.nodes[a], self.nodes[b])
            if forward:
                self.edges.setdefault(a, []).append((b, length, two_way, weight))
            if backward:
                self.edges.setdefault(b, []).append((a, length, two_way, weight))

    def nearest_node(self, lat, lon, rng=None):
        point = self._to_local(lat, lon)
        return min(self.edges, key=lambda n: math.dist(self.nodes[n], point))

    def random_node(self, lat, lon, radius_m, rng):
        point = self._to_local(lat, lon)
        near = [n for n in self.edges if math.dist(self.nodes[n], point) <= radius_m]
        return rng.choice(near) if near else self.nearest_node(lat, lon)

    # Routes -------------------------------------------------------------
    def route(self, start, end):
        """Shortest path between the network points nearest two [lat, lon]."""
        source, target = self.nearest_node(*start), self.nearest_node(*end)
        dist, prev = {source: 0.0}, {}
        queue = [(0.0, source)]
        while queue:
            d, node = heapq.heappop(queue)
            if node == target:
                break
            if d > dist[node]:
                continue
            for nxt, length, two_way, _ in self.edges.get(node, []):
                if d + length < dist.get(nxt, math.inf):
                    dist[nxt] = d + length
                    prev[nxt] = (node, two_way)
                    heapq.heappush(queue, (d + length, nxt))
        if target not in dist:
            raise RuntimeError("no route between those points on the OSM network")
        path, two_way = [target], []
        while path[-1] != source:
            node, tw = prev[path[-1]]
            path.append(node)
            two_way.append(tw)
        return self._finish(path[::-1], two_way[::-1])

    def random_route(self, length_m, rng, start=None, start_radius_m=400.0):
        """A random drive/walk of about length_m, from `start` ([lat, lon]) or
        from a random point within start_radius_m of the network's centre."""
        node = (
            self.nearest_node(*start)
            if start
            else self.random_node(self.lat0, self.lon0, start_radius_m, rng)
        )
        path, two_way, travelled, previous = [node], [], 0.0, None
        visited = {node}
        while travelled < length_m:
            options = self.edges.get(node, [])
            if not options:
                break
            # Prefer roads not driven yet, then anything but straight back
            # (unless it's a dead end), favouring bigger roads
            onward = (
                [e for e in options if e[0] not in visited]
                or [e for e in options if e[0] != previous]
                or options
            )
            nxt, length, tw, _ = rng.choices(onward, weights=[e[3] for e in onward])[0]
            previous, node = node, nxt
            visited.add(node)
            path.append(node)
            two_way.append(tw)
            travelled += length
        if len(path) < 2:
            raise RuntimeError("random route start has no onward path")
        return self._finish(path, two_way)

    def _finish(self, path, two_way):
        """Node ids to [lat, lon] points, shifted into the left lane."""
        points = [self.nodes[n] for n in path]
        offset = LANE_OFFSET_M[self.kind]
        shifted = []
        for i, (n, e) in enumerate(points):
            a = points[max(i - 1, 0)]
            b = points[min(i + 1, len(points) - 1)]
            dn, de = b[0] - a[0], b[1] - a[1]
            norm = math.hypot(dn, de)
            seg_two_way = two_way[min(i, len(two_way) - 1)]
            if norm > 0 and offset and seg_two_way:
                # Left of the direction of travel is (de, -dn) in north/east
                n, e = n + offset * de / norm, e - offset * dn / norm
            if not shifted or math.dist(shifted[-1], (n, e)) > 0.5:
                shifted.append((n, e))
        return [self._to_geo(n, e) for n, e in shifted]


def path_length_m(points):
    total = 0.0
    for (lat1, lon1), (lat2, lon2) in zip(points, points[1:]):
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
