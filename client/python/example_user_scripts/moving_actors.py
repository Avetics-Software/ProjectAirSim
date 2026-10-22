"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

People and vehicles that move along paths in the Cesium world, from
config_moving_actors.yml or from commands typed while the sim runs. The sim
moves them every frame (SetObjectPath), so dozens can move at once.

Where every actor is gets logged to a CSV (sim time, name, kind, lat, lon,
alt, heading) as ground truth for scoring detection and re-identification,
with how visible it is from the drone's camera (in view, visible fraction,
under cover) for scoring detection under occlusion.
"""

import csv
import glob
import math
import os
import random
import time

import yaml

from projectairsim.utils import geo_to_ned_coordinates, projectairsim_log, quaternion_to_rpy
from road_routes import RoadNetwork, path_length_m
from spawned_objects import ObjectSpawner

# Used for anything neither the actor nor the config's `defaults` set
KIND_DEFAULTS = {
    "person": {
        "asset": "SKM_Manny_Simple",
        "speed": 1.4,
        "yaw_offset": -90.0,
        "walk_animation": "MF_Unarmed_Walk_Fwd",
        "idle_animation": "MM_Idle",
        "animation_speed": 1.6,
    },
    "vehicle": {
        "asset": "SKM_SportsCar",
        "length": 4.5,
        "speed": 10.0,
        "align_to_ground": True,
    },
}
EARTH_M_PER_DEG = 111_320.0
GROUND_TRUTH_PERIOD_S = 0.2
GROUND_TRUTH_KEEP = 5  # ground-truth logs kept in the log directory
ROAD_RADIUS_M = 1500.0  # OSM network downloaded around the drone's home

ACTORS_HELP = """  actors [list]                         moving people and vehicles
  actors pause | resume                 stop or restart all of them
  actors reload                         delete them, then respawn from the config file
  actors delete NAME                    remove one"""


def follows_roads(opts):
    """Waypoints are joined along roads/footpaths unless `follow_roads: false`;
    by default vehicles follow roads and people walk straight between points."""
    return bool(opts.get("follow_roads", opts.get("kind") == "vehicle"))


def resolve_path(opts, network):
    """[lat, lon] points of an actor's path from its config entry. `network(kind)`
    returns the RoadNetwork around home. Shared with actor_editor.py, so its
    preview shows exactly the path the sim will use."""
    if "waypoints" in opts:
        points = [(float(p[0]), float(p[1])) for p in opts["waypoints"]]
        if len(points) < 2:
            raise ValueError("waypoints needs at least two [lat, lon] points")
        if follows_roads(opts):
            # Along the roads between waypoints; a loop's last leg goes back to
            # the first waypoint along the roads too
            closed = opts.get("mode", "loop") == "loop"
            return network(opts["kind"]).route_via(points, closed=closed)
        return points
    if "route" in opts:
        route = opts["route"] or {}
        if not route.get("from") or not route.get("to"):
            raise ValueError("route needs both `from` (A) and `to` (B) points")
        return network(opts["kind"]).route(route["from"], route["to"])
    if "random_route" in opts:
        spec = opts["random_route"] or {}
        # Seeded by name, so `actors reload` and restarts drive the same routes
        rng = random.Random(f"{opts['name']}:{spec.get('seed', 0)}")
        return network(opts["kind"]).random_route(
            float(spec.get("length_m", 1000.0)),
            rng,
            start=spec.get("start"),
            start_radius_m=float(spec.get("start_radius_m", 400.0)),
        )
    raise ValueError("needs waypoints, route or random_route")


def plan_path(opts, network):
    """The path and follower mode the sim will use, and a note for the user:
    ([lat, lon] points, "once" | "loop" | "pingpong", str).

    A vehicle on roads never drives back over its outbound path, which would
    take it the wrong way down one-way streets and onto the wrong side of the
    road. Instead it gets a legal return route from the end to the start and
    loops the round trip ("pingpong" and "loop" both do this)."""
    points = resolve_path(opts, network)
    mode = opts.get("mode", "loop" if "waypoints" in opts else "pingpong")
    if mode not in ("once", "loop", "pingpong"):
        raise ValueError(f"mode must be once, loop or pingpong, not {mode}")
    closed = path_length_m([points[0], points[-1]]) < 10.0
    on_roads = "waypoints" not in opts or follows_roads(opts)
    if mode == "once":
        return points, mode, ""
    if opts["kind"] == "vehicle" and on_roads:
        if closed:
            return points, "loop", ""
        try:
            back = _return_path(opts, network, points)
        except (ValueError, RuntimeError) as err:
            return points, "pingpong", f"no legal way back ({err}); reversing instead"
        return points + back[1:], "loop", ""
    if mode == "loop" and not closed and "waypoints" not in opts:
        # Looping would cut straight across from the end back to the start
        return points, "pingpong", "path doesn't end where it starts; using pingpong"
    return points, mode, ""


def _return_path(opts, network, points):
    """A legal route from the end of an actor's path back to its start."""
    roads = network(opts["kind"])
    if "waypoints" in opts:
        return roads.route_via([tuple(p) for p in reversed(opts["waypoints"])])
    if "route" in opts:
        return roads.route(opts["route"]["to"], opts["route"]["from"])
    return roads.route(points[-1][:2], points[0][:2])


def expand_actors(config):
    """Config entries with defaults applied and `count` copies expanded, as
    (kind, opts) pairs in spawn order."""
    defaults = config.get("defaults") or {}
    expanded = []
    for i, entry in enumerate(config.get("actors") or []):
        kind = entry.get("kind", "person")
        opts = {**KIND_DEFAULTS.get(kind, {}), **(defaults.get(kind) or {}), **entry}
        opts.setdefault("name", f"{kind.capitalize()}{i + 1}")
        opts["kind"] = kind
        count = int(opts.pop("count", 1))
        if count > 1 and "random_route" not in opts:
            projectairsim_log().warning(f"{opts['name']}: count needs random_route; spawning one")
            count = 1
        for n in range(count):
            name = f"{opts['name']}_{n + 1}" if count > 1 else opts["name"]
            expanded.append((kind, dict(opts, name=name)))
    return expanded


class MovingActorManager:
    def __init__(self, world, config_path, log_dir=None, camera=None):
        """camera: (robot name, camera id) whose view the ground-truth log
        scores visibility from, e.g. ("Drone1", "GimbalCam")."""
        self.world = world
        self.camera = camera
        self.config_path = config_path
        # A spawner of its own, so `delete all` / `reload` for static objects
        # leave the moving actors alone
        self.spawner = ObjectSpawner(world, None)
        self.actors = {}  # name -> {"kind", "yaw_offset", "desc"}
        self.log_dir = log_dir
        self.log_file = None
        self.log_writer = None
        self.last_log = 0.0
        self.networks = {}  # kind -> RoadNetwork around home

    # ------------------------------------------------------------------
    # Spawning

    def spawn_from_config(self):
        if not self.config_path or not os.path.exists(self.config_path):
            return
        with open(self.config_path) as f:
            config = yaml.safe_load(f) or {}
        base_dir = os.path.dirname(os.path.abspath(self.config_path))
        for kind, opts in expand_actors(config):
            if opts.get("file"):
                opts["file"] = os.path.join(base_dir, os.path.expanduser(opts["file"]))
                opts.pop("asset", None)
            try:
                self.spawn(kind, opts)
            except Exception as err:
                projectairsim_log().warning(f"Could not spawn moving actor {opts['name']}: {err}")
        self._open_log()

    def spawn(self, kind, opts):
        """Spawn one actor at the start of its path and set it moving."""
        opts = dict(opts, kind=kind)
        geo_path, mode, note = plan_path(opts, self._network)
        if note:
            projectairsim_log().warning(f"{opts['name']}: {note}")
        # [north, east] or [north, east, 1] on bridges
        ned_path = [self._to_ned(p[0], p[1]) + list(p[2:3]) for p in geo_path]
        yaw_offset = float(opts.get("yaw_offset", 0.0))
        (n0, e0), (n1, e1) = ned_path[0][:2], ned_path[1][:2]
        heading = math.degrees(math.atan2(e1 - e0, n1 - n0))

        name = self.spawner.spawn(
            name=opts["name"],
            lat=geo_path[0][0],
            lon=geo_path[0][1],
            heading=heading + yaw_offset,
            asset=None if opts.get("file") else opts["asset"],
            model_file=opts.get("file"),
            length=opts.get("length"),
            scale=opts.get("scale"),
        )
        speed = float(opts["speed"])
        self.world.set_object_path(
            name,
            ned_path,
            speed,
            mode=mode,
            yaw_offset=yaw_offset,
            align_to_ground=bool(opts.get("align_to_ground", False)),
            walk_animation=opts.get("walk_animation", ""),
            idle_animation=opts.get("idle_animation", ""),
            animation_speed=float(opts.get("animation_speed", 0.0)),
            start_delay=float(opts.get("start_delay", 0.0)),
        )
        length = sum(math.dist(a[:2], b[:2]) for a, b in zip(ned_path, ned_path[1:]))
        self.actors[name] = {
            "kind": kind,
            "yaw_offset": yaw_offset,
            "desc": f"{kind}, {speed:g} m/s, {mode}, {len(ned_path)} points / {length:.0f} m",
        }
        return name

    def _network(self, kind):
        if kind not in self.networks:
            home = self.world.home_geo_point
            projectairsim_log().info(f"Loading OpenStreetMap {kind} paths around home")
            self.networks[kind] = RoadNetwork(
                home["latitude"], home["longitude"], ROAD_RADIUS_M, kind
            )
        return self.networks[kind]

    def _to_ned(self, lat, lon):
        home = self.world.home_geo_point
        north, east, _ = geo_to_ned_coordinates(home, [lat, lon, home["altitude"]])
        return [north, east]

    def _to_geo(self, north, east):
        home = self.world.home_geo_point
        lat = home["latitude"] + north / EARTH_M_PER_DEG
        lon = home["longitude"] + east / (
            EARTH_M_PER_DEG * math.cos(math.radians(home["latitude"]))
        )
        return lat, lon

    def delete(self, name):
        if name == "all":
            for actor in list(self.actors):
                self.delete(actor)
            return
        self.spawner.delete(name)
        self.actors.pop(name, None)

    # ------------------------------------------------------------------
    # Ground truth

    def _open_log(self):
        if not self.log_dir or not self.actors:
            return
        os.makedirs(self.log_dir, exist_ok=True)
        old = sorted(glob.glob(os.path.join(self.log_dir, "ground_truth_*.csv")))
        for path in old[: max(len(old) - (GROUND_TRUTH_KEEP - 1), 0)]:
            os.remove(path)
        path = os.path.join(self.log_dir, f"ground_truth_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        self.log_file = open(path, "w", newline="")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(
            ["sim_time_s", "name", "kind", "lat", "lon", "alt_m", "north_m", "east_m",
             "down_m", "heading_deg", "in_view", "visible_fraction", "covered",
             "camera_distance_m"]
        )
        projectairsim_log().info(f"Logging moving-actor ground truth to {path}")

    def log_ground_truth(self):
        """Append every actor's pose, at most every GROUND_TRUTH_PERIOD_S."""
        now = time.monotonic()
        if not self.log_writer or now - self.last_log < GROUND_TRUTH_PERIOD_S:
            return
        self.last_log = now
        names = list(self.actors)
        poses = self.world.get_object_poses(names)
        visibility = [{}] * len(names)
        if self.camera:
            try:
                visibility = self.world.get_object_visibility(*self.camera, names)
            except Exception as err:  # e.g. an older sim without the service
                projectairsim_log().debug(f"No visibility for the ground truth: {err}")
        sim_time = self.world.get_sim_time() / 1e9
        home_alt = self.world.home_geo_point["altitude"]
        for name, pose, seen in zip(names, poses, visibility):
            t, q = pose["translation"], pose["rotation"]
            if math.isnan(t["x"]):
                continue
            lat, lon = self._to_geo(t["x"], t["y"])
            _, _, yaw = quaternion_to_rpy(q["w"], q["x"], q["y"], q["z"])
            heading = (math.degrees(yaw) - self.actors[name]["yaw_offset"]) % 360.0
            self.log_writer.writerow(
                [f"{sim_time:.3f}", name, self.actors[name]["kind"], f"{lat:.7f}",
                 f"{lon:.7f}", f"{home_alt - t['z']:.2f}", f"{t['x']:.2f}",
                 f"{t['y']:.2f}", f"{t['z']:.2f}", f"{heading:.1f}",
                 int(seen.get("in_view", 0)), f"{seen.get('visible_fraction', 0.0):.2f}",
                 int(seen.get("covered", 0)), f"{seen.get('distance_m', float('nan')):.1f}"]
            )
        self.log_file.flush()

    def close(self):
        if self.log_file:
            self.log_file.close()
            self.log_file = self.log_writer = None

    # ------------------------------------------------------------------
    # Console

    def run_command(self, args):
        """Handle `actors ...`; args are the words after `actors`."""
        cmd = args[0].lower() if args else "list"
        try:
            if cmd == "list":
                if not self.actors:
                    print("No moving actors.")
                for name, actor in self.actors.items():
                    print(f"  {name}: {actor['desc']}")
            elif cmd in ("pause", "resume"):
                self.world.pause_object_paths(cmd == "pause")
            elif cmd == "reload":
                self.close()
                self.delete("all")
                self.spawn_from_config()
            elif cmd == "delete" and len(args) == 2:
                self.delete(args[1])
            else:
                print(ACTORS_HELP)
        except Exception as err:
            print(f"Error: {err}")
