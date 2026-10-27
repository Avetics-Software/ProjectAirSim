"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Map editor, in the browser, for the two things a mission is made of:

- Drone mission (--mission FILE): plan the drones' steps on the map the way QGC's
  plan view does. Click to add waypoints, drag them, and set each step's height,
  heading, camera, hold, look-at point, landing ... in the list beside the map. It
  checks every drone's steps as the ground station will upload them, for that
  drone's aircraft, and shows the route, its length and time, or the step that is
  wrong. Save writes the mission file (the old one is kept as .bak); a new file
  starts an empty mission. In a running sim: `mission load NAME`.
- Traffic (the moving actors): pick each vehicle's or person's path by clicking on
  the map instead of typing coordinates. With --mission it edits the mission's own
  `actors:` file, and the mission's route is drawn faintly beside the traffic so
  the two can be timed against each other.

Traffic, in detail:

- Route: click point A and point B; the path follows OpenStreetMap roads
  (vehicles) or footpaths (people) between them.
- Waypoints: click points in order; "loop" drives back to the first point,
  like a mission that returns to its takeoff point.
- Random: click a start point (optional); the random drive/walk is shown.

The map previews the exact path the sim will use. Save writes the YAML (the
old file is kept as .bak); in a running sim, type `actors reload` to apply it.

  python actor_editor.py                      # edit config_moving_actors.yml
  python actor_editor.py --lat 1.35 --lon 103.82 --config my_actors.yml
  python actor_editor.py --mission missions/my_mission.yml   # plan a mission (new or existing)
  python actor_editor.py --mission my_flight --fleet config.yml --model x8_arduplane

run_cesium_sim.sh --edit-actors (traffic) or --edit-mission FILE opens this before
starting the sim and starts the sim once you press "Save & start".
"""

import argparse
import json
import os
import shutil
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

import yaml

from gcs import mission_plan
from gcs.airframe import airframe_of, load_models
from gcs.fleet import load_fleet
from moving_actors import ROAD_RADIUS_M, expand_actors, plan_path
from road_routes import RoadNetwork

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PAGE = os.path.join(SCRIPT_DIR, "actor_editor.html")
PLANNER = os.path.join(SCRIPT_DIR, "mission_planner.js")
MISSIONS_DIR = os.path.join(SCRIPT_DIR, "missions")
# The drone's default spawn (px4_gimbal_cesium.py); the road network and random
# routes are centred on it, so pass the sim's --lat/--lon if you change it
DEFAULT_LAT, DEFAULT_LON = 1.413425, 103.865000


def resolve_mission(path):
    """A mission file given as a path, or as a name in missions/ (as `mission load`).
    One that doesn't exist yet is to be created: in missions/ when it is just a name."""
    for candidate in (path, os.path.join(MISSIONS_DIR, path)):
        for name in (candidate, candidate + ".yml", candidate + ".yaml"):
            if os.path.isfile(name):
                return os.path.abspath(name)
    if os.path.dirname(path):
        return os.path.abspath(path if path.endswith((".yml", ".yaml")) else path + ".yml")
    return os.path.join(MISSIONS_DIR, path if path.endswith((".yml", ".yaml")) else path + ".yml")


def mission_actors_path(mission_path):
    """The traffic file a mission names (`actors:`, relative to the mission), or None."""
    if not os.path.isfile(mission_path):
        return None
    with open(mission_path) as f:
        data = yaml.safe_load(f) or {}
    actors = data.get("actors") if isinstance(data, dict) else None
    return os.path.abspath(os.path.join(os.path.dirname(mission_path), actors)) if actors else None


class Editor:
    def __init__(self, config_path, lat, lon, mission_path=None, fleet_path=None, model=None):
        self.config_path = config_path
        self.home = {"latitude": lat, "longitude": lon}
        self.mission_path = mission_path
        # The drones a mission can name, and each one's aircraft (the config file's)
        self.default_model = model
        self.fleet = []
        if fleet_path and os.path.exists(fleet_path):
            self.fleet = [(d.name, d.model) for d in load_fleet(fleet_path).drones]
        self.networks = {}
        self.lock = threading.Lock()
        self.done = threading.Event()

    def network(self, kind):
        with self.lock:
            if kind not in self.networks:
                self.networks[kind] = RoadNetwork(
                    self.home["latitude"], self.home["longitude"], ROAD_RADIUS_M, kind
                )
            return self.networks[kind]

    @property
    def origin(self):
        """Where the drones start: the sim's spawn, which a file's north/east steps
        are measured from."""
        return (self.home["latitude"], self.home["longitude"])

    def mission_state(self):
        """What the page edits for --mission: the mission as data, with its places
        as lat/lon, and the drones and aircraft it can be flown by. None without
        --mission."""
        if not self.mission_path:
            return None
        raw = {}
        if os.path.exists(self.mission_path):
            with open(self.mission_path) as f:
                raw = yaml.safe_load(f) or {}
        names = [name for name, _ in self.fleet] or ["Drone1"]
        data = mission_plan.normalize(raw, self.origin, names)
        data.setdefault("mission", os.path.splitext(os.path.basename(self.mission_path))[0])
        names += [n for n in data["drones"] if n not in names]
        fleet_models = dict(self.fleet)
        models = {n: fleet_models.get(n) or self.default_model or "quad_px4" for n in names}
        known = {name: {"hovers": airframe_of(name).hovers, "label": entry.get("label", name)}
                 for name, entry in load_models()["models"].items()}
        return {"path": self.mission_path, "exists": os.path.exists(self.mission_path),
                "data": data, "drones": names, "models": models, "known_models": known,
                "origin": list(self.origin), "traffic": self.config_path}

    def mission_preview(self, data, models):
        return {"drones": mission_plan.preview(data, self.origin, models)}

    def mission_save(self, data, models):
        """Write the mission file (the old one kept as .bak); returns the mistakes
        the ground station would refuse it for, so they can be fixed now."""
        text = mission_plan.dump(data, mission_plan.header_of(self.mission_path))
        if os.path.exists(self.mission_path):
            shutil.copy2(self.mission_path, self.mission_path + ".bak")
        os.makedirs(os.path.dirname(self.mission_path), exist_ok=True)
        with open(self.mission_path, "w") as f:
            f.write(text)
        problems = [r["error"] if name in r["error"] else f"{name}: {r['error']}" for name, r in
                    mission_plan.preview(data, self.origin, models).items() if r.get("error")]
        return problems

    def load(self):
        config = {}
        if os.path.exists(self.config_path):
            with open(self.config_path) as f:
                config = yaml.safe_load(f) or {}
        return {
            "home": self.home,
            "road_radius_m": ROAD_RADIUS_M,
            "config_path": self.config_path,
            "mission": self.mission_state(),
            "defaults": config.get("defaults") or {},
            "actors": config.get("actors") or [],
        }

    def preview(self, defaults, actor):
        """The paths an entry produces (one per copy with `count`)."""
        paths = []
        for _, opts in expand_actors({"defaults": defaults, "actors": [actor]}):
            points, mode, note = plan_path(opts, self.network)
            paths.append({"name": opts["name"], "points": points, "mode": mode, "note": note})
        return paths

    def save(self, defaults, actors):
        # Keep the explanatory comment block at the top of the file
        header = ""
        if os.path.exists(self.config_path):
            with open(self.config_path) as f:
                for line in f:
                    if line.strip() and not line.lstrip().startswith("#"):
                        break
                    header += line
            shutil.copy2(self.config_path, self.config_path + ".bak")
        body = yaml.safe_dump(
            {"defaults": defaults, "actors": actors},
            sort_keys=False, default_flow_style=None, width=100,
        )
        with open(self.config_path, "w") as f:
            f.write(header + body)
        # Report entries the sim would refuse, so they can be fixed now
        problems = []
        for actor in actors:
            try:
                self.preview(defaults, actor)
            except Exception as err:
                problems.append(f"{actor.get('name', '?')}: {err}")
        return problems


def make_handler(editor):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, body, content_type="application/json"):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = urlsplit(self.path).path
            if path in ("/", "/index.html"):
                with open(PAGE, "rb") as f:
                    self._send(200, f.read(), "text/html; charset=utf-8")
            elif path == "/mission_planner.js":
                with open(PLANNER, "rb") as f:
                    self._send(200, f.read(), "text/javascript; charset=utf-8")
            elif path == "/api/state":
                self._send(200, editor.load())
            else:
                self._send(404, {"error": "not found"})

        def do_POST(self):
            length = int(self.headers.get("Content-Length", 0))
            request = json.loads(self.rfile.read(length) or b"{}")
            try:
                if self.path == "/api/preview":
                    self._send(200, {"paths": editor.preview(request["defaults"], request["actor"])})
                elif self.path == "/api/mission/preview":
                    self._send(200, editor.mission_preview(request["data"], request.get("models")))
                elif self.path == "/api/mission/save":
                    problems = editor.mission_save(request["data"], request.get("models"))
                    self._send(200, {"saved": editor.mission_path, "problems": problems})
                    if request.get("finish"):
                        editor.done.set()
                elif self.path == "/api/save":
                    problems = editor.save(request["defaults"], request["actors"])
                    self._send(200, {"saved": editor.config_path, "problems": problems})
                    if request.get("finish"):
                        editor.done.set()
                else:
                    self._send(404, {"error": "not found"})
            except Exception as err:
                self._send(200, {"error": str(err)})

    return Handler


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--config", default=None,
                        help="the traffic file [config_moving_actors.yml, or the mission's own]")
    parser.add_argument("--mission", metavar="FILE",
                        help="plan this mission on the map: a path, or a name in missions/ "
                        "(a file that doesn't exist yet is created when you save)")
    parser.add_argument("--fleet", default=os.path.join(SCRIPT_DIR, "config.yml"),
                        help="the config file whose drones (and their `model:`) the mission "
                        "is for [config.yml]")
    parser.add_argument("--model", default=None,
                        help="the aircraft of drones the config file doesn't type [quad_px4]")
    parser.add_argument("--lat", type=float, default=DEFAULT_LAT, help="drone spawn latitude")
    parser.add_argument("--lon", type=float, default=DEFAULT_LON, help="drone spawn longitude")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--wait", action="store_true",
                        help="exit after 'Save & start' (used by run_cesium_sim.sh)")
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    mission_path = resolve_mission(args.mission) if args.mission else None
    config = args.config or (mission_path and mission_actors_path(mission_path)) \
        or os.path.join(SCRIPT_DIR, "config_moving_actors.yml")
    editor = Editor(os.path.abspath(config), args.lat, args.lon, mission_path,
                    args.fleet, args.model)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(editor))
    url = f"http://127.0.0.1:{args.port}/" + ("?wait=1" if args.wait else "")
    print(f"Actor editor: {url}  (editing {editor.config_path})")
    if not args.no_browser:
        webbrowser.open(url)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        if args.wait:
            editor.done.wait()
            print("Saved; starting the sim")
        else:
            threading.Event().wait()
    except KeyboardInterrupt:
        pass
    server.shutdown()


if __name__ == "__main__":
    main()
