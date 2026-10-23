"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Map editor for config_moving_actors.yml: pick each vehicle's or person's path
by clicking on a map in the browser instead of typing coordinates.

- Route: click point A and point B; the path follows OpenStreetMap roads
  (vehicles) or footpaths (people) between them.
- Waypoints: click points in order; "loop" drives back to the first point,
  like a mission that returns to its takeoff point.
- Random: click a start point (optional); the random drive/walk is shown.

The map previews the exact path the sim will use. Save writes the YAML (the
old file is kept as .bak); in a running sim, type `actors reload` to apply it.

  python actor_editor.py                      # edit config_moving_actors.yml
  python actor_editor.py --lat 1.35 --lon 103.82 --config my_actors.yml

run_cesium_sim.sh --edit-actors opens this before starting the sim and starts
the sim once you press "Save & start".
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

from moving_actors import ROAD_RADIUS_M, expand_actors, plan_path
from road_routes import RoadNetwork

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PAGE = os.path.join(SCRIPT_DIR, "actor_editor.html")
# The drone's default spawn (px4_gimbal_cesium.py); the road network and random
# routes are centred on it, so pass the sim's --lat/--lon if you change it
DEFAULT_LAT, DEFAULT_LON = 1.413425, 103.865000


class Editor:
    def __init__(self, config_path, lat, lon):
        self.config_path = config_path
        self.home = {"latitude": lat, "longitude": lon}
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

    def load(self):
        config = {}
        if os.path.exists(self.config_path):
            with open(self.config_path) as f:
                config = yaml.safe_load(f) or {}
        return {
            "home": self.home,
            "road_radius_m": ROAD_RADIUS_M,
            "config_path": self.config_path,
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
    parser.add_argument("--config", default=os.path.join(SCRIPT_DIR, "config_moving_actors.yml"))
    parser.add_argument("--lat", type=float, default=DEFAULT_LAT, help="drone spawn latitude")
    parser.add_argument("--lon", type=float, default=DEFAULT_LON, help="drone spawn longitude")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--wait", action="store_true",
                        help="exit after 'Save & start' (used by run_cesium_sim.sh)")
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    editor = Editor(os.path.abspath(args.config), args.lat, args.lon)
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
