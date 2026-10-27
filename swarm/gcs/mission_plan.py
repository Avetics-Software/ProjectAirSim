"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Planning a mission on a map: the data side of the map editor (actor_editor.py,
its "Drone mission" tab). The page edits a mission as plain data, like QGC's plan
view edits a list of items; this module is what stands behind it:

  normalize   a mission as the page edits it: places as lat/lon (a file's
              north/east steps are converted from where the drones start), drones
              by name
  preview     each drone's steps checked and measured as the ground station will
              upload them, for that drone's aircraft: its route, length, time, or
              the step that is wrong
  dump        the mission file's text, with the keys in their usual order

Nothing here needs the sim; gcs/ stays free of Project AirSim.
"""

import copy
import math
import os
import re
import time

import yaml

from .airframe import airframe_of
from .mission import EARTH_M_PER_DEG, _place, build_items, parse_mission

KEY_ORDER = ("mission", "origin", "speed", "record", "actors", "drones")
PLACE_KEYS = ("goto", "look_at", "land")  # steps that carry a place


def drone_name(key, drone_names):
    """The drone a mission file's key names: a name, or its 1-based number."""
    text = str(key)
    if text.isdigit():
        n = int(text)
        return drone_names[n - 1] if 0 < n <= len(drone_names) else f"Drone{n}"
    return text


def _to_latlon(spec, origin):
    """A place given as north/east of `origin` becomes lat/lon; other keys stay."""
    if not isinstance(spec, dict) or ("lat" in spec and "lon" in spec):
        return spec
    if "north" not in spec and "east" not in spec:
        return spec
    lat, lon, _ = _place(spec, origin, 0.0)
    out = {"lat": round(lat, 6), "lon": round(lon, 6)}
    out.update({k: v for k, v in spec.items() if k not in ("north", "east", "lat", "lon")})
    return out


def normalize(data, origin, drone_names=()):
    """Mission data for the map: every place as lat/lon (north/east measured
    from `origin`, where the drones start), every drone by name."""
    data = copy.deepcopy(data or {})
    drones = {}
    for key, entry in (data.get("drones") or {}).items():
        entry = dict(entry or {})
        steps = []
        for step in entry.get("steps") or []:
            step = dict(step) if isinstance(step, dict) else step
            if isinstance(step, dict):
                for place in PLACE_KEYS:
                    if place in step:
                        step[place] = _to_latlon(step[place], origin)
            steps.append(step)
        entry["steps"] = steps
        drones[drone_name(key, drone_names)] = entry
    out = {k: data[k] for k in KEY_ORDER if k in data and k != "drones"}
    out.update({k: v for k, v in data.items() if k not in KEY_ORDER})
    out["drones"] = drones
    return out


def preview(data, origin, models=None):
    """Check and measure each drone's steps as the ground station will upload them.

    data: normalized mission data; models: {drone name: aircraft model} (default
    the default model). Returns {drone name: {steps, items, route: [[lat, lon]],
    alts, length_m, time_s, max_alt} or {steps, error, error_step}}: a drone's
    mistake doesn't hide the others."""
    models = models or {}
    out = {}
    for name, entry in (data.get("drones") or {}).items():
        steps = (entry or {}).get("steps") or []
        result = {"steps": len(steps), "route": [], "alts": [], "items": 0,
                  "length_m": 0.0, "time_s": None, "max_alt": 0.0}
        out[name] = result
        if not steps:
            continue
        single = {k: v for k, v in data.items() if k not in ("drones", "actors", "record")}
        single["drones"] = {name: entry}
        try:
            mission = parse_mission(single, [name], "mission")
            dm = mission.drones[0]
            build_items(dm, tuple(origin), tuple(origin), 0.0, False, 1,
                        airframe=airframe_of(models.get(name)))
        except ValueError as err:
            text = re.sub(r"^mission: ", "", str(err))
            result["error"] = text
            found = re.search(r"step (\d+)", text)
            result["error_step"] = int(found.group(1)) if found else None
            continue
        length = sum(
            math.hypot((b[0] - a[0]) * EARTH_M_PER_DEG,
                       (b[1] - a[1]) * EARTH_M_PER_DEG * math.cos(math.radians(a[0])))
            for a, b in zip([tuple(origin)] + dm.route, dm.route))
        result.update(route=[list(p) for p in dm.route],
                      alts=[dm.items[k][5] for k in dm.route_seq], items=len(dm.items),
                      length_m=round(length), max_alt=dm.max_alt,
                      time_s=round(length / dm.speed) if dm.speed and length else None)
    return out


def header_of(path):
    """The comment block at the top of a mission file (kept when it is saved)."""
    header = ""
    if os.path.exists(path):
        with open(path) as f:
            for line in f:
                if line.strip() and not line.lstrip().startswith("#"):
                    break
                header += line
    return header


def dump(data, header=""):
    """The mission file's text: keys in their usual order, drones with no steps
    left out. Raises ValueError if there is nothing to fly."""
    data = normalize(data, (0.0, 0.0))  # already lat/lon; this only orders the keys
    drones = {}
    for name, entry in data["drones"].items():
        if entry.get("steps"):
            ordered = {k: entry[k] for k in ("speed",) if k in entry}
            ordered["steps"] = entry["steps"]
            drones[name] = ordered
    if not drones:
        raise ValueError("no steps yet: add a takeoff and waypoints")
    data["drones"] = drones
    if not header:
        header = (f"# Planned on the map editor ({time.strftime('%Y-%m-%d')}). Places are lat/lon.\n"
                  "# Every step is explained in docs/features/missions.md.\n")
    return header + yaml.safe_dump(data, sort_keys=False, default_flow_style=None, width=100)
