"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Mission files: scripted flights for repeatable experiments. A YAML file gives
each drone a list of steps (where to fly, its heading, the gimbal's pitch and
yaw or a point to look at, holds, photos); the ground station turns each
drone's steps into a PX4 mission and uploads it, so it flies on the drone at
PX4's own timing, the same way every run, and keeps flying if the link drops.

    mission: seletar_pass             # a name, for the run logs
    origin: start                     # north/east steps are metres from each drone's
                                      #   position at upload (start) or from home (home)
    speed: 8                          # m/s, for every drone unless a drone sets its own
    drones:
      Drone1:                         # a drone's name, or its number
        steps:
          - takeoff: 30               # metres above home (only needed on the ground)
          - goto: {north: 100, east: 0, alt: 30}
            heading: 90               # degrees from north, while flying there and after
            gimbal: {pitch: -30, yaw: 0}      # applied on arrival; yaw from the nose
            hold: 5                   # seconds at the waypoint
          - gimbal: {pitch: -45, heading: 270}  # yaw as a compass heading (yaw lock)
          - goto: {lat: 1.4150, lon: 103.8660, alt: 40}
          - look_at: {north: 150, east: 50}   # point the camera at a place (alt: ground)
          - wait: 10                  # seconds
          - look_at: none
          - photo: true
          - track: {target: Vehicle1} # hand the camera to the tracker (X, or the sim's
                                      #   truth tracker) while it flies on; track: off
          - rtl: true                 # or land: true; else it holds at the last step

Mission items: NAV_TAKEOFF, NAV_WAYPOINT (param1 hold, param4 heading),
DO_CHANGE_SPEED, DO_GIMBAL_MANAGER_PITCHYAW (flags: yaw lock for a compass
heading), DO_SET_ROI_LOCATION / DO_SET_ROI_NONE, NAV_DELAY,
IMAGE_START_CAPTURE, NAV_LAND, NAV_RETURN_TO_LAUNCH. PX4 doesn't accept
camera zoom in a mission, so there is no zoom step.

A fixed-wing (gcs/airframe.py) flies the same steps where it can: it climbs
out along the runway for takeoff, circles for a hold (NAV_LOITER_TIME), has no
heading of its own (a heading step is an error) and lands only through a
landing approach, which missions don't plan yet (a land step is an error; use
rtl).

Runs: each `mission start` gets a run id; every drone's position, heading,
gimbal angle and mission step are logged at 5 Hz to runs/<mission>_<run>.csv,
and `mission compare` lines two runs up by time and reports how far apart the
drones flew.
"""

import asyncio
import csv
import math
import os
import re
import time
from dataclasses import dataclass, field

import yaml

from .airframe import Airframe
from .autopilot import dialect
from .log import log
from .mavlink_link import FIXEDWING_TAKEOFF_PITCH, M

EARTH_M_PER_DEG = 111_320.0
F_REL, F_MISSION = M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, M.MAV_FRAME_MISSION
STEP_KEYS = {"takeoff", "goto", "heading", "gimbal", "hold", "look_at", "wait", "photo",
             "land", "rtl", "speed", "track"}
TRACKERS = ("auto", "x", "truth")


def _number(value, label, minimum=None, maximum=None):
    if isinstance(value, bool):
        raise ValueError(f"{label}: expected a number")
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{label}: expected a number") from None
    if not math.isfinite(value) or (minimum is not None and value < minimum) \
            or (maximum is not None and value > maximum):
        raise ValueError(f"{label}: invalid value {value}")
    return value


def _validate_place(spec, label):
    if not isinstance(spec, dict) or set(spec) - {"lat", "lon", "north", "east", "alt"}:
        raise ValueError(f"{label}: expected lat/lon or north/east, with optional alt")
    geographic = "lat" in spec or "lon" in spec
    if geographic:
        if not {"lat", "lon"} <= spec.keys() or {"north", "east"} & spec.keys():
            raise ValueError(f"{label}: use either lat/lon or north/east")
        _number(spec["lat"], label + " latitude", -90, 90)
        _number(spec["lon"], label + " longitude", -180, 180)
    elif not {"north", "east"} & spec.keys():
        raise ValueError(f"{label}: expected lat/lon or north/east")
    for key in ("north", "east", "alt"):
        if key in spec:
            _number(spec[key], label + " " + key, 0 if key == "alt" else None)


def _validate_step(step, label):
    for key in ("takeoff", "speed", "heading", "hold", "wait"):
        if key == "heading" and step.get(key) == "travel":
            continue
        if key in step:
            _number(step[key], label + " " + key,
                    0.01 if key in ("takeoff", "speed") else (0 if key != "heading" else None))
    if "goto" in step:
        _validate_place(step["goto"], label + " goto")
    if "look_at" in step and step["look_at"] not in (None, "none", False):
        _validate_place(step["look_at"], label + " look_at")
    if "gimbal" in step:
        g = step["gimbal"]
        if not isinstance(g, dict) or not g or set(g) - {"pitch", "yaw", "heading"}:
            raise ValueError(f"{label}: gimbal needs pitch, yaw or heading")
        if "yaw" in g and "heading" in g:
            raise ValueError(f"{label}: gimbal yaw and heading are mutually exclusive")
        for key, value in g.items():
            _number(value, label + " gimbal " + key, -90 if key == "pitch" else None,
                    90 if key == "pitch" else None)
    if "track" in step:
        track_spec(step["track"], label)
        if "gimbal" in step or "look_at" in step:
            raise ValueError(f"{label}: track hands the camera to the tracker; it can't share "
                             "a step with gimbal or look_at")
    for key in ("land", "rtl", "photo"):
        if key in step and not isinstance(step[key], bool):
            raise ValueError(f"{label}: {key} must be true or false")
    if sum(bool(step.get(k)) for k in ("takeoff", "goto", "land", "rtl")) > 1:
        raise ValueError(f"{label}: use one flight action per step")


def track_spec(value, label="track"):
    """A `track` step's value as {"target": name or None, "tracker": ...}, or
    None for `track: off`."""
    if value in (False, None, "off"):
        return None
    if value in (True, "on"):
        return {"target": None, "tracker": "auto"}
    if isinstance(value, str):
        return {"target": value, "tracker": "auto"}
    if not isinstance(value, dict) or set(value) - {"target", "tracker"}:
        raise ValueError(f"{label}: track needs on, off, a target name, or "
                         "{target: NAME, tracker: auto|x|truth}")
    tracker = str(value.get("tracker", "auto")).lower()
    if tracker not in TRACKERS:
        raise ValueError(f"{label}: tracker must be one of {', '.join(TRACKERS)}")
    target = value.get("target")
    return {"target": str(target) if target is not None else None, "tracker": tracker}


@dataclass
class DroneMission:
    name: str
    steps: list
    speed: float = None  # m/s
    items: list = field(default_factory=list)  # PX4 mission items, built on upload
    route: list = field(default_factory=list)  # [(lat, lon)] for the map
    route_seq: list = field(default_factory=list)  # each route point's mission item
    route_alt: float = None
    max_alt: float = 0.0
    camera_steps: list = field(default_factory=list)
    track_steps: list = field(default_factory=list)  # [(item seq, track spec or None)]


@dataclass
class Mission:
    name: str
    origin: str  # "start" or "home"
    drones: dict  # drone index -> DroneMission
    path: str = ""
    record: list = field(default_factory=list)  # drone indices whose runs are recorded (sim)
    actors: str = None  # the sim's moving actors for this mission (a config file), or None


def load_mission(path, drone_names):
    """Read and check a mission file; drones are named or numbered (1-based)."""
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a mission mapping")
    unknown = set(data) - {"mission", "origin", "speed", "drones", "record", "actors"}
    if unknown:
        raise ValueError(f"{path}: unknown key(s) {', '.join(sorted(unknown))}")
    origin = str(data.get("origin", "start")).lower()
    if origin not in ("start", "home"):
        raise ValueError(f"{path}: origin must be start or home")
    drones = {}
    entries = data.get("drones") or {}
    if not isinstance(entries, dict):
        raise ValueError(f"{path}: drones must be a mapping")
    for key, entry in entries.items():
        if isinstance(key, int) or str(key).isdigit():
            index = int(key) - 1
        elif key in drone_names:
            index = drone_names.index(key)
        else:
            raise ValueError(f"{path}: no drone {key!r} (drones: {', '.join(drone_names)})")
        if not 0 <= index < len(drone_names):
            raise ValueError(f"{path}: no drone {key} (there are {len(drone_names)})")
        entry = entry or {}
        if index in drones:
            raise ValueError(f"{path}: drone {key} listed twice")
        if not isinstance(entry, dict) or set(entry) - {"steps", "speed"}:
            raise ValueError(f"{path}: {key} needs steps and optional speed")
        steps = entry.get("steps") or []
        if not isinstance(steps, list) or not steps:
            raise ValueError(f"{path}: {key} needs at least one step")
        ended = False
        for n, step in enumerate(steps, start=1):
            if not isinstance(step, dict) or not step:
                raise ValueError(f"{path}: {key} step {n} is empty")
            bad = set(step) - STEP_KEYS
            if bad:
                raise ValueError(f"{path}: {key} step {n}: unknown {', '.join(sorted(bad))} "
                                 f"(steps: {', '.join(sorted(STEP_KEYS))})")
            if ended:
                raise ValueError(f"{path}: {key} step {n}: no steps allowed after land or rtl")
            _validate_step(step, f"{path}: {key} step {n}")
            if "takeoff" in step and n != 1:
                raise ValueError(f"{path}: {key}: takeoff must be the first step")
            ended = bool(step.get("land") or step.get("rtl"))
        speed = entry.get("speed", data.get("speed"))
        drones[index] = DroneMission(drone_names[index], steps,
                                     _number(speed, "speed", 0.01) if speed is not None else None)
    if not drones:
        raise ValueError(f"{path}: no drones")
    name = str(data.get("mission") or os.path.splitext(os.path.basename(path))[0])
    if not re.fullmatch(r"[\w.-]+", name) or name in (".", ".."):
        raise ValueError(f"{path}: mission name must be a simple filename")
    # record: true (every drone in the file) or a list of drones; actors: a
    # moving-actor config, relative to the mission file
    record = data.get("record", False)
    if record is True:
        record = sorted(drones)
    elif record in (False, None):
        record = []
    elif isinstance(record, list):
        indices = []
        for key in record:
            index = (int(key) - 1 if isinstance(key, int) or str(key).isdigit()
                     else drone_names.index(key) if key in drone_names else None)
            if index not in drones:
                raise ValueError(f"{path}: record: {key} has no steps in this mission")
            indices.append(index)
        record = sorted(set(indices))
    else:
        raise ValueError(f"{path}: record must be true, false or a list of drones")
    actors = data.get("actors")
    if actors is not None:
        actors = os.path.join(os.path.dirname(os.path.abspath(path)), str(actors))
        if not os.path.isfile(actors):
            raise ValueError(f"{path}: no actors file {actors}")
    return Mission(name, origin, drones, path, record, actors)


def _place(spec, origin, alt_default):
    """(lat, lon, alt) of a goto / look_at: lat/lon, or north/east of origin."""
    if not isinstance(spec, dict):
        raise ValueError(f"expected lat/lon or north/east, got {spec!r}")
    alt = float(spec.get("alt", alt_default))
    if "lat" in spec and "lon" in spec:
        return float(spec["lat"]), float(spec["lon"]), alt
    if "north" in spec or "east" in spec:
        lat0, lon0 = origin
        north, east = float(spec.get("north", 0)), float(spec.get("east", 0))
        return (lat0 + north / EARTH_M_PER_DEG,
                lon0 + east / (EARTH_M_PER_DEG * math.cos(math.radians(lat0))), alt)
    raise ValueError(f"expected lat/lon or north/east, got {spec!r}")


def build_items(dm, origin, here, start_alt, airborne, sysid, default_alt=30.0,
                ground_alt=0.0, airframe=None):
    """Turn a drone's steps into mission items (frame, command, (p1..p4),
    lat, lon, alt). origin: (lat, lon) north/east are measured from; here:
    where the drone is; start_alt: its height (for gotos without an alt).
    On the ground without a takeoff step, it takes off to default_alt first;
    in the air, takeoff steps are skipped. sysid: the drone's, whose autopilot
    claims the gimbal for each gimbal step. airframe: gcs.airframe.Airframe,
    default a PX4 multirotor. Fills dm.route for the map."""
    airframe = airframe or Airframe()
    hovers = airframe.hovers
    if not hovers:
        for n, step in enumerate(dm.steps, start=1):
            if "heading" in step and step["heading"] != "travel":
                raise ValueError(f"step {n}: a fixed-wing has no heading of its own")
            if step.get("land"):
                raise ValueError(f"step {n}: a fixed-wing lands only from a landing approach, "
                                 "which missions don't plan yet; use rtl")
    takeoff_params = (0, 0, 0, math.nan) if hovers else (FIXEDWING_TAKEOFF_PITCH, 0, 0, 0)
    speed_type = 1 if hovers else 0  # ground speed; a fixed-wing's airspeed
    accept_m = 3.0 if hovers else 0.0  # a fixed-wing's own (WP_RADIUS)
    claims_gimbal = dialect(airframe.autopilot).mission_gimbal_claim
    items, route, route_seq, track_steps = [], [], [], []
    heights = []
    camera_steps = []
    roi = None  # the look_at in force, re-issued at each waypoint
    # The heading in force: a heading holds for later steps until another
    # one, or "travel" (face the way it flies, PX4's own choice: NaN)
    held_heading = nan_heading = math.nan
    alt = start_alt
    if not airborne and not any("takeoff" in s for s in dm.steps):
        alt = default_alt
        items.append((F_REL, M.MAV_CMD_NAV_TAKEOFF, takeoff_params, here[0], here[1], alt))
    # Who controls the gimbal: this drone's autopilot (the mission) unless a
    # `track` step hands it to the tracker. PX4 obeys a mission's gimbal
    # commands only while the autopilot holds the gimbal, and a companion
    # (X) stands down while it does.
    claim = (F_MISSION, M.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE, (sysid, 1, -1, -1), 0, 0, 0)
    release = (F_MISSION, M.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE, (0, 0, -1, -1), 0, 0, 0)
    claim_items = [claim] if claims_gimbal else []
    release_items = [release] if claims_gimbal else []
    tracking = False
    if not any("takeoff" in s for s in dm.steps) or airborne:
        items.extend(claim_items)
    if dm.speed:
        items.append((F_MISSION, M.MAV_CMD_DO_CHANGE_SPEED, (speed_type, dm.speed, -1, 0), 0, 0, 0))
    nan = math.nan

    def circle(seconds, lat, lon, alt):
        """A fixed-wing's hold: circle the point for `seconds` (radius: its own)."""
        return (F_REL, M.MAV_CMD_NAV_LOITER_TIME, (seconds, 0, 0, 0), lat, lon, alt)

    def gimbal_items(g):
        pitch = float(g.get("pitch", nan))
        if "heading" in g:  # a compass heading: yaw lock
            yaw, flags = float(g["heading"]), M.GIMBAL_MANAGER_FLAGS_YAW_LOCK
        else:
            yaw, flags = float(g.get("yaw", 0.0)), 0
        flags |= M.GIMBAL_MANAGER_FLAGS_ROLL_LOCK | M.GIMBAL_MANAGER_FLAGS_PITCH_LOCK
        # PX4 obeys a mission's gimbal commands only while the autopilot itself
        # controls the gimbal, which a mission doesn't claim by itself: claim it
        # (primary control: this drone's autopilot; secondary unchanged), also
        # back from a tracker
        # param5 is the flags, param7 the gimbal device (0: all); params 5-7
        # travel in the lat/lon/alt fields of a mission item
        return claim_items + [(F_MISSION, M.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                               (pitch, yaw, nan, nan), flags, 0, 0)]

    for step in dm.steps:
        if "speed" in step:
            items.append((F_MISSION, M.MAV_CMD_DO_CHANGE_SPEED,
                          (speed_type, float(step["speed"]), -1, 0), 0, 0, 0))
        if "takeoff" in step and not airborne:
            alt = float(step["takeoff"])
            items.append((F_REL, M.MAV_CMD_NAV_TAKEOFF, takeoff_params, here[0], here[1], alt))
            heights.append(alt)
            items.extend(claim_items)
        if "heading" in step:
            held_heading = (nan_heading if step["heading"] == "travel"
                            else float(step["heading"]))
        heading = held_heading
        hold = float(step.get("hold", 0))
        # A hold comes after the step's camera action (PX4 runs a waypoint's
        # hold before the items following it), including a re-aimed look_at
        reaim = bool(roi) and "goto" in step and "look_at" not in step and "gimbal" not in step
        hold_after_camera = hold > 0 and ("gimbal" in step or "look_at" in step or reaim)
        waypoint_hold = 0 if hold_after_camera else hold
        if "goto" in step:
            lat, lon, alt = _place(step["goto"], origin, alt)
            here = (lat, lon)
            # param2: acceptance radius (m); param4: heading (deg; NaN: keep
            # the current heading's mode)
            route_seq.append(len(items))
            if not hovers and waypoint_hold:
                items.append(circle(waypoint_hold, lat, lon, alt))
            elif hovers:
                items.append((F_REL, M.MAV_CMD_NAV_WAYPOINT, (waypoint_hold, accept_m, 0, heading),
                              lat, lon, alt))
            else:
                items.append((F_REL, M.MAV_CMD_NAV_WAYPOINT, (0, accept_m, 0, 0), lat, lon, alt))
            route.append(here)
            heights.append(alt)
            if reaim:
                # PX4 aims the camera at a look_at point once, when the item
                # runs, not as the drone moves: aim it again from here
                items.append(roi)
        elif not hovers and waypoint_hold:
            items.append(circle(waypoint_hold, here[0], here[1], alt))
        elif hovers and (("heading" in step and step["heading"] != "travel") or "hold" in step):
            # Turn (and/or hold) where it is
            items.append((F_REL, M.MAV_CMD_NAV_WAYPOINT, (waypoint_hold, accept_m, 0, heading),
                          here[0], here[1], alt))
        if "track" in step:
            spec = track_spec(step["track"])
            if spec and not tracking:
                if roi:
                    # Clearing the ROI centres the camera, so hand it over
                    # still pointing at the look_at place (from here): a fixed
                    # pitch and compass heading in its stead
                    roi_lat, roi_lon, roi_alt = roi[3:6]
                    roi = None
                    north = (roi_lat - here[0]) * EARTH_M_PER_DEG
                    east = (roi_lon - here[1]) * EARTH_M_PER_DEG * math.cos(math.radians(here[0]))
                    pitch = -math.degrees(math.atan2(alt - roi_alt, math.hypot(north, east)))
                    bearing = math.degrees(math.atan2(east, north)) % 360.0
                    items.append((F_MISSION, M.MAV_CMD_DO_SET_ROI_NONE, (0, 0, 0, 0), 0, 0, 0))
                    items.extend(gimbal_items({"pitch": max(-90.0, min(90.0, pitch)),
                                               "heading": bearing}))
                # The gimbal to nobody: the tracker (X re-claims it) takes over
                items.extend(release_items)
                track_steps.append((len(items) - 1, spec))
                camera_steps.append((*here, "TRACK" + (f" {spec['target']}" if spec["target"]
                                                       else "")))
            elif not spec and tracking:
                items.extend(claim_items)
                track_steps.append((len(items) - 1, None))
            tracking = bool(spec)
        if "gimbal" in step:
            roi = None
            items.extend(gimbal_items(step["gimbal"]))
            if tracking:  # the gimbal step's claim ends the tracking
                tracking = False
                track_steps.append((len(items) - 1 - len(claim_items), None))
            g = step["gimbal"]
            text = " ".join(f"{k[0].upper()}{float(v):g}" for k, v in g.items())
            camera_steps.append((*here, text))
        if "look_at" in step:
            if tracking:  # the mission takes the camera back to point it
                tracking = False
                items.extend(claim_items)
                track_steps.append((len(items) - 1, None))
            spec = step["look_at"]
            if spec in (None, "none", False):
                roi = None
                items.append((F_MISSION, M.MAV_CMD_DO_SET_ROI_NONE, (0, 0, 0, 0), 0, 0, 0))
            else:
                lat, lon, a = _place(spec, origin, ground_alt)
                # ROI altitude: above home (relative frame)
                roi = (F_REL, M.MAV_CMD_DO_SET_ROI_LOCATION, (0, 0, 0, 0), lat, lon, a)
                items.append(roi)
                camera_steps.append((*here, "ROI"))
        if hold_after_camera:
            items.append((F_MISSION, M.MAV_CMD_NAV_DELAY, (hold, -1, -1, -1), 0, 0, 0) if hovers
                         else circle(hold, here[0], here[1], alt))
        if "wait" in step:
            items.append((F_MISSION, M.MAV_CMD_NAV_DELAY, (float(step["wait"]), -1, -1, -1), 0, 0, 0)
                         if hovers else circle(float(step["wait"]), here[0], here[1], alt))
        if step.get("photo"):
            items.append((F_MISSION, M.MAV_CMD_IMAGE_START_CAPTURE, (0, 0, 1, 0), 0, 0, 0))
        if step.get("land"):
            items.append((F_REL, M.MAV_CMD_NAV_LAND, (0, 0, 0, nan), here[0], here[1], 0))
        if step.get("rtl"):
            items.append((F_MISSION, M.MAV_CMD_NAV_RETURN_TO_LAUNCH, (0, 0, 0, 0), 0, 0, 0))
    # A varying-height route has no single safe height at its crossings.
    dm.items, dm.route, dm.route_seq = items, route, route_seq
    dm.route_alt = heights[0] if heights and len(set(heights)) == 1 else None
    dm.max_alt = max(heights, default=alt)
    dm.camera_steps = camera_steps
    dm.track_steps = track_steps
    return items


def track_summaries(run_path):
    """{drone: (share of frames with the target in frame, mean offset from the
    centre in degrees while in it)} from a run's tracking files
    (<run>_track_<drone>.csv, written by the sim's track step)."""
    out = {}
    stem = run_path[:-len(".csv")]
    directory = os.path.dirname(run_path) or "."
    for name in os.listdir(directory):
        path = os.path.join(directory, name)
        if not path.startswith(stem + "_track_") or not name.endswith(".csv"):
            continue
        with open(path) as f:
            rows = list(csv.DictReader(f))
        if not rows:
            continue
        inside = [r for r in rows if r["in_frame"] == "1"]
        offset = (sum(max(abs(float(r["h_offset_deg"])), abs(float(r["v_offset_deg"])))
                      for r in inside) / len(inside)) if inside else math.nan
        out[name[len(os.path.basename(stem)) + len("_track_"):-len(".csv")]] = (
            len(inside) / len(rows), offset)
    return out


class RunLog:
    """Each drone's track during a mission run, at 5 Hz, to a CSV."""

    def __init__(self, directory, mission, run_id, links, indices):
        os.makedirs(directory, exist_ok=True)
        self.path = os.path.join(directory, f"{mission}_{run_id}.csv")
        self.links = links
        self.indices = list(indices)
        self.t0 = time.monotonic()
        self._f = open(self.path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(["t", "drone", "lat", "lon", "height", "heading", "mode",
                          "mission_step", "gimbal_pitch", "gimbal_yaw"])

    def sample(self):
        t = round(time.monotonic() - self.t0, 2)
        for i in self.indices:
            link = self.links[i]
            if not link.position:
                continue
            lat, lon, _, rel = link.position
            g = link.gimbal_attitude or (math.nan, math.nan)
            self._w.writerow([t, link.name, f"{lat:.7f}", f"{lon:.7f}", round(rel, 2),
                              round(link.heading or 0.0, 1), link.mode, link.mission_seq,
                              round(g[0], 1), round(g[1], 1)])
        self._f.flush()

    def close(self):
        if not self._f.closed:
            self._f.close()


def compare_runs(path_a, path_b):
    """Per drone: how far apart two runs of a mission flew, matched by time
    since the start: (drone, samples, mean m, max m, max heading diff deg)."""
    def read(path):
        tracks = {}
        with open(path) as f:
            for r in csv.DictReader(f):
                tracks.setdefault(r["drone"], []).append(
                    (float(r["t"]), float(r["lat"]), float(r["lon"]), float(r["height"]),
                     float(r["heading"]), float(r.get("gimbal_pitch", "nan")),
                     float(r.get("gimbal_yaw", "nan"))))
        return tracks
    a, b = read(path_a), read(path_b)
    out = []
    for drone in sorted(set(a) & set(b)):
        tb = b[drone]
        dists, headings, pitches, yaws = [], [], [], []
        j = 0
        for t, lat, lon, h, hdg, pitch, yaw in a[drone]:
            while j + 1 < len(tb) and abs(tb[j + 1][0] - t) <= abs(tb[j][0] - t):
                j += 1
            _, lat2, lon2, h2, hdg2, pitch2, yaw2 = tb[j]
            if abs(tb[j][0] - t) > 0.5:
                continue
            dn = (lat2 - lat) * EARTH_M_PER_DEG
            de = (lon2 - lon) * EARTH_M_PER_DEG * math.cos(math.radians(lat))
            dists.append(math.sqrt(dn * dn + de * de + (h2 - h) ** 2))
            headings.append(abs((hdg2 - hdg + 180) % 360 - 180))
            if math.isfinite(pitch) and math.isfinite(pitch2):
                pitches.append(abs(pitch2 - pitch))
            if math.isfinite(yaw) and math.isfinite(yaw2):
                yaws.append(abs((yaw2 - yaw + 180) % 360 - 180))
        if dists:
            out.append((drone, len(dists), sum(dists) / len(dists), max(dists), max(headings),
                        max(pitches, default=math.nan), max(yaws, default=math.nan)))
    return out


MISSION_HELP = """Missions (scripted flights for repeatable experiments; see docs/features/missions.md):
  mission load FILE                     read a mission file (also looked for in missions/)
  mission show                          each drone's steps, and what was uploaded
  mission upload [N|all]                turn it into PX4 missions and upload them
                                        [every drone in the file]
  mission start [N|all]                 arm, take off and fly them, all together; logs the
                                        run to runs/ (X stands down while the mission has
                                        the camera; a track step hands it back)
  mission stop                          hold every drone flying the mission
  mission clear [N|all]                 remove the uploaded missions
  mission runs                          the logged runs of this mission
  mission compare [RUN_A RUN_B]         how far apart two runs flew, and their tracking
                                        scores [the last two]"""


class MissionControl:
    """The `mission` console commands for the ground station.

    swarm: the SwarmControl (the drones' tasks, plans for the map and the
    separation monitor); home(): (lat, lon) of the map's home, or None;
    before_start: optional callable (sync or async) run before a start, e.g.
    the sim putting its moving actors back where they began. on_track(i, spec,
    run): optional, called when drone i's mission hands the camera to a
    tracker (spec: the `track` step's {"target", "tracker"}) and with spec None
    when it takes it back; the sim locks the tracker onto the target and
    scores it."""

    def __init__(self, links, swarm, home, run_dir, search_dirs=(), before_start=None,
                 default_alt=30.0, companions=None, on_track=None, on_run=None):
        self.links = links
        self.swarm = swarm
        self.home = home
        self.run_dir = run_dir
        self.search_dirs = list(search_dirs)
        self.before_start = before_start
        self.default_alt = default_alt
        self.companions = companions or {}
        self.mission = None
        self.uploaded = set()  # drone indices holding the loaded mission
        self.flying = set()  # drone indices flying it now
        self.run_log = None
        self.run_id = None  # of the run in progress (or the last one)
        self.on_track = on_track
        self.on_run = on_run  # on_run(event "start"/"end", mission, run_id, indices)
        self.tracking = {}  # drone index -> the track spec in force
        self._log_task = None

    # ------------------------------------------------------------------

    def _find(self, path):
        path = os.fspath(path)
        for candidate in [path] + [os.path.join(d, path) for d in self.search_dirs]:
            for name in (candidate, candidate + ".yml", candidate + ".yaml"):
                if os.path.isfile(name):
                    return name
        raise ValueError(f"no mission file {path}")

    def _targets(self, parts):
        """Drone indices a command names (a number or all), within the mission."""
        if not self.mission:
            raise ValueError("load a mission first (mission load FILE)")
        if len(parts) < 3 or parts[2].lower() == "all":
            return sorted(self.mission.drones)
        i = int(parts[2]) - 1
        if i not in self.mission.drones:
            raise ValueError(f"the mission has no steps for drone {parts[2]}")
        return [i]

    async def run_command(self, parts):
        """Handle `mission ...`; True if it was one."""
        if not parts or parts[0].lower() != "mission":
            return False
        sub = parts[1].lower() if len(parts) > 1 else "show"
        try:
            if sub == "load" and len(parts) == 3:
                self.load(parts[2])
            elif sub == "show":
                self.show()
            elif sub == "upload":
                await self.upload(self._targets(parts))
            elif sub == "start":
                await self.start(self._targets(parts))
            elif sub == "stop":
                self.stop()
            elif sub == "clear":
                await self.clear(self._targets(parts))
            elif sub == "runs":
                for path in self.runs():
                    print(f"  {os.path.basename(path)}")
            elif sub == "compare":
                self.compare(parts[2:])
            else:
                print(MISSION_HELP)
        except (OSError, ValueError, RuntimeError, yaml.YAMLError) as err:
            print(f"mission {sub}: {err}")
        return True

    def load(self, path):
        self._require_idle()
        path = self._find(path)
        self.mission = load_mission(path, [link.name for link in self.links])
        self.uploaded.clear()
        log.info(f"Mission {self.mission.name} loaded from {path}: " + ", ".join(
            f"{dm.name} {len(dm.steps)} steps" for dm in self.mission.drones.values()))

    def show(self):
        if not self.mission:
            print("No mission loaded (mission load FILE)")
            return
        m = self.mission
        print(f"Mission {m.name} ({m.path}), north/east from each drone's "
              f"{'start' if m.origin == 'start' else 'home'}:")
        for i, dm in sorted(m.drones.items()):
            state = (f"uploaded, {len(dm.items)} items" if i in self.uploaded
                     else "not uploaded")
            if i in self.flying:
                state += f", flying (item {self.links[i].mission_seq + 1})"
            speed = f", {dm.speed:g} m/s" if dm.speed else ""
            print(f"  {dm.name}{speed}: {state}")
            for n, step in enumerate(dm.steps, start=1):
                print(f"    {n}. " + "  ".join(f"{k}: {v}" for k, v in step.items()))

    async def upload(self, indices):
        self._require_idle()
        if not self.mission:
            raise ValueError("load a mission first")
        m = self.mission

        async def one(i):
            link, dm = self.links[i], m.drones[i]
            self.uploaded.discard(i)
            self.swarm._clear_plan(i)
            if not link.position:
                raise RuntimeError(f"{link.name}: no position yet")
            lat, lon, _, rel = link.position
            airborne = bool(link.armed and rel > 2.0)
            home = self.home() if m.origin == "home" else None
            if m.origin == "home" and not home:
                raise RuntimeError("no home position yet")
            build_items(dm, home or (lat, lon), (lat, lon), rel if airborne else 0.0, airborne,
                        link.sysid, self.default_alt, airframe=link.airframe)
            await asyncio.to_thread(link.upload_items, dm.items)
            self.uploaded.add(i)
            self.swarm.plans[i] = list(dm.route)
            self.swarm.plan_alts[i] = dm.route_alt
            self.swarm.plan_max_alts[i] = dm.max_alt
            log.info(f"{link.name}: mission {m.name} uploaded ({len(dm.items)} items, "
                     f"{len(dm.route)} waypoints)")

        results = await asyncio.gather(*(one(i) for i in indices), return_exceptions=True)
        for i, r in zip(indices, results):
            if isinstance(r, Exception):
                log.warning(f"{self.links[i].name}: mission upload failed: {r}")
        self.swarm.check_crossings(indices)

    async def start(self, indices):
        self._require_idle()
        if not self.mission:
            raise ValueError("load a mission first")
        missing = [self.links[i].name for i in indices if i not in self.uploaded]
        if missing:
            raise ValueError(f"upload it first: {', '.join(missing)} (mission upload)")
        # (made first, so before_start can tag the run, e.g. in a capture)
        run_id = time.strftime("%Y%m%d_%H%M%S") + f"_{time.time_ns() % 1_000_000_000:09d}"
        self.run_id = run_id
        if self.before_start:
            result = self.before_start()
            if asyncio.iscoroutine(result):
                await result
        if self._log_task and not self._log_task.done():
            self._log_task.cancel()
        self.run_id = run_id
        self.run_log = RunLog(self.run_dir, self.mission.name, run_id, self.links, indices)
        for i in indices:
            link = self.links[i]
            link.mission_seq = link.mission_reached = -1
            link.mission_state = 0
            companion = self.companions.get(i)
            if companion:
                companion.set_mission_active(True)
            self.flying.add(i)
            self.swarm._start(i, self._mission(i))
            # A command can be replaced before its coroutine starts (and thus
            # before its finally block can clean up).
            self.swarm.tasks[i].add_done_callback(lambda task, i=i: self.flying.discard(i))
        self._log_task = asyncio.get_running_loop().create_task(
            self._log_run(self.run_log, self.mission, run_id, list(indices)))
        self._run_hook("start", self.mission, run_id, indices)
        log.info(f"Mission {self.mission.name} started on "
                 f"{', '.join(self.links[i].name for i in indices)}; run log {self.run_log.path}")

    async def _mission(self, i):
        link, dm = self.links[i], self.mission.drones[i]
        try:
            self.swarm.plans[i] = list(dm.route)
            self.swarm.plan_alts[i] = dm.route_alt
            self.swarm.plan_max_alts[i] = dm.max_alt
            started = time.monotonic()
            # (a previous run's progress mustn't count as this one's)
            link.mission_seq = link.mission_reached = -1
            await self.swarm._arm_and(i, link.start_mission)
            # PX4 switches to MISSION within a moment; then follow it until it
            # leaves. Armed too: a drone that landed at the end of a mission is
            # still in MISSION, disarmed, until this start takes effect
            for _ in range(20):
                if link.mode == "MISSION" and link.armed:
                    break
                await asyncio.sleep(0.5)
            else:
                raise RuntimeError("PX4 accepted the start but did not enter MISSION mode")
            landing = dm.items[-1][1] in (M.MAV_CMD_NAV_LAND, M.MAV_CMD_NAV_RETURN_TO_LAUNCH)
            if landing:
                self.swarm.mission_ends[i] = (len(dm.items) - 1, "RTL" if dm.items[-1][1] ==
                                              M.MAV_CMD_NAV_RETURN_TO_LAUNCH else "LAND")
            complete = False
            observed_active = False
            pending_tracks = list(dm.track_steps)
            # (giving way to another drone pauses it; the swarm resumes it)
            while link.mode == "MISSION" or i in self.swarm.giving_way \
                    or (landing and link.mode in ("RTL", "LAND")):
                # Wait for PX4's completion report, including trailing camera
                # actions; reaching the last navigation waypoint is too early.
                observed_active = observed_active or (link.mission_time >= started
                                                      and link.mission_state == M.MISSION_STATE_ACTIVE)
                complete = (observed_active and link.mission_time >= started
                            and link.mission_state == M.MISSION_STATE_COMPLETE)
                complete = complete or link.mission_reached >= len(dm.items) - 1
                # Track steps PX4 has run (the current item is past them)
                while pending_tracks and link.mission_seq >= pending_tracks[0][0]:
                    self._set_tracking(i, pending_tracks.pop(0)[1])
                if not link.armed or (complete and not landing):
                    break
                k = max((n for n, seq in enumerate(dm.route_seq) if seq <= link.mission_seq),
                        default=0)
                if dm.route:
                    self.swarm.plans[i] = dm.route[k:]
                    self.swarm.destinations[i] = dm.route[k]
                await asyncio.sleep(0.5)
            done = complete or not link.armed
            if complete and not landing and link.mode == "MISSION":
                # X stands down based on the flight mode, so explicitly leave
                # MISSION after the last action instead of hovering in it.
                result = await self.swarm._call(link.hold)
                if result != M.MAV_RESULT_ACCEPTED:
                    raise RuntimeError("mission complete, but PX4 did not confirm HOLD; X remains stood down")
            log.info(f"{link.name}: mission {'done' if done else 'left'} "
                     f"({link.mode} at item {link.mission_seq + 1} of {len(dm.items)})")
        finally:
            self.swarm.mission_ends.pop(i, None)
            self._set_tracking(i, None)
            self.flying.discard(i)
            if self.swarm.tasks.get(i) is asyncio.current_task():
                self.swarm._clear_plan(i)
            companion = self.companions.get(i)
            if companion:
                companion.set_mission_active(link.mode == "MISSION")

    def _set_tracking(self, i, spec):
        """The mission handed drone i's camera to a tracker (spec), or took it back."""
        if spec == self.tracking.get(i):
            return
        if spec is None:
            self.tracking.pop(i, None)
            if self.companions.get(i):
                # X stops tracking; it would carry on if the mission left
                # MISSION mode next (e.g. for RTL), where it doesn't stand down
                self.companions[i].cancel()
        else:
            self.tracking[i] = spec
        name = self.links[i].name
        companion = self.companions.get(i)
        if companion and spec:
            # Operator input reaches X again (the station keeps this in step
            # with who PX4 says controls the gimbal)
            companion.set_mission_active(False)
        if spec:
            log.info(f"{name}: the mission hands the camera to the tracker"
                     + (f" ({spec['target']})" if spec["target"] else "")
                     + ("" if self.on_track or not companion else
                        ": pick the target in the Tracker view"))
        else:
            log.info(f"{name}: tracking ended; the mission has the camera")
        if self.on_track:
            try:
                self.on_track(i, spec, self.run_id)
            except Exception as err:
                log.warning(f"{name}: tracking hook failed: {err}")

    def _require_idle(self):
        if self.flying:
            raise ValueError("a mission is running; stop it before loading, uploading or starting another")

    def invalidate(self):
        """The sim reloaded, so uploaded missions no longer exist on PX4."""
        self.uploaded.clear()

    def close(self):
        for i in list(self.flying):
            task = self.swarm.tasks.get(i)
            if task:
                task.cancel()
        if self._log_task:
            self._log_task.cancel()
        if self.run_log:
            self.run_log.close()

    def _run_hook(self, event, mission, run_id, indices):
        if not self.on_run:
            return
        try:
            self.on_run(event, mission, run_id, list(indices))
        except Exception as err:
            log.warning(f"Mission run hook ({event}) failed: {err}")

    async def _log_run(self, run_log, mission, run_id, indices):
        try:
            while self.flying:
                run_log.sample()
                await asyncio.sleep(0.2)
            # A few seconds more: the drones settling after the last step
            for _ in range(15):
                run_log.sample()
                await asyncio.sleep(0.2)
        finally:
            run_log.close()
            self._run_hook("end", mission, run_id, indices)
            log.info(f"Mission run logged: {run_log.path} (mission compare: compare "
                     "the last two runs)")

    def stop(self):
        if not self.flying:
            print("No drone is flying the mission")
            return
        self.swarm.act(sorted(self.flying), "hold")

    async def clear(self, indices):
        self._require_idle()
        for i in indices:
            await asyncio.to_thread(self.links[i].clear_mission)
            self.uploaded.discard(i)
            self.swarm._clear_plan(i)
        log.info(f"Mission cleared on {', '.join(self.links[i].name for i in indices)}")

    def runs(self):
        if not os.path.isdir(self.run_dir):
            return []
        # <mission>_<date>_<time>_<ns>.csv: the run logs, not the files tagged
        # with a run (e.g. <run>_track_<drone>.csv)
        name = re.escape(self.mission.name) if self.mission else r"[\w.-]+"
        pattern = re.compile(name + r"_\d{8}_\d{6}(_\d+)?\.csv")
        return sorted(os.path.join(self.run_dir, f) for f in os.listdir(self.run_dir)
                      if pattern.fullmatch(f))

    def compare(self, names):
        if len(names) == 2:
            paths = [n if os.path.isfile(n) else os.path.join(self.run_dir, n) for n in names]
        else:
            paths = self.runs()[-2:]
            if len(paths) < 2:
                raise ValueError("there aren't two runs of this mission yet")
        rows = compare_runs(*paths)
        print(f"{os.path.basename(paths[0])} vs {os.path.basename(paths[1])}, matched by time "
              "since the start:")
        for drone, n, mean, worst, heading, pitch, yaw in rows:
            print(f"  {drone}: {n} samples, apart by {mean:.1f} m on average, at most "
                  f"{worst:.1f} m; heading at most {heading:.1f} deg apart; "
                  f"gimbal pitch {pitch:.1f}, yaw {yaw:.1f} deg apart (nan = no feedback)")
        if not rows:
            print("  no drone in both runs")
        # Tracking scores (a `track` step in the sim), side by side
        for drone in sorted({d for p in paths for d in track_summaries(p)}):
            scores = [track_summaries(p).get(drone) for p in paths]
            print(f"  {drone} tracking: " + " vs ".join(
                f"in frame {s[0]:.0%}, offset {s[1]:.1f} deg" if s else "none" for s in scores))
