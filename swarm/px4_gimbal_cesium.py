"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Flies a PX4 quadrotor with a gimbaled payload camera in a real-world location
(Google Photorealistic 3D Tiles streamed through Cesium for Unreal). The default
spawn is Seletar Airport's runway, Singapore.

- The GimbalCam is published as RTSP (rtsp://127.0.0.1:8554/gimbal by default)
  and announced to QGroundControl as a MAVLink camera, so QGC shows the video
  and its zoom controls work.
- The gimbal can be steered from QGC, or from the dashboard window: W/A/S/D to
  move, mouse wheel or +/- to zoom, C to centre, X to look straight down.
- Objects from config_spawned_objects.yml are placed on the ground; type `help`
  in this terminal for commands to spawn, list and delete objects while running.
- The GimbalCam's resolution, sensor and zoom lens come from
  camera_properties.json; zooming moves along the lens's focal-length range.
- --spotter opens a window to spawn and walk a human around the world.
- --drones N flies a swarm: N PX4 drones side by side, each with its own
  PX4, gimbal camera and RTSP stream (/gimbal, /gimbal2, ...). Fly them from
  QGC (one UDP link, servers 127.0.0.1:18570, 18571, ...) or from this
  terminal (`all takeoff`, `2 goto LAT LON ALT`, `all search ...`; see help).
  Keys 1-9 in the dashboard pick which drone it shows and steers.

Gimbal commands go through PX4's gimbal manager on the onboard payload MAVLink
link (PX4 sends to UDP 14030), the same path a companion-computer tracker would
use. Replace gimbal_setpoint() with your own tracking controller.

Usually started through run_cesium_sim.sh, which also launches the simulator,
PX4 and the RTSP server. To run it by hand:
  1. Start Project AirSim on the GISMap level with CESIUM_ION_TOKEN set.
  2. Start MediaMTX (RTSP server) and PX4 SITL with the gimbal enabled at boot:
       cd PX4-Autopilot
       PX4_PARAM_MNT_MODE_IN=4 PX4_PARAM_MNT_MODE_OUT=2 make px4_sitl none_iris
  3. python px4_gimbal_cesium.py [options]

Examples:
  python px4_gimbal_cesium.py                                  # Seletar runway demo
  python px4_gimbal_cesium.py --lat 1.3521 --lon 103.8198 --heading 90
  python px4_gimbal_cesium.py --qgc                            # fly from QGroundControl
  python px4_gimbal_cesium.py --scene scene_px4_gimbal_payload_blocks.jsonc
"""

import argparse
import asyncio
import csv
import faulthandler
import json
import logging
import math
import os
import signal
import subprocess
import sys
import threading
import time
import urllib.request

import commentjson
import cv2

from projectairsim import ProjectAirSimClient, Drone, World
from projectairsim.types import Pose, Quaternion, Vector3
from projectairsim.utils import load_scene_config_as_dict, projectairsim_log, quaternion_to_rpy

from aircraft import Aircraft, list_models
from ardupilot_sitl import ArduPilotSitl
from camera_properties import CameraProperties
from gcs.airframe import airframe_of
from gcs.fleet import Fleet, load_fleet, sim_drones, sim_models
from gcs.log import log as gcs_log
from gcs.mavlink_link import M, VehicleLink
from gcs.minimap import road_segments
from gcs.station import DroneCamera, GroundStation
from gcs.swarm import search_demo
from gt_capture import GroundTruthRecorder, add_segmentation_capture, kind_of_asset
from payload_mavlink import SimCamera, payload_connection
from payload_video import RtspStreamer, image_to_bgr
from px4_sitl import Px4Sitl
from shared_frame import SharedFrameWriter
from moving_actors import ACTORS_HELP, GROUND_TRUTH_KEEP, MovingActorManager
from spawned_objects import CONSOLE_HELP, ObjectSpawner
from experiment_recorder import ExperimentRecorder
from mission_track import MissionTrack
from track_test import run_track_test

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_CONFIG_DIR = os.path.join(SCRIPT_DIR, "sim_config")
TEMPLATE_SCENE = "scene_px4_gimbal_cesium_seletar.jsonc"
GENERATED_SCENE = "scene_px4_gimbal_cesium_custom.jsonc"
GENERATED_ROBOT = "robot_{}_custom{}.jsonc"  # aircraft model, drone suffix
FIXEDWING_SPACING = 40.0  # m between fixed-wings on the runway, ahead of the spawn

# Seletar Airport runway 03/21 centreline, facing up the runway
DEFAULT_LAT, DEFAULT_LON, DEFAULT_HEADING = 1.413425, 103.865000, 33.0
DEFAULT_GROUND_ALT = 12.0  # m MSL at the default spawn

SPOTTER_POV_ROBOT = "PovCam"  # must not share a prefix with spawned object names
GIMBAL_CAM_INITIAL_PITCH = -45.0  # matches the gimbal origin in the robot config


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--lat", type=float, help="spawn latitude (deg)")
    parser.add_argument("--lon", type=float, help="spawn longitude (deg)")
    parser.add_argument(
        "--heading", type=float, default=None, help="spawn heading (deg from north)"
    )
    parser.add_argument(
        "--ground-alt",
        type=float,
        default=None,
        help="ground elevation at the spawn (m MSL). Looked up online if omitted.",
    )
    parser.add_argument(
        "--spawn-height",
        type=float,
        default=15.0,
        help="spawn this far above --ground-alt; the drone then settles onto "
        "the real tile surface (default 15)",
    )
    parser.add_argument(
        "--geoid",
        type=float,
        default=8.1,
        help="geoid height at the spawn (m); ~8 across Singapore. Only affects "
        "how the Cesium origin lines up with MSL.",
    )
    parser.add_argument(
        "--objects",
        default=os.path.join(SCRIPT_DIR, "config_spawned_objects.yml"),
        help="YAML file of objects to place in the world ('' for none)",
    )
    parser.add_argument(
        "--drones",
        type=int,
        default=None,
        help="number of PX4 drones (a swarm), each with its own PX4 instance "
        "(default: as many as the config file lists)",
    )
    parser.add_argument(
        "--drone-spacing",
        type=float,
        default=8.0,
        help="metres between the drones at the spawn, in a line to the right (default 8)",
    )
    parser.add_argument(
        "--model",
        choices=list(list_models()),
        help="aircraft of the drones whose `model:` the config file doesn't give "
        "(default quad_px4; see python aircraft.py --list)",
    )
    parser.add_argument(
        "--companion",
        default=os.environ.get("PI_IP", ""),
        help="IP of a companion computer running the X vision pipeline on Drone1 "
        "(default: run_cesium_sim.sh's --pi-ip). The dashboard shows its tracker "
        "video and sends it click-to-track and gimbal commands.",
    )
    parser.add_argument(
        "--config",
        "--fleet",
        dest="config",
        default=os.path.join(SCRIPT_DIR, "config.yml"),
        help="config file: the drones, their companions and the ground station's "
        "settings (default: config.yml here; gcs/fleet.example.yml has every key). "
        "--drones, --companion and --layer-step override it",
    )
    parser.add_argument(
        "--layer-step",
        type=float,
        default=None,
        help="drone N flies (N-1) x this many metres above the height asked for, so "
        "drones whose paths cross pass at different heights (default: the config "
        "file's swarm: layer_step_m, 5; 0 = off)",
    )
    parser.add_argument(
        "--actors",
        default=os.path.join(SCRIPT_DIR, "config_moving_actors.yml"),
        help="YAML file of people and vehicles moving along paths ('' for none)",
    )
    parser.add_argument(
        "--camera-props",
        default=os.path.join(SCRIPT_DIR, "camera_properties.json"),
        help="camera_properties.json for the GimbalCam (resolution, sensor, zoom lens)",
    )
    parser.add_argument(
        "--spotter",
        action="store_true",
        help="open the spotter window to spawn and walk a human in the world",
    )
    parser.add_argument(
        "--qgc",
        action="store_true",
        help="load the world and wait; arm, fly missions and steer the gimbal "
        "from QGroundControl or the dashboard window instead of the scripted demo",
    )
    parser.add_argument(
        "--rtsp-url",
        default="rtsp://127.0.0.1:8554/gimbal",
        help="where to publish the GimbalCam video",
    )
    parser.add_argument(
        "--rtsp-advertise-url",
        help="video URL announced to QGC and companion computers, if different "
        "from --rtsp-url (e.g. this laptop's LAN address)",
    )
    parser.add_argument("--no-rtsp", action="store_true", help="don't publish video")
    parser.add_argument(
        "--stream-bitrate",
        type=int,
        default=0,
        metavar="KBPS",
        help="H.264 bitrate of the published video, kbit/s (0: the encoder's "
        "default, about 2000 at 1080p)",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="also print debug messages in the terminal (they always go to "
        "projectairsim_client.log)",
    )
    parser.add_argument(
        "--tile-wait",
        type=float,
        default=90.0,
        help="longest time to keep the sim paused while Cesium streams the ground "
        "under the drone (default 90)",
    )
    parser.add_argument(
        "--ground-truth",
        nargs="?",
        const=os.path.join(os.path.dirname(SCRIPT_DIR), "datasets", "sim_gt"),
        metavar="DIR",
        help="save labelled GimbalCam frames from every drone (YOLO labels, plus "
        "each box's actor for scoring tracking and ReID) under DIR "
        "[datasets/sim_gt in the ProjectAirSim checkout]; see gt_capture.py. Costs a second render per GimbalCam",
    )
    parser.add_argument(
        "--gt-rate",
        type=float,
        default=2.0,
        help="labelled frames saved per second per drone with --ground-truth (default 2)",
    )
    parser.add_argument(
        "--scene",
        help="load this scene config from sim_config/ as-is instead of building "
        "one from --lat/--lon",
    )
    return parser.parse_args()


def lookup_ground_alt(lat, lon):
    """Ground elevation (m MSL) from Open-Meteo's free Copernicus DEM service."""
    url = f"https://api.open-meteo.com/v1/elevation?latitude={lat}&longitude={lon}"
    try:
        with urllib.request.urlopen(url, timeout=10) as resp:
            return float(json.load(resp)["elevation"][0])
    except Exception as err:
        projectairsim_log().warning(
            f"Elevation lookup failed ({err}); pass --ground-alt. Assuming 0 m."
        )
        return 0.0


def build_robot(camera, aircraft, index=0, ground_truth=False, home=(DEFAULT_LAT, DEFAULT_LON)):
    """Write the robot config for drone `index` (0-based), an aircraft.Aircraft,
    and return its name. Each drone talks to its own autopilot instance (ports
    offset by the index); home (lat, lon) is the scene's, for PX4's estimator.
    ground_truth: the GimbalCam also renders segmentation."""
    robot = aircraft.robot(index, home)
    sensors = {s["id"]: s for s in robot["sensors"]}
    camera.apply_to_camera_config(sensors["GimbalCam"])
    if ground_truth:
        add_segmentation_capture(sensors["GimbalCam"])
    name = GENERATED_ROBOT.format(aircraft.name, "" if index == 0 else f"_{index + 1}")
    with open(os.path.join(SIM_CONFIG_DIR, name), "w") as f:
        f.write("// Generated by px4_gimbal_cesium.py; overwritten on every run.\n")
        json.dump(robot, f, indent=2)
    return name


def build_scene(args, aircraft, camera):
    """Write a scene config for the requested spawn point, and each drone's robot
    config, and return the scene's name. aircraft: aircraft.Aircraft per drone."""
    with open(os.path.join(SIM_CONFIG_DIR, TEMPLATE_SCENE)) as f:
        scene = commentjson.load(f)

    if args.lat is None and args.lon is None:
        lat, lon = DEFAULT_LAT, DEFAULT_LON
        heading = DEFAULT_HEADING if args.heading is None else args.heading
        ground_alt = DEFAULT_GROUND_ALT if args.ground_alt is None else args.ground_alt
    elif args.lat is None or args.lon is None:
        raise SystemExit("--lat and --lon must be given together")
    else:
        lat, lon = args.lat, args.lon
        heading = 0.0 if args.heading is None else args.heading
        ground_alt = (
            lookup_ground_alt(lat, lon) if args.ground_alt is None else args.ground_alt
        )

    projectairsim_log().info(
        f"Spawn {lat:.6f}, {lon:.6f}, heading {heading:.0f} deg, ground {ground_alt:.1f} m MSL"
    )

    # Drone1 at the spawn point, the others in a line to its right; aircraft
    # that can't hover on the runway centreline ahead of it, facing along it
    template = scene["actors"][0]
    drones = []
    h = math.radians(heading)
    fixedwings = 0
    for i, craft in enumerate(aircraft):
        robot_config = build_robot(camera, craft, i, bool(args.ground_truth), (lat, lon))
        if craft.capabilities.hovers:
            ahead, right = 0.0, i * args.drone_spacing
        else:
            fixedwings += 1
            ahead, right = fixedwings * FIXEDWING_SPACING, 0.0
        north = ahead * math.cos(h) - right * math.sin(h)
        east = ahead * math.sin(h) + right * math.cos(h)
        d_lat = north / 111_320.0
        d_lon = east / (111_320.0 * math.cos(math.radians(lat)))
        drone = json.loads(json.dumps(template))
        drone["name"] = f"Drone{i + 1}"
        drone["origin"] = {
            "geo-point": f"{lat + d_lat} {lon + d_lon} {ground_alt + args.spawn_height}",
            "rpy-deg": f"0 0 {heading}",
        }
        drone["robot-config"] = robot_config
        drones.append(drone)
    scene["actors"][0:1] = drones
    scene["home-geo-point"] = {"latitude": lat, "longitude": lon, "altitude": ground_alt}
    scene["tiles-altitude-offset"] = args.geoid
    # Only ground-truth capture uses segmentation. Its IDs put the Cesium tiles
    # in the custom depth pass too (to occlude), which costs GPU otherwise wasted
    scene.setdefault("segmentation", {})["initialize-ids"] = bool(args.ground_truth)
    if args.spotter:
        # Carries the spotter's first-person camera; spotter_control.py moves it
        scene["actors"].append(
            {
                "type": "robot",
                "name": SPOTTER_POV_ROBOT,
                "origin": {"xyz": "0 0 -50", "rpy-deg": "0 0 0"},
                "robot-config": "robot_spotter_pov.jsonc",
            }
        )

    with open(os.path.join(SIM_CONFIG_DIR, GENERATED_SCENE), "w") as f:
        f.write("// Generated by px4_gimbal_cesium.py; overwritten on every run.\n")
        json.dump(scene, f, indent=2)
    return GENERATED_SCENE


def gimbal_setpoint(t):
    """Demo trajectory: slow pan while nodding between nadir and horizon."""
    pitch = -50.0 + 40.0 * math.sin(2 * math.pi * t / 12.0)
    yaw = 60.0 * math.sin(2 * math.pi * t / 20.0)
    return pitch, yaw


class SimPayload(DroneCamera):
    """One sim drone's cameras: frames from the sim (to the dashboard while it
    is the selected drone, and to RTSP), the sim zoom, and the MAVLink camera
    that QGC and companion computers zoom through. The gimbal is steered over
    MAVLink like a real drone's (DroneCamera).

    Everything that talks to the sim runs on the asyncio thread via `tick()`.
    """

    def __init__(self, link, windows, sim_camera, rtsp, camera):
        super().__init__(link, windows)
        self.drone = None
        self.sim_camera = sim_camera  # SimCamera
        self.rtsp = rtsp
        self.camera = camera  # CameraProperties
        self.zoom_speed = float(
            camera.props.get("zoom", {}).get("zoom_speed_percent_per_s", 30.0)
        )
        self.zoom_percent = 0.0
        self.zoom_rate = 0.0
        self.applied_zoom = None
        self.last_gimbal_frame = None
        self.frame_counts = {"ChaseCam": 0, "GimbalCam": 0}
        # `sim attitude`: (csv writer, file, end time) while recording
        self.attitude_log = None
        self.camera_rotation = None
        self.camera_pose = None  # ((x, y, z), (w, x, y, z)) of the latest frame
        self.pose_wanted = 0  # >0 while a test needs camera_pose
        self.ground_truth = None  # GroundTruthRecorder with --ground-truth
        self.frame_taps = []  # callables given every GimbalCam image (a recording)
        self._attitude_t0 = 0.0

    def attach(self, client, drone):
        """Follow a (re)loaded drone: subscribe to its cameras and reapply zoom."""
        self.drone = drone
        self.applied_zoom = None
        self.pitch, self.yaw = GIMBAL_CAM_INITIAL_PITCH, 0.0
        client.subscribe(drone.sensors["Chase"]["scene_camera"], self.on_chase_image)
        client.subscribe(drone.sensors["GimbalCam"]["scene_camera"], self.on_gimbal_image)

    def on_chase_image(self, _, image_msg):
        self.frame_counts["ChaseCam"] += 1
        if self.selected:
            self.windows.receive("ChaseCam", image_to_bgr(image_msg))

    def on_gimbal_image(self, _, image_msg):
        self.frame_counts["GimbalCam"] += 1
        if self.attitude_log is not None or self.pose_wanted:
            # The camera's world pose (NED) when this frame was rendered
            self.camera_rotation = tuple(image_msg[k] for k in ("rot_w", "rot_x", "rot_y", "rot_z"))
            self.camera_pose = (tuple(image_msg[k] for k in ("pos_x", "pos_y", "pos_z")),
                                self.camera_rotation)
        for tap in self.frame_taps:
            tap(image_msg)
        bgr = image_to_bgr(image_msg)
        self.last_gimbal_frame = bgr
        if self.selected:
            self.windows.receive("GimbalCam", bgr)
        if self.rtsp:
            self.rtsp.push(bgr)

    def set_zoom(self, percent):
        self.zoom_percent, self.zoom_rate = max(0.0, min(100.0, percent)), 0.0

    def nudge_zoom(self, delta_percent):
        self.zoom_percent = max(0.0, min(100.0, self.zoom_percent + delta_percent))

    def zoom_text(self):
        focal = self.camera.focal_length(self.zoom_percent)
        return f"{focal:.1f} mm ({focal / self.camera.focal_min_mm:.1f}x)"

    def record_attitude(self, seconds, path):
        """Log the drone's attitude and the GimbalCam's actual (rendered) world
        attitude, about 20 times a second, to a CSV: how well the camera is
        stabilised against the drone's motion."""
        f = open(path, "w", newline="")
        writer = csv.writer(f)
        writer.writerow(["t", "drone_roll", "drone_pitch", "drone_yaw",
                         "cam_roll", "cam_pitch", "cam_yaw", "gimbal_pitch_cmd", "gimbal_yaw_cmd"])
        self._attitude_t0 = time.monotonic()
        self.attitude_log = (writer, f, self._attitude_t0 + seconds)
        self._attitude_next = 0.0

    def _log_attitude(self):
        writer, f, end = self.attitude_log
        now = time.monotonic()
        if now >= end:
            f.close()
            self.attitude_log = None
            projectairsim_log().info(f"{self.name}: attitude log written to {f.name}")
            return
        if now < self._attitude_next or self.camera_rotation is None or not self.drone:
            return
        self._attitude_next = now + 0.05
        q = self.drone.get_ground_truth_kinematics()["pose"]["orientation"]
        drone = [math.degrees(v) for v in quaternion_to_rpy(q["w"], q["x"], q["y"], q["z"])]
        cam = [math.degrees(v) for v in quaternion_to_rpy(*self.camera_rotation)]
        att = self.link.gimbal_attitude or (math.nan, math.nan)
        writer.writerow([round(now - self._attitude_t0, 3), *(round(v, 2) for v in drone + cam),
                         round(att[0], 2), round(att[1], 2)])

    def tick(self, dt):
        if self.attitude_log is not None:
            self._log_attitude()
        requests = self.sim_camera.zoom_requests
        while not requests.empty():
            kind, value = requests.get()
            if kind == "range":
                self.zoom_percent, self.zoom_rate = value, 0.0
            else:
                self.zoom_rate = self.zoom_speed * value
        if self.zoom_rate:
            self.nudge_zoom(self.zoom_rate * dt)

        if self.drone and (
            self.applied_zoom is None or abs(self.zoom_percent - self.applied_zoom) > 0.5
        ):
            focal = self.camera.focal_length(self.zoom_percent)
            self.drone.set_field_of_view("GimbalCam", 0, self.camera.hfov_deg(focal))
            if self.ground_truth:
                self.ground_truth.set_fov(self.drone, self.camera.hfov_deg(focal))
            self.applied_zoom = self.zoom_percent
            self.sim_camera.publish_zoom(self.zoom_percent)

        while not self.sim_camera.capture_requests.empty():
            self.sim_camera.capture_requests.get()
            self.save_snapshot()

    def save_snapshot(self):
        if self.last_gimbal_frame is None:
            return
        out_dir = os.path.expanduser("~/Pictures/projectairsim")
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, time.strftime(f"{self.name}_GimbalCam_%Y%m%d_%H%M%S.png"))
        cv2.imwrite(path, self.last_gimbal_frame)
        self.sim_camera.report_image_captured(path)
        projectairsim_log().info(f"Saved {path}")


GOTO_HELP = """  goto LAT LON [HEADING]                move the drone(s) to a new place (restarts PX4)
  sim attitude [SECONDS] [N]            log drone N's attitude and its camera's (stabilisation)
  sim tracktest [N] [follow|lock]       fly hard while pointing the gimbal at a ground target,
                                        and measure how well it stays in frame
  gt                                    --ground-truth: frames and boxes saved so far
  sim cut N | sim battery N PCT | sim reset N|all
                                        simulate a lost link or low battery on drone N;
                                        reset undoes them and restarts the drone's
                                        simulated hardware in PX4 (also the safety
                                        panel's Sim tab, F9)"""
SIM_HELP = CONSOLE_HELP + "\n" + ACTORS_HELP + "\n" + GOTO_HELP


def reset_actors_for_mission(session):
    """Before each `mission start`: the moving actors start again from where
    they began (their paths are seeded), so every run sees the same scene."""
    missions = session.station.missions
    if session.actors:
        # The mission's own traffic, or the usual actors (--actors)
        wanted = (missions.mission.actors if missions.mission else None) or session.args.actors
        if wanted != session.actors.config_path or session.actors.actors:
            projectairsim_log().info(f"Mission start: moving actors from {os.path.basename(wanted)} "
                                     "back to their start")
            session.actors.reload(wanted)
            if session.ground_truth:
                session.refresh_ground_truth()
    if session.ground_truth and missions.mission:
        session.ground_truth.note_run(missions.mission.name, missions.run_id, "start")


def run_tag(session):
    """'<mission>_<run>_' while a mission runs, to tag measurement files with it."""
    missions = session.station.missions if session.station else None
    if missions and missions.flying and missions.mission:
        return f"{missions.mission.name}_{missions.run_id}_"
    return ""


def record_mission(session, event, mission, run_id, indices):
    """Record a mission run that asks for it (`record:` in the mission file;
    experiment_recorder.py), from its start until just after its end."""
    loop = asyncio.get_running_loop()
    if event == "start":
        wanted = [i for i in indices if i in mission.record]
        if not wanted:
            return
        if session.recording:
            loop.create_task(session.recording.stop())
        recorder = ExperimentRecorder(session, mission, run_id, wanted,
                                      session.station.missions.run_dir)
        recorder.start()
        session.recording = recorder
    elif event == "end" and session.recording and session.recording.run_id == run_id:
        recorder, session.recording = session.recording, None
        loop.create_task(recorder.stop())


def mission_track(session, index, spec, run_id):
    """A mission's `track` step on drone `index` (spec None: tracking ended):
    lock the tracker onto the target and score it (mission_track.py)."""
    missions = session.station.missions
    old = session.mission_tracks.pop(index, None)
    loop = asyncio.get_running_loop()
    if old:
        loop.create_task(old.stop())
    name = session.station.links[index].name
    if session.ground_truth and missions.mission:
        target = f" {spec['target']}" if spec and spec["target"] else ""
        session.ground_truth.note_run(missions.mission.name, run_id,
                                      f"track{target}" if spec else "track off", name)
    if not spec:
        return
    try:
        track = MissionTrack(session, index, spec, missions.mission.name, run_id,
                             missions.run_dir)
        track.start()
    except ValueError as err:
        projectairsim_log().warning(f"{name}: can't track: {err}")
        return
    session.mission_tracks[index] = track


async def sim_command(session, line, parts):
    """The sim's own console commands, after the ground station's; anything
    else goes to the object spawner, which reports what it doesn't know."""
    word = parts[0].lower()
    if word == "goto":
        try:
            lat, lon = float(parts[1]), float(parts[2])
            heading = float(parts[3]) if len(parts) > 3 else 0.0
        except (IndexError, ValueError):
            print("Usage: goto LAT LON [HEADING]")
            return True
        if not session.args.qgc:
            print("goto is only available in --qgc mode")
            return True
        try:
            await session.load(lat, lon, heading)
        except Exception as err:
            print(f"goto failed: {err}")
    elif word == "actors":
        if session.actors:
            session.actors.run_command(parts[1:])
    elif word == "gt":
        print(session.ground_truth.status() if session.ground_truth
              else "Ground-truth capture is off (start with --ground-truth)")
    elif word == "sim" and len(parts) >= 2 and parts[1] == "tracktest":
        # sim tracktest [N] [follow|lock]
        args = parts[2:]
        mode = next((a for a in args if a in ("follow", "lock")), "follow")
        number = next((int(a) for a in args if a.isdigit()), 1)
        if not 1 <= number <= len(session.station.cameras):
            print("Usage: sim tracktest [DRONE] [follow|lock]")
            return True
        if session.station.companions.get(number - 1):
            print(f"Drone{number}'s gimbal is its companion's (X); run the test on a drone without one")
            return True
        camera = session.station.cameras[number - 1]
        asyncio.get_running_loop().create_task(run_track_test(
            session, camera, mode, os.environ.get("LOG_DIR", "/tmp/projectairsim-run"),
            run_tag(session)))
        print(f"Track test on {camera.name}, yaw {mode}: about a minute; results in the messages")
    elif word == "sim" and len(parts) >= 2 and parts[1] == "attitude":
        # sim attitude [SECONDS] [N]: how well drone N's camera is stabilised
        try:
            seconds = float(parts[2]) if len(parts) > 2 else 30.0
            camera = session.station.cameras[int(parts[3]) - 1 if len(parts) > 3 else 0]
        except (ValueError, IndexError):
            print("Usage: sim attitude [SECONDS] [DRONE]")
            return True
        path = os.path.join(os.environ.get("LOG_DIR", "/tmp/projectairsim-run"),
                            run_tag(session) + time.strftime(f"attitude_{camera.name}_%H%M%S.csv"))
        camera.record_attitude(seconds, path)
        print(f"Recording {camera.name}'s attitude and its camera's for {seconds:g} s to {path}")
    elif word == "sim":
        faults = session.station.panel.sim_faults
        try:
            count = len(session.station.links)
            if parts[1] in ("reset", "restore") and parts[2].lower() == "all":
                drones = range(count)
            else:
                drones = [int(parts[2]) - 1]
                if not 0 <= drones[0] < count:
                    raise IndexError
            for i in drones:
                if parts[1] == "cut":
                    faults.cut_link(i, True)
                elif parts[1] == "battery":
                    faults.set_battery(i, float(parts[3]))
                elif parts[1] in ("reset", "restore"):
                    faults.reset(i)
                else:
                    raise ValueError
        except (IndexError, ValueError):
            print("Usage: sim cut N | sim battery N PCT | sim reset N|all")
    elif session.spawner:
        session.spawner.run_command(line)
    return True


async def run_ground_truth(session):
    """Log where the moving actors are, for scoring detection and ReID, and
    keep --ground-truth's labels in step with actors spawned or deleted."""
    next_refresh = 0.0
    while True:
        if not session.loading and session.actors:
            try:
                session.actors.log_ground_truth()
            except Exception as err:  # e.g. the scene is reloading under us
                projectairsim_log().debug(f"Ground-truth log skipped: {err}")
        if not session.loading and session.ground_truth and time.monotonic() >= next_refresh:
            next_refresh = time.monotonic() + 2.0
            try:
                session.refresh_ground_truth()
            except Exception as err:
                projectairsim_log().debug(f"Ground-truth labels not refreshed: {err}")
        await asyncio.sleep(0.05)


def sim_objects(session):
    """Every spawned object and moving actor, for the map, as (name, kind,
    north, east, heading) from the sim's ground truth."""
    if session.loading or not session.world:
        return []
    kinds = {}
    if session.spawner:
        kinds.update(dict.fromkeys(session.spawner.spawned, "object"))
    if session.actors:
        kinds.update({name: a["kind"] for name, a in session.actors.actors.items()})
    names = list(kinds)
    poses = session.world.get_object_poses(names) if names else []
    objects = []
    for name, pose in zip(names, poses):
        t, q = pose["translation"], pose["rotation"]
        if math.isnan(t["x"]):
            continue
        yaw = math.degrees(quaternion_to_rpy(q["w"], q["x"], q["y"], q["z"])[2])
        if session.actors and name in session.actors.actors:
            yaw -= session.actors.actors[name]["yaw_offset"]
        objects.append((name, kinds[name], t["x"], t["y"], yaw))
    session.object_counts = {}
    for _, kind, *_ in objects:
        session.object_counts[kind] = session.object_counts.get(kind, 0) + 1
    return objects


def sim_status(session):
    counts = session.object_counts
    return [
        "Objects: " + (", ".join(f"{n} {k}" for k, n in sorted(counts.items())) or "none"),
        f"QGC: UDP, listen 14550, {qgc_servers(session.autopilots)}",
    ]


def qgc_servers(autopilots):
    """Where QGC finds the drones: PX4 instance N serves UDP 18570+N; ArduPilot
    sends to QGC's 14550 itself."""
    ports = [f"127.0.0.1:{18570 + a.instance}" for a in autopilots if isinstance(a, Px4Sitl)]
    text = f"server{'s' if len(ports) > 1 else ''} {', '.join(ports)}" if ports else ""
    if len(ports) < len(autopilots):
        text += (" (" if text else "") + "ArduPilot drones connect by themselves" + (")" if text else "")
    return text


async def wait_for_ground(world, max_wait, min_wait=10.0, stable_reads=3):
    """Ground height (NED down) under the drone once Cesium's collision there
    has loaded and stopped changing, or None after max_wait seconds."""
    projectairsim_log().info("Waiting for the Cesium ground under the drone to load")
    t0 = time.monotonic()
    last, stable = None, 0
    while time.monotonic() - t0 < max_wait:
        z = world.get_surface_elevation_at_point(0.0, 0.0)
        if z is not None and not math.isnan(z):
            stable = stable + 1 if last is not None and abs(z - last) < 0.05 else 0
            last = z
            if stable >= stable_reads and time.monotonic() - t0 >= min_wait:
                projectairsim_log().info(
                    f"Ground ready after {time.monotonic() - t0:.0f}s (NED z = {z:.2f} m)"
                )
                return z
        await asyncio.sleep(1)
    return None


def place_drone_above_ground(drone, world, clearance=0.5):
    """Set the drone just above the ground under it; False if no ground yet."""
    pose = drone.get_ground_truth_kinematics()["pose"]
    x, y = pose["position"]["x"], pose["position"]["y"]
    ground_down = world.get_surface_elevation_at_point(x, y)
    if ground_down is None or math.isnan(ground_down):
        return False
    drone.set_pose(
        Pose(
            {
                "translation": Vector3({"x": x, "y": y, "z": ground_down - clearance}),
                "rotation": Quaternion(pose["orientation"]),
                "frame_id": "DEFAULT_ID",
            }
        ),
        True,
    )
    return True


async def wait_while_sim_alive(session):
    """Return when the simulator stops answering (e.g. its window was closed)."""
    failures = 0
    while failures < 3:
        await asyncio.sleep(2)
        if session.loading:
            continue
        try:
            session.world.get_sim_time()
            failures = 0
        except Exception:
            failures += 1
    projectairsim_log().info("Simulator closed")


async def fly_gimbal_demo(drone, link, sweep=True):
    """Take off, sweep the gimbal for a minute and land. With a companion
    computer the gimbal is X's, so the sweep is skipped (sweep=False)."""
    if sweep:
        link.take_gimbal_control()
    drone.enable_api_control()

    # PX4 refuses to arm until its estimator has converged (a few seconds)
    for _ in range(60):
        if drone.arm():
            break
        z = drone.get_ground_truth_kinematics()["pose"]["position"]["z"]
        projectairsim_log().info(f"Waiting for PX4 to be ready to arm (NED z = {z:.2f} m)")
        await asyncio.sleep(1)
    else:
        raise RuntimeError("PX4 did not become ready to arm within 60 s")

    projectairsim_log().info("Takeoff")
    await (await drone.takeoff_async())

    projectairsim_log().info("Climb to 40 m")
    await (await drone.move_to_position_async(
        north=0.0, east=0.0, down=-40.0, velocity=4.0
    ))

    if sweep:
        projectairsim_log().info("Sweeping gimbal for 60 s")
        t0 = time.monotonic()
        while (t := time.monotonic() - t0) < 60.0:
            pitch, yaw = gimbal_setpoint(t)
            link.set_gimbal_attitude(pitch, yaw)
            await asyncio.sleep(0.05)  # 20 Hz setpoint stream
        link.set_gimbal_attitude(-90.0, 0.0)
    else:
        projectairsim_log().info("Hovering for 60 s; the gimbal is the companion's (X)")
        await asyncio.sleep(60.0)

    projectairsim_log().info("Landing")
    await (await drone.land_async())
    drone.disarm()
    drone.disable_api_control()


class Session:
    """The running world: loads a location, (re)starts PX4 for it and keeps the
    ground station's cameras and map, the objects and the spotter following
    the current scene."""

    def __init__(self, args, client, camera, models, companions):
        self.args = args
        self.client = client
        self.camera = camera
        self.station = None  # GroundStation, set once it is made
        # Each drone's aircraft (aircraft/models.yml) and its autopilot, which
        # also opens MAVLink links to the drone's companion computer
        self.aircraft = [Aircraft(model) for model in models]
        self.autopilots = [
            (Px4Sitl if craft.family == "px4" else ArduPilotSitl)(
                spec=craft.sitl(i), pi_ip=companions.get(i, ""))
            for i, craft in enumerate(self.aircraft)
        ]
        if args.scene:
            # The scene names the generated robot config; build it once here
            build_robot(camera, self.aircraft[0], 0, bool(args.ground_truth))
        self.object_counts = {}
        self.world = None
        self.drones = []
        self.spawner = None
        self.actors = None
        self.ground_truth = None  # GroundTruthRecorder with --ground-truth
        self.mission_tracks = {}  # drone index -> MissionTrack during a mission's track step
        self.recording = None  # ExperimentRecorder of the mission run being recorded
        self._labelled_generation = 0  # the actors' reload whose meshes are labelled
        self.spotter = None
        self.loading = False
        # The spotter window runs in its own process, which can't receive camera
        # topics, so its POV frames are passed on through shared memory
        self.pov_frames = (
            SharedFrameWriter(f"projectairsim_pov_{os.getpid()}") if args.spotter else None
        )

    async def load(self, lat=None, lon=None, heading=None):
        """Load the scene at a location (the --lat/--lon ones if not given)."""
        args = self.args
        if lat is not None:
            args.lat, args.lon, args.heading, args.ground_alt = lat, lon, heading, None
        scene_name = args.scene or build_scene(args, self.aircraft, self.camera)
        is_cesium = args.scene is None or "cesium" in args.scene

        self.loading = True
        try:
            self._stop_spotter()
            if self.actors:
                self.actors.close()
                self.actors = None
            # Load the scene first: it drops the old PX4's connection cleanly.
            # Then start a fresh PX4, which connects to the new scene.
            if self.world is None:
                self.world = World(self.client, scene_name, delay_after_load_sec=2)
            else:
                projectairsim_log().info(f"Moving to {lat:.6f}, {lon:.6f}")
                config, _ = load_scene_config_as_dict(scene_name, sim_config_path=SIM_CONFIG_DIR)
                self.world.load_scene(config, delay_after_load_sec=2)
            home = self.world.home_geo_point
            heading = DEFAULT_HEADING if args.heading is None else args.heading
            for autopilot in self.autopilots:
                autopilot.stop()
            for autopilot in self.autopilots:
                # ArduPilot's home is the scene's, so their positions agree
                autopilot.home = (home["latitude"], home["longitude"], home.get("altitude", 0.0), heading)
                autopilot.start()

            self.drones = [
                Drone(self.client, self.world, f"Drone{i + 1}") for i in range(len(self.autopilots))
            ]
            for payload, drone in zip(self.station.cameras, self.drones):
                payload.attach(self.client, drone)
            if self.pov_frames:
                self.client.subscribe(
                    f"{self.world.parent_topic}/robots/{SPOTTER_POV_ROBOT}/sensors/POV/scene_camera",
                    self.pov_frames.write_image_msg,
                )

            if is_cesium:
                # Cesium scenes start paused so the ground tiles (and their
                # collision) can stream in first. Wait until the ground under
                # the drones is solid, then set each just above it.
                ground = await wait_for_ground(self.world, args.tile_wait)
                if ground is None:
                    projectairsim_log().warning(
                        "No stable ground under the drone yet; resuming anyway"
                    )
                else:
                    for drone, craft in zip(self.drones, self.aircraft):
                        # Fixed-wings too, held there for launch: ArduPlane measures heights
                        # from where it arms, so its landing point (height 0) is the runway
                        # only if it starts on it
                        if not place_drone_above_ground(drone, self.world):
                            projectairsim_log().warning(f"No ground under {drone.name} yet")

            self.world.resume()

            # The drones should come to rest on the ground. Still falling well
            # below the home altitude means one went through tiles that hadn't
            # loaded yet.
            await asyncio.sleep(5)
            for drone in self.drones:
                z = drone.get_ground_truth_kinematics()["pose"]["position"]["z"]
                if z > 20.0:
                    raise RuntimeError(
                        f"{drone.name} fell through the Cesium ground (NED z = {z:.1f} m). "
                        "The tiles probably hadn't loaded; try again (they are cached "
                        "now) or raise --tile-wait."
                    )

            for link in self.station.links:
                link.wait_for_autopilot()
            await self._clear_stored_plans()
            self.station.swarm.start()

            # Ground lookups only see Cesium's collision once the sim is running
            self.spawner = ObjectSpawner(self.world, args.objects)
            self.spawner.spawn_from_config()
            if args.ground_truth and self.ground_truth is None:
                self.ground_truth = GroundTruthRecorder(
                    self.world, args.ground_truth, rate_hz=args.gt_rate
                )
            if args.actors:
                # With --ground-truth, the actor log goes beside the labelled
                # frames, every one of them (one per scene load)
                self.actors = MovingActorManager(
                    self.world, args.actors,
                    self.ground_truth.dir if self.ground_truth
                    else os.environ.get("LOG_DIR", "/tmp/projectairsim-run"),
                    camera=("Drone1", "GimbalCam"),
                    keep_logs=None if self.ground_truth else GROUND_TRUTH_KEEP,
                )
                self.actors.spawn_from_config()
            if args.ground_truth:
                self._start_ground_truth()
            self._load_map_roads()

            if args.spotter:
                self.spotter = subprocess.Popen(
                    [sys.executable, os.path.join(SCRIPT_DIR, "spotter_control.py"),
                     "--scene-file", os.path.join(SIM_CONFIG_DIR, scene_name),
                     "--pov-shm", self.pov_frames.shm.name]
                )
        finally:
            self.loading = False

    async def _clear_stored_plans(self):
        """Start without a geofence or a mission. PX4 keeps both on disk across
        restarts, so ones from an earlier session would otherwise still be
        there: a fence refusing targets outside it, a mission a "start mission"
        would fly, neither shown on the dashboard."""
        links = self.station.links
        for what, clear in (("geofence", lambda link: link.upload_fence([])),
                            ("mission", lambda link: link.clear_mission())):
            results = await asyncio.gather(
                *(asyncio.to_thread(clear, link) for link in links), return_exceptions=True
            )
            failed = [f"{link.name} ({err})" for link, err in zip(links, results) if err]
            if failed:
                projectairsim_log().warning(f"Couldn't clear the {what} on {', '.join(failed)}")
        projectairsim_log().info("Geofence and missions cleared on every drone")
        self.station.missions.invalidate()
        self.station.safety.fence = []
        self.station.windows.set_fence([])
        self.station.fence_status = ""

    def _load_map_roads(self):
        """Centre the map on the scene's home, with its OpenStreetMap roads
        (cached after the first download; moving actors on roads share them)."""
        home = self.world.home_geo_point
        roads = []
        try:
            manager = self.actors or MovingActorManager(self.world, None)
            roads = road_segments(manager._network("vehicle"))
        except Exception as err:
            projectairsim_log().warning(f"Minimap has no roads: {err}")
        self.station.set_home(home["latitude"], home["longitude"], roads)

    def _start_ground_truth(self):
        """Label the new scene's actors and record every drone's GimbalCam."""
        self.ground_truth.new_scene(self.world)
        for payload, drone in zip(self.station.cameras, self.drones):
            self.ground_truth.attach(self.client, drone)
            payload.ground_truth = self.ground_truth
            payload.applied_zoom = None  # reapplies the FOV, the segmentation's with it
        self.refresh_ground_truth()

    def refresh_ground_truth(self):
        """Keep the recorder's segmentation IDs in step with what is spawned:
        moving actors, and static objects that are vehicles or people."""
        kinds = {name: a["kind"] for name, a in (self.actors.actors if self.actors else {}).items()}
        if self.actors and self.actors.generation != self._labelled_generation:
            # Respawned actors are new meshes under the same names: label them again
            self.ground_truth.forget(self.actors.actors)
            self._labelled_generation = self.actors.generation
        others = []
        for name, source in (self.spawner.sources if self.spawner else {}).items():
            kind = kind_of_asset(source)
            if kind:
                kinds[name] = kind
            else:
                others.append(name)
        self.ground_truth.refresh(kinds, others)

    def _stop_spotter(self):
        if self.spotter:
            self.spotter.terminate()
            self.spotter = None

    def close(self):
        self._stop_spotter()
        if self.actors:
            self.actors.close()
        if self.ground_truth:
            self.ground_truth.close()
        for autopilot in self.autopilots:
            autopilot.stop()
        if self.pov_frames:
            self.pov_frames.close()


def quiet_terminal(verbose):
    """Debug messages (e.g. camera frame rates) to the log file only, unless
    --verbose, so they don't break up commands typed in the terminal."""
    for handler in projectairsim_log().handlers:
        if type(handler) is logging.StreamHandler:
            handler.setLevel(logging.DEBUG if verbose else logging.INFO)


def drone_url(url, index):
    """Drone 1 streams at the URL given, drone N at the same path plus N."""
    return url if index == 0 or not url else f"{url}{index + 1}"


class SimFaults:
    """The safety panel's Sim tab: faults injected into the sim's drones to
    test the ground station's and the autopilots' responses. What differs
    between PX4 and ArduPilot (the battery, the link-loss failsafe) is each
    drone's SITL's (px4_sitl.py, ardupilot_sitl.py)."""

    def __init__(self, connections, links, autopilots):
        self.connections = connections
        self.links = links
        self.autopilots = autopilots  # Px4Sitl or ArduPilotSitl per drone

    def is_cut(self, i):
        return self.connections[i].outage

    def cut_link(self, i, cut):
        """Stop (or restart) all MAVLink between the ground station and drone i's
        autopilot, like a radio dropping out.

        ArduPilot notices by itself: its ground station is the payload link
        (MAV_GCS_SYSID), whose heartbeats stop, and after FS_LONG_TIMEOUT it
        runs FS_LONG_ACTN. PX4 can't notice in the sim: the simulator sends it
        GCS heartbeats of its own on its control link (mavlink_api.cpp), so
        PX4 never loses "a ground station". This stands in for PX4's link-loss
        failsafe: after COM_DL_LOSS_T it sends PX4 its NAV_DLL_ACT action."""
        if not cut:
            self.connections[i].outage = False
            projectairsim_log().warning(f"Sim: {self.links[i].name} link restored")
            return
        if self.connections[i].outage:
            return
        link = self.links[i]
        if self.autopilots[i].detects_link_loss:
            self.connections[i].outage = True
            projectairsim_log().warning(
                f"Sim: {link.name} link CUT; {link.ap.label}'s own link-loss failsafe acts "
                f"after {link.ap.link_loss_timeout_param}")
            return

        def run():
            # Read PX4's own setting while the link is still up
            action = link.get_param("NAV_DLL_ACT", timeout=1.0)
            delay = link.get_param("COM_DL_LOSS_T", timeout=1.0) or 10.0
            self.connections[i].outage = True
            projectairsim_log().warning(f"Sim: {link.name} link CUT")
            deadline = time.monotonic() + delay
            while time.monotonic() < deadline:
                if not self.connections[i].outage:
                    return  # restored in time
                time.sleep(0.2)
            command = {1: (M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 3),
                       2: (M.MAV_CMD_NAV_RETURN_TO_LAUNCH,),
                       3: (M.MAV_CMD_NAV_LAND,)}.get(int(action or 0))
            if command is None or not link.armed:
                return
            params = list(command[1:]) + [0.0] * (7 - len(command[1:]))
            link.conn.send_as(link.compid, lambda mav: mav.command_long_send(
                link.sysid, 1, command[0], 0, *params), bypass_outage=True)
            projectairsim_log().warning(
                f"Sim: {link.name} without its ground station for {delay:.0f} s: PX4's "
                f"link-loss failsafe (NAV_DLL_ACT {int(action)}) acts")

        threading.Thread(target=run, daemon=True).start()

    def set_battery(self, i, pct):
        """Bring drone i's simulated battery to pct within seconds (each SITL's
        set_battery says how it holds there)."""
        self._background(lambda: self.autopilots[i].set_battery(self.links[i], pct))
        projectairsim_log().warning(f"Sim: {self.links[i].name} battery set to {pct}%")

    def restore(self, i):
        """Undo the faults: reconnect, and the battery back to normal."""
        self.cut_link(i, False)
        self._background(lambda: self.autopilots[i].restore_battery(self.links[i]))

    def reset(self, i):
        """restore(), and restart drone i's simulated hardware in PX4 (the
        battery and power simulators), e.g. after PX4's battery simulator has
        hung and PX4 refuses to arm with "Battery unhealthy". Restarting all of
        PX4 isn't possible without reloading the scene for every drone (goto).
        ArduPilot has no such restart (restart_modules says so)."""
        self.restore(i)
        name = self.links[i].name

        def run():
            failed = self.autopilots[i].restart_modules()
            if failed:
                projectairsim_log().warning(f"Sim: {name} reset: {'; '.join(failed)}")
            else:
                projectairsim_log().info(f"Sim: {name} reset (link, battery, PX4's simulated "
                                         "hardware)")

        threading.Thread(target=run, daemon=True).start()

    @staticmethod
    def _background(action):
        # The autopilot's confirmation can't arrive while the link is cut
        threading.Thread(target=action, daemon=True).start()


def share_sim_logging():
    """The ground station logs to Project AirSim's log file and terminal too."""
    gcs_log.setLevel(logging.DEBUG)
    gcs_log.propagate = False
    for handler in projectairsim_log().handlers:
        gcs_log.addHandler(handler)


async def main():
    args = parse_args()
    quiet_terminal(args.verbose)
    share_sim_logging()
    # The config file, then the command line on top
    config = load_fleet(args.config) if os.path.exists(args.config) else Fleet(drones=[])
    args.drones, companions = sim_drones(config if config.drones else None, args.drones,
                                         args.companion)
    models = sim_models(config if config.drones else None, args.drones, args.model)
    if args.layer_step is not None:
        config.swarm.layer_step_m = args.layer_step
    projectairsim_log().info(
        f"Config {args.config if os.path.exists(args.config) else '(defaults)'}: "
        f"{args.drones} drone{'s' if args.drones != 1 else ''} ({', '.join(models)})"
        + (f", companions {', '.join(f'Drone{i + 1} {ip}' for i, ip in companions.items())}"
           if companions else "")
        + f", height layers {config.swarm.layer_step_m:g} m")
    if args.scene and args.drones != 1:
        raise SystemExit("--drones needs a generated Cesium scene; it can't be used with --scene")
    if args.scene and args.ground_truth:
        raise SystemExit("--ground-truth needs a generated Cesium scene; it can't be used with --scene")
    camera = CameraProperties(args.camera_props if os.path.exists(args.camera_props) else None)
    projectairsim_log().info(f"GimbalCam {camera.describe()}")

    client = ProjectAirSimClient()
    rtsps, connections = [], []
    station = None
    session = None
    tasks = []

    try:
        client.connect()
        # Each drone's PX4 on its own payload link, with the sim's MAVLink camera
        advertised_url = args.rtsp_advertise_url or args.rtsp_url
        links, sim_cameras = [], []
        for i in range(args.drones):
            rtsp = None if args.no_rtsp else RtspStreamer(drone_url(args.rtsp_url, i),
                                                          camera.resolution,
                                                          bitrate_kbps=args.stream_bitrate)
            rtsps.append(rtsp)
            conn = payload_connection(i)
            connections.append(conn)
            links.append(VehicleLink(conn, i + 1, f"Drone{i + 1}", airframe_of(models[i])))
            sim_cameras.append(SimCamera(conn, i, drone_url(advertised_url, i) if rtsp else "",
                                         camera))

        session = Session(args, client, camera, models, companions)
        for i, ip in sorted(companions.items()):
            profile = session.aircraft[i].companion_profile
            projectairsim_log().info(f"Drone{i + 1} ({models[i]}): its companion {ip} should run "
                                     f"X's {profile or 'sim'} profile")
        station = GroundStation(
            links,
            lambda i, link, windows: SimPayload(link, windows, sim_cameras[i], rtsps[i], camera),
            companions=companions,

            extra_commands=lambda line, parts: sim_command(session, line, parts),
            extra_help=SIM_HELP,
            extra_objects=lambda: sim_objects(session),
            extra_status=lambda: sim_status(session),
            config=config,
            sim_faults=SimFaults(connections, links, session.autopilots),
            before_mission=lambda: reset_actors_for_mission(session),
            on_mission_track=lambda i, spec, run_id: mission_track(session, i, spec, run_id),
            on_mission_run=lambda event, mission, run_id, indices: record_mission(
                session, event, mission, run_id, indices),
        )
        session.station = station
        projectairsim_log().addHandler(station.dashboard_log)
        station.start_ui()

        await session.load()
        station.start()
        tasks.append(asyncio.create_task(run_ground_truth(session)))

        if args.qgc:
            projectairsim_log().info(
                f"World ready. Connect QGroundControl (UDP, listen 14550, "
                f"{qgc_servers(session.autopilots)}) to fly"
                + (", or type commands here (e.g. 'all takeoff', 'help')" if args.drones > 1 else "")
                + ". Type 'goto LAT LON [HEADING]' to move. Ctrl+C here or close the sim to stop."
            )
            await wait_while_sim_alive(session)
        elif args.drones == 1 and not session.aircraft[0].capabilities.hovers:
            projectairsim_log().info("The scripted demo is a quadrotor's; fly the "
                                     f"{session.aircraft[0].name} from QGC or this terminal")
            await wait_while_sim_alive(session)
        elif args.drones == 1:
            await fly_gimbal_demo(session.drones[0], links[0], sweep=0 not in companions)
        else:
            home = session.world.home_geo_point
            heading = DEFAULT_HEADING if args.heading is None else args.heading
            await search_demo(station.swarm, home["latitude"], home["longitude"], heading)

    finally:
        for task in tasks:
            task.cancel()
        if session:
            session.close()
        client.disconnect()
        if station:
            projectairsim_log().removeHandler(station.dashboard_log)
            station.close()
        for conn in connections:
            conn.close()
        for rtsp in rtsps:
            if rtsp:
                rtsp.close()


if __name__ == "__main__":
    # `kill -USR1 <pid>` prints every thread's stack, for diagnosing hangs
    faulthandler.register(signal.SIGUSR1, all_threads=True)
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    os._exit(0)  # don't wait on the console thread blocked reading stdin
