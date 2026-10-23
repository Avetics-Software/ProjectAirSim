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
import faulthandler
import json
import logging
import math
import os
import signal
import subprocess
import sys
import time
import urllib.request

import commentjson
import cv2

from projectairsim import ProjectAirSimClient, Drone, World
from projectairsim.types import Pose, Quaternion, Vector3
from projectairsim.utils import load_scene_config_as_dict, projectairsim_log, quaternion_to_rpy

from camera_properties import CameraProperties
from gcs.fleet import load_fleet
from gcs.log import log as gcs_log
from gcs.mavlink_link import VehicleLink
from gcs.minimap import road_segments
from gcs.station import DroneCamera, GroundStation
from gcs.swarm import search_demo
from payload_mavlink import SimCamera, payload_connection
from payload_video import RtspStreamer, image_to_bgr
from px4_sitl import Px4Sitl
from shared_frame import SharedFrameWriter
from moving_actors import ACTORS_HELP, MovingActorManager
from spawned_objects import CONSOLE_HELP, ObjectSpawner

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_CONFIG_DIR = os.path.join(SCRIPT_DIR, "sim_config")
TEMPLATE_SCENE = "scene_px4_gimbal_cesium_seletar.jsonc"
GENERATED_SCENE = "scene_px4_gimbal_cesium_custom.jsonc"
TEMPLATE_ROBOT = "robot_quadrotor_px4_gimbal_payload.jsonc"
GENERATED_ROBOT = "robot_quadrotor_px4_gimbal_payload_custom{}.jsonc"  # drone suffix

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
        default=1,
        help="number of PX4 drones (a swarm); each gets its own PX4 instance",
    )
    parser.add_argument(
        "--drone-spacing",
        type=float,
        default=8.0,
        help="metres between the drones at the spawn, in a line to the right (default 8)",
    )
    parser.add_argument(
        "--companion",
        default=os.environ.get("PI_IP", ""),
        help="IP of a companion computer running the X vision pipeline on Drone1 "
        "(default: run_cesium_sim.sh's --pi-ip). The dashboard shows its tracker "
        "video and sends it click-to-track and gimbal commands.",
    )
    parser.add_argument(
        "--fleet",
        help="fleet file (gcs/fleet.example.yml): the number of drones and each one's "
        "companion computer. Replaces --drones and --companion.",
    )
    parser.add_argument(
        "--layer-step",
        type=float,
        default=5.0,
        help="drone N flies (N-1) x this many metres above the height asked for, so "
        "drones whose paths cross pass at different heights (default 5; 0 = off)",
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


def build_robot(camera, index=0):
    """Write the payload robot config for drone `index` (0-based) and return its
    name. Each drone talks to its own PX4 instance, whose ports are offset by
    the index."""
    with open(os.path.join(SIM_CONFIG_DIR, TEMPLATE_ROBOT)) as f:
        robot = commentjson.load(f)
    sensors = {s["id"]: s for s in robot["sensors"]}
    camera.apply_to_camera_config(sensors["GimbalCam"])
    px4 = robot["controller"]["px4-settings"]
    px4["tcp-port"] += index
    px4["control-port"] += index
    px4["gimbal-port"] += index
    if index > 0:
        # Save GPU for the swarm: a smaller chase view, no down camera
        for setting in sensors["Chase"].get("capture-settings", []):
            setting["width"], setting["height"] = 640, 360
        if "DownCamera" in sensors:
            sensors["DownCamera"]["enabled"] = False
    name = GENERATED_ROBOT.format("" if index == 0 else f"_{index + 1}")
    with open(os.path.join(SIM_CONFIG_DIR, name), "w") as f:
        f.write("// Generated by px4_gimbal_cesium.py; overwritten on every run.\n")
        json.dump(robot, f, indent=2)
    return name


def build_scene(args, robot_configs):
    """Write a scene config for the requested spawn point and return its name."""
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

    # Drone1 at the spawn point, the others in a line to its right
    template = scene["actors"][0]
    drones = []
    h = math.radians(heading)
    for i, robot_config in enumerate(robot_configs):
        right = i * args.drone_spacing
        north, east = -right * math.sin(h), right * math.cos(h)
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

    def tick(self, dt):
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


GOTO_HELP = """  goto LAT LON [HEADING]                move the drone(s) to a new place (restarts PX4)"""
SIM_HELP = CONSOLE_HELP + "\n" + ACTORS_HELP + "\n" + GOTO_HELP


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
    elif session.spawner:
        session.spawner.run_command(line)
    return True


async def run_ground_truth(session):
    """Log where the moving actors are, for scoring detection and ReID."""
    while True:
        if not session.loading and session.actors:
            try:
                session.actors.log_ground_truth()
            except Exception as err:  # e.g. the scene is reloading under us
                projectairsim_log().debug(f"Ground-truth log skipped: {err}")
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
    ports = ", ".join(f"{18570 + i}" for i in range(len(session.px4s)))
    return [
        "Objects: " + (", ".join(f"{n} {k}" for k, n in sorted(counts.items())) or "none"),
        f"QGC: UDP, listen 14550, servers 127.0.0.1: {ports}",
    ]


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


async def fly_gimbal_demo(drone, link):
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

    projectairsim_log().info("Sweeping gimbal for 60 s")
    t0 = time.monotonic()
    while (t := time.monotonic() - t0) < 60.0:
        pitch, yaw = gimbal_setpoint(t)
        link.set_gimbal_attitude(pitch, yaw)
        await asyncio.sleep(0.05)  # 20 Hz setpoint stream

    link.set_gimbal_attitude(-90.0, 0.0)

    projectairsim_log().info("Landing")
    await (await drone.land_async())
    drone.disarm()
    drone.disable_api_control()


class Session:
    """The running world: loads a location, (re)starts PX4 for it and keeps the
    ground station's cameras and map, the objects and the spotter following
    the current scene."""

    def __init__(self, args, client, camera, drones, companions):
        self.args = args
        self.client = client
        self.camera = camera
        self.station = None  # GroundStation, set once it is made
        # Each PX4 also opens MAVLink links to its drone's companion computer
        self.px4s = [Px4Sitl(i, pi_ip=companions.get(i, "")) for i in range(drones)]
        self.robot_configs = (
            None if args.scene else [build_robot(camera, i) for i in range(drones)]
        )
        self.object_counts = {}
        self.world = None
        self.drones = []
        self.spawner = None
        self.actors = None
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
        scene_name = args.scene or build_scene(args, self.robot_configs)
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
            for px4 in self.px4s:
                px4.stop()
            for px4 in self.px4s:
                px4.start()

            self.drones = [
                Drone(self.client, self.world, f"Drone{i + 1}") for i in range(len(self.px4s))
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
                    for drone in self.drones:
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
                link.wait_for_px4()
            self.station.swarm.start()

            # Ground lookups only see Cesium's collision once the sim is running
            self.spawner = ObjectSpawner(self.world, args.objects)
            self.spawner.spawn_from_config()
            if args.actors:
                self.actors = MovingActorManager(
                    self.world, args.actors, os.environ.get("LOG_DIR", "/tmp/projectairsim-run"),
                    camera=("Drone1", "GimbalCam"),
                )
                self.actors.spawn_from_config()
            self._load_map_roads()

            if args.spotter:
                self.spotter = subprocess.Popen(
                    [sys.executable, os.path.join(SCRIPT_DIR, "spotter_control.py"),
                     "--scene-file", os.path.join(SIM_CONFIG_DIR, scene_name),
                     "--pov-shm", self.pov_frames.shm.name]
                )
        finally:
            self.loading = False

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

    def _stop_spotter(self):
        if self.spotter:
            self.spotter.terminate()
            self.spotter = None

    def close(self):
        self._stop_spotter()
        if self.actors:
            self.actors.close()
        for px4 in self.px4s:
            px4.stop()
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
    if args.fleet:
        fleet = load_fleet(args.fleet)
        args.drones = len(fleet.drones)
        companions = fleet.companions
    else:
        companions = {0: args.companion} if args.companion else {}
    if args.scene and args.drones != 1:
        raise SystemExit("--drones needs a generated Cesium scene; it can't be used with --scene")
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
                                                          camera.resolution)
            rtsps.append(rtsp)
            conn = payload_connection(i)
            connections.append(conn)
            links.append(VehicleLink(conn, i + 1, f"Drone{i + 1}"))
            sim_cameras.append(SimCamera(conn, i, drone_url(advertised_url, i) if rtsp else "",
                                         camera))

        session = Session(args, client, camera, args.drones, companions)
        station = GroundStation(
            links,
            lambda i, link, windows: SimPayload(link, windows, sim_cameras[i], rtsps[i], camera),
            companions=companions,
            layer_step=args.layer_step,
            extra_commands=lambda line, parts: sim_command(session, line, parts),
            extra_help=SIM_HELP,
            extra_objects=lambda: sim_objects(session),
            extra_status=lambda: sim_status(session),
        )
        session.station = station
        projectairsim_log().addHandler(station.dashboard_log)
        station.start_ui()

        await session.load()
        station.start()
        tasks.append(asyncio.create_task(run_ground_truth(session)))

        if args.qgc:
            servers = ", ".join(f"127.0.0.1:{18570 + i}" for i in range(args.drones))
            projectairsim_log().info(
                f"World ready. Connect QGroundControl (UDP, listen 14550, server{'s' if args.drones > 1 else ''} "
                f"{servers}) to fly"
                + (", or type commands here (e.g. 'all takeoff', 'help')" if args.drones > 1 else "")
                + ". Type 'goto LAT LON [HEADING]' to move. Ctrl+C here or close the sim to stop."
            )
            await wait_while_sim_alive(session)
        elif args.drones == 1:
            await fly_gimbal_demo(session.drones[0], links[0])
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
