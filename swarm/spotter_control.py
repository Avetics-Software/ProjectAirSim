"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Control windows for a walking human "spotter" in a running Cesium scene.

Spotter window: pick a point from config_spotter_points.yml and Spawn, Teleport
or Walk to it, or walk the spotter by hand: click the window, then hold W/S to
walk forward/back and A/D to turn.

Spotter POV window (when the scene has the PovCam camera robot, i.e. it was
started with px4_gimbal_cesium.py --spotter): the spotter's own eyes. Click it
to capture the mouse and look around, W/A/S/D to walk and strafe, Shift to run,
Esc to release the mouse.

The spotter follows the ground and plays walk and idle animations (UE's template
Mannequin, installed by install_spotter_assets.sh).

Started by px4_gimbal_cesium.py --spotter, or by hand while a scene is running:
    python spotter_control.py --scene-file sim_config/scene_px4_gimbal_cesium_custom.jsonc
"""

import argparse
import math
import os
import time
import tkinter as tk
from tkinter import ttk

import commentjson
import numpy as np
import yaml
from PIL import Image, ImageTk

from shared_frame import SharedFrameReader

from projectairsim import ProjectAirSimClient, World
from projectairsim.types import Pose, Quaternion, Vector3
from projectairsim.utils import geo_to_ned_coordinates, rpy_to_quaternion

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

SPOTTER_NAME = "Spotter"
MESH = "SKM_Manny_Simple"
WALK_ANIMS = {
    "forward": "MF_Unarmed_Walk_Fwd",
    "back": "MF_Unarmed_Walk_Bwd",
    "left": "MF_Unarmed_Walk_Left",
    "right": "MF_Unarmed_Walk_Right",
}
IDLE_ANIM = "MM_Idle"
WALK_ANIM_SPEED = 1.6  # m/s the walk animation is authored for
MESH_YAW_OFFSET_DEG = -90.0  # the Mannequin mesh faces +Y; rotate it to face +X
TICK_MS = 50
TURN_RATE_DPS = 90.0
ARRIVE_M = 0.3
GROUND_PROBE_M = 1.0  # sample the ground this far ahead/behind, clear of the body
RUN_FACTOR = 2.2  # speed multiplier while Shift is held
POV_ROBOT = "PovCam"  # camera robot added to the scene by --spotter
EYE_HEIGHT_M = 1.62
EYE_FORWARD_M = 0.18  # just in front of the face, so the head doesn't block the view
LOOK_DEG_PER_PIXEL = 0.15
KEY_RELEASE_DEBOUNCE_MS = 40  # X11 auto-repeat sends release/press pairs while held
EARTH_M_PER_DEG = 111_320.0


def attach_world(client, scene_file):
    """World handle for an already-loaded scene, without reloading it."""
    with open(scene_file) as f:
        scene = commentjson.load(f)
    world = World.__new__(World)
    world.client = client
    world.parent_topic = f"/Sim/{scene['id']}"
    world.home_geo_point = scene["home-geo-point"]
    names = [a["name"] for a in scene["actors"] if a.get("type") == "robot"]
    robots = f"/Sim/{scene['id']}/robots"
    pov_topic = f"{robots}/{POV_ROBOT}" if POV_ROBOT in names else None
    return world, f"{robots}/{names[0]}", pov_topic


class Spotter:
    """A Mannequin on the ground, moved in NED metres around the scene origin.

    If `pov_topic` is given, the spotter's first-person camera robot is kept at
    its eyes, looking where the spotter looks.
    """

    def __init__(self, world, pov_topic=None):
        self.world = world
        self.pov_topic = pov_topic
        self.home = world.home_geo_point
        self.north = self.east = 0.0
        self.down = 0.0
        self.heading = 0.0  # deg from north; also where the POV looks
        self.look_pitch = 0.0  # deg, + up
        self.speed = 1.4  # m/s
        self.running = False
        self.target = None  # (north, east) while walking to a point
        self.manual = {"forward": 0, "strafe": 0, "turn": 0}
        self.anim = None
        self.spawned = False
        self.name = SPOTTER_NAME  # the sim may make it unique, e.g. Spotter1
        self._look_changed = False

    @property
    def walking(self):
        return self.anim not in (None, IDLE_ANIM)

    # Geodesy -----------------------------------------------------------
    def to_ned(self, lat, lon):
        north, east, _ = geo_to_ned_coordinates(
            self.home, [lat, lon, self.home["altitude"]]
        )
        return north, east

    def to_geo(self):
        lat = self.home["latitude"] + self.north / EARTH_M_PER_DEG
        lon = self.home["longitude"] + self.east / (
            EARTH_M_PER_DEG * math.cos(math.radians(self.home["latitude"]))
        )
        return lat, lon

    # Sim ---------------------------------------------------------------
    def ground_down(self, north, east):
        """Ground height (NED down) under a point, probing clear of the body."""
        h = math.radians(self.heading)
        samples = []
        for sign in (1, -1):
            z = self.world.get_surface_elevation_at_point(
                north + sign * GROUND_PROBE_M * math.cos(h),
                east + sign * GROUND_PROBE_M * math.sin(h),
            )
            if z is not None and not math.isnan(z):
                samples.append(z)
        return sum(samples) / len(samples) if samples else self.down

    @staticmethod
    def _pose(north, east, down, roll_deg, pitch_deg, yaw_deg):
        w, x, y, z = rpy_to_quaternion(
            math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
        )
        return Pose(
            {
                "translation": Vector3({"x": north, "y": east, "z": down}),
                "rotation": Quaternion({"w": w, "x": x, "y": y, "z": z}),
                "frame_id": "DEFAULT_ID",
            }
        )

    def _push_pose(self):
        try:
            self.world.set_object_pose(
                self.name,
                self._pose(self.north, self.east, self.down, 0.0, 0.0,
                           self.heading + MESH_YAW_OFFSET_DEG),
                True,
            )
        except RuntimeError as err:
            # The sim reports setting an unchanged pose as a failure; ignore that
            if "Unable to move" not in str(err):
                raise
        if self.pov_topic:
            h = math.radians(self.heading)
            eye = self._pose(
                self.north + EYE_FORWARD_M * math.cos(h),
                self.east + EYE_FORWARD_M * math.sin(h),
                self.down - EYE_HEIGHT_M,
                0.0, self.look_pitch, self.heading,
            )
            self.world.client.request(
                {"method": f"{self.pov_topic}/SetPose",
                 "params": {"pose": eye, "reset_kinematics": True}, "version": 1.0}
            )

    def spawn(self, north, east):
        if self.spawned:
            self.world.destroy_object(self.name)
        self.north, self.east = north, east
        self.down = self.ground_down(north, east)
        self.name = self.world.spawn_object(
            SPOTTER_NAME, MESH,
            self._pose(north, east, self.down, 0.0, 0.0, self.heading + MESH_YAW_OFFSET_DEG),
            [1.0, 1.0, 1.0], False,
        )
        self.spawned = True
        self.anim = None
        self._set_anim(IDLE_ANIM)
        self._push_pose()

    def remove(self):
        if self.spawned:
            self.world.destroy_object(self.name)
        self.spawned = False
        self.target = None

    def teleport(self, north, east):
        self.target = None
        self.north, self.east = north, east
        self.down = self.ground_down(north, east)
        self._push_pose()

    def look(self, d_yaw_deg, d_pitch_deg):
        """Turn the head/body (mouse look)."""
        self.heading = (self.heading + d_yaw_deg) % 360
        self.look_pitch = max(-80.0, min(80.0, self.look_pitch + d_pitch_deg))
        self._look_changed = True

    def _set_anim(self, anim, speed=None):
        if anim == self.anim:
            return
        self.anim = anim
        rate = 1.0 if anim == IDLE_ANIM else (speed or self.speed) / WALK_ANIM_SPEED
        self.world.set_object_animation(self.name, anim, True, rate)

    def step(self, dt):
        """Advance walking (to a target or under manual control) by dt seconds."""
        if not self.spawned:
            return
        speed = self.speed * (RUN_FACTOR if self.running else 1.0)
        move_dir = None  # bearing to move along, deg from north
        anim = IDLE_ANIM
        if self.manual["turn"]:
            self.heading = (self.heading + self.manual["turn"] * TURN_RATE_DPS * dt) % 360
            self._look_changed = True
        fwd, strafe = self.manual["forward"], self.manual["strafe"]
        if fwd or strafe:
            self.target = None
            move_dir = (self.heading + math.degrees(math.atan2(strafe, fwd))) % 360
            if fwd > 0:
                anim = WALK_ANIMS["forward"]
            elif fwd < 0:
                anim = WALK_ANIMS["back"]
            else:
                anim = WALK_ANIMS["right" if strafe > 0 else "left"]
            distance = speed * dt
        elif self.target:
            dn, de = self.target[0] - self.north, self.target[1] - self.east
            dist = math.hypot(dn, de)
            if dist < ARRIVE_M:
                self.target = None
            else:
                self.heading = math.degrees(math.atan2(de, dn)) % 360
                move_dir, anim = self.heading, WALK_ANIMS["forward"]
                distance = min(speed * dt, dist)

        moved = move_dir is not None
        if moved:
            b = math.radians(move_dir)
            self.north += distance * math.cos(b)
            self.east += distance * math.sin(b)
            self.down = self.ground_down(self.north, self.east)
        if moved or self._look_changed:
            self._push_pose()
            self._look_changed = False
        self._set_anim(anim, speed)


class KeyState:
    """Held-key tracking that ignores X11 auto-repeat's release/press pairs."""

    def __init__(self, widget, bindings, on_change):
        self.widget = widget
        self.bindings = bindings  # key -> (field, value)
        self.on_change = on_change  # called with (field, value)
        self._pending = {}
        for key in bindings:
            widget.bind(f"<KeyPress-{key}>", lambda e, k=key: self._press(k))
            widget.bind(f"<KeyRelease-{key}>", lambda e, k=key: self._release(k))

    def _press(self, key):
        pending = self._pending.pop(key, None)
        if pending:
            self.widget.after_cancel(pending)
        field, value = self.bindings[key]
        self.on_change(field, value)

    def _release(self, key):
        def released():
            self._pending.pop(key, None)
            self.on_change(self.bindings[key][0], 0)

        self._pending[key] = self.widget.after(KEY_RELEASE_DEBOUNCE_MS, released)


class PovWindow:
    """The spotter's first-person view, with mouse look and WASD walking."""

    def __init__(self, root, spotter, frames, size=(960, 540)):
        self.spotter = spotter
        self.size = size
        self.top = tk.Toplevel(root)
        self.top.title("Spotter POV")
        self.canvas = tk.Canvas(self.top, width=size[0], height=size[1],
                                bg="black", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.image_item = self.canvas.create_image(0, 0, anchor="nw")
        self.hint = self.canvas.create_text(
            size[0] // 2, size[1] - 18, fill="white",
            text="Click to look around · W/A/S/D walk · Shift run · Esc release mouse",
        )
        self.photo = None
        self.frames = frames  # SharedFrameReader
        self.captured = False
        KeyState(self.top, {
            "w": ("forward", 1), "s": ("forward", -1),
            "a": ("strafe", -1), "d": ("strafe", 1),
            "W": ("forward", 1), "S": ("forward", -1),
            "A": ("strafe", -1), "D": ("strafe", 1),
        }, self._key)
        self.top.bind("<KeyPress-Shift_L>", lambda e: setattr(spotter, "running", True))
        self.top.bind("<KeyRelease-Shift_L>", lambda e: setattr(spotter, "running", False))
        self.canvas.bind("<Button-1>", lambda e: self._capture(True))
        self.top.bind("<Escape>", lambda e: self._capture(False))
        self.top.bind("<FocusOut>", lambda e: self._capture(False))
        self.canvas.bind("<Motion>", self._on_motion)
        self.top.after(33, self._draw)

    def _key(self, field, value):
        self.spotter.manual[field] = value

    def _centre(self):
        return self.canvas.winfo_width() // 2, self.canvas.winfo_height() // 2

    def _capture(self, on):
        self.captured = on
        self.canvas.config(cursor="none" if on else "")
        self.canvas.itemconfigure(self.hint, state="hidden" if on else "normal")
        if on:
            self.top.focus_set()
            cx, cy = self._centre()
            self.canvas.event_generate("<Motion>", warp=True, x=cx, y=cy)

    def _on_motion(self, event):
        if not self.captured:
            return
        cx, cy = self._centre()
        dx, dy = event.x - cx, event.y - cy
        if dx or dy:
            self.spotter.look(dx * LOOK_DEG_PER_PIXEL, -dy * LOOK_DEG_PER_PIXEL)
            # Keep the pointer in the middle so you can keep turning
            self.canvas.event_generate("<Motion>", warp=True, x=cx, y=cy)

    def _draw(self):
        frame = self.frames.read_new()
        if frame is not None:
            w, h = max(self.canvas.winfo_width(), 1), max(self.canvas.winfo_height(), 1)
            img = Image.fromarray(np.ascontiguousarray(frame[:, :, ::-1]))  # BGR -> RGB
            if img.size != (w, h):
                img = img.resize((w, h))
            self.photo = ImageTk.PhotoImage(img)
            self.canvas.itemconfigure(self.image_item, image=self.photo)
        self.top.after(33, self._draw)


class SpotterWindow:
    def __init__(self, root, spotter, points_file, drone_topic):
        self.root = root
        self.spotter = spotter
        self.points_file = points_file
        self.drone_topic = drone_topic
        self.points = self._load_points()

        root.title("Spotter")
        frame = ttk.Frame(root, padding=10)
        frame.grid(sticky="nsew")

        ttk.Label(frame, text="Points").grid(row=0, column=0, sticky="w")
        self.listbox = tk.Listbox(frame, height=10, width=34, exportselection=False)
        self.listbox.grid(row=1, column=0, rowspan=7, sticky="nsew")
        for p in self.points:
            self.listbox.insert(tk.END, p["name"])
        if self.points:
            self.listbox.selection_set(0)

        buttons = [
            ("Spawn here", self.on_spawn),
            ("Teleport", self.on_teleport),
            ("Walk to", self.on_walk),
            ("Stop", self.on_stop),
            ("Remove spotter", self.on_remove),
            ("Save position as point", self.on_save_point),
        ]
        for i, (label, cmd) in enumerate(buttons):
            ttk.Button(frame, text=label, command=cmd, width=24).grid(
                row=1 + i, column=1, padx=(10, 0), pady=2, sticky="ew"
            )

        ttk.Label(frame, text="Speed (m/s)").grid(row=8, column=0, sticky="w", pady=(10, 0))
        self.speed = tk.DoubleVar(value=spotter.speed)
        ttk.Scale(frame, from_=0.5, to=4.0, variable=self.speed,
                  command=self.on_speed).grid(row=9, column=0, columnspan=2, sticky="ew")

        self.status = tk.StringVar(value="Pick a point and press Spawn here.")
        ttk.Label(frame, textvariable=self.status, justify="left").grid(
            row=10, column=0, columnspan=2, sticky="w", pady=(10, 0)
        )
        ttk.Label(
            frame,
            text="Click this window, then hold W/S to walk and A/D to turn.",
            foreground="gray",
        ).grid(row=11, column=0, columnspan=2, sticky="w")

        KeyState(root, {
            "w": ("forward", 1), "s": ("forward", -1), "a": ("turn", -1), "d": ("turn", 1),
        }, self._key)

        self.last_tick = time.monotonic()
        self.root.after(TICK_MS, self.tick)

    def _load_points(self):
        if not os.path.exists(self.points_file):
            return []
        with open(self.points_file) as f:
            return (yaml.safe_load(f) or {}).get("points") or []

    def _selected(self):
        sel = self.listbox.curselection()
        if not sel:
            self.status.set("Select a point first.")
            return None
        p = self.points[sel[0]]
        return self.spotter.to_ned(float(p["lat"]), float(p["lon"]))

    def _run(self, action):
        try:
            action()
        except Exception as err:
            self.status.set(f"Error: {err}")

    def _key(self, field, value):
        self.spotter.manual[field] = value

    def on_spawn(self):
        target = self._selected()
        if target:
            self._run(lambda: self.spotter.spawn(*target))

    def on_teleport(self):
        target = self._selected()
        if target and self.spotter.spawned:
            self._run(lambda: self.spotter.teleport(*target))

    def on_walk(self):
        target = self._selected()
        if target and self.spotter.spawned:
            self.spotter.target = target

    def on_stop(self):
        self.spotter.target = None

    def on_remove(self):
        self._run(self.spotter.remove)

    def on_speed(self, _):
        self.spotter.speed = self.speed.get()
        if self.spotter.walking:
            self.spotter.anim = None  # restart the walk at the new play rate

    def on_save_point(self):
        if not self.spotter.spawned:
            return
        lat, lon = self.spotter.to_geo()
        name = f"Point {len(self.points) + 1}"
        self.points.append({"name": name, "lat": round(lat, 7), "lon": round(lon, 7)})
        self.listbox.insert(tk.END, name)
        with open(self.points_file, "w") as f:
            yaml.safe_dump({"points": self.points}, f, sort_keys=False)
        self.status.set(f"Saved {name} to {os.path.basename(self.points_file)}")

    def _drone_distance(self):
        try:
            k = self.spotter.world.client.request(
                {"method": f"{self.drone_topic}/GetGroundTruthKinematics",
                 "params": {}, "version": 1.0}
            )
            p = k["pose"]["position"]
            return math.dist(
                (p["x"], p["y"], p["z"]),
                (self.spotter.north, self.spotter.east, self.spotter.down - 1.7),
            )
        except Exception:
            return None

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now
        try:
            self.spotter.step(dt)
            if self.spotter.spawned:
                lat, lon = self.spotter.to_geo()
                state = "walking" if self.spotter.walking else "standing"
                dist = self._drone_distance()
                drone = f"   drone {dist:.0f} m away" if dist is not None else ""
                self.status.set(
                    f"{state} at {lat:.6f}, {lon:.6f}, heading {self.spotter.heading:.0f}{drone}"
                )
        except Exception as err:
            self.status.set(f"Error: {err}")
        self.root.after(TICK_MS, self.tick)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--scene-file",
        default=os.path.join(SCRIPT_DIR, "sim_config", "scene_px4_gimbal_cesium_custom.jsonc"),
        help="scene config of the running scene (for its id and home point)",
    )
    parser.add_argument(
        "--pov-shm",
        help="shared-memory block with the POV camera frames (set by px4_gimbal_cesium.py)",
    )
    parser.add_argument(
        "--points",
        default=os.path.join(SCRIPT_DIR, "config_spotter_points.yml"),
        help="YAML file of named points",
    )
    args = parser.parse_args()

    client = ProjectAirSimClient()
    # Requests only: the sim's topic channel takes a single client, and that is
    # px4_gimbal_cesium.py (it passes the POV frames on through shared memory)
    client.connect_services()
    world, drone_topic, pov_topic = attach_world(client, args.scene_file)
    spotter = Spotter(world, pov_topic)

    root = tk.Tk()
    SpotterWindow(root, spotter, args.points, drone_topic)
    if pov_topic and args.pov_shm:
        PovWindow(root, spotter, SharedFrameReader(args.pov_shm))
    try:
        root.mainloop()
    finally:
        try:
            spotter.remove()
        except Exception:
            pass
        client.disconnect()


if __name__ == "__main__":
    main()
