"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Ground-truth capture: the drones' GimbalCam frames with a box around every
moving actor in view, for training a detector (YOLO labels) and for scoring
tracking and re-identification (each box carries the actor's name, the same
identity across frames and across drones).

How the boxes are made: every moving actor gets a segmentation ID of its own
and everything else ID 0, the GimbalCam renders a segmentation image beside
its scene image, and each actor's box is the extent of its pixels there. So
the boxes are tight to what is actually drawn: a car half behind a building
gets a box around its visible half, and one fully hidden gets none. (The
Cesium tiles join the segmentation as ID 0 when the scene initializes
segmentation IDs; see UnrealSimLoader.cpp.)

Output, under OUT_DIR/<YYYYmmdd_HHMMSS>/:
  images/<drone>_s<scene>_<sim ns>.jpg
                                the GimbalCam frame; <scene> counts `goto`
                                reloads, in case sim time restarts
  labels/<same>.txt             YOLO: class cx cy w h, normalised 0..1
  classes.txt                   the YOLO class names, one per line
  boxes.csv                     every box: sim time, drone, frame, actor
                                (identity), class, x1 y1 x2 y2 in pixels,
                                visible pixels, whether cut by the frame edge
  frames.csv                    every frame: sim time, drone, image, camera
                                world pose (NED) and horizontal FOV
  ground_truth_<time>.csv       where every moving actor was, one per scene
                                load (moving_actors.py writes it here)
  mission_runs.csv              when each ground-station mission run started
                                and ended (sim time), and track steps, so
                                frames can be matched to a run

gt_playback.py plays a capture back with its boxes and a map.

Images and labels share a stem, the layout Ultralytics expects.
"""

import csv
import os
import queue
import re
import threading
import time

import cv2
import numpy as np

from projectairsim.image_utils import SEGMENTATION_PALLETE
from projectairsim.types import ImageType
from projectairsim.utils import projectairsim_log

# YOLO class of each actor kind; the order is the class index
CLASSES = ["person", "car"]
CLASS_OF_KIND = {"person": "person", "vehicle": "car"}
# The kind of a spawned static object, from its asset or model file name;
# anything else is background
KIND_OF_ASSET = [
    ("vehicle", re.compile(r"car|vehicle|truck|van|bus", re.I)),
    ("person", re.compile(r"manny|quinn|mannequin|person|human", re.I)),
]
MAX_ACTORS = 255  # segmentation IDs 1..255; 0 is the background

MIN_PIXELS = 12  # fewer visible pixels than this: no box
MIN_SIDE_PX = 3  # nor for a box thinner than this
JPEG_QUALITY = 95
PENDING_FRAMES = 8  # unpaired scene/segmentation frames kept per drone


def _packed(bgr):
    """One int per pixel, 0xRRGGBB, so colours compare as numbers."""
    bgr = bgr.astype(np.int32)
    return (bgr[..., 2] << 16) | (bgr[..., 1] << 8) | bgr[..., 0]


# The segmentation image's colour of each ID (the palette is RGB, pixels BGR)
_ID_OF_COLOR = {(r << 16) | (g << 8) | b: i for i, (r, g, b) in enumerate(SEGMENTATION_PALLETE)}
_BACKGROUND = (SEGMENTATION_PALLETE[0][0] << 16) | (SEGMENTATION_PALLETE[0][1] << 8) | SEGMENTATION_PALLETE[0][2]


def kind_of_asset(source):
    """'vehicle', 'person' or None for an asset or model file name."""
    return next((kind for kind, pattern in KIND_OF_ASSET if pattern.search(source or "")), None)


def add_segmentation_capture(camera):
    """Make a robot config camera (the GimbalCam) also render a segmentation
    image matching its scene image: same size and field of view."""
    settings = camera["capture-settings"]
    scene = next(s for s in settings if s.get("image-type", 0) == ImageType.SCENE)
    settings[:] = [s for s in settings if s.get("image-type") != ImageType.SEGMENTATION]
    settings.append(
        {
            "image-type": int(ImageType.SEGMENTATION),
            "width": scene["width"],
            "height": scene["height"],
            "fov-degrees": scene["fov-degrees"],
            "capture-enabled": True,
            "streaming-enabled": False,
            "pixels-as-float": False,
            "compress": False,
        }
    )


def boxes_from_segmentation(seg_bgr, names_by_id, min_pixels=MIN_PIXELS, min_side=MIN_SIDE_PX):
    """[(name, x1, y1, x2, y2, pixels)] for each actor visible in a
    segmentation image; x2/y2 are exclusive pixel bounds."""
    packed = _packed(seg_bgr)
    ys, xs = np.nonzero(packed != _BACKGROUND)
    if len(xs) == 0:
        return []
    colors = packed[ys, xs]
    boxes = []
    for color in np.unique(colors):
        name = names_by_id.get(_ID_OF_COLOR.get(int(color)))
        if name is None:  # an ID we didn't hand out, e.g. an object spawned since
            continue
        sel = colors == color
        count = int(sel.sum())
        x1, x2 = int(xs[sel].min()), int(xs[sel].max()) + 1
        y1, y2 = int(ys[sel].min()), int(ys[sel].max()) + 1
        if count < min_pixels or min(x2 - x1, y2 - y1) < min_side:
            continue
        boxes.append((name, x1, y1, x2, y2, count))
    return boxes


class GroundTruthRecorder:
    """Writes labelled GimbalCam frames from every drone. Subscribe it to the
    drones with `attach()`; keep its segmentation IDs current with `refresh()`
    from the asyncio thread, which is the one that talks to the sim."""

    def __init__(self, world, out_dir, rate_hz=2.0, keep_empty=0.1):
        """rate_hz: frames saved per second per drone. keep_empty: fraction of
        frames with nobody in view to keep anyway (hard negatives)."""
        self.world = world
        self.period_ns = int(1e9 / rate_hz) if rate_hz > 0 else 0
        self.keep_empty = keep_empty
        self.ids = {}  # actor name -> segmentation ID
        self.kinds = {}  # actor name -> kind
        self.names_by_id = {}
        self.background_painted = False
        self.background = set()  # objects already set to ID 0
        self.scene = -1  # scene loads so far, for unique file names
        self.pending = {}  # drone -> {time_stamp: {"scene": msg, "seg": msg}}
        self.last_saved = {}  # drone -> sim time (ns) of its last saved frame
        self.empty_skipped = {}  # drone -> empty frames since the last kept one
        self.counts = {"frames": 0, "boxes": 0}
        # drone -> callable(stamp, scene msg, seg msg, names_by_id, kinds) given
        # EVERY paired frame, whatever the rate (an experiment recording)
        self.taps = {}
        self._lock = threading.Lock()

        self.dir = os.path.join(os.path.expanduser(out_dir), time.strftime("%Y%m%d_%H%M%S"))
        for sub in ("images", "labels"):
            os.makedirs(os.path.join(self.dir, sub), exist_ok=True)
        with open(os.path.join(self.dir, "classes.txt"), "w") as f:
            f.write("\n".join(CLASSES) + "\n")
        self._boxes_file = open(os.path.join(self.dir, "boxes.csv"), "w", newline="")
        self._boxes = csv.writer(self._boxes_file)
        self._boxes.writerow(["sim_time", "drone", "frame", "actor", "class",
                              "x1", "y1", "x2", "y2", "pixels", "truncated"])
        self._frames_file = open(os.path.join(self.dir, "frames.csv"), "w", newline="")
        self._frames = csv.writer(self._frames_file)
        self._frames.writerow(["sim_time", "drone", "frame", "width", "height", "boxes",
                               "cam_x", "cam_y", "cam_z", "cam_qw", "cam_qx", "cam_qy",
                               "cam_qz", "hfov_deg"])
        self.hfov = {}  # drone -> GimbalCam horizontal FOV (deg), for frames.csv

        # Encoding and writing happen off the sim's message thread
        self._work = queue.Queue(maxsize=32)
        self._writer = threading.Thread(target=self._write_loop, daemon=True)
        self._writer.start()
        projectairsim_log().info(f"Ground-truth capture to {self.dir}")

    # ------------------------------------------------------------------
    # Segmentation IDs

    def new_scene(self, world):
        """After a scene (re)load: nothing has our IDs any more."""
        self.world = world
        self.ids = {}
        self.background = set()
        self.background_painted = False
        self.scene += 1

    def forget(self, names):
        """Their meshes were replaced (respawned under the same names): label
        them again at the next refresh."""
        for name in names:
            self.ids.pop(name, None)

    def refresh(self, actor_kinds, others=()):
        """Give each actor in `actor_kinds` (name -> kind) a segmentation ID of
        its own, and `others` (e.g. spawned static objects) ID 0. Cheap when
        nothing changed; call it after actors are spawned or reloaded."""
        if not self.background_painted:
            # Once: whatever the scene's own ID initialization gave the level's
            # meshes and drones becomes background. Touches every mesh, so the
            # sim stalls briefly.
            self.world.set_segmentation_id_by_name(".*", 0, True, True)
            self.background_painted = True
        changed = False
        for name in list(self.ids):
            if name not in actor_kinds:
                del self.ids[name]
                changed = True
        for name, kind in actor_kinds.items():
            if name in self.ids:
                continue
            free = sorted(set(range(1, MAX_ACTORS + 1)) - set(self.ids.values()))
            if not free:
                projectairsim_log().warning(f"Ground truth: no segmentation ID left for {name}")
                continue
            if self.world.set_segmentation_id_by_name(name, free[0], False, True):
                self.ids[name] = free[0]
                changed = True
            else:
                projectairsim_log().warning(f"Ground truth: no mesh named {name}")
        for name in set(others) - self.background:
            self.world.set_segmentation_id_by_name(name, 0, False, True)
        self.background = set(others)
        if changed:
            with self._lock:
                self.kinds = dict(actor_kinds)
                self.names_by_id = {i: n for n, i in self.ids.items()}
            projectairsim_log().info(f"Ground truth: {len(self.ids)} actors labelled")

    # ------------------------------------------------------------------
    # Frames

    def attach(self, client, drone, camera_id="GimbalCam"):
        """Subscribe to a (re)loaded drone's scene and segmentation images."""
        name = drone.name
        already = name in self.pending
        self.pending[name] = {}
        self.last_saved.pop(name, None)  # sim time restarts with a new scene
        if already:  # same topic names after a reload; still subscribed
            return
        sensor = drone.sensors[camera_id]
        client.subscribe(sensor["scene_camera"], lambda _, msg: self._receive(name, "scene", msg))
        client.subscribe(sensor["segmentation_camera"], lambda _, msg: self._receive(name, "seg", msg))

    def set_fov(self, drone, hfov_deg):
        """Zoom the segmentation image with the scene image: call it wherever
        the GimbalCam's field of view is set."""
        drone.set_field_of_view("GimbalCam", int(ImageType.SEGMENTATION), hfov_deg)
        self.hfov[drone.name] = hfov_deg

    def _receive(self, drone, kind, msg):
        """Pair a drone's scene and segmentation images by sim time; both are
        rendered on the same tick, but arrive on separate topics."""
        stamp = msg["time_stamp"]
        pending = self.pending.get(drone)
        if pending is None:
            return
        tap = self.taps.get(drone)
        with self._lock:
            due = stamp - self.last_saved.get(drone, -self.period_ns) >= self.period_ns
            if not due and not tap:
                return  # not due yet: skip the pairing, and the decode
            pair = pending.setdefault(stamp, {})
            pair[kind] = msg
            if len(pair) < 2:
                for old in sorted(pending)[:-PENDING_FRAMES]:
                    del pending[old]
                return
            del pending[stamp]
            for old in [s for s in pending if s < stamp]:
                del pending[old]
            if due:
                self.last_saved[drone] = stamp
            names_by_id, kinds, scene = self.names_by_id, self.kinds, self.scene
        if tap:
            tap(stamp, pair["scene"], pair["seg"], names_by_id, kinds)
        if not due:
            return
        try:
            self._work.put_nowait((drone, scene, stamp, pair["scene"], pair["seg"], names_by_id, kinds))
        except queue.Full:
            projectairsim_log().debug("Ground truth: writer behind, frame dropped")

    def _write_loop(self):
        while True:
            item = self._work.get()
            if item is None:
                return
            try:
                self._write(*item)
            except Exception as err:
                projectairsim_log().warning(f"Ground truth: frame not written: {err}")

    def _write(self, drone, scene_index, stamp, scene, seg, names_by_id, kinds):
        h, w = scene["height"], scene["width"]
        seg_img = np.frombuffer(seg["data"], np.uint8).reshape(seg["height"], seg["width"], -1)[:, :, :3]
        boxes = boxes_from_segmentation(seg_img, names_by_id)
        if seg_img.shape[:2] != (h, w):  # a mismatched config: scale into the scene image
            sx, sy = w / seg_img.shape[1], h / seg_img.shape[0]
            boxes = [(n, x1 * sx, y1 * sy, x2 * sx, y2 * sy, c) for n, x1, y1, x2, y2, c in boxes]
        if not boxes:
            skipped = self.empty_skipped.get(drone, 0) + 1
            if self.keep_empty <= 0 or skipped < 1.0 / self.keep_empty:
                self.empty_skipped[drone] = skipped
                return
        self.empty_skipped[drone] = 0

        stem = f"{drone}_s{max(scene_index, 0)}_{stamp}"
        frame = np.frombuffer(scene["data"], np.uint8).reshape(h, w, -1)[:, :, :3]
        cv2.imwrite(os.path.join(self.dir, "images", stem + ".jpg"), frame,
                    [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        sim_time = f"{stamp / 1e9:.3f}"
        lines = []
        for name, x1, y1, x2, y2, pixels in boxes:
            cls = CLASS_OF_KIND.get(kinds.get(name), "car")
            lines.append(f"{CLASSES.index(cls)} {(x1 + x2) / 2 / w:.6f} {(y1 + y2) / 2 / h:.6f} "
                         f"{(x2 - x1) / w:.6f} {(y2 - y1) / h:.6f}")
            truncated = int(x1 <= 0 or y1 <= 0 or x2 >= w or y2 >= h)
            self._boxes.writerow([sim_time, drone, stem, name, cls, round(x1, 1), round(y1, 1),
                                  round(x2, 1), round(y2, 1), pixels, truncated])
        with open(os.path.join(self.dir, "labels", stem + ".txt"), "w") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))
        self._frames.writerow([sim_time, drone, stem, w, h, len(boxes),
                               *(round(scene[k], 3) for k in ("pos_x", "pos_y", "pos_z")),
                               *(round(scene[k], 6) for k in ("rot_w", "rot_x", "rot_y", "rot_z")),
                               round(self.hfov.get(drone, float("nan")), 3)])
        # A few frames a second: flush each, so a killed run loses nothing
        self._boxes_file.flush()
        self._frames_file.flush()
        self.counts["frames"] += 1
        self.counts["boxes"] += len(boxes)

    def note_run(self, mission, run_id, event, drone=""):
        """Mark a mission run's start/end or a track step at the current sim
        time, so its frames can be picked out of the capture."""
        path = os.path.join(self.dir, "mission_runs.csv")
        new = not os.path.exists(path)
        with open(path, "a", newline="") as f:
            out = csv.writer(f)
            if new:
                out.writerow(["sim_time", "mission", "run", "event", "drone"])
            out.writerow([f"{self.world.get_sim_time() / 1e9:.3f}", mission, run_id, event,
                          drone])

    def status(self):
        return f"{self.counts['frames']} frames, {self.counts['boxes']} boxes in {self.dir}"

    def close(self):
        self._work.put(None)
        self._writer.join(timeout=5)
        self._boxes_file.close()
        self._frames_file.close()
        projectairsim_log().info(f"Ground truth: {self.status()}")
