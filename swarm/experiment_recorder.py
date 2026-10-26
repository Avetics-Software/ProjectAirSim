"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Records a mission run for multi-object-tracking experiments: every GimbalCam
frame, what the drone was doing at that frame, and where every vehicle really
was, so a tracker's ID switches can be related to the drone's motion.

A mission with `record: true` (or `record: [Drone1, ...]`) is recorded from
`mission start` until a few seconds after it ends, to runs/<mission>_<run>/:

  gimbal_<drone>.mp4      every rendered GimbalCam frame once, in order
                          (near-lossless H.264; the frame rate tag is
                          nominal, the true times are in frames_<drone>.csv)
  frames_<drone>.csv      per video frame: index, sim and wall time, the
                          camera's world pose (NED; Euler too: the gimbal's
                          actual attitude), field of view and zoom, PX4's
                          gimbal angles, the drone's mode and mission item
  boxes_<drone>.csv       per video frame, each vehicle (or person) in view:
                          its name (true identity), box in pixels, visible
                          pixels, cut by the frame edge (needs --ground-truth,
                          which renders the segmentation image)
  telemetry.csv           each recorded drone's true kinematics at ~50 Hz in
                          sim time: position, attitude, angular rates,
                          velocity, acceleration; and its lat/lon/height
  actors.csv              every moving actor's position and heading at 10 Hz
  meta.json               camera, run, counts (frames, any dropped), and
                          copies of the mission and actor files

mot_eval.py scores a tracker's output against it.
"""

import asyncio
import csv
import json
import math
import os
import queue
import shutil
import subprocess
import threading
import time

import numpy as np

from projectairsim.utils import projectairsim_log, quaternion_to_rpy

from gt_capture import boxes_from_segmentation

TELEMETRY_HZ = 50.0
ACTORS_HZ = 10.0
QUEUE_FRAMES = 240  # frames waiting to be encoded (~8 s at 30 Hz; ~5 MB each)
TAIL_S = 3.0  # recorded after the mission run ends


def _encoder():
    encoders = subprocess.run(["ffmpeg", "-hide_banner", "-encoders"],
                              capture_output=True, text=True).stdout
    if "h264_nvenc" in encoders:
        return ["-c:v", "h264_nvenc", "-preset", "p4", "-rc", "constqp", "-qp", "14"]
    return ["-c:v", "libx264", "-preset", "veryfast", "-crf", "12"]


def _euler_deg(qw, qx, qy, qz):
    return [math.degrees(v) for v in quaternion_to_rpy(qw, qx, qy, qz)]


class DroneRecording:
    """One drone's GimbalCam video, per-frame metadata and boxes. Frames come
    in on the sim's message thread (`add`); a writer thread encodes them."""

    def __init__(self, out_dir, name, payload, link):
        self.name = name
        self.payload = payload  # SimPayload: camera, zoom
        self.link = link
        self.dir = out_dir
        self.frames = 0
        self.dropped = 0
        self.boxes = 0
        self.size = None
        self.last_stamp = None
        self._proc = None
        self._queue = queue.Queue(maxsize=QUEUE_FRAMES)
        self._frames_file = open(os.path.join(out_dir, f"frames_{name}.csv"), "w", newline="")
        self._frames = csv.writer(self._frames_file)
        self._frames.writerow([
            "frame", "sim_time", "wall_time", "cam_x", "cam_y", "cam_z",
            "cam_qw", "cam_qx", "cam_qy", "cam_qz", "cam_roll", "cam_pitch", "cam_yaw",
            "hfov_deg", "zoom_percent", "gimbal_pitch", "gimbal_yaw", "mode", "mission_item",
            "boxes"])
        self._boxes_file = open(os.path.join(out_dir, f"boxes_{name}.csv"), "w", newline="")
        self._boxes = csv.writer(self._boxes_file)
        self._boxes.writerow(["frame", "sim_time", "actor", "kind", "x1", "y1", "x2", "y2",
                              "pixels", "truncated"])
        self._thread = threading.Thread(target=self._write_loop, daemon=True)
        self._thread.start()

    def add(self, stamp, scene, seg=None, names_by_id=None, kinds=None):
        """One rendered frame (and its segmentation image, if there is one)."""
        if stamp == self.last_stamp:
            return
        self.last_stamp = stamp
        link, payload = self.link, self.payload
        cam = payload.camera
        state = {
            "wall": time.time(),
            "hfov": cam.hfov_deg(cam.focal_length(payload.zoom_percent)),
            "zoom": payload.zoom_percent,
            "gimbal": link.gimbal_attitude or (math.nan, math.nan),
            "mode": link.mode,
            "item": link.mission_seq,
        }
        try:
            self._queue.put_nowait((stamp, scene, seg, names_by_id or {}, kinds or {}, state))
        except queue.Full:
            self.dropped += 1

    def _start_ffmpeg(self, w, h):
        cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
               "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{w}x{h}", "-framerate", "30",
               "-i", "-", *_encoder(), "-pix_fmt", "yuv420p",
               os.path.join(self.dir, f"gimbal_{self.name}.mp4")]
        self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        self.size = (w, h)

    def _write_loop(self):
        while True:
            item = self._queue.get()
            if item is None:
                return
            try:
                self._write(*item)
            except Exception as err:
                projectairsim_log().warning(f"Recording {self.name}: frame not written: {err}")

    def _write(self, stamp, scene, seg, names_by_id, kinds, state):
        h, w = scene["height"], scene["width"]
        if self._proc is None:
            self._start_ffmpeg(w, h)
        if (w, h) != self.size:
            return  # the camera changed size mid-run; keep the video consistent
        frame = np.frombuffer(scene["data"], np.uint8).reshape(h, w, -1)[:, :, :3]
        self._proc.stdin.write(np.ascontiguousarray(frame).tobytes())
        index = self.frames
        self.frames += 1
        sim_time = f"{stamp / 1e9:.4f}"
        boxes = []
        if seg is not None:
            seg_img = np.frombuffer(seg["data"], np.uint8).reshape(seg["height"], seg["width"], -1)[:, :, :3]
            boxes = boxes_from_segmentation(seg_img, names_by_id)
            if seg_img.shape[:2] != (h, w):
                sx, sy = w / seg_img.shape[1], h / seg_img.shape[0]
                boxes = [(n, x1 * sx, y1 * sy, x2 * sx, y2 * sy, c) for n, x1, y1, x2, y2, c in boxes]
            for name, x1, y1, x2, y2, pixels in boxes:
                truncated = int(x1 <= 0 or y1 <= 0 or x2 >= w or y2 >= h)
                self._boxes.writerow([index, sim_time, name, kinds.get(name, ""), round(x1, 1),
                                      round(y1, 1), round(x2, 1), round(y2, 1), pixels, truncated])
            self.boxes += len(boxes)
        q = [scene[k] for k in ("rot_w", "rot_x", "rot_y", "rot_z")]
        roll, pitch, yaw = _euler_deg(*q)
        gp, gy = state["gimbal"]
        self._frames.writerow([
            index, sim_time, f"{state['wall']:.4f}",
            *(round(scene[k], 3) for k in ("pos_x", "pos_y", "pos_z")),
            *(round(v, 6) for v in q), round(roll, 2), round(pitch, 2), round(yaw, 2),
            round(state["hfov"], 3), round(state["zoom"], 1), round(gp, 2), round(gy, 2),
            state["mode"], state["item"], len(boxes) if seg is not None else ""])

    def close(self):
        self._queue.put(None)
        self._thread.join()
        if self._proc:
            self._proc.stdin.close()
            self._proc.wait(timeout=30)
        self._frames_file.close()
        self._boxes_file.close()


class ExperimentRecorder:
    """Records one mission run (see the module docstring)."""

    def __init__(self, session, mission, run_id, indices, run_dir):
        self.session = session
        self.mission = mission
        self.run_id = run_id
        self.dir = os.path.join(run_dir, f"{mission.name}_{run_id}")
        os.makedirs(self.dir, exist_ok=True)
        station = session.station
        self.ground_truth = session.ground_truth  # pairs scene + segmentation frames
        self.recordings = {}
        for i in indices:
            payload, link = station.cameras[i], station.links[i]
            self.recordings[i] = DroneRecording(self.dir, link.name, payload, link)
        self._tasks = []
        self._t0 = time.time()

    def start(self):
        log = projectairsim_log()
        if not shutil.which("ffmpeg"):
            raise RuntimeError("ffmpeg not found")
        for i, rec in self.recordings.items():
            if self.ground_truth:
                self.ground_truth.taps[rec.name] = rec.add
            else:
                rec.payload.frame_taps.append(self._scene_tap(rec))
        if not self.ground_truth:
            log.warning("Recording without --ground-truth: video and metadata, but no "
                        "vehicle boxes (start the sim with --ground-truth for them)")
        loop = asyncio.get_running_loop()
        self._tasks = [loop.create_task(self._telemetry()), loop.create_task(self._actors())]
        for name in ("mission.yml", "actors.yml"):
            source = self.mission.path if name == "mission.yml" else self.mission.actors
            if source and os.path.isfile(source):
                shutil.copy(source, os.path.join(self.dir, name))
        log.info(f"Recording {', '.join(r.name for r in self.recordings.values())} "
                 f"to {self.dir}")

    @staticmethod
    def _scene_tap(rec):
        def tap(msg):
            rec.add(msg["time_stamp"], msg)
        tap.recording = rec
        return tap

    async def stop(self):
        """Stop after the tail; close everything and write meta.json."""
        await asyncio.sleep(TAIL_S)
        for i, rec in self.recordings.items():
            if self.ground_truth:
                self.ground_truth.taps.pop(rec.name, None)
            else:
                rec.payload.frame_taps[:] = [t for t in rec.payload.frame_taps
                                             if getattr(t, "recording", None) is not rec]
        for task in self._tasks:
            task.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)
        await asyncio.to_thread(lambda: [rec.close() for rec in self.recordings.values()])
        self._write_meta()
        log = projectairsim_log()
        for rec in self.recordings.values():
            dropped = f", {rec.dropped} DROPPED (the encoder fell behind)" if rec.dropped else ""
            log.info(f"Recorded {rec.name}: {rec.frames} frames, {rec.boxes} boxes{dropped}")
        log.info(f"Recording written to {self.dir} (score a tracker with mot_eval.py)")

    def _write_meta(self):
        cams = {}
        for rec in self.recordings.values():
            cam = rec.payload.camera
            w, h = rec.size or cam.resolution
            hfov = cam.hfov_deg(cam.focal_length(rec.payload.zoom_percent))
            cams[rec.name] = {
                "video": f"gimbal_{rec.name}.mp4", "width": w, "height": h,
                "hfov_deg_at_end": round(hfov, 3),
                "fx_px_at_end": round((w / 2) / math.tan(math.radians(hfov) / 2), 2),
                "frames": rec.frames, "dropped": rec.dropped, "boxes": rec.boxes,
            }
        meta = {
            "mission": self.mission.name, "run": self.run_id,
            "recorded_s": round(time.time() - self._t0, 1),
            "ground_truth_boxes": bool(self.ground_truth),
            "drones": cams,
            "files": {"frames": "frames_<drone>.csv", "boxes": "boxes_<drone>.csv",
                      "telemetry": "telemetry.csv", "actors": "actors.csv"},
            "frames": "sim_time: seconds of sim time; wall_time: Unix seconds on the "
                      "laptop when the frame arrived",
            "telemetry": "angular rates and velocity as the sim reports them "
                         "(get_ground_truth_kinematics twist), NED",
        }
        with open(os.path.join(self.dir, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)

    async def _telemetry(self):
        session = self.session
        with open(os.path.join(self.dir, "telemetry.csv"), "w", newline="") as f:
            out = csv.writer(f)
            out.writerow(["sim_time", "drone", "north", "east", "down", "lat", "lon", "height",
                          "roll", "pitch", "yaw", "roll_rate", "pitch_rate", "yaw_rate",
                          "vn", "ve", "vd", "an", "ae", "ad", "mode", "mission_item"])
            last = {}
            while True:
                for i, rec in self.recordings.items():
                    drone = session.drones[i] if i < len(session.drones) else None
                    if drone is None:
                        continue
                    k = drone.get_ground_truth_kinematics()
                    stamp = k["time_stamp"]
                    if last.get(i) == stamp:
                        continue
                    last[i] = stamp
                    p, o = k["pose"]["position"], k["pose"]["orientation"]
                    ang, lin = k["twist"]["angular"], k["twist"]["linear"]
                    acc = k.get("accels", {}).get("linear", {"x": math.nan, "y": math.nan,
                                                              "z": math.nan})
                    roll, pitch, yaw = _euler_deg(o["w"], o["x"], o["y"], o["z"])
                    pos = rec.link.position or (math.nan,) * 4
                    out.writerow([
                        f"{stamp / 1e9:.4f}", rec.name, round(p["x"], 3), round(p["y"], 3),
                        round(p["z"], 3), f"{pos[0]:.7f}", f"{pos[1]:.7f}", round(pos[3], 2),
                        round(roll, 3), round(pitch, 3), round(yaw, 3),
                        *(round(math.degrees(ang[a]), 3) for a in ("x", "y", "z")),
                        *(round(lin[a], 3) for a in ("x", "y", "z")),
                        *(round(acc[a], 3) for a in ("x", "y", "z")),
                        rec.link.mode, rec.link.mission_seq])
                await asyncio.sleep(1.0 / TELEMETRY_HZ)

    async def _actors(self):
        actors = self.session.actors
        world = self.session.world
        with open(os.path.join(self.dir, "actors.csv"), "w", newline="") as f:
            out = csv.writer(f)
            out.writerow(["sim_time", "actor", "kind", "north", "east", "down", "heading"])
            while True:
                if actors and actors.actors:
                    names = list(actors.actors)
                    sim_time = world.get_sim_time() / 1e9
                    for name, pose in zip(names, world.get_object_poses(names)):
                        t, q = pose["translation"], pose["rotation"]
                        if math.isnan(t["x"]):
                            continue
                        _, _, yaw = quaternion_to_rpy(q["w"], q["x"], q["y"], q["z"])
                        heading = (math.degrees(yaw) - actors.actors[name]["yaw_offset"]) % 360
                        out.writerow([f"{sim_time:.3f}", name, actors.actors[name]["kind"],
                                      round(t["x"], 2), round(t["y"], 2), round(t["z"], 2),
                                      round(heading, 1)])
                await asyncio.sleep(1.0 / ACTORS_HZ)
