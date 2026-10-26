"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Tracking a moving target from a moving drone, inside a mission (the sim side
of a mission's `track` step, gcs/mission.py):

    - track: {target: Vehicle1}       # hand the camera to the tracker
    - goto: ...                       # the drone keeps flying its route
    - track: off                      # the mission takes the camera back

When PX4 reaches the step it gives the gimbal to nobody, and the tracker takes
it:

  x       the drone's companion (X, vision): once the target is in the frame
          (and big enough to lock onto), the station locks X onto it from the
          ground truth, as an operator's drag would (BOX + CLICK), then leaves
          it to X
  truth   no vision: the gimbal is pointed at the target's true position 20
          times a second, a best case for the gimbal and the flight; X's
          result can be measured against it
  auto    x if the drone has a companion, else truth

Either way every rendered frame is scored against where the target really
is: its offset from the image centre, whether it is in the frame, its size in
pixels. The frames go to runs/<mission>_<run>_track_<drone>.csv and a summary
to the log when the tracking ends.
"""

import asyncio
import csv
import math
import os
import time

import numpy as np

from projectairsim.utils import projectairsim_log, quaternion_to_rpy

from track_test import rotate_inverse

TRACKER_HZ = 20.0
TARGET_HEIGHT_M = 0.8  # aim this far above the actor's origin (a car's centre)
TARGET_SIZE_M = 4.5  # for the box sent to X: about a car's length
MIN_BOX_PX = 24
# X is locked on once the target is this big in the GimbalCam frame (about
# 100 m away for a car at the widest zoom) and well inside it: a smaller box
# is a few pixels in X's own video
ACQUIRE_MIN_PX = 50
ACQUIRE_MARGIN = 0.1  # of the frame's width/height, kept clear of the edges
HAND_OVER_TIMEOUT_S = 3.0  # wait this long for PX4 to report the gimbal released


class MissionTrack:
    """One drone tracking one target for one `track` stretch of a mission."""

    def __init__(self, session, index, spec, mission, run_id, run_dir):
        station = session.station
        self.session = session
        self.world = session.world
        self.camera = station.cameras[index]  # SimPayload
        self.link = station.links[index]
        self.companion = station.companions.get(index)
        self.target = spec["target"]
        tracker = spec["tracker"]
        if tracker == "auto":
            tracker = "x" if self.companion else "truth"
        if tracker == "x" and not self.companion:
            raise ValueError(f"{self.camera.name} has no companion for tracker x")
        if tracker == "truth" and self.companion:
            raise ValueError(f"{self.camera.name}'s companion (X) would take the gimbal back "
                             "from the truth tracker; use tracker x (or auto)")
        self.tracker = tracker
        self.path = os.path.join(run_dir, f"{mission}_{run_id}_track_{self.camera.name}.csv")
        self.rows = []  # (t, h_off, v_off, in_frame, distance, size_px)
        self.acquired_at = None  # s after the start, when X was given the target
        self._stop = asyncio.Event()
        self._task = None
        self._t0 = time.monotonic()

    def start(self):
        if not self.target:
            raise ValueError("track needs a target name to score (track: {target: NAME})")
        self._target_pose()  # fails now if there is no such object
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        self._task = asyncio.get_running_loop().create_task(self._run())
        projectairsim_log().info(
            f"{self.camera.name}: tracking {self.target} ({'X, vision' if self.tracker == 'x' else 'truth: aimed from its true position'}); scoring every frame")

    async def stop(self):
        self._stop.set()
        if self._task:
            await self._task

    # ------------------------------------------------------------------

    def _target_pose(self):
        """The target's aim point (NED, metres)."""
        pose = self.world.get_object_poses([self.target])[0]
        t = pose["translation"]
        if math.isnan(t["x"]):
            raise ValueError(f"no object {self.target} in the sim")
        return np.array([t["x"], t["y"], t["z"] - TARGET_HEIGHT_M])

    def _optics(self):
        cam = self.camera.camera
        hfov = math.radians(cam.hfov_deg(cam.focal_length(self.camera.zoom_percent)))
        w, h = cam.resolution
        fx = (w / 2) / math.tan(hfov / 2)
        return w, h, fx, hfov

    async def _run(self):
        log = projectairsim_log()
        camera, link = self.camera, self.link
        camera.pose_wanted += 1
        last_frame = None
        try:
            # PX4 hands the gimbal over when it runs the step; wait to hear it
            # (a click sent before then would be dropped as the mission's)
            t_end = time.monotonic() + HAND_OVER_TIMEOUT_S
            while link.autopilot_has_gimbal and time.monotonic() < t_end:
                await asyncio.sleep(0.1)
            if self.tracker == "truth":
                link.take_gimbal_control()
            while not self._stop.is_set():
                target = self._target_pose()
                if self.tracker == "truth":
                    self._aim(target)
                frame = camera.camera_pose
                if frame and frame is not last_frame:
                    last_frame = frame
                    self._score(frame, target)
                await asyncio.sleep(1.0 / TRACKER_HZ)
        except Exception as err:
            log.warning(f"{camera.name}: tracking {self.target} stopped: {err}")
        finally:
            camera.pose_wanted -= 1
            self._write()

    def _aim(self, target):
        """Point the gimbal at the target's true position (tracker truth)."""
        kin = self.camera.drone.get_ground_truth_kinematics()
        p, o = kin["pose"]["position"], kin["pose"]["orientation"]
        yaw_now = quaternion_to_rpy(o["w"], o["x"], o["y"], o["z"])[2]
        d = target - np.array([p["x"], p["y"], p["z"]])
        pitch = -math.degrees(math.atan2(d[2], math.hypot(d[0], d[1])))
        yaw = math.degrees(math.atan2(d[1], d[0]) - yaw_now)
        self.link.set_gimbal_attitude(pitch, (yaw + 180) % 360 - 180)

    def _score(self, frame, target):
        cam_pos, cam_rot = frame
        c = rotate_inverse(cam_rot, target - np.array(cam_pos))  # x ahead, y right, z down
        w, h, fx, hfov = self._optics()
        distance = float(np.linalg.norm(c))
        h_off = math.degrees(math.atan2(c[1], c[0]))
        v_off = math.degrees(math.atan2(c[2], c[0]))
        ahead = c[0] > 0
        u = w / 2 + fx * c[1] / c[0] if ahead else math.nan
        v = h / 2 + fx * c[2] / c[0] if ahead else math.nan
        inside = ahead and 0 <= u < w and 0 <= v < h
        size_px = fx * TARGET_SIZE_M / distance if distance > 0 else math.nan
        t = time.monotonic() - self._t0
        self.rows.append((t, h_off, v_off, inside, distance, size_px))
        lockable = inside and size_px >= ACQUIRE_MIN_PX and \
            ACQUIRE_MARGIN * w <= u <= (1 - ACQUIRE_MARGIN) * w and \
            ACQUIRE_MARGIN * h <= v <= (1 - ACQUIRE_MARGIN) * h
        if self.tracker == "x" and self.acquired_at is None and lockable:
            self._lock_x(u, v, size_px, w, h)
            self.acquired_at = t

    def _lock_x(self, u, v, size_px, w, h):
        """Give X the target, as an operator's drag on the Tracker view would."""
        companion = self.companion
        frame = companion.frame
        tw, th = (frame.shape[1], frame.shape[0]) if frame is not None else (w, h)
        sx, sy = tw / w, th / h
        half = max(MIN_BOX_PX, size_px * sx) / 2
        x, y = u * sx, v * sy
        companion.drag(x - half, y - half, x + half, y + half)
        projectairsim_log().info(
            f"{self.camera.name}: {self.target} in the frame; X locked onto it at "
            f"({x:.0f}, {y:.0f}) in its video, box {2 * half:.0f} px")

    def _write(self):
        rows = self.rows
        with open(self.path, "w", newline="") as f:
            out = csv.writer(f)
            out.writerow(["t", "h_offset_deg", "v_offset_deg", "in_frame", "distance_m",
                          "target_px"])
            for t, h_off, v_off, inside, dist, size in rows:
                out.writerow([round(t, 3), round(h_off, 2), round(v_off, 2), int(inside),
                              round(dist, 1), round(size, 1)])
        log = projectairsim_log()
        if not rows:
            log.warning(f"{self.camera.name}: no frames scored for {self.target}")
            return
        scored = [r for r in rows if self.acquired_at is None or r[0] >= self.acquired_at] \
            if self.tracker == "x" else rows
        if not scored:
            scored = rows
        in_frame = sum(r[3] for r in scored) / len(scored)
        offsets = [max(abs(r[1]), abs(r[2])) for r in scored if r[3]]
        longest_out, run_start = 0.0, None
        for r in scored:
            if not r[3] and run_start is None:
                run_start = r[0]
            elif r[3] and run_start is not None:
                longest_out = max(longest_out, r[0] - run_start)
                run_start = None
        if run_start is not None:
            longest_out = max(longest_out, scored[-1][0] - run_start)
        _, _, _, hfov = self._optics()
        acquired = ("" if self.tracker != "x" else
                    f"; X locked on after {self.acquired_at:.1f} s" if self.acquired_at is not None
                    else "; never in the frame, so X was never locked on")
        offset_text = (f"offset from centre mean {sum(offsets) / len(offsets):.1f} deg, "
                       f"max {max(offsets):.1f} (frame half-width "
                       f"{math.degrees(hfov / 2):.1f})" if offsets else "never in the frame")
        log.info(f"{self.camera.name} tracking {self.target} ({self.tracker}), "
                 f"{scored[-1][0] - scored[0][0]:.0f} s: in frame {in_frame:.0%} of "
                 f"{len(scored)} frames, longest out {longest_out:.1f} s; {offset_text}; "
                 f"distance {min(r[4] for r in scored):.0f}-{max(r[4] for r in scored):.0f} m"
                 f"{acquired}; {self.path}")
