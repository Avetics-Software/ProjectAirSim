"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

`sim tracktest [N] [follow|lock]`: how well drone N's gimbal camera keeps a
ground target in frame while the drone flies hard.

A ground point 50 m ahead of the drone is the target. An idealised tracker
points the gimbal at it 20 times a second from the drone's true position (so
what's measured is the gimbal: its stabilisation, slew limits and yaw mode,
not a vision tracker), while the drone

  1. dashes 120 m sideways at full speed, keeping its heading
  2. stops hard
  3. turns 90 degrees on the spot (the target stays within the gimbal's yaw
     travel; a turn putting it behind the drone would lose it in any mode)
  4. dashes back, keeping its new heading

From each rendered frame's camera pose, the target's offset from the image
centre is measured: the summary gives the share of frames with the target in
frame, its largest offsets, and the time it spent out of frame; a CSV has every
frame. Compare `follow` (the camera turns with the drone, trailing it) with
`lock` (it holds its compass heading).
"""

import asyncio
import csv
import math
import time

import numpy as np

from projectairsim.utils import projectairsim_log, quaternion_to_rpy

from gcs.swarm import offset_geo

TARGET_AHEAD_M = 50.0
DASH_M = 120.0
TRACKER_HZ = 20.0


def rotate_inverse(q, v):
    """v expressed in the frame of the rotation q (w, x, y, z)."""
    w, x, y, z = q
    r = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    return r.T @ np.asarray(v)


async def run_track_test(session, camera, mode, log_dir):
    """Run the test on one drone (a SimPayload with no companion)."""
    log = projectairsim_log()
    link, drone, world = camera.link, camera.drone, session.world
    if not (link.armed and link.position and link.position[3] > 5.0):
        log.warning(f"tracktest: take {camera.name} off first (5 m or higher)")
        return
    lock = mode == "lock"

    # The target: on the ground, TARGET_AHEAD_M ahead of the drone
    k = drone.get_ground_truth_kinematics()
    p = k["pose"]["position"]
    q = k["pose"]["orientation"]
    heading = quaternion_to_rpy(q["w"], q["x"], q["y"], q["z"])[2]
    tn = p["x"] + TARGET_AHEAD_M * math.cos(heading)
    te = p["y"] + TARGET_AHEAD_M * math.sin(heading)
    ground = world.get_surface_elevation_at_point(tn, te)
    target = np.array([tn, te, ground if ground is not None and not math.isnan(ground)
                       else p["z"] + link.position[3]])

    hfov = math.radians(camera.camera.hfov_deg(camera.camera.focal_length(camera.zoom_percent)))
    w, h = camera.camera.resolution
    vfov = 2 * math.atan(math.tan(hfov / 2) * h / w)
    rows = []
    phase = ["acquire"]  # the manoeuvre in progress, recorded with each frame
    camera.pose_wanted += 1
    stop = asyncio.Event()

    async def tracker():
        link.take_gimbal_control()
        last_frame = None
        while not stop.is_set():
            kin = drone.get_ground_truth_kinematics()
            pos = kin["pose"]["position"]
            o = kin["pose"]["orientation"]
            yaw_now = quaternion_to_rpy(o["w"], o["x"], o["y"], o["z"])[2]
            d = target - np.array([pos["x"], pos["y"], pos["z"]])
            bearing = math.atan2(d[1], d[0])
            pitch = -math.degrees(math.atan2(d[2], math.hypot(d[0], d[1])))
            yaw = math.degrees(bearing if lock else bearing - yaw_now)
            link.set_gimbal_attitude(pitch, (yaw + 180) % 360 - 180, yaw_lock=lock)
            # Where the target is in the latest rendered frame
            frame = camera.camera_pose
            if frame and frame is not last_frame:
                last_frame = frame
                cam_pos, cam_rot = frame
                c = rotate_inverse(cam_rot, target - np.array(cam_pos))  # x ahead, y right, z down
                h_off = math.degrees(math.atan2(c[1], c[0]))
                v_off = math.degrees(math.atan2(c[2], c[0]))
                inside = c[0] > 0 and abs(math.radians(h_off)) < hfov / 2 and \
                    abs(math.radians(v_off)) < vfov / 2
                rows.append((time.monotonic(), h_off, v_off, inside, math.degrees(yaw_now),
                             phase[0]))
            await asyncio.sleep(1.0 / TRACKER_HZ)

    async def wait_arrival(lat, lon, timeout):
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            dist = link.distance_to(lat, lon)
            if dist is not None and dist < 4.0:
                return
            await asyncio.sleep(0.2)

    task = asyncio.create_task(tracker())
    lat0, lon0, _, alt = link.position
    heading_deg = math.degrees(heading) % 360.0
    side = heading_deg + 90.0
    lat1, lon1 = offset_geo(lat0, lon0, DASH_M * math.cos(math.radians(side)),
                            DASH_M * math.sin(math.radians(side)))
    log.info(f"tracktest {camera.name} ({mode}): target {TARGET_AHEAD_M:.0f} m ahead; "
             "dash sideways, stop, turn 90, dash back")
    t0 = time.monotonic()
    try:
        # Acquire: the gimbal slews onto the target first (not measured)
        t_end = time.monotonic() + 8.0
        while time.monotonic() < t_end:
            recent = rows[-10:]
            if len(recent) == 10 and all(abs(r[1]) < 2 and abs(r[2]) < 2 for r in recent):
                break
            await asyncio.sleep(0.1)
        phase[0] = "dash"
        # Dash sideways, keeping the heading (PX4 would otherwise turn to face
        # the way it flies)
        await asyncio.to_thread(link.fly_to, lat1, lon1, alt, -1.0, heading_deg)
        await wait_arrival(lat1, lon1, 40.0)
        phase[0] = "stop"
        await asyncio.to_thread(link.hold)                             # hard stop
        await asyncio.sleep(3.0)
        phase[0] = "turn"
        turned = (heading_deg - 90.0) % 360.0  # towards the target, which is behind-left now
        await asyncio.to_thread(link.turn_to, turned)
        await asyncio.sleep(8.0)                                       # turn on the spot
        phase[0] = "dash back"
        await asyncio.to_thread(link.fly_to, lat0, lon0, alt, -1.0, turned)
        await wait_arrival(lat0, lon0, 40.0)
        await asyncio.sleep(3.0)
    finally:
        stop.set()
        await task
        camera.pose_wanted -= 1

    measured = [r for r in rows if r[5] != "acquire"]
    if not measured:
        log.warning("tracktest: no camera frames were measured")
        return
    path = f"{log_dir}/tracktest_{camera.name}_{mode}_{time.strftime('%H%M%S')}.csv"
    with open(path, "w", newline="") as f:
        out = csv.writer(f)
        out.writerow(["t", "phase", "h_offset_deg", "v_offset_deg", "in_frame",
                      "drone_yaw_deg"])
        for t, h_off, v_off, inside, yaw, ph in rows:
            out.writerow([round(t - t0, 3), ph, round(h_off, 2), round(v_off, 2), int(inside),
                          round(yaw, 1)])
    in_frame = sum(r[3] for r in measured) / len(measured)
    out_time = sum(b[0] - a[0] for a, b in zip(measured, measured[1:]) if not a[3])
    worst = {}
    for r in measured:
        worst[r[5]] = max(worst.get(r[5], 0.0), abs(r[1]), abs(r[2]))
    per_phase = ", ".join(f"{ph} {w:.1f}" for ph, w in worst.items())
    log.info(f"tracktest {camera.name} ({mode}): target in frame {in_frame:.0%} of "
             f"{len(measured)} frames, out of frame {out_time:.1f} s; largest offset "
             f"(deg) by phase: {per_phase}; mean "
             f"{sum(max(abs(r[1]), abs(r[2])) for r in measured) / len(measured):.1f} "
             f"(frame half-width {math.degrees(hfov / 2):.1f}, half-height "
             f"{math.degrees(vfov / 2):.1f}); {path}")
