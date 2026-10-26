"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Score X's camera motion compensation (CMC) against the true camera motion of
a recorded mission run, and write the true motion for X to replay.

    python cmc_eval.py RUN_DIR [TRACKER ...] [--drone Drone1]

RUN_DIR is a recording (experiment_recorder.py); each TRACKER is a folder
written by X's mot_replay.py with CMC on, which logs every estimate to
cmc.csv. Nothing here needs the detector: the true motion comes from the
recorded camera poses, field of view and vehicle positions.

The true motion of a car between two processed frames is where the camera
motion alone would move it: its 3D position at the earlier frame, projected
through both frames' cameras. That is what CMC is for (the Kalman filter
handles the car's own motion), so each estimate is scored at the true cars,
not over the whole image:

  error         pixels between the CMC-warped position and the true one
  overlap       the IoU a car's box keeps with itself after that error
                (box size from the true box); below 0.2 ByteTrack's stage 1
                can no longer match it, whatever the Kalman filter predicts

The same is reported for no CMC (the camera motion itself), for the best
4-parameter warp (rotation, scale, shift: CMC's model) fitted to the true
ground motion, and for a rotation-only warp (what perfect gimbal and attitude
telemetry, without the drone's movement, could give).

RUN_DIR/cmc_truth.csv: per recorded frame, the true warp from the frame
before (truth_a11..a23) and the rotation-only one (rotation_a11..a23), in
recorded pixels. Feed it to X with mot_replay.py --cmc-warps to see what a
perfect CMC would do to ID switches (--cmc-warp-column rotation for the
rotation-only one).

Per TRACKER: cmc_eval.csv (one row per car and frame pair), cmc_eval.json
(the summary printed) and cmc_eval.png (error and failures over time).

Pose lag: the sim's image metadata runs a frame ahead of the pixels. The
recorder now writes each frame the pose its pixels were rendered from
(meta.json camera_pose_aligned); in older recordings frame k's pose describes
the image of frame k+1, so it is read from row k - --pose-lag [0 for aligned
recordings, else 1]. The fit of projected vehicles to their true boxes is
printed for lag 0 and the lag used.
"""

import argparse
import csv
import json
import math
import os
from collections import Counter, defaultdict

import numpy as np

TURN_BINS = [0.0, 5.0, 15.0, 30.0, math.inf]  # deg/s
GRID = (16, 9)


def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def similarity_fit(src, dst):
    """Least-squares rotation + uniform scale + shift (cv2's partial affine) as 2x3."""
    ms, md = src.mean(0), dst.mean(0)
    s, d = src - ms, dst - md
    norm = (s ** 2).sum()
    a = (s * d).sum() / norm
    b = (s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0]).sum() / norm
    M = np.array([[a, -b], [b, a]])
    return np.hstack([M, (md - M @ ms)[:, None]])


def warp_points(W, p):
    return p @ W[:, :2].T + W[:, 2]


def shifted_iou(w, h, dx, dy):
    """IoU of a w x h box with itself moved by (dx, dy)."""
    iw, ih = max(0.0, w - abs(dx)), max(0.0, h - abs(dy))
    inter = iw * ih
    return inter / (2 * w * h - inter)


class Scene:
    """Camera poses and vehicle positions of one recording."""

    def __init__(self, run_dir, drone, pose_lag):
        self.dir = run_dir
        self.rows = read_csv(os.path.join(run_dir, f"frames_{drone}.csv"))
        self.index = {int(r["frame"]): i for i, r in enumerate(self.rows)}
        self.pose_lag = pose_lag
        meta = {}
        if os.path.exists(os.path.join(run_dir, "meta.json")):
            with open(os.path.join(run_dir, "meta.json")) as f:
                meta = json.load(f)
        if pose_lag is None:
            pose_lag = 0 if meta.get("camera_pose_aligned") else 1
        self.pose_lag = pose_lag
        cam = meta.get("drones", {}).get(drone, {})
        self.size = (int(cam.get("width", 1920)), int(cam.get("height", 1080)))
        self.boxes = defaultdict(list)
        for r in read_csv(os.path.join(run_dir, f"boxes_{drone}.csv")):
            self.boxes[int(r["frame"])].append(r)
        tracks = defaultdict(list)
        for r in read_csv(os.path.join(run_dir, "actors.csv")):
            tracks[r["actor"]].append([float(r[k]) for k in ("sim_time", "north", "east", "down")])
        self.actors = {k: np.array(v) for k, v in tracks.items()}
        # The ground the cars drive on, as a plane: down = a*north + b*east + c
        allpos = np.vstack(list(self.actors.values()))[:, 1:]
        A = np.column_stack([allpos[:, 0], allpos[:, 1], np.ones(len(allpos))])
        self.plane, *_ = np.linalg.lstsq(A, allpos[:, 2], rcond=None)
        self.plane_rms = float(np.sqrt(np.mean((A @ self.plane - allpos[:, 2]) ** 2)))

    def sim_time(self, frame):
        return float(self.rows[self.index[frame]]["sim_time"])

    def pose(self, frame, lag=None):
        """(R camera-to-NED, position, fx) for frame's image."""
        lag = self.pose_lag if lag is None else lag
        r = self.rows[max(0, self.index[frame] - lag)]
        w, x, y, z = (float(r[k]) for k in ("cam_qw", "cam_qx", "cam_qy", "cam_qz"))
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        C = np.array([float(r[k]) for k in ("cam_x", "cam_y", "cam_z")])
        fx = (self.size[0] / 2) / math.tan(math.radians(float(r["hfov_deg"])) / 2)
        return R, C, fx

    def project(self, pose, P):
        """Pixel of NED point(s) P (N x 3), and their depth along the view axis."""
        R, C, fx = pose
        c = (np.atleast_2d(P) - C) @ R  # camera frame: x forward, y right, z down
        depth = c[:, 0]
        with np.errstate(divide="ignore", invalid="ignore"):
            uv = np.column_stack([self.size[0] / 2 + fx * c[:, 1] / depth,
                                  self.size[1] / 2 + fx * c[:, 2] / depth])
        return uv, depth

    def rays(self, pose, uv):
        R, _, fx = pose
        d = np.column_stack([np.full(len(uv), fx), uv[:, 0] - self.size[0] / 2,
                             uv[:, 1] - self.size[1] / 2])
        return d @ R.T

    def actor_at(self, name, t):
        a = self.actors[name]
        return np.array([np.interp(t, a[:, 0], a[:, i]) for i in (1, 2, 3)])

    def turn_deg(self, f0, f1):
        R0, R1 = self.pose(f0)[0], self.pose(f1)[0]
        return math.degrees(math.acos(np.clip((np.trace(R0.T @ R1) - 1) / 2, -1.0, 1.0)))

    def inside(self, uv, depth):
        w, h = self.size
        return (depth > 0) & (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)

    def true_warps(self, f0, f1):
        """(truth, rotation-only, truth fit rms px) warps from f0's image to f1's."""
        p0, p1 = self.pose(f0), self.pose(f1)
        gx, gy = GRID
        u, v = np.meshgrid((np.arange(gx) + 0.5) * self.size[0] / gx,
                           (np.arange(gy) + 0.5) * self.size[1] / gy)
        uv0 = np.column_stack([u.ravel(), v.ravel()])
        d = self.rays(p0, uv0)
        # Rotation only: the same directions seen from the second camera's attitude
        uv_rot, depth_rot = self.project((p1[0], np.zeros(3), p1[2]), d)
        ok = depth_rot > 0
        rot = similarity_fit(uv0[ok], uv_rot[ok]) if ok.sum() >= 3 else None
        # Truth: where those rays meet the ground, seen from the second camera
        a, b, c = self.plane
        C = p0[1]
        denom = d[:, 2] - a * d[:, 0] - b * d[:, 1]
        with np.errstate(divide="ignore", invalid="ignore"):
            s = (c + a * C[0] + b * C[1] - C[2]) / denom
        hit = (s > 0) & (s < 5000)
        if hit.sum() < 6:
            return None, rot, math.nan
        uv1, depth1 = self.project(p1, C + s[hit, None] * d[hit])
        ok = self.inside(uv1, depth1)
        if ok.sum() < 6:
            return None, rot, math.nan
        truth = similarity_fit(uv0[hit][ok], uv1[ok])
        rms = float(np.sqrt(np.mean(np.sum((warp_points(truth, uv0[hit][ok]) - uv1[ok]) ** 2, 1))))
        return truth, rot, rms

    def projection_check(self, lag):
        """|x| error (px) between projected vehicles and their true box centres."""
        errs = []
        for f in list(self.index)[5::5]:
            pose = self.pose(f, lag)
            for b in self.boxes.get(f, []):
                if b["truncated"] != "0" or b["actor"] not in self.actors:
                    continue
                uv, depth = self.project(pose, self.actor_at(b["actor"], self.sim_time(f)))
                if depth[0] > 0:
                    errs.append(abs(uv[0, 0] - (float(b["x1"]) + float(b["x2"])) / 2))
        return (float(np.median(errs)), float(np.percentile(errs, 90))) if errs else (math.nan, math.nan)


def write_truth(scene):
    path = os.path.join(scene.dir, "cmc_truth.csv")
    keys = ["a11", "a12", "a13", "a21", "a22", "a23"]
    frames = sorted(scene.index)
    with open(path, "w", newline="") as f:
        out = csv.writer(f)
        out.writerow(["frame", "prev_frame", *(f"truth_{k}" for k in keys),
                      *(f"rotation_{k}" for k in keys), "truth_fit_rms_px"])
        for f0, f1 in zip(frames, frames[1:]):
            truth, rot, rms = scene.true_warps(f0, f1)
            out.writerow([f1, f0,
                          *(np.round(truth.ravel(), 6) if truth is not None else [""] * 6),
                          *(np.round(rot.ravel(), 6) if rot is not None else [""] * 6),
                          round(rms, 3) if not math.isnan(rms) else ""])
    return path


def evaluate(scene, tracker_dir, min_px):
    with open(os.path.join(tracker_dir, "run_info.json")) as f:
        info = json.load(f)
    sx, sy = info.get("scale", [1.0, 1.0])
    to_rec = np.diag([1 / sx, 1 / sy, 1.0])
    from_rec = np.diag([sx, sy, 1.0])
    rows, pairs = [], []
    for r in read_csv(os.path.join(tracker_dir, "cmc.csv")):
        if r["prev_frame"] in ("", "None"):
            continue
        f0, f1 = int(r["prev_frame"]), int(r["frame"])
        W = None
        if r.get("a11") not in (None, ""):
            W_in = np.vstack([np.array([float(r[k]) for k in
                                        ("a11", "a12", "a13", "a21", "a22", "a23")]).reshape(2, 3),
                              [0, 0, 1]])
            W = (to_rec @ W_in @ from_rec)[:2]
        truth, rot, fit_rms = scene.true_warps(f0, f1)
        dt = scene.sim_time(f1) - scene.sim_time(f0)
        turn = scene.turn_deg(f0, f1)
        status = r["status"] or ("ok" if W is not None else "none")
        pairs.append({"frame": f1, "t": scene.sim_time(f1), "turn_rate": turn / dt if dt > 0 else 0.0,
                      "status": status, "inliers": r.get("inliers", ""), "fit_rms": fit_rms})
        p0, p1 = scene.pose(f0), scene.pose(f1)
        t0 = scene.sim_time(f0)
        for b in scene.boxes.get(f0, []):
            w, h = float(b["x2"]) - float(b["x1"]), float(b["y2"]) - float(b["y1"])
            if b["truncated"] != "0" or b["actor"] not in scene.actors or w * sx < min_px:
                continue
            X = scene.actor_at(b["actor"], t0)
            uv0, d0 = scene.project(p0, X)
            uv1, d1 = scene.project(p1, X)
            if not (scene.inside(uv0, d0)[0] and scene.inside(uv1, d1)[0]):
                continue
            row = {"prev_frame": f0, "frame": f1, "sim_time": round(scene.sim_time(f1), 3),
                   "actor": b["actor"], "box_w": round(w, 1), "box_h": round(h, 1),
                   "turn_deg": round(turn, 3), "turn_rate": round(turn / dt if dt > 0 else 0.0, 2),
                   "status": status, "inliers": r.get("inliers", "")}
            for name, warp in (("none", None), ("cmc", W), ("truth", truth), ("rotation", rot)):
                pred = uv0 if warp is None else warp_points(warp, uv0)
                dx, dy = (pred - uv1)[0]
                row[f"err_{name}"] = round(float(math.hypot(dx, dy)), 2)
                row[f"iou_{name}"] = round(shifted_iou(w, h, dx, dy), 3)
            rows.append(row)
    return rows, pairs, info


def summarise(rows, pairs):
    def stats(name, sel):
        e = np.array([r[f"err_{name}"] for r in sel])
        iou = np.array([r[f"iou_{name}"] for r in sel])
        if not len(e):
            return None
        return {"err_p50": round(float(np.median(e)), 1), "err_p90": round(float(np.percentile(e, 90)), 1),
                "err_p99": round(float(np.percentile(e, 99)), 1),
                "iou_below_0.5": round(float(np.mean(iou < 0.5)), 4),
                "iou_below_0.2": round(float(np.mean(iou < 0.2)), 4)}

    statuses = Counter(p["status"] for p in pairs)
    out = {"pairs": len(pairs), "car_pairs": len(rows),
           "estimate_failed": round(1 - statuses.get("ok", 0) / len(pairs), 4) if pairs else None,
           "statuses": dict(statuses),
           "truth_fit_rms_p90": round(float(np.nanpercentile([p["fit_rms"] for p in pairs], 90)), 2)
           if pairs else None,
           "all": {n: stats(n, rows) for n in ("none", "cmc", "truth", "rotation")},
           "by_turn_rate": []}
    for lo, hi in zip(TURN_BINS, TURN_BINS[1:]):
        sel = [r for r in rows if lo <= r["turn_rate"] < hi]
        psel = [p for p in pairs if lo <= p["turn_rate"] < hi]
        out["by_turn_rate"].append({
            "deg_s": f"{lo:g}-{hi:g}", "pairs": len(psel), "car_pairs": len(sel),
            "estimate_failed": round(sum(p["status"] != "ok" for p in psel) / len(psel), 4) if psel else None,
            **{n: stats(n, sel) for n in ("none", "cmc", "truth", "rotation")}})
    return out


def print_summary(name, s):
    print(f"{name}: {s['pairs']} frame pairs, {s['car_pairs']} car pairs; "
          f"estimate failed {100 * (s['estimate_failed'] or 0):.1f}% {s['statuses']}")
    print(f"  4-parameter model's own fit to the true ground motion: p90 {s['truth_fit_rms_p90']} px")
    print("  turn deg/s   pairs fail%  | error p50/p90/p99 px  (IoU<0.5 / IoU<0.2 share of car pairs)")
    for b in [{"deg_s": "all", "pairs": s["pairs"], "estimate_failed": s["estimate_failed"],
               **s["all"]}] + s["by_turn_rate"]:
        if not b.get("none"):
            continue
        print(f"  {b['deg_s']:>9s} {b['pairs']:7d} {100 * (b['estimate_failed'] or 0):5.1f}")
        for n in ("none", "cmc", "truth", "rotation"):
            v = b[n]
            print(f"      {n:9s} {v['err_p50']:6.1f} {v['err_p90']:6.1f} {v['err_p99']:7.1f}"
                  f"   ({100 * v['iou_below_0.5']:5.1f}% / {100 * v['iou_below_0.2']:5.1f}%)")


def plot(path, rows, pairs):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    t = [p["t"] for p in pairs]
    ax1.plot(t, [p["turn_rate"] for p in pairs], lw=0.8)
    bad = [p for p in pairs if p["status"] != "ok"]
    ax1.scatter([p["t"] for p in bad], [p["turn_rate"] for p in bad], c="red", s=8,
                label="no estimate applied")
    ax1.set_ylabel("camera turn deg/s")
    ax1.legend(loc="upper right")
    for name, colour in (("none", "0.6"), ("cmc", "C0"), ("truth", "C2")):
        ax2.scatter([r["sim_time"] for r in rows], [r[f"err_{name}"] for r in rows], s=3,
                    c=colour, label=name)
    ax2.set_yscale("symlog", linthresh=1)
    ax2.set_ylabel("error at true cars, px")
    ax2.set_xlabel("sim time s")
    ax2.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run_dir")
    parser.add_argument("trackers", nargs="*", help="mot_replay.py output folders with cmc.csv")
    parser.add_argument("--drone", default="Drone1")
    parser.add_argument("--pose-lag", type=int, default=None,
                        help="frames the recorded camera pose runs ahead of the image "
                             "[0 for recordings with camera_pose_aligned, else 1]")
    parser.add_argument("--min-px", type=float, default=8.0,
                        help="ignore true boxes narrower than this, in tracker pixels [8]")
    args = parser.parse_args()

    scene = Scene(args.run_dir, args.drone, args.pose_lag)
    for lag in sorted({0, scene.pose_lag}):
        p50, p90 = scene.projection_check(lag)
        print(f"pose lag {lag}: projected vehicles vs true boxes, |x| p50 {p50:.1f} px, p90 {p90:.1f} px")
    print(f"ground plane fit to the vehicles' positions: rms {scene.plane_rms:.2f} m")
    print(f"wrote {write_truth(scene)}")

    for path in args.trackers:
        rows, pairs, info = evaluate(scene, path, args.min_px)
        summary = summarise(rows, pairs)
        summary.update(tracker=os.path.basename(os.path.normpath(path)), source=info.get("source"),
                       overrides=info.get("overrides"), rate_hz=info.get("rate_hz"),
                       pose_lag=scene.pose_lag)
        if rows:
            with open(os.path.join(path, "cmc_eval.csv"), "w", newline="") as f:
                out = csv.DictWriter(f, fieldnames=list(rows[0]))
                out.writeheader()
                out.writerows(rows)
        with open(os.path.join(path, "cmc_eval.json"), "w") as f:
            json.dump(summary, f, indent=2)
        plot(os.path.join(path, "cmc_eval.png"), rows, pairs)
        print()
        print_summary(summary["tracker"], summary)


if __name__ == "__main__":
    main()
