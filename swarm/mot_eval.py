"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Score a multi-object tracker against a recorded mission run: how often it
switched a vehicle's ID, and what the drone was doing when it did.

    python mot_eval.py RUN_DIR TRACKER [TRACKER ...] [--drone Drone1]

RUN_DIR is a recording (experiment_recorder.py: frames, true boxes,
telemetry); each TRACKER is a tracker output folder written by X's
mot_replay.py (tracks.csv, detections.csv, run_info.json), e.g.
RUN_DIR/tracker_20261026_111310. Only the frames the tracker processed are
scored, with the true boxes scaled to its frame size.

Per frame, true boxes and tracks are matched CLEAR-MOT style (a match is kept
while its IoU holds, the rest are paired by IoU with the Hungarian method);
true boxes smaller than --min-px are ignored, as are tracks on them. Reported:

  ID switches     a vehicle matched to a different track ID than before
  IDs per vehicle how many track IDs each vehicle was given (1 is perfect)
  fragmentations  a vehicle's track lost and picked up again
  IDF1, MOTA, MOTP, and the detector's own recall and precision (from
                  detections.csv), to tell detector misses from tracker errors

Each ID switch and fragmentation is written to TRACKER/events.csv with what
was happening at that frame: the drone's attitude, its rates, speed and
acceleration (telemetry.csv), how fast the camera was turning (frames.csv),
the vehicle's box size, how close it was to the frame edge and to the
nearest other vehicle, how long since it was last detected, and a likely
cause:

  detector-gap    not detected for a while just before (the detector missed it)
  crowding        overlapping or close to another vehicle in the image
  camera-motion   the camera turning faster than --rate-thresh deg/s
  frame-edge      at or cut by the frame edge (leaving or re-entering)
  other           none of the above

TRACKER/summary.json has the totals and TRACKER/timeline.png shows each
vehicle's track IDs over time above the drone's yaw rate and speed.
"""

import argparse
import bisect
import csv
import json
import math
import os
from collections import Counter, defaultdict

import numpy as np
from scipy.optimize import linear_sum_assignment

CAUSE_DETECTOR_GAP_FRAMES = 2
CAUSE_CROWD_IOU = 0.05
CAUSE_CROWD_CENTRES = 1.5  # centres closer than this many box widths
CAUSE_EDGE_WIDTHS = 0.5


def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def iou_matrix(a, b):
    """IoU of boxes a (N,4) and b (M,4), x1 y1 x2 y2."""
    if len(a) == 0 or len(b) == 0:
        return np.zeros((len(a), len(b)))
    a, b = np.asarray(a, float)[:, None, :], np.asarray(b, float)[None, :, :]
    iw = np.clip(np.minimum(a[..., 2], b[..., 2]) - np.maximum(a[..., 0], b[..., 0]), 0, None)
    ih = np.clip(np.minimum(a[..., 3], b[..., 3]) - np.maximum(a[..., 1], b[..., 1]), 0, None)
    inter = iw * ih
    area_a = (a[..., 2] - a[..., 0]) * (a[..., 3] - a[..., 1])
    area_b = (b[..., 2] - b[..., 0]) * (b[..., 3] - b[..., 1])
    union = area_a + area_b - inter
    return np.where(union > 0, inter / np.maximum(union, 1e-9), 0.0)


def hungarian(iou, threshold):
    """[(row, col)] pairs maximising IoU, each at least threshold."""
    if iou.size == 0:
        return []
    rows, cols = linear_sum_assignment(-iou)
    return [(r, c) for r, c in zip(rows, cols) if iou[r, c] >= threshold]


class Recording:
    """A recorded run's true boxes and metadata for one drone."""

    def __init__(self, run_dir, drone):
        self.dir = run_dir
        self.frames = {int(r["frame"]): r for r in read_csv(os.path.join(run_dir, f"frames_{drone}.csv"))}
        self.boxes = defaultdict(list)  # frame -> [(actor, kind, x1, y1, x2, y2, pixels, truncated)]
        for r in read_csv(os.path.join(run_dir, f"boxes_{drone}.csv")):
            self.boxes[int(r["frame"])].append(
                (r["actor"], r.get("kind", ""), *(float(r[k]) for k in ("x1", "y1", "x2", "y2")),
                 int(r["pixels"]), int(r["truncated"])))
        self.telemetry = [r for r in read_csv(os.path.join(run_dir, "telemetry.csv"))
                          if r["drone"] == drone]
        self.tel_times = [float(r["sim_time"]) for r in self.telemetry]
        meta_path = os.path.join(run_dir, "meta.json")
        self.meta = {}
        if os.path.exists(meta_path):
            with open(meta_path) as f:
                self.meta = json.load(f)
        self._camera_rates = self._camera_rate()

    def sim_time(self, frame):
        return float(self.frames[frame]["sim_time"])

    def telemetry_at(self, t):
        if not self.telemetry:
            return {}
        i = bisect.bisect_left(self.tel_times, t)
        i = min(range(max(0, i - 1), min(len(self.tel_times), i + 1)),
                key=lambda j: abs(self.tel_times[j] - t))
        return self.telemetry[i]

    def _camera_rate(self):
        """frame -> how fast the camera was turning (deg/s), from its pose."""
        out = {}
        prev = None
        for f in sorted(self.frames):
            r = self.frames[f]
            q = np.array([float(r[k]) for k in ("cam_qw", "cam_qx", "cam_qy", "cam_qz")])
            t = float(r["sim_time"])
            if prev is not None and t > prev[1]:
                dot = min(1.0, abs(float(np.dot(q, prev[0]))))
                out[f] = math.degrees(2 * math.acos(dot)) / (t - prev[1])
            prev = (q, t)
        return out

    def camera_rate(self, frame):
        return self._camera_rates.get(frame, 0.0)


class Tracker:
    """A tracker's output (mot_replay.py)."""

    def __init__(self, path):
        self.dir = path if os.path.isdir(path) else os.path.dirname(path)
        self.name = os.path.basename(os.path.normpath(self.dir))
        info_path = os.path.join(self.dir, "run_info.json")
        self.info = {}
        if os.path.exists(info_path):
            with open(info_path) as f:
                self.info = json.load(f)
        self.tracks = defaultdict(list)  # frame -> [(id, x1, y1, x2, y2)]
        for r in read_csv(os.path.join(self.dir, "tracks.csv")):
            self.tracks[int(r["frame"])].append(
                (int(r["track_id"]), *(float(r[k]) for k in ("x1", "y1", "x2", "y2"))))
        self.detections = defaultdict(list)
        det_path = os.path.join(self.dir, "detections.csv")
        if os.path.exists(det_path):
            for r in read_csv(det_path):
                self.detections[int(r["frame"])].append(
                    tuple(float(r[k]) for k in ("x1", "y1", "x2", "y2")))
        self.processed = self.info.get("processed")
        self.scale = self.info.get("scale", [1.0, 1.0])
        self.size = self.info.get("input_size")


def evaluate(rec, trk, iou_thresh=0.5, min_px=8.0, kinds=None, rate_thresh=15.0):
    sx, sy = trk.scale
    frames = sorted(trk.processed if trk.processed is not None else rec.frames)
    width, height = trk.size or (None, None)
    matched_to = {}  # actor -> track id it is matched to now (CLEAR-MOT continuity)
    last_id = {}  # actor -> the track id it was last matched to
    was_matched = {}  # actor -> matched in the previous frame it was present
    last_detected = {}  # actor -> last processed frame index (position) it was detected
    gt_total = matches = fp = fn = switches = frags = 0
    iou_sum = 0.0
    pair_counts = Counter()  # (actor, track id) -> frames overlapping (for IDF1)
    gt_counts, trk_counts = Counter(), Counter()
    ids_per_actor = defaultdict(set)
    det_tp = det_fp = det_fn = 0
    events = []
    timeline = defaultdict(list)  # actor -> [(sim time, track id or None)]

    for position, f in enumerate(frames):
        if f not in rec.frames:
            continue
        t = rec.sim_time(f)
        gts, ignored = [], []
        for actor, kind, x1, y1, x2, y2, pixels, truncated in rec.boxes.get(f, []):
            if kinds and kind not in kinds:
                continue
            box = (x1 * sx, y1 * sy, x2 * sx, y2 * sy, pixels, truncated)
            if min(box[2] - box[0], box[3] - box[1]) < min_px:
                ignored.append(box[:4])
            else:
                gts.append((actor, box))
        tracks = trk.tracks.get(f, [])
        gt_boxes = [b[:4] for _, b in gts]
        trk_boxes = [tb[1:] for tb in tracks]
        iou = iou_matrix(gt_boxes, trk_boxes)

        # The detector alone (detections.csv), against the same true boxes
        dets = trk.detections.get(f, [])
        diou = iou_matrix(gt_boxes, dets)
        dpairs = hungarian(diou, iou_thresh)
        det_tp += len(dpairs)
        det_fn += len(gts) - len(dpairs)
        ign_det = iou_matrix(dets, ignored)
        matched_dets = {c for _, c in dpairs}
        det_fp += sum(1 for j in range(len(dets)) if j not in matched_dets
                      and not (ign_det.shape[1] and ign_det[j].max() >= iou_thresh))
        # Frames since each vehicle was last detected, before this frame (1:
        # detected in the previous frame tracked too)
        gap_before = {}
        for actor, _ in gts:
            gap_before[actor] = (position - last_detected[actor]) if actor in last_detected else None
        for r, _ in dpairs:
            last_detected[gts[r][0]] = position

        # IDF1 overlaps
        for r, c in hungarian(iou, iou_thresh):
            pair_counts[(gts[r][0], tracks[c][0])] += 1
        # CLEAR-MOT: keep last frame's pairs that still overlap, then Hungarian
        pairs = {}
        used_t = set()
        for r, (actor, _) in enumerate(gts):
            tid = matched_to.get(actor)
            c = next((c for c, tb in enumerate(tracks) if tb[0] == tid), None)
            if c is not None and iou[r, c] >= iou_thresh and c not in used_t:
                pairs[r] = c
                used_t.add(c)
        rest_r = [r for r in range(len(gts)) if r not in pairs]
        rest_c = [c for c in range(len(tracks)) if c not in used_t]
        if rest_r and rest_c:
            sub = iou[np.ix_(rest_r, rest_c)]
            for a, b in hungarian(sub, iou_thresh):
                pairs[rest_r[a]] = rest_c[b]

        ign_trk = iou_matrix(trk_boxes, ignored)
        matched_c = set(pairs.values())
        fp += sum(1 for c in range(len(tracks)) if c not in matched_c
                  and not (ign_trk.shape[1] and ign_trk[c].max() >= iou_thresh))
        trk_counts.update(tb[0] for c, tb in enumerate(tracks)
                          if not (ign_trk.shape[1] and ign_trk[c].max() >= iou_thresh))
        gt_total += len(gts)
        gt_counts.update(actor for actor, _ in gts)
        present = set()
        for r, (actor, box) in enumerate(gts):
            present.add(actor)
            c = pairs.get(r)
            if c is None:
                fn += 1
                matched_to.pop(actor, None)
                was_matched[actor] = False
                timeline[actor].append((t, None))
                continue
            tid = tracks[c][0]
            matches += 1
            iou_sum += iou[r, c]
            ids_per_actor[actor].add(tid)
            timeline[actor].append((t, tid))
            kind = None
            if actor in last_id and last_id[actor] != tid:
                switches += 1
                kind = "id-switch"
            elif was_matched.get(actor) is False:
                frags += 1
                kind = "fragmentation"
            if kind:
                events.append(_event(rec, f, t, actor, box, gts, kind, last_id.get(actor), tid,
                                     gap_before.get(actor), width, height, rate_thresh))
            last_id[actor] = tid
            matched_to[actor] = tid
            was_matched[actor] = True
        for actor in list(matched_to):
            if actor not in present:
                matched_to.pop(actor)  # left the frame: no continuity across the gap

    # IDF1: the best one-to-one pairing of vehicles and track IDs over the run
    actors = sorted(gt_counts)
    tids = sorted(trk_counts)
    idtp = 0
    if actors and tids:
        m = np.zeros((len(actors), len(tids)))
        ai, ti = {a: i for i, a in enumerate(actors)}, {t: i for i, t in enumerate(tids)}
        for (a, tid), n in pair_counts.items():
            if tid in ti:
                m[ai[a], ti[tid]] = n
        rows, cols = linear_sum_assignment(-m)
        idtp = int(m[rows, cols].sum())
    idfn = gt_total - idtp
    idfp = sum(trk_counts.values()) - idtp
    summary = {
        "tracker": trk.name, "frames_scored": len(frames), "true_boxes": gt_total,
        "vehicles": len(gt_counts),
        "id_switches": switches, "fragmentations": frags,
        "ids_per_vehicle": {a: len(ids_per_actor[a]) for a in actors},
        "mean_ids_per_vehicle": round(sum(len(ids_per_actor[a]) for a in actors) / len(actors), 2) if actors else None,
        "idf1": round(2 * idtp / max(1, 2 * idtp + idfp + idfn), 4),
        "mota": round(1 - (fn + fp + switches) / max(1, gt_total), 4),
        "motp_iou": round(iou_sum / max(1, matches), 4),
        "recall": round(matches / max(1, gt_total), 4), "false_positives": fp, "misses": fn,
        "detector_recall": round(det_tp / max(1, det_tp + det_fn), 4) if trk.detections else None,
        "detector_precision": round(det_tp / max(1, det_tp + det_fp), 4) if trk.detections else None,
        "causes": dict(Counter(c for e in events for c in e["cause"].split("+"))),
        "settings": trk.info.get("config", {}), "overrides": trk.info.get("overrides", {}),
        "source": trk.info.get("source"), "rate_hz": trk.info.get("rate_hz"),
        "iou_threshold": iou_thresh, "min_px": min_px,
    }
    return summary, events, timeline


def _event(rec, f, t, actor, box, gts, kind, old_id, new_id, gap, width, height,
           rate_thresh):
    x1, y1, x2, y2, pixels, truncated = box
    bw, bh = x2 - x1, y2 - y1
    others = [b for a, b in gts if a != actor]
    crowd_iou = float(iou_matrix([box[:4]], [b[:4] for b in others]).max()) if others else 0.0
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    nearest = min((math.hypot((b[0] + b[2]) / 2 - cx, (b[1] + b[3]) / 2 - cy) for b in others),
                  default=math.inf)
    edge = min(x1, y1, (width - x2) if width else math.inf, (height - y2) if height else math.inf)
    tel = rec.telemetry_at(t)
    cam_rate = rec.camera_rate(f)
    causes = []
    if gap is None or gap >= CAUSE_DETECTOR_GAP_FRAMES:
        causes.append("detector-gap")
    if crowd_iou >= CAUSE_CROWD_IOU or nearest < CAUSE_CROWD_CENTRES * max(bw, 1.0):
        causes.append("crowding")
    if cam_rate >= rate_thresh:
        causes.append("camera-motion")
    if truncated or edge < CAUSE_EDGE_WIDTHS * max(bw, 1.0):
        causes.append("frame-edge")

    def num(key):
        try:
            return round(float(tel[key]), 2)
        except (KeyError, TypeError, ValueError):
            return ""
    speed = (round(math.hypot(float(tel["vn"]), float(tel["ve"])), 2)
             if tel.get("vn") not in (None, "") else "")
    return {
        "frame": f, "sim_time": round(t, 3), "event": kind, "vehicle": actor,
        "old_id": old_id if old_id is not None else "", "new_id": new_id,
        "cause": "+".join(causes) or "other",
        "box_w": round(bw, 1), "box_h": round(bh, 1), "visible_px": pixels,
        "truncated": truncated, "edge_px": round(edge, 1),
        "nearest_other_iou": round(crowd_iou, 3),
        "nearest_other_px": round(nearest, 1) if nearest != math.inf else "",
        "frames_since_detected": gap if gap is not None else "",
        "camera_rate_dps": round(cam_rate, 1),
        "roll": num("roll"), "pitch": num("pitch"), "yaw": num("yaw"),
        "roll_rate": num("roll_rate"), "pitch_rate": num("pitch_rate"),
        "yaw_rate": num("yaw_rate"), "speed": speed,
        "accel_n": num("an"), "accel_e": num("ae"), "height": num("height"),
        "mode": tel.get("mode", ""), "mission_item": tel.get("mission_item", ""),
    }


def write_outputs(trk, summary, events, timeline, rec):
    with open(os.path.join(trk.dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    fields = list(events[0]) if events else ["frame", "sim_time", "event", "vehicle"]
    with open(os.path.join(trk.dir, "events.csv"), "w", newline="") as f:
        out = csv.DictWriter(f, fieldnames=fields)
        out.writeheader()
        out.writerows(events)
    try:
        _plot(trk, timeline, rec, events)
    except Exception as err:  # matplotlib missing or headless trouble: the CSVs stand
        print(f"  (no timeline.png: {err})")


def _plot(trk, timeline, rec, events):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    actors = sorted(timeline)
    if not actors:
        return
    t0 = min(t for a in actors for t, _ in timeline[a])
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(12, 1.2 + 0.45 * len(actors) + 2.5),
                                  sharex=True, gridspec_kw={"height_ratios": [max(2, len(actors)), 2]})
    cmap = plt.get_cmap("tab20")
    for row, actor in enumerate(actors):
        for t, tid in timeline[actor]:
            if tid is None:
                ax.plot(t - t0, row, "|", color="0.8", ms=6)
            else:
                ax.plot(t - t0, row, "s", color=cmap(tid % 20), ms=3)
    for e in events:
        row = actors.index(e["vehicle"])
        ax.plot(e["sim_time"] - t0, row, "x" if e["event"] == "id-switch" else "o",
                color="red", ms=8, mfc="none")
    ax.set_yticks(range(len(actors)))
    ax.set_yticklabels(actors)
    ax.set_title(f"{trk.name}: track IDs per vehicle (colour = ID, grey = no track, "
                 "red x = ID switch, red o = fragmentation)")
    tel = rec.telemetry
    if tel:
        ts = [float(r["sim_time"]) - t0 for r in tel]
        ax2.plot(ts, [float(r["yaw_rate"]) for r in tel], label="drone yaw rate (deg/s)")
        ax2.plot(ts, [math.hypot(float(r["vn"]), float(r["ve"])) for r in tel],
                 label="drone speed (m/s)")
        frames = sorted(rec.frames)
        ax2.plot([rec.sim_time(f) - t0 for f in frames], [rec.camera_rate(f) for f in frames],
                 label="camera turn rate (deg/s)", alpha=0.6)
        ax2.legend(loc="upper right", fontsize=8)
    ax2.set_xlabel("sim time since the recording started (s)")
    fig.tight_layout()
    fig.savefig(os.path.join(trk.dir, "timeline.png"), dpi=110)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run_dir")
    parser.add_argument("trackers", nargs="+", help="tracker output folders (mot_replay.py)")
    parser.add_argument("--drone", default="Drone1")
    parser.add_argument("--iou", type=float, default=0.5, help="IoU for a match [0.5]")
    parser.add_argument("--min-px", type=float, default=8.0,
                        help="ignore true boxes narrower than this, in tracker pixels [8]")
    parser.add_argument("--kinds", nargs="*", default=["vehicle"],
                        help="actor kinds scored [vehicle]; none: all")
    parser.add_argument("--rate-thresh", type=float, default=15.0,
                        help="camera turn rate counted as camera motion, deg/s [15]")
    args = parser.parse_args()

    rec = Recording(args.run_dir, args.drone)
    rows = []
    for path in args.trackers:
        trk = Tracker(path)
        summary, events, timeline = evaluate(rec, trk, args.iou, args.min_px, args.kinds or None,
                                             args.rate_thresh)
        write_outputs(trk, summary, events, timeline, rec)
        rows.append(summary)
        print(f"{trk.name} ({summary['source']}): {summary['vehicles']} vehicles, "
              f"{summary['frames_scored']} frames")
        print(f"  ID switches {summary['id_switches']}, fragmentations "
              f"{summary['fragmentations']}, IDs per vehicle {summary['mean_ids_per_vehicle']} "
              f"{summary['ids_per_vehicle']}")
        print(f"  IDF1 {summary['idf1']:.3f}  MOTA {summary['mota']:.3f}  MOTP {summary['motp_iou']:.3f}"
              f"  recall {summary['recall']:.3f}  false positives {summary['false_positives']}"
              + (f"  | detector recall {summary['detector_recall']:.3f} precision "
                 f"{summary['detector_precision']:.3f}" if summary["detector_recall"] is not None else ""))
        if summary["causes"]:
            print("  causes: " + ", ".join(f"{k} {v}" for k, v in sorted(summary["causes"].items())))
        print(f"  {os.path.join(trk.dir, 'events.csv')}, summary.json, timeline.png")
    if len(rows) > 1:
        print("\ntracker                                 IDsw  frag  IDs/veh  IDF1   MOTA   overrides")
        for s in rows:
            print(f"{s['tracker'][:38]:38s} {s['id_switches']:5d} {s['fragmentations']:5d} "
                  f"{s['mean_ids_per_vehicle'] or 0:8.2f} {s['idf1']:.3f}  {s['mota']:.3f}  "
                  f"{s['overrides']}")


if __name__ == "__main__":
    main()
