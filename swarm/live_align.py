"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Line up X's live log of a flight with the sim's recording of the same flight,
so mot_eval.py can score what X did live.

    python live_align.py RUN_DIR LOG_DIR [--clock-offset S] [--drone Drone1]

RUN_DIR is a recording (experiment_recorder.py) and LOG_DIR an X mot_log
folder (MOT_LOG_DIR in X's config_cv.yml: frames.csv, detections.csv,
tracks.csv), copied from the companion.

X stamps each frame with the companion's clock when it read it from the video
stream; the recording stamps each frame with the laptop's clock when the sim
rendered it. The two differ by the clocks' offset plus the stream's delay.
That shift is found from the data:
- start from --clock-offset (the companion's clock minus the laptop's, in
  seconds; measure it, e.g. with `date +%s.%N` over ssh, or leave 0 if both
  run NTP)
- try shifts within --search seconds of it
- keep the one at which X's detections overlap the true boxes best

The score is the mean best IoU over all of X's detections (false ones
count as 0), printed with the best score more than 0.1 s away. Check that the
shift is the same for every flight of a session (it was 0.15-0.18 s for six
flights on 2026-10-26): it is the clocks' offset plus the stream's delay.

Writes RUN_DIR/tracker_<log name>_live/: tracks.csv and detections.csv keyed
by recording frame, and run_info.json (the frames X processed, the shift, the
live frame rate and the frames it skipped). Score it with:

    python mot_eval.py RUN_DIR RUN_DIR/tracker_<log name>_live

The video stream repeats the newest frame to keep a steady rate, so X can
process one rendered frame twice; only the last processing of each recording
frame is kept (counted as repeats).

The stream's delay jitters, so the shift alone puts about one X frame in five
on a neighbouring recording frame. When X also logged thumbnails
(MOT_LOG_THUMBS: thumbs.u8), each X frame is instead matched to the
recording frame within --match-window frames of the shift's guess whose
thumbnail is closest (mean absolute difference after scaling each thumbnail
to zero mean and unit spread, since X's videobalance changes its contrast):
the exact frame X processed. run_info.json then has "mapping": "thumbnail" and how far the
matches moved from the shift's guess.
"""

import argparse
import bisect
import csv
import json
import os
from collections import defaultdict

import cv2
import numpy as np

from mot_eval import iou_matrix, read_csv


def load_log(log_dir):
    frames = read_csv(os.path.join(log_dir, "frames.csv"))
    dets, tracks = defaultdict(list), defaultdict(list)
    for r in read_csv(os.path.join(log_dir, "detections.csv")):
        dets[int(r["capture_ns"])].append(r)
    for r in read_csv(os.path.join(log_dir, "tracks.csv")):
        tracks[int(r["capture_ns"])].append(r)
    return frames, dets, tracks


THUMB_W, THUMB_H = 128, 72  # X's MotLog thumbnail size


def recording_thumbs(video):
    """Every recorded frame shrunk the way X's MotLog shrinks it, decoded one
    frame at a time (a few MB for a whole run)."""
    cap = cv2.VideoCapture(video)
    out = []
    while True:
        ok, img = cap.read()
        if not ok:
            break
        if (img.shape[1], img.shape[0]) != (1920, 1080):
            img = cv2.resize(img, (1920, 1080))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        out.append(normalise(cv2.resize(gray, (THUMB_W, THUMB_H), interpolation=cv2.INTER_AREA)))
    cap.release()
    return np.array(out, np.float32)


def normalise(thumb):
    """Zero mean, unit spread: X's videobalance (CONTRAST, BRIGHTNESS) changes
    the grey levels of what it logs, not the picture."""
    t = thumb.astype(np.float32)
    return (t - t.mean()) / max(float(t.std()), 1e-3)


def score_shift(shift, captures, det_boxes, rec_times, gt_boxes):
    """Mean best IoU of X's detections with the true boxes of the recording
    frame each X frame maps to, for one shift (seconds, companion - laptop)."""
    total, n = 0.0, 0
    for c, boxes in zip(captures, det_boxes):
        t = c - shift
        i = bisect.bisect_right(rec_times, t) - 1  # the newest frame rendered by then
        if i < 0:
            continue
        gt = gt_boxes[i]
        if len(gt) == 0:
            n += len(boxes)
            continue
        total += float(iou_matrix(boxes, gt).max(axis=1).sum())
        n += len(boxes)
    return total / n if n else 0.0


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run_dir")
    parser.add_argument("log_dir")
    parser.add_argument("--drone", default="Drone1")
    parser.add_argument("--clock-offset", type=float, default=0.0,
                        help="companion clock minus laptop clock, seconds [0]")
    parser.add_argument("--search", type=float, default=3.0,
                        help="search this many seconds either side of it [3]")
    parser.add_argument("--step", type=float, default=0.01, help="search step, seconds [0.01]")
    parser.add_argument("--match-window", type=int, default=5,
                        help="with thumbnails: frames either side of the shift's guess [5]")
    args = parser.parse_args()

    rec = read_csv(os.path.join(args.run_dir, f"frames_{args.drone}.csv"))
    rec_frames = [int(r["frame"]) for r in rec]
    rec_times = [float(r["wall_time"]) for r in rec]
    boxes = defaultdict(list)
    for r in read_csv(os.path.join(args.run_dir, f"boxes_{args.drone}.csv")):
        if r.get("kind", "vehicle") == "vehicle":
            boxes[int(r["frame"])].append([float(r[k]) for k in ("x1", "y1", "x2", "y2")])
    gt_boxes = [np.array(boxes.get(f, []), float).reshape(-1, 4) for f in rec_frames]

    frames, dets, tracks = load_log(args.log_dir)
    captures_all = [int(r["capture_ns"]) for r in frames]
    # Score on up to 600 frames with detections, spread over the run
    with_dets = [c for c in captures_all if dets.get(c)]
    step = max(1, len(with_dets) // 600)
    sample = with_dets[::step]
    caps = [c / 1e9 for c in sample]
    det_boxes = [np.array([[float(r[k]) for k in ("x1", "y1", "x2", "y2")] for r in dets[c]])
                 for c in sample]

    shifts = np.arange(args.clock_offset - args.search, args.clock_offset + args.search + 1e-9,
                       args.step)
    scores = np.array([score_shift(s, caps, det_boxes, rec_times, gt_boxes) for s in shifts])
    best = int(scores.argmax())
    # runner-up outside +-0.1 s of the best
    far = np.abs(shifts - shifts[best]) > 0.1
    runner = float(scores[far].max()) if far.any() else float("nan")
    shift = float(shifts[best])
    print(f"shift {shift:+.3f} s (companion - laptop, clocks plus stream delay): "
          f"mean IoU {scores[best]:.3f}; best elsewhere {runner:.3f}")
    # The score averages every detection, misses and false ones included, so
    # live detections score 0.15-0.7 on these recordings; near 0 means the log
    # and the recording are not the same flight
    if scores[best] < 0.1:
        print("  WARNING: no match; check that the log and the recording are the same flight")

    # Map every logged frame to its recording frame; keep the last per frame
    thumbs_path = os.path.join(args.log_dir, "thumbs.u8")
    thumbs = None
    if os.path.exists(thumbs_path):
        thumbs = np.memmap(thumbs_path, np.uint8, "r").reshape(-1, THUMB_H, THUMB_W)
        # Thumbnails and rows are written in the same order; a run that was
        # killed can leave a few more thumbnails than rows (rows are flushed
        # once a second), never fewer
        if len(thumbs) < len(captures_all):
            print(f"  WARNING: {len(thumbs)} thumbnails for {len(captures_all)} frames; "
                  "mapping by time")
            thumbs = None
    if thumbs is not None:
        rec_thumbs = recording_thumbs(os.path.join(args.run_dir, f"gimbal_{args.drone}.mp4"))
        n_rec = min(len(rec_thumbs), len(rec_times))
    moves, diffs, margins = defaultdict(int), [], []
    mapped = {}
    repeats = 0
    for n, c in enumerate(captures_all):
        i = bisect.bisect_right(rec_times, c / 1e9 - shift) - 1
        if i < 0 or (i == len(rec_times) - 1 and c / 1e9 - shift > rec_times[-1] + 0.5):
            continue
        if thumbs is not None:
            lo, hi = max(0, i - args.match_window), min(n_rec, i + args.match_window + 1)
            if lo >= hi:
                continue
            d = np.abs(rec_thumbs[lo:hi] - normalise(thumbs[n])).mean(axis=(1, 2))
            best = int(d.argmin())
            moves[lo + best - i] += 1
            diffs.append(float(d[best]))
            if len(d) > 1:
                margins.append(float(np.partition(d, 1)[1] - d[best]))
            i = lo + best
        if rec_frames[i] in mapped:
            repeats += 1
        mapped[rec_frames[i]] = c

    name = os.path.basename(os.path.normpath(args.log_dir))
    out = os.path.join(args.run_dir, f"tracker_{name}_live")
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "tracks.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "track_id", "x1", "y1", "x2", "y2", "conf", "label", "state"])
        for frame, c in sorted(mapped.items()):
            for r in tracks.get(c, []):
                w.writerow([frame, r["track_id"], r["x1"], r["y1"], r["x2"], r["y2"],
                            r["conf"], r["label"], r["state"]])
    with open(os.path.join(out, "detections.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "label", "conf", "x1", "y1", "x2", "y2"])
        for frame, c in sorted(mapped.items()):
            for r in dets.get(c, []):
                w.writerow([frame, r["label"], r["conf"], r["x1"], r["y1"], r["x2"], r["y2"]])

    # The live rate: processed frames per second over the mapped span
    span = (max(mapped.values()) - min(mapped.values())) / 1e9 if mapped else 0.0
    processed_in_span = [c for c in captures_all
                         if mapped and min(mapped.values()) <= c <= max(mapped.values())]
    ms = [float(r["infer_ms"]) + float(r["track_ms"]) for r in frames]
    loop = [(int(b["done_ns"]) - int(a["done_ns"])) / 1e6 for a, b in zip(frames, frames[1:])]
    info = {
        "source": f"live X log {name}", "log_dir": os.path.abspath(args.log_dir),
        "mapping": "thumbnail" if thumbs is not None else "time",
        "shift_s": round(shift, 3), "match_iou": round(float(scores[best]), 3),
        "runner_up_iou": round(runner, 3),
        "scale": [1.0, 1.0], "input_size": [1920, 1080],
        "frames_logged": len(captures_all), "frames_mapped": len(mapped),
        "repeats": repeats,
        "stale_skipped": sum(int(r["stale_skipped"]) for r in frames),
        "live_fps": round(len(processed_in_span) / span, 2) if span else None,
        "detect_track_ms_mean": round(sum(ms) / len(ms), 1) if ms else None,
        "loop_ms_median": round(sorted(loop)[len(loop) // 2], 1) if loop else None,
        "processed": sorted(mapped),
    }
    if thumbs is not None:
        total = sum(moves.values())
        info["thumbnail_moves"] = {f"{k:+d}": round(v / total, 4) for k, v in sorted(moves.items())}
        info["thumbnail_diff_median"] = round(float(np.median(diffs)), 3)
        info["thumbnail_margin_median"] = round(float(np.median(margins)), 3)
        print("thumbnail matches, frames from the shift's guess: "
              + " ".join(f"{k}: {100 * v:.0f}%" for k, v in info["thumbnail_moves"].items())
              + f"; difference median {info['thumbnail_diff_median']} (normalised), "
              f"margin to the next best {info['thumbnail_margin_median']}")
    with open(os.path.join(out, "run_info.json"), "w") as f:
        json.dump(info, f, indent=2)
    print(f"{len(mapped)} recording frames tracked live ({repeats} repeats dropped, "
          f"{info['stale_skipped']} frames skipped while busy; {info['live_fps']} frames/s; "
          f"detect+track {info['detect_track_ms_mean']} ms); {out}")


if __name__ == "__main__":
    main()
