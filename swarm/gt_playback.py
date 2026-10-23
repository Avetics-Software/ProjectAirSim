"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Plays back a ground-truth capture (--ground-truth) after the sim has stopped:
each saved GimbalCam frame with its boxes and actor names, beside a map of
where every moving actor and drone camera was at that moment.

    python gt_playback.py                         # the newest capture
    python gt_playback.py ../datasets/sim_gt/20261023_144412 --drone 2
    python gt_playback.py --only-boxes --save review.mp4   # to a video, no window

Keys:
  space          play / pause
  right, left    next / previous frame (also d / a)
  . ,            10 frames forward / back
  0-9            show only drone N (0 = all drones)
  b              only frames with boxes, or all frames
  + -            faster / slower
  q, Esc         quit
"""

import argparse
import bisect
import csv
import glob
import math
import os
import re
import sys

import cv2
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_ROOT = os.path.join(os.path.dirname(SCRIPT_DIR), "datasets", "sim_gt")
STEM = re.compile(r"^(?P<drone>.+?)(?:_s(?P<scene>\d+))?_(?P<ns>\d+)$")

VIEW_W = 1280  # the frame is scaled to this width
MAP_W = 420
KEYS_LEFT = (81, 2424832, 65361, ord("a"))
KEYS_RIGHT = (83, 2555904, 65363, ord("d"))


def actor_color(name):
    """A steady, distinct BGR colour per actor name."""
    hue = (sum(ord(c) * 31 ** i for i, c in enumerate(name)) % 180)
    hsv = np.uint8([[[hue, 200, 255]]])
    return tuple(int(v) for v in cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0])


def camera_yaw(qw, qx, qy, qz):
    """Heading of the camera's forward (x) axis, radians from north."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class Capture:
    """One capture folder: frames, boxes and the moving actors' logs."""

    def __init__(self, path):
        self.path = path
        self.frames = []  # dicts, in time order within each scene
        by_stem = {}
        frames_csv = os.path.join(path, "frames.csv")
        if os.path.exists(frames_csv):
            for row in csv.DictReader(open(frames_csv)):
                by_stem[row["frame"]] = row
        # The images are the record; frames.csv adds the camera pose when it
        # has the row (a killed run may have lost the last few)
        for image in glob.glob(os.path.join(path, "images", "*.jpg")):
            stem = os.path.splitext(os.path.basename(image))[0]
            match = STEM.match(stem)
            if not match:
                continue
            row = by_stem.get(stem, {})
            self.frames.append({
                "stem": stem,
                "drone": match["drone"],
                "scene": int(match["scene"] or 0),
                "t": int(match["ns"]) / 1e9,
                "cam": (float(row["cam_x"]), float(row["cam_y"]),
                        camera_yaw(*(float(row[k]) for k in ("cam_qw", "cam_qx", "cam_qy", "cam_qz"))))
                if row else None,
            })
        self.frames.sort(key=lambda f: (f["scene"], f["t"], f["drone"]))
        self.drones = sorted({f["drone"] for f in self.frames},
                             key=lambda d: int(re.sub(r"\D", "", d) or 0))

        self.boxes = {}  # stem -> [(actor, class, x1, y1, x2, y2, truncated)]
        boxes_csv = os.path.join(path, "boxes.csv")
        if os.path.exists(boxes_csv):
            for row in csv.DictReader(open(boxes_csv)):
                self.boxes.setdefault(row["frame"], []).append(
                    (row["actor"], row["class"], float(row["x1"]), float(row["y1"]),
                     float(row["x2"]), float(row["y2"]), row["truncated"] == "1"))
        names = os.path.join(path, "classes.txt")
        self.classes = open(names).read().split() if os.path.exists(names) else []

        # One actor log per scene load, in order: scene N is the Nth
        self.tracks = []  # per scene: {actor: ([t], [(north, east, heading)])}
        for log in sorted(glob.glob(os.path.join(path, "ground_truth_*.csv"))):
            tracks = {}
            for row in csv.DictReader(open(log)):
                t, xy = tracks.setdefault(row["name"], ([], []))
                t.append(float(row["sim_time_s"]))
                xy.append((float(row["north_m"]), float(row["east_m"]), float(row["heading_deg"])))
            self.tracks.append(tracks)

    def frame_boxes(self, frame, image_shape):
        """The frame's boxes from boxes.csv, else from its YOLO label file
        (which has the class but not the actor's name)."""
        if frame["stem"] in self.boxes:
            return self.boxes[frame["stem"]]
        label = os.path.join(self.path, "labels", frame["stem"] + ".txt")
        if not os.path.exists(label):
            return []
        h, w = image_shape[:2]
        boxes = []
        for line in open(label):
            c, cx, cy, bw, bh = line.split()
            cls = int(c)
            name = self.classes[cls] if cls < len(self.classes) else c
            cx, cy, bw, bh = float(cx) * w, float(cy) * h, float(bw) * w, float(bh) * h
            boxes.append(("?", name, cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2, False))
        return boxes

    def actors_at(self, scene, t):
        """{actor: (north, east, heading)} nearest to sim time t."""
        if scene >= len(self.tracks):
            return {}
        where = {}
        for name, (times, xy) in self.tracks[scene].items():
            i = bisect.bisect_left(times, t)
            near = [j for j in (i - 1, i) if 0 <= j < len(times)]
            j = min(near, key=lambda j: abs(times[j] - t))
            if abs(times[j] - t) < 1.0:
                where[name] = xy[j]
        return where

    def cameras_at(self, scene, t):
        """{drone: (north, east, yaw)}: each drone's latest frame at or before t."""
        cams = {}
        for f in self.frames:
            if f["scene"] == scene and f["t"] <= t and f["cam"]:
                cams[f["drone"]] = f["cam"]
        return cams


class MapPanel:
    """A north-up map of one scene: the actors' paths, and where everyone is."""

    def __init__(self, capture, scene, height):
        self.capture, self.height = capture, height
        tracks = capture.tracks[scene] if scene < len(capture.tracks) else {}
        points = [(n, e) for _, xy in tracks.values() for n, e, _ in xy]
        points += [f["cam"][:2] for f in capture.frames if f["scene"] == scene and f["cam"]]
        if not points:
            points = [(0.0, 0.0)]
        ns, es = [p[0] for p in points], [p[1] for p in points]
        self.n0, self.e0 = (min(ns) + max(ns)) / 2, (min(es) + max(es)) / 2
        span = max(max(ns) - min(ns), max(es) - min(es), 50.0) * 1.1
        self.scale = (min(MAP_W, height) - 40) / span  # px per metre
        self.base = np.full((height, MAP_W, 3), 32, np.uint8)
        for name, (_, xy) in tracks.items():
            path = np.array([self.px(n, e) for n, e, _ in xy[::5]], np.int32)
            if len(path) > 1:
                cv2.polylines(self.base, [path], False, (70, 70, 70), 1, cv2.LINE_AA)
        bar = 10 ** math.floor(math.log10(span / 4))
        y = height - 15
        cv2.line(self.base, (15, y), (15 + int(bar * self.scale), y), (200, 200, 200), 2)
        cv2.putText(self.base, f"{bar:g} m", (20 + int(bar * self.scale), y + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(self.base, "N", (MAP_W - 22, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (200, 200, 200), 1, cv2.LINE_AA)

    def px(self, north, east):
        return (int(MAP_W / 2 + (east - self.e0) * self.scale),
                int(self.height / 2 - (north - self.n0) * self.scale))

    def draw(self, frame, boxed):
        img = self.base.copy()
        scene, t = frame["scene"], frame["t"]
        for name, (n, e, _) in self.capture.actors_at(scene, t).items():
            p = self.px(n, e)
            seen = name in boxed
            cv2.circle(img, p, 6 if seen else 4, actor_color(name), -1 if seen else 1, cv2.LINE_AA)
            cv2.putText(img, name, (p[0] + 7, p[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                        actor_color(name) if seen else (150, 150, 150), 1, cv2.LINE_AA)
        for drone, (n, e, yaw) in self.capture.cameras_at(scene, t).items():
            p = self.px(n, e)
            colour = (255, 255, 255) if drone == frame["drone"] else (140, 140, 140)
            tip = (int(p[0] + 18 * math.sin(yaw)), int(p[1] - 18 * math.cos(yaw)))
            cv2.arrowedLine(img, p, tip, colour, 2, cv2.LINE_AA, tipLength=0.4)
            cv2.putText(img, drone, (p[0] + 6, p[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        colour, 1, cv2.LINE_AA)
        return img


class Player:
    def __init__(self, capture, drone=None, only_boxes=False, fps=4.0):
        self.capture = capture
        self.drone, self.only_boxes, self.fps = drone, only_boxes, fps
        self.maps = {}
        self._select()
        self.i = 0

    def _select(self, keep=None):
        """The frames shown with the current filters; stay near `keep`."""
        self.shown = [
            f for f in self.capture.frames
            if (self.drone is None or f["drone"] == self.drone)
            and (not self.only_boxes or self._has_boxes(f))
        ]
        if keep is not None and self.shown:
            key = (keep["scene"], keep["t"])
            self.i = min(bisect.bisect_left([(f["scene"], f["t"]) for f in self.shown], key),
                         len(self.shown) - 1)

    def _has_boxes(self, f):
        if f["stem"] in self.capture.boxes:
            return True
        label = os.path.join(self.capture.path, "labels", f["stem"] + ".txt")
        return os.path.exists(label) and os.path.getsize(label) > 0

    def render(self, playing):
        frame = self.shown[self.i]
        image = cv2.imread(os.path.join(self.capture.path, "images", frame["stem"] + ".jpg"))
        boxes = self.capture.frame_boxes(frame, image.shape)
        k = VIEW_W / image.shape[1]
        view = cv2.resize(image, (VIEW_W, int(image.shape[0] * k)), interpolation=cv2.INTER_AREA)
        for actor, cls, x1, y1, x2, y2, truncated in boxes:
            colour = actor_color(actor)
            p1, p2 = (int(x1 * k), int(y1 * k)), (int(x2 * k), int(y2 * k))
            cv2.rectangle(view, p1, p2, colour, 2)
            text = f"{actor} ({cls})" + (" cut" if truncated else "")
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            # Above the box, unless that is under the status bar
            y = p1[1] - 6 if p1[1] > th + 34 else p2[1] + th + 6
            cv2.rectangle(view, (p1[0], y - th - 3), (p1[0] + tw + 4, y + 3), (0, 0, 0), -1)
            cv2.putText(view, text, (p1[0] + 2, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1,
                        cv2.LINE_AA)

        status = (f"{frame['drone']}  scene {frame['scene']}  t {frame['t']:.2f} s  "
                  f"frame {self.i + 1}/{len(self.shown)}  {len(boxes)} boxes  "
                  f"[{'playing' if playing else 'paused'} {self.fps:g}/s"
                  f"{', boxes only' if self.only_boxes else ''}"
                  f"{', ' + self.drone + ' only' if self.drone else ''}]")
        cv2.rectangle(view, (0, 0), (VIEW_W, 26), (0, 0, 0), -1)
        cv2.putText(view, status, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1,
                    cv2.LINE_AA)

        key = (frame["scene"], view.shape[0])
        if key not in self.maps:
            self.maps[key] = MapPanel(self.capture, frame["scene"], view.shape[0])
        panel = self.maps[key].draw(frame, {b[0] for b in boxes})
        return np.hstack([view, panel])

    def run(self):
        playing = True
        cv2.namedWindow("Ground truth", cv2.WINDOW_NORMAL)
        while True:
            cv2.imshow("Ground truth", self.render(playing))
            key = cv2.waitKeyEx(int(1000 / self.fps) if playing else 0)
            if key in (ord("q"), 27) or cv2.getWindowProperty("Ground truth", cv2.WND_PROP_VISIBLE) < 1:
                break
            current = self.shown[self.i]
            if key == ord(" "):
                playing = not playing
            elif key in KEYS_RIGHT:
                self.i, playing = min(self.i + 1, len(self.shown) - 1), False
            elif key in KEYS_LEFT:
                self.i, playing = max(self.i - 1, 0), False
            elif key == ord("."):
                self.i = min(self.i + 10, len(self.shown) - 1)
            elif key == ord(","):
                self.i = max(self.i - 10, 0)
            elif key in (ord("+"), ord("=")):
                self.fps = min(self.fps * 1.5, 60.0)
            elif key == ord("-"):
                self.fps = max(self.fps / 1.5, 0.5)
            elif key == ord("b") or ord("0") <= key <= ord("9"):
                if key == ord("b"):
                    self.only_boxes = not self.only_boxes
                else:
                    n = key - ord("0")
                    self.drone = None if n == 0 else f"Drone{n}"
                self._select(keep=current)
                if not self.shown:
                    print("No frames match that filter")
                    self.drone, self.only_boxes = None, False
                    self._select(keep=current)
            elif key == -1 and playing:
                if self.i + 1 < len(self.shown):
                    self.i += 1
                else:
                    playing = False
        cv2.destroyAllWindows()

    def save(self, path):
        first = self.render(True)
        writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), self.fps,
                                 (first.shape[1], first.shape[0]))
        for self.i in range(len(self.shown)):
            writer.write(self.render(True))
        writer.release()
        print(f"Wrote {len(self.shown)} frames to {path}")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("capture", nargs="?",
                        help=f"a capture folder (default: the newest in {DEFAULT_ROOT})")
    parser.add_argument("--drone", type=int, help="show only drone N")
    parser.add_argument("--only-boxes", action="store_true",
                        help="skip frames with nobody in view")
    parser.add_argument("--fps", type=float, default=4.0, help="playback speed, frames/s")
    parser.add_argument("--save", metavar="VIDEO.mp4",
                        help="write the playback to a video instead of opening a window")
    args = parser.parse_args()

    path = args.capture
    if path is None:
        captures = sorted(glob.glob(os.path.join(DEFAULT_ROOT, "*", "images")))
        if not captures:
            sys.exit(f"No captures in {DEFAULT_ROOT}; record one with --ground-truth")
        path = os.path.dirname(captures[-1])
    capture = Capture(path)
    if not capture.frames:
        sys.exit(f"No frames in {path}")
    print(f"{path}: {len(capture.frames)} frames from {', '.join(capture.drones)}, "
          f"{sum(len(b) for b in capture.boxes.values())} boxes, "
          f"{len(capture.tracks)} actor log(s)")

    player = Player(capture, f"Drone{args.drone}" if args.drone else None,
                    args.only_boxes, args.fps)
    if not player.shown:
        sys.exit("No frames match those filters")
    if args.save:
        player.save(args.save)
    else:
        player.run()


if __name__ == "__main__":
    main()
