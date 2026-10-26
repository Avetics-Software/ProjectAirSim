"""mot_eval.py on synthetic recordings with known ID switches, gaps and crowding."""

import csv
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mot_eval import Recording, Tracker, evaluate  # noqa: E402


def write(path, header, rows):
    with open(path, "w", newline="") as f:
        out = csv.writer(f)
        out.writerow(header)
        out.writerows(rows)


def make_run(directory, gt, yaw_rate=0.0, cam_turn=False):
    """gt: {frame: [(actor, x1, y1, x2, y2)]} at 10 frames/s."""
    frames = sorted(gt)
    rows = []
    for f in frames:
        # The camera turns 3 deg per frame (30 deg/s) when cam_turn
        import math
        half = math.radians(3 * f / 2) if cam_turn else 0.0
        rows.append([f, f"{f / 10:.4f}", "0", 0, 0, -40, math.cos(half), 0, 0, math.sin(half),
                     0, 0, 0, 60, 0, -30, 0, "MISSION", 3, len(gt[f])])
    write(os.path.join(directory, "frames_Drone1.csv"),
          ["frame", "sim_time", "wall_time", "cam_x", "cam_y", "cam_z", "cam_qw", "cam_qx",
           "cam_qy", "cam_qz", "cam_roll", "cam_pitch", "cam_yaw", "hfov_deg", "zoom_percent",
           "gimbal_pitch", "gimbal_yaw", "mode", "mission_item", "boxes"], rows)
    write(os.path.join(directory, "boxes_Drone1.csv"),
          ["frame", "sim_time", "actor", "kind", "x1", "y1", "x2", "y2", "pixels", "truncated"],
          [[f, f / 10, a, "vehicle", x1, y1, x2, y2, 400, 0]
           for f in frames for a, x1, y1, x2, y2 in gt[f]])
    write(os.path.join(directory, "telemetry.csv"),
          ["sim_time", "drone", "north", "east", "down", "lat", "lon", "height", "roll",
           "pitch", "yaw", "roll_rate", "pitch_rate", "yaw_rate", "vn", "ve", "vd", "an", "ae",
           "ad", "mode", "mission_item"],
          [[f / 10, "Drone1", 0, 0, -40, 1.4, 103.8, 40, 0, 0, 0, 0, 0, yaw_rate, 3, 4, 0, 0,
            0, 0, "MISSION", 3] for f in frames])


def make_tracker(directory, tracks, detections=None, processed=None):
    """tracks: {frame: [(id, x1, y1, x2, y2)]}."""
    os.makedirs(directory)
    write(os.path.join(directory, "tracks.csv"),
          ["frame", "track_id", "x1", "y1", "x2", "y2", "conf", "label", "state"],
          [[f, tid, x1, y1, x2, y2, 0.9, "car", "confirmed"]
           for f in sorted(tracks) for tid, x1, y1, x2, y2 in tracks[f]])
    if detections is not None:
        write(os.path.join(directory, "detections.csv"),
              ["frame", "label", "conf", "x1", "y1", "x2", "y2"],
              [[f, "car", 0.9, *box] for f in sorted(detections) for box in detections[f]])
    with open(os.path.join(directory, "run_info.json"), "w") as f:
        json.dump({"scale": [1, 1], "input_size": [1920, 1080], "source": "test",
                   "processed": processed}, f)
    return Tracker(directory)


A = (100, 100, 140, 120)
B = (400, 100, 440, 120)


class MotEvalTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.run = self.dir.name

    def tearDown(self):
        self.dir.cleanup()

    def score(self, gt, tracks, **kw):
        make_run(self.run, gt, **{k: kw.pop(k) for k in ("yaw_rate", "cam_turn") if k in kw})
        trk = make_tracker(os.path.join(self.run, "trk"), tracks, kw.pop("detections", None),
                           kw.pop("processed", None))
        return evaluate(Recording(self.run, "Drone1"), trk, **kw)

    def test_perfect_tracking(self):
        gt = {f: [("A", *A), ("B", *B)] for f in range(10)}
        tracks = {f: [(1, *A), (2, *B)] for f in range(10)}
        s, events, _ = self.score(gt, tracks)
        self.assertEqual((s["id_switches"], s["fragmentations"]), (0, 0))
        self.assertEqual(s["idf1"], 1.0)
        self.assertEqual(s["mota"], 1.0)
        self.assertEqual(s["ids_per_vehicle"], {"A": 1, "B": 1})
        self.assertEqual(events, [])

    def test_new_id_after_a_missed_frame_is_a_switch_with_a_detector_gap(self):
        gt = {f: [("A", *A)] for f in range(10)}
        tracks = {f: [(1 if f < 4 else 7, *A)] for f in range(10) if f != 4}
        dets = {f: [A] for f in range(10) if f not in (3, 4)}  # the detector missed 3 and 4
        s, events, _ = self.score(gt, tracks, detections=dets)
        self.assertEqual(s["id_switches"], 1)
        self.assertEqual(s["ids_per_vehicle"], {"A": 2})
        switch = next(e for e in events if e["event"] == "id-switch")
        self.assertEqual((switch["old_id"], switch["new_id"], switch["frame"]), (1, 7, 5))
        self.assertIn("detector-gap", switch["cause"])
        self.assertEqual(s["detector_recall"], 0.8)

    def test_same_id_after_a_gap_is_a_fragmentation_not_a_switch(self):
        gt = {f: [("A", *A)] for f in range(10)}
        tracks = {f: [(1, *A)] for f in range(10) if f not in (4, 5)}
        s, events, _ = self.score(gt, tracks)
        self.assertEqual((s["id_switches"], s["fragmentations"]), (0, 1))
        self.assertEqual(s["misses"], 2)

    def test_swapped_ids_between_neighbours_are_crowding(self):
        a, b = (100, 100, 140, 120), (130, 100, 170, 120)  # overlapping in the image
        gt = {f: [("A", *a), ("B", *b)] for f in range(10)}
        tracks = {f: [(1, *a), (2, *b)] if f < 5 else [(2, *a), (1, *b)] for f in range(10)}
        s, events, _ = self.score(gt, tracks)
        self.assertEqual(s["id_switches"], 2)
        self.assertTrue(all("crowding" in e["cause"] for e in events))
        self.assertLess(s["idf1"], 1.0)

    def test_camera_motion_and_telemetry_join(self):
        gt = {f: [("A", *A)] for f in range(10)}
        tracks = {f: [(1 if f < 5 else 2, *A)] for f in range(10)}
        s, events, _ = self.score(gt, tracks, detections={f: [A] for f in range(10)},
                                  yaw_rate=25.0, cam_turn=True)
        (switch,) = events
        self.assertIn("camera-motion", switch["cause"])
        self.assertNotIn("detector-gap", switch["cause"])
        self.assertAlmostEqual(switch["camera_rate_dps"], 30.0, delta=1.0)
        self.assertEqual(switch["yaw_rate"], 25.0)
        self.assertEqual(switch["speed"], 5.0)

    def test_only_processed_frames_are_scored(self):
        gt = {f: [("A", *A)] for f in range(10)}
        tracks = {f: [(1, *A)] for f in range(0, 10, 2)}
        s, _, _ = self.score(gt, tracks, processed=list(range(0, 10, 2)))
        self.assertEqual((s["frames_scored"], s["misses"], s["fragmentations"]), (5, 0, 0))

    def test_small_true_boxes_are_ignored_with_their_tracks(self):
        tiny = (500, 500, 504, 503)
        gt = {f: [("A", *A), ("T", *tiny)] for f in range(5)}
        tracks = {f: [(1, *A), (9, *tiny)] for f in range(5)}
        s, _, _ = self.score(gt, tracks, min_px=8)
        self.assertEqual((s["vehicles"], s["false_positives"], s["misses"]), (1, 0, 0))


if __name__ == "__main__":
    unittest.main()
